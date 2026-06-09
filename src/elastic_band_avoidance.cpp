// Elastic Band Obstacle Avoidance Node
// ──────────────────────────────────────────────────────────────────────────────
// Subscribes to:
//   • waypointarray          (geometry_msgs/PoseArray)  – original waypoints
//   • current_pose           (geometry_msgs/PoseStamped) – vehicle pose
//   • pointcloud_obstacles   (sensor_msgs/PointCloud2)  – filtered obstacle cloud
//
// Publishes:
//   • waypointarray_modified (geometry_msgs/PoseArray)  – deformed waypoints
//   • eb_viz                 (visualization_msgs/MarkerArray) – RViz debug view
//
// Algorithm: Elastic Band
//   Each waypoint in a lookahead window is treated as a rubber-band node.
//   Every 50 ms (20 Hz) the following forces are integrated for
//   `eband_iterations` steps:
//     1. Spring (internal) : pulls node back toward its original anchor position
//     2. Repulsion         : pushes node away from nearby obstacle points using
//                            the standard Khatib-style potential gradient
//     3. Smoothness        : pulls node toward the average of its two neighbours
//   The resulting displacement from the anchor is hard-clamped to
//   `max_lateral_deviation` (default 9 m).
//   Waypoints outside the window snap back toward their anchor at a fixed rate.
// ─────────────────────────────────────────────────────────────────────────────

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// ─── Internal state for one band node ────────────────────────────────────────
struct BandNode
{
    double orig_x{0.0}, orig_y{0.0};   // anchor (from original waypoint)
    double cur_x{0.0},  cur_y{0.0};    // current elastic position
};

// ─── Node ────────────────────────────────────────────────────────────────────
class ElasticBandAvoidance : public rclcpp::Node
{
    // ── Parameter callback ─────────────────────────────────────────────────
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason     = "success";
        for (const auto & param : parameters) {
            RCLCPP_INFO_STREAM(get_logger(),
                "Param update: " << param.get_name() << " = " << param.value_to_string());
            if      (param.get_name() == "max_lateral_deviation") max_lateral_deviation_ = param.as_double();
            else if (param.get_name() == "spring_constant")       spring_constant_       = param.as_double();
            else if (param.get_name() == "repulsion_gain")        repulsion_gain_        = param.as_double();
            else if (param.get_name() == "influence_radius")      influence_radius_      = param.as_double();
            else if (param.get_name() == "robot_radius")          robot_radius_          = param.as_double();
            else if (param.get_name() == "smoothness_weight")     smoothness_weight_     = param.as_double();
            else if (param.get_name() == "snapback_rate")         snapback_rate_         = param.as_double();
            else if (param.get_name() == "eband_iterations")      eband_iterations_      = param.as_int();
            else if (param.get_name() == "lookahead_count")       lookahead_count_       = param.as_int();
            else if (param.get_name() == "map_frame")             map_frame_             = param.as_string();
        }
        return result;
    }

public:
    ElasticBandAvoidance() : Node("elastic_band_avoidance")
    {
        // ── Declare & read parameters ──────────────────────────────────────
        declare_parameter<double>("max_lateral_deviation", max_lateral_deviation_);
        declare_parameter<double>("spring_constant",       spring_constant_);
        declare_parameter<double>("repulsion_gain",        repulsion_gain_);
        declare_parameter<double>("influence_radius",      influence_radius_);
        declare_parameter<double>("robot_radius",          robot_radius_);
        declare_parameter<double>("smoothness_weight",     smoothness_weight_);
        declare_parameter<double>("snapback_rate",         snapback_rate_);
        declare_parameter<int>   ("eband_iterations",      eband_iterations_);
        declare_parameter<int>   ("lookahead_count",       lookahead_count_);
        declare_parameter<std::string>("waypoint_topic",   waypoint_topic_);
        declare_parameter<std::string>("cloud_topic",      cloud_topic_);
        declare_parameter<std::string>("output_topic",     output_topic_);
        declare_parameter<std::string>("map_frame",        map_frame_);

        get_parameter("max_lateral_deviation", max_lateral_deviation_);
        get_parameter("spring_constant",       spring_constant_);
        get_parameter("repulsion_gain",        repulsion_gain_);
        get_parameter("influence_radius",      influence_radius_);
        get_parameter("robot_radius",          robot_radius_);
        get_parameter("smoothness_weight",     smoothness_weight_);
        get_parameter("snapback_rate",         snapback_rate_);
        get_parameter("eband_iterations",      eband_iterations_);
        get_parameter("lookahead_count",       lookahead_count_);
        get_parameter("waypoint_topic",        waypoint_topic_);
        get_parameter("cloud_topic",           cloud_topic_);
        get_parameter("output_topic",          output_topic_);
        get_parameter("map_frame",             map_frame_);

        callback_handle_ = add_on_set_parameters_callback(
            std::bind(&ElasticBandAvoidance::parametersCallback, this, _1));

        // ── TF ─────────────────────────────────────────────────────────────
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ── Subscriptions ──────────────────────────────────────────────────
        sub_waypoints_ = create_subscription<geometry_msgs::msg::PoseArray>(
            waypoint_topic_, 10,
            std::bind(&ElasticBandAvoidance::waypointCallback, this, _1));

        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "current_pose", 10,
            std::bind(&ElasticBandAvoidance::poseCallback, this, _1));

        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, 10,
            std::bind(&ElasticBandAvoidance::cloudCallback, this, _1));

        // ── Publishers ─────────────────────────────────────────────────────
        pub_modified_ = create_publisher<geometry_msgs::msg::PoseArray>(output_topic_, 10);
        pub_viz_      = create_publisher<visualization_msgs::msg::MarkerArray>("eb_viz", 10);

        // ── 20 Hz optimisation timer ───────────────────────────────────────
        timer_ = create_wall_timer(50ms, std::bind(&ElasticBandAvoidance::timerCallback, this));

        RCLCPP_INFO(get_logger(), "elastic_band_avoidance node started");
        RCLCPP_INFO(get_logger(), "  waypoint_topic      : %s", waypoint_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  cloud_topic         : %s", cloud_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  output_topic        : %s", output_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  max_lateral_deviation: %.2f m", max_lateral_deviation_);
        RCLCPP_INFO(get_logger(), "  influence_radius    : %.2f m", influence_radius_);
        RCLCPP_INFO(get_logger(), "  lookahead_count     : %d wp", lookahead_count_);
    }

private:
    // ── Tunable parameters ─────────────────────────────────────────────────
    double max_lateral_deviation_  = 9.0;  // hard clamp on displacement from anchor (m)
    double spring_constant_        = 0.5;  // spring pull-back gain (per metre of displacement)
    double repulsion_gain_         = 3.0;  // Khatib repulsion gain
    double influence_radius_       = 3.5;  // obstacle influence sphere radius (m)
    double robot_radius_           = 1.2;  // minimum clearance / singularity guard (m)
    double smoothness_weight_      = 0.4;  // weight for neighbour-averaging force
    double snapback_rate_          = 0.08; // fraction snapped back per 50 ms outside window
    int    eband_iterations_       = 8;    // force integration steps per timer tick
    int    lookahead_count_        = 60;   // number of waypoints ahead to process
    std::string waypoint_topic_    = "waypointarray";
    std::string cloud_topic_       = "pointcloud_obstacles";
    std::string output_topic_      = "waypointarray_modified";
    std::string map_frame_         = "map";

    // ── Internal state ─────────────────────────────────────────────────────
    std::vector<BandNode> band_;
    bool band_initialized_ = false;
    bool pose_received_    = false;
    int  closest_idx_      = 0;

    geometry_msgs::msg::PoseStamped current_pose_;

    struct Point2D { double x, y; };
    std::vector<Point2D> obstacle_pts_;   // cached, in map_frame_

    // ── ROS handles ───────────────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr    sub_waypoints_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  sub_pose_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr    sub_cloud_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr       pub_modified_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;
    rclcpp::TimerBase::SharedPtr                                      timer_;
    OnSetParametersCallbackHandle::SharedPtr                          callback_handle_;
    std::unique_ptr<tf2_ros::Buffer>                                  tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>                       tf_listener_;

    // ── Waypoint callback ──────────────────────────────────────────────────
    // Reinitialise band on size change; otherwise only update anchors so that
    // an ongoing deformation is not discarded on every incoming message.
    void waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty()) return;

        const std::size_t n = msg->poses.size();

        if (!band_initialized_ || band_.size() != n) {
            band_.resize(n);
            for (std::size_t i = 0; i < n; ++i) {
                band_[i].orig_x = msg->poses[i].position.x;
                band_[i].orig_y = msg->poses[i].position.y;
                band_[i].cur_x  = band_[i].orig_x;
                band_[i].cur_y  = band_[i].orig_y;
            }
            band_initialized_ = true;
            RCLCPP_INFO(get_logger(), "Elastic band initialised with %zu waypoints", n);
        } else {
            // Shift anchors; keep existing deformation (cur positions unchanged)
            for (std::size_t i = 0; i < n; ++i) {
                band_[i].orig_x = msg->poses[i].position.x;
                band_[i].orig_y = msg->poses[i].position.y;
            }
        }
    }

    // ── Pose callback ──────────────────────────────────────────────────────
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        pose_received_ = true;

        if (!band_initialized_) return;

        // Find closest original anchor (not deformed position) to vehicle
        const double cx = msg->pose.position.x;
        const double cy = msg->pose.position.y;
        double min_d2   = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(band_.size()); ++i) {
            double dx = band_[i].orig_x - cx;
            double dy = band_[i].orig_y - cy;
            double d2 = dx * dx + dy * dy;
            if (d2 < min_d2) { min_d2 = d2; closest_idx_ = i; }
        }
    }

    // ── PointCloud2 callback ───────────────────────────────────────────────
    // Transforms all points to map_frame_ using the latest available TF and
    // caches them for the optimisation loop.
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        obstacle_pts_.clear();
        if (msg->width * msg->height == 0) return;

        // Resolve transform once per cloud message
        geometry_msgs::msg::TransformStamped transform;
        bool need_transform = (msg->header.frame_id != map_frame_);
        if (need_transform) {
            try {
                transform = tf_buffer_->lookupTransform(
                    map_frame_, msg->header.frame_id,
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.05));
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "TF lookup %s→%s failed: %s  (treating cloud as already in map frame)",
                    msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
                need_transform = false;
            }
        }

        try {
            sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

            for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
                if (!std::isfinite(*it_x) || !std::isfinite(*it_y)) continue;

                double px = static_cast<double>(*it_x);
                double py = static_cast<double>(*it_y);
                double pz = static_cast<double>(*it_z);

                if (need_transform) {
                    geometry_msgs::msg::PointStamped p_in, p_out;
                    p_in.header   = msg->header;
                    p_in.point.x  = px;
                    p_in.point.y  = py;
                    p_in.point.z  = pz;
                    tf2::doTransform(p_in, p_out, transform);
                    px = p_out.point.x;
                    py = p_out.point.y;
                }
                obstacle_pts_.push_back({px, py});
            }
        } catch (const std::exception & ex) {
            RCLCPP_WARN(get_logger(), "PointCloud2 parsing error: %s", ex.what());
        }
    }

    // ── Main optimisation loop (20 Hz) ─────────────────────────────────────
    void timerCallback()
    {
        if (!band_initialized_ || !pose_received_) return;

        const int n            = static_cast<int>(band_.size());
        const int win_start    = std::max(0, closest_idx_ + 1);
        const int win_end      = std::min(n - 1, closest_idx_ + lookahead_count_);

        // ── Force integration ──────────────────────────────────────────────
        for (int iter = 0; iter < eband_iterations_; ++iter) {
            for (int i = win_start; i <= win_end; ++i) {
                const double x = band_[i].cur_x;
                const double y = band_[i].cur_y;

                // 1. Spring force – pulls node back to anchor
                double fx = (band_[i].orig_x - x) * spring_constant_;
                double fy = (band_[i].orig_y - y) * spring_constant_;

                // 2. Repulsion – Khatib potential gradient
                //    U = 0.5 * gain * (1/d - 1/rho)^2   for d < rho
                //    F = -dU/dr  (pointing away from obstacle)
                for (const auto & obs : obstacle_pts_) {
                    const double dx   = x - obs.x;
                    const double dy   = y - obs.y;
                    double dist = std::sqrt(dx * dx + dy * dy);
                    if (dist < robot_radius_) dist = robot_radius_; // singularity guard
                    if (dist < influence_radius_) {
                        const double coeff = repulsion_gain_
                            * (1.0 / dist - 1.0 / influence_radius_)
                            / (dist * dist);
                        fx += coeff * (dx / dist);
                        fy += coeff * (dy / dist);
                    }
                }

                // 3. Smoothness – pull toward average of neighbours
                if (i > 0 && i < n - 1) {
                    const double mid_x = 0.5 * (band_[i - 1].cur_x + band_[i + 1].cur_x);
                    const double mid_y = 0.5 * (band_[i - 1].cur_y + band_[i + 1].cur_y);
                    fx += (mid_x - x) * smoothness_weight_;
                    fy += (mid_y - y) * smoothness_weight_;
                }

                double new_x = x + fx;
                double new_y = y + fy;

                // 4. Hard clamp: displacement from anchor ≤ max_lateral_deviation_
                const double sh_x = new_x - band_[i].orig_x;
                const double sh_y = new_y - band_[i].orig_y;
                const double sh_d = std::sqrt(sh_x * sh_x + sh_y * sh_y);
                if (sh_d > max_lateral_deviation_) {
                    const double inv = max_lateral_deviation_ / sh_d;
                    new_x = band_[i].orig_x + sh_x * inv;
                    new_y = band_[i].orig_y + sh_y * inv;
                }

                band_[i].cur_x = new_x;
                band_[i].cur_y = new_y;
            }
        }

        // ── Snap-back for nodes outside the active window ──────────────────
        for (int i = 0; i < n; ++i) {
            if (i >= win_start && i <= win_end) continue;
            band_[i].cur_x += (band_[i].orig_x - band_[i].cur_x) * snapback_rate_;
            band_[i].cur_y += (band_[i].orig_y - band_[i].cur_y) * snapback_rate_;
        }

        publishModified(n);
        publishVisualization(win_start, win_end, n);
    }

    // ── Publish the full modified waypoint array ───────────────────────────
    void publishModified(int n)
    {
        geometry_msgs::msg::PoseArray out;
        out.header.stamp    = now();
        out.header.frame_id = map_frame_;
        out.poses.resize(static_cast<std::size_t>(n));

        for (int i = 0; i < n; ++i) {
            out.poses[i].position.x = band_[i].cur_x;
            out.poses[i].position.y = band_[i].cur_y;
            out.poses[i].position.z = 0.0;

            // Recompute yaw from direction to next node
            double yaw = 0.0;
            if (i < n - 1) {
                yaw = std::atan2(band_[i + 1].cur_y - band_[i].cur_y,
                                 band_[i + 1].cur_x - band_[i].cur_x);
            } else if (i > 0) {
                yaw = std::atan2(band_[i].cur_y - band_[i - 1].cur_y,
                                 band_[i].cur_x - band_[i - 1].cur_x);
            }
            const double half_yaw = yaw * 0.5;
            out.poses[i].orientation.w = std::cos(half_yaw);
            out.poses[i].orientation.z = std::sin(half_yaw);
            out.poses[i].orientation.x = 0.0;
            out.poses[i].orientation.y = 0.0;
        }
        pub_modified_->publish(out);
    }

    // ── Publish RViz visualisation ─────────────────────────────────────────
    void publishVisualization(int win_start, int win_end, int n)
    {
        if (pub_viz_->get_subscription_count() == 0) return;

        visualization_msgs::msg::MarkerArray ma;
        const auto now_stamp = now();

        auto add_sphere = [&](const std::string & ns, int id,
                              double x, double y,
                              float r, float g, float b, double scale)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = map_frame_;
            m.header.stamp    = now_stamp;
            m.ns              = ns;
            m.id              = id;
            m.type            = visualization_msgs::msg::Marker::SPHERE;
            m.action          = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = x;
            m.pose.position.y = y;
            m.pose.position.z = 0.05;
            m.pose.orientation.w = 1.0;
            m.scale.x = scale; m.scale.y = scale; m.scale.z = scale;
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.85f;
            m.lifetime = rclcpp::Duration::from_seconds(0.25);
            ma.markers.push_back(m);
        };

        // Anchor positions in window → green
        for (int i = win_start; i <= win_end; ++i)
            add_sphere("eb_anchor",   i,      band_[i].orig_x, band_[i].orig_y, 0.0f, 0.85f, 0.0f, 0.22);

        // Active band positions → orange
        for (int i = win_start; i <= win_end; ++i)
            add_sphere("eb_modified", i + n,  band_[i].cur_x,  band_[i].cur_y,  1.0f, 0.5f, 0.0f, 0.32);

        // Obstacle points → red
        for (std::size_t i = 0; i < obstacle_pts_.size(); ++i)
            add_sphere("eb_obstacles", static_cast<int>(i),
                       obstacle_pts_[i].x, obstacle_pts_[i].y, 0.9f, 0.1f, 0.1f, 0.18);

        // Line strip: original path in window → thin white
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = map_frame_;
            line.header.stamp    = now_stamp;
            line.ns              = "eb_anchor_line";
            line.id              = 0;
            line.type            = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action          = visualization_msgs::msg::Marker::ADD;
            line.scale.x         = 0.06;
            line.color.r = 0.6f; line.color.g = 0.6f; line.color.b = 0.6f; line.color.a = 0.6f;
            line.lifetime        = rclcpp::Duration::from_seconds(0.25);
            line.pose.orientation.w = 1.0;
            for (int i = win_start; i <= win_end; ++i) {
                geometry_msgs::msg::Point p;
                p.x = band_[i].orig_x; p.y = band_[i].orig_y; p.z = 0.05;
                line.points.push_back(p);
            }
            ma.markers.push_back(line);
        }

        // Line strip: modified path in window → bright yellow
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = map_frame_;
            line.header.stamp    = now_stamp;
            line.ns              = "eb_modified_line";
            line.id              = 0;
            line.type            = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action          = visualization_msgs::msg::Marker::ADD;
            line.scale.x         = 0.10;
            line.color.r = 1.0f; line.color.g = 0.9f; line.color.b = 0.0f; line.color.a = 1.0f;
            line.lifetime        = rclcpp::Duration::from_seconds(0.25);
            line.pose.orientation.w = 1.0;
            for (int i = win_start; i <= win_end; ++i) {
                geometry_msgs::msg::Point p;
                p.x = band_[i].cur_x; p.y = band_[i].cur_y; p.z = 0.05;
                line.points.push_back(p);
            }
            ma.markers.push_back(line);
        }

        pub_viz_->publish(ma);
    }
};

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ElasticBandAvoidance>());
    rclcpp::shutdown();
    return 0;
}
