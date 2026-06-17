// Pointcloud Trajectory Avoidance Node
// ─────────────────────────────────────────────────────────────────────────────
// Subscribes to:
//   • targetpoints            (geometry_msgs/PoseArray)   – local trajectory
//   • pointcloud_obstacles    (sensor_msgs/PointCloud2)   – obstacle cloud
//
// Publishes:
//   • targetpoints_modified   (geometry_msgs/PoseArray)   – avoidance trajectory
//   • ptc_avoidance_viz       (visualization_msgs/MarkerArray) – RViz debug view
//
// Algorithm:
//   1. For every incoming PointCloud2 message, iterate over all (x,y) points.
//   2. For each point check the perpendicular distance to every segment of the
//      local trajectory.  A point is "on the path" when it falls within
//      `corridor_width` (half-width) of any segment, within `lookahead_count`
//      waypoints ahead.
//   3. Count the total number of such points.  When the count reaches or
//      exceeds `min_obstacle_points` the avoidance offset is activated.
//   4. Each waypoint in the trajectory is shifted laterally by `offset_distance`
//      metres in the `avoidance_direction` ("left", "right", or "auto").
//      "auto" shifts away from the centroid of the detected obstacle points.
//   5. When no obstacle is detected the original trajectory is forwarded
//      unchanged.
// ─────────────────────────────────────────────────────────────────────────────

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// ─── Node ────────────────────────────────────────────────────────────────────
class PointcloudTrajectoryAvoidance : public rclcpp::Node
{
    // ── Parameter callback ────────────────────────────────────────────────
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason     = "success";
        for (const auto & param : parameters) {
            RCLCPP_INFO_STREAM(get_logger(),
                "Param update: " << param.get_name() << " = " << param.value_to_string());
            if      (param.get_name() == "corridor_width")      corridor_width_      = param.as_double();
            else if (param.get_name() == "min_obstacle_points") min_obstacle_points_ = param.as_int();
            else if (param.get_name() == "offset_distance")     offset_distance_     = param.as_double();
            else if (param.get_name() == "avoidance_direction") avoidance_direction_ = param.as_string();
            else if (param.get_name() == "lookahead_count")     lookahead_count_     = param.as_int();
            else if (param.get_name() == "trajectory_topic")    trajectory_topic_    = param.as_string();
            else if (param.get_name() == "cloud_topic")         cloud_topic_         = param.as_string();
            else if (param.get_name() == "output_topic")        output_topic_        = param.as_string();
        }
        return result;
    }

public:
    PointcloudTrajectoryAvoidance() : Node("pointcloud_trajectory_avoidance")
    {
        // ── Declare & read parameters ─────────────────────────────────────
        declare_parameter<double>("corridor_width",      corridor_width_);
        declare_parameter<int>   ("min_obstacle_points", min_obstacle_points_);
        declare_parameter<double>("offset_distance",     offset_distance_);
        declare_parameter<std::string>("avoidance_direction", avoidance_direction_);
        declare_parameter<int>   ("lookahead_count",     lookahead_count_);
        declare_parameter<std::string>("trajectory_topic", trajectory_topic_);
        declare_parameter<std::string>("cloud_topic",    cloud_topic_);
        declare_parameter<std::string>("output_topic",   output_topic_);

        get_parameter("corridor_width",      corridor_width_);
        get_parameter("min_obstacle_points", min_obstacle_points_);
        get_parameter("offset_distance",     offset_distance_);
        get_parameter("avoidance_direction", avoidance_direction_);
        get_parameter("lookahead_count",     lookahead_count_);
        get_parameter("trajectory_topic",    trajectory_topic_);
        get_parameter("cloud_topic",         cloud_topic_);
        get_parameter("output_topic",        output_topic_);

        callback_handle_ = add_on_set_parameters_callback(
            std::bind(&PointcloudTrajectoryAvoidance::parametersCallback, this, _1));

        // ── TF ────────────────────────────────────────────────────────────
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ── Subscriptions ─────────────────────────────────────────────────
        sub_trajectory_ = create_subscription<geometry_msgs::msg::PoseArray>(
            trajectory_topic_, 10,
            std::bind(&PointcloudTrajectoryAvoidance::trajectoryCallback, this, _1));

        sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, 10,
            std::bind(&PointcloudTrajectoryAvoidance::cloudCallback, this, _1));

        // ── Publishers ────────────────────────────────────────────────────
        pub_modified_ = create_publisher<geometry_msgs::msg::PoseArray>(output_topic_, 10);
        pub_viz_      = create_publisher<visualization_msgs::msg::MarkerArray>("ptc_avoidance_viz", 10);

        RCLCPP_INFO(get_logger(), "pointcloud_trajectory_avoidance node started");
        RCLCPP_INFO(get_logger(), "  trajectory_topic  : %s", trajectory_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  cloud_topic       : %s", cloud_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  output_topic      : %s", output_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  corridor_width    : %.2f m", corridor_width_);
        RCLCPP_INFO(get_logger(), "  min_obstacle_pts  : %d", min_obstacle_points_);
        RCLCPP_INFO(get_logger(), "  offset_distance   : %.2f m", offset_distance_);
        RCLCPP_INFO(get_logger(), "  avoidance_dir     : %s", avoidance_direction_.c_str());
        RCLCPP_INFO(get_logger(), "  lookahead_count   : %d", lookahead_count_);
    }

private:
    // ── Tunable parameters ────────────────────────────────────────────────
    double      corridor_width_      = 1.5;   // half-width of the trajectory corridor (m)
    int         min_obstacle_points_ = 5;     // point count threshold to trigger avoidance
    double      offset_distance_     = 2.0;   // lateral avoidance offset (m)
    std::string avoidance_direction_ = "left"; // "left", "right", or "auto"
    int         lookahead_count_     = 20;    // max waypoints ahead to check
    std::string trajectory_topic_    = "targetpoints";
    std::string cloud_topic_         = "pointcloud_obstacles";
    std::string output_topic_        = "targetpoints_modified";

    // ── Internal state ────────────────────────────────────────────────────
    geometry_msgs::msg::PoseArray::SharedPtr trajectory_;

    struct Point2D { double x, y; };
    std::vector<Point2D> obstacle_pts_in_corridor_; // points that fall inside the corridor

    // ── ROS handles ───────────────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr  sub_trajectory_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  sub_cloud_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr     pub_modified_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;
    OnSetParametersCallbackHandle::SharedPtr                         callback_handle_;
    std::unique_ptr<tf2_ros::Buffer>                                  tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>                       tf_listener_;

    // ── Helper: perpendicular distance from point P to segment AB ─────────
    // Returns the minimum distance from (px,py) to the finite line segment
    // between (ax,ay) and (bx,by).
    static double distPointToSegment(double px, double py,
                                     double ax, double ay,
                                     double bx, double by)
    {
        const double dx = bx - ax;
        const double dy = by - ay;
        const double len2 = dx * dx + dy * dy;

        if (len2 < 1e-9) {
            // Degenerate segment – treat as a point
            return std::hypot(px - ax, py - ay);
        }

        // Parameter t of the closest point on the infinite line
        double t = ((px - ax) * dx + (py - ay) * dy) / len2;
        t = std::max(0.0, std::min(1.0, t));   // clamp to segment

        const double cx = ax + t * dx;
        const double cy = ay + t * dy;
        return std::hypot(px - cx, py - cy);
    }

    // ── Helper: determine the sign of a point relative to segment ─────────
    // Returns +1 if point is on the left side of AB (from A looking toward B),
    // −1 if on the right side.
    static double sideOfSegment(double px, double py,
                                 double ax, double ay,
                                 double bx, double by)
    {
        // Cross product of (AB) x (AP)
        return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    }

    // ── Trajectory callback ───────────────────────────────────────────────
    void trajectoryCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        trajectory_ = msg;
    }

    // ── PointCloud2 callback ──────────────────────────────────────────────
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!trajectory_ || trajectory_->poses.empty()) return;

        const int n_wp = static_cast<int>(trajectory_->poses.size());
        const int check_count = std::min(lookahead_count_, n_wp);

        // Collect cloud points that fall within the trajectory corridor
        obstacle_pts_in_corridor_.clear();

        // ── Frame transform: cloud frame → trajectory frame ───────────────
        const std::string traj_frame  = trajectory_->header.frame_id;
        const std::string cloud_frame = msg->header.frame_id;
        double tf_tx = 0.0, tf_ty = 0.0, tf_cos = 1.0, tf_sin = 0.0;
        const bool same_frame = (traj_frame == cloud_frame)
                                || traj_frame.empty() || cloud_frame.empty();
        if (!same_frame) {
            try {
                const geometry_msgs::msg::TransformStamped ts =
                    tf_buffer_->lookupTransform(traj_frame, cloud_frame, tf2::TimePointZero);

                tf_tx = ts.transform.translation.x;
                tf_ty = ts.transform.translation.y;

                tf2::Quaternion q;
                tf2::fromMsg(ts.transform.rotation, q);
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                tf_cos = std::cos(yaw);
                tf_sin = std::sin(yaw);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "TF lookup failed (%s → %s): %s – skipping cloud message",
                    cloud_frame.c_str(), traj_frame.c_str(), ex.what());
                return;
            }
        }

        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");

        for (; it_x != it_x.end(); ++it_x, ++it_y) {
            const double px_raw = static_cast<double>(*it_x);
            const double py_raw = static_cast<double>(*it_y);

            // Skip NaN/Inf points
            if (!std::isfinite(px_raw) || !std::isfinite(py_raw)) continue;

            // Apply 2-D rigid transform into the trajectory frame
            const double px = tf_cos * px_raw - tf_sin * py_raw + tf_tx;
            const double py = tf_sin * px_raw + tf_cos * py_raw + tf_ty;

            // Check against each trajectory segment within lookahead window
            for (int i = 0; i < check_count - 1; ++i) {
                const double ax = trajectory_->poses[i].position.x;
                const double ay = trajectory_->poses[i].position.y;
                const double bx = trajectory_->poses[i + 1].position.x;
                const double by = trajectory_->poses[i + 1].position.y;

                if (distPointToSegment(px, py, ax, ay, bx, by) <= corridor_width_) {
                    obstacle_pts_in_corridor_.push_back({px, py});
                    break; // count each cloud point once
                }
            }
        }

        const int obstacle_count = static_cast<int>(obstacle_pts_in_corridor_.size());
        const bool avoidance_active = (obstacle_count >= min_obstacle_points_);

        if (avoidance_active) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Obstacle detected: %d points in corridor – activating avoidance",
                obstacle_count);
        }

        // Build and publish modified trajectory
        geometry_msgs::msg::PoseArray modified = *trajectory_;

        if (avoidance_active) {
            applyLateralOffset(modified);
        }

        modified.header.stamp = now();
        pub_modified_->publish(modified);

        // Publish visualisation markers
        publishViz(modified, avoidance_active);
    }

    // ── Lateral offset ────────────────────────────────────────────────────
    void applyLateralOffset(geometry_msgs::msg::PoseArray & traj)
    {
        const int n = static_cast<int>(traj.poses.size());
        if (n < 2) return;

        // Determine offset sign from avoidance_direction_
        // For "auto": compute centroid of obstacle points and offset away from it
        double sign = 1.0; // +1 = left, −1 = right

        if (avoidance_direction_ == "right") {
            sign = -1.0;
        } else if (avoidance_direction_ == "auto" && !obstacle_pts_in_corridor_.empty()) {
            // Compute centroid of obstacle points in corridor
            double cx = 0.0, cy = 0.0;
            for (const auto & p : obstacle_pts_in_corridor_) { cx += p.x; cy += p.y; }
            cx /= obstacle_pts_in_corridor_.size();
            cy /= obstacle_pts_in_corridor_.size();

            // Use first segment direction to decide side
            const double ax = traj.poses[0].position.x;
            const double ay = traj.poses[0].position.y;
            const double bx = traj.poses[1].position.x;
            const double by = traj.poses[1].position.y;

            // Obstacle is on the left side when cross product > 0 → offset right
            const double cross = sideOfSegment(cx, cy, ax, ay, bx, by);
            sign = (cross > 0.0) ? -1.0 : 1.0;
        }
        // "left" uses default sign = +1.0

        // Apply lateral offset to every waypoint
        for (int i = 0; i < n; ++i) {
            // Compute local tangent direction at waypoint i
            double tx, ty;
            if (i < n - 1) {
                tx = traj.poses[i + 1].position.x - traj.poses[i].position.x;
                ty = traj.poses[i + 1].position.y - traj.poses[i].position.y;
            } else {
                tx = traj.poses[i].position.x - traj.poses[i - 1].position.x;
                ty = traj.poses[i].position.y - traj.poses[i - 1].position.y;
            }

            const double len = std::hypot(tx, ty);
            if (len < 1e-6) continue;

            // Left-perpendicular: rotate tangent +90°
            const double nx = -ty / len;
            const double ny =  tx / len;

            traj.poses[i].position.x += sign * offset_distance_ * nx;
            traj.poses[i].position.y += sign * offset_distance_ * ny;
        }
    }

    // ── Visualisation ─────────────────────────────────────────────────────
    void publishViz(const geometry_msgs::msg::PoseArray & traj, bool avoidance_active)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete all previous markers
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        // Marker 1: trajectory line strip
        {
            visualization_msgs::msg::Marker m;
            m.header   = traj.header;
            m.header.stamp = now();
            m.ns       = "trajectory";
            m.id       = 0;
            m.type     = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action   = visualization_msgs::msg::Marker::ADD;
            m.scale.x  = 0.1;
            m.color.a  = 1.0;
            if (avoidance_active) {
                m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; // orange
            } else {
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; // green
            }
            for (const auto & pose : traj.poses) {
                geometry_msgs::msg::Point p;
                p.x = pose.position.x;
                p.y = pose.position.y;
                p.z = pose.position.z;
                m.points.push_back(p);
            }
            marker_array.markers.push_back(m);
        }

        // Marker 2: corridor left edge (cyan line strip)
        {
            visualization_msgs::msg::Marker m;
            m.header        = traj.header;
            m.header.stamp  = now();
            m.ns            = "corridor";
            m.id            = 1;
            m.type          = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action        = visualization_msgs::msg::Marker::ADD;
            m.scale.x       = 0.05;
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.6;

            const int n_viz = static_cast<int>(traj.poses.size());
            for (int i = 0; i < n_viz; ++i) {
                double tx, ty;
                if (i < n_viz - 1) {
                    tx = traj.poses[i + 1].position.x - traj.poses[i].position.x;
                    ty = traj.poses[i + 1].position.y - traj.poses[i].position.y;
                } else {
                    tx = traj.poses[i].position.x - traj.poses[i - 1].position.x;
                    ty = traj.poses[i].position.y - traj.poses[i - 1].position.y;
                }
                const double len = std::hypot(tx, ty);
                if (len < 1e-6) continue;
                // left-perpendicular
                const double nx = -ty / len;
                const double ny =  tx / len;
                geometry_msgs::msg::Point p;
                p.x = traj.poses[i].position.x + corridor_width_ * nx;
                p.y = traj.poses[i].position.y + corridor_width_ * ny;
                p.z = traj.poses[i].position.z;
                m.points.push_back(p);
            }
            marker_array.markers.push_back(m);
        }

        // Marker 3: corridor right edge (cyan line strip)
        {
            visualization_msgs::msg::Marker m;
            m.header        = traj.header;
            m.header.stamp  = now();
            m.ns            = "corridor";
            m.id            = 2;
            m.type          = visualization_msgs::msg::Marker::LINE_STRIP;
            m.action        = visualization_msgs::msg::Marker::ADD;
            m.scale.x       = 0.05;
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.6;

            const int n_viz = static_cast<int>(traj.poses.size());
            for (int i = 0; i < n_viz; ++i) {
                double tx, ty;
                if (i < n_viz - 1) {
                    tx = traj.poses[i + 1].position.x - traj.poses[i].position.x;
                    ty = traj.poses[i + 1].position.y - traj.poses[i].position.y;
                } else {
                    tx = traj.poses[i].position.x - traj.poses[i - 1].position.x;
                    ty = traj.poses[i].position.y - traj.poses[i - 1].position.y;
                }
                const double len = std::hypot(tx, ty);
                if (len < 1e-6) continue;
                // right-perpendicular
                const double nx =  ty / len;
                const double ny = -tx / len;
                geometry_msgs::msg::Point p;
                p.x = traj.poses[i].position.x + corridor_width_ * nx;
                p.y = traj.poses[i].position.y + corridor_width_ * ny;
                p.z = traj.poses[i].position.z;
                m.points.push_back(p);
            }
            marker_array.markers.push_back(m);
        }

        // Marker 4: obstacle points in corridor (red spheres)
        if (!obstacle_pts_in_corridor_.empty()) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = traj.header.frame_id;
            m.header.stamp    = now();
            m.ns              = "obstacle_pts";
            m.id              = 1;
            m.type            = visualization_msgs::msg::Marker::SPHERE_LIST;
            m.action          = visualization_msgs::msg::Marker::ADD;
            m.scale.x = m.scale.y = m.scale.z = 0.3;
            m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.8;
            for (const auto & p : obstacle_pts_in_corridor_) {
                geometry_msgs::msg::Point pt;
                pt.x = p.x; pt.y = p.y; pt.z = 0.0;
                m.points.push_back(pt);
            }
            marker_array.markers.push_back(m);
        }

        pub_viz_->publish(marker_array);
    }
};

// ─── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointcloudTrajectoryAvoidance>());
    rclcpp::shutdown();
    return 0;
}
