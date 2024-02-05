#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "wayp_plan_tools/common.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
class WaypointToTarget : public rclcpp::Node
{
public:
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param : parameters)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
            if (param.get_name() == "lookahead_min")
            {
                lookahead_min = param.as_double();
            }
            if (param.get_name() == "lookahead_max")
            {
                lookahead_max = param.as_double();
            }
            if (param.get_name() == "mps_alpha")
            {
                mps_alpha = param.as_double();
            }
            if (param.get_name() == "mps_beta")
            {
                mps_beta = param.as_double();
            }
            if (param.get_name() == "static_speed_enabled")
            {
                static_speed_enabled = param.as_bool();
            }
            if (param.get_name() == "static_speed")
            {
                static_speed = param.as_double();
            }
            if (param.get_name() == "interpolate_waypoints")
            {
                interpolate_waypoints = param.as_bool();
            }
        }
        return result;
    }
    WaypointToTarget() : Node("waypoint_to_target_node")
    {
        metrics_arr.data.resize(common_wpt::NOT_USED_YET); // initialize the metrics array
        pursuit_vizu_arr.markers.resize(3);                // initialize visualization
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Call timer_callback function 20 Hz, 50 milliseconds
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&WaypointToTarget::timer_callback, this));

        this->declare_parameter<std::string>("waypoint_topic", "");
        this->get_parameter("waypoint_topic", waypoint_topic);
        this->declare_parameter<double>("lookahead_min", 9.0);
        this->get_parameter("lookahead_min", lookahead_min);
        this->declare_parameter<double>("lookahead_max", 14.5);
        this->get_parameter("lookahead_max", lookahead_max);
        this->declare_parameter<double>("mps_alpha", 3.2);
        this->get_parameter("mps_alpha", mps_alpha);
        this->declare_parameter<double>("mps_beta", 5.2);
        this->get_parameter("mps_beta", mps_beta);
        this->declare_parameter<double>("static_speed", 4.0);
        this->get_parameter("static_speed", static_speed);
        this->declare_parameter<bool>("static_speed_enabled", false);
        this->get_parameter("static_speed_enabled", static_speed_enabled);
        this->declare_parameter<std::string>("tf_frame_id", tf_frame_id);
        this->get_parameter("tf_frame_id", tf_frame_id);
        this->declare_parameter<std::string>("tf_child_frame_id", tf_child_frame_id);
        this->get_parameter("tf_child_frame_id", tf_child_frame_id);
        this->declare_parameter<bool>("interpolate_waypoints", false);
        this->get_parameter("interpolate_waypoints", interpolate_waypoints);

        if (waypoint_topic == "")
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "waypoint_topic is not set using default value: waypointarray");
            waypoint_topic = "waypointarray";
        }

        goal_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pursuitviz", 10);
        speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("pursuitspeedtarget", 10);
        target_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("targetpoints", 10);
        metrics_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("metrics_wayp", 10);
        sub_w_ = this->create_subscription<geometry_msgs::msg::PoseArray>(waypoint_topic, 10, std::bind(&WaypointToTarget::waypointCallback, this, _1));
        sub_s_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("waypointarray_speeds", 10, std::bind(&WaypointToTarget::speedCallback, this, _1));
        sub_reinit_ = this->create_subscription<std_msgs::msg::Bool>("control_reinit", 10, std::bind(&WaypointToTarget::reinitCallback, this, _1));
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&WaypointToTarget::parametersCallback, this, std::placeholders::_1));
        RCLCPP_INFO_STREAM(this->get_logger(), "waypoint_to_target node started");
        RCLCPP_INFO_STREAM(this->get_logger(), "lookahead_min: " << lookahead_min << " lookahead_max: " << lookahead_max << " mps_alpha: " << mps_alpha << " mps_beta: " << mps_beta);
    }

private:
    double distanceFromWayPoint(const geometry_msgs::msg::Pose &waypoint, const geometry_msgs::msg::Pose &pose_curr)
    {
        double dx = waypoint.position.x - pose_curr.position.x;
        double dy = waypoint.position.y - pose_curr.position.y;
        return sqrt(dx * dx + dy * dy);
    }

    double calcLookahead(const double speed_mps)
    {
        double actual_lookahead = lookahead_min;
        if (speed_mps < mps_alpha)
        {
            actual_lookahead = lookahead_min;
        }
        else if (speed_mps > mps_beta)
        {
            actual_lookahead = lookahead_max;
        }
        else
        {
            // slope from lookahead_min to lookahead_max between mps_alpha and mps_beta
            actual_lookahead = (lookahead_max - lookahead_min) / (mps_beta - mps_alpha) * (speed_mps - mps_alpha) + lookahead_min;
        }
        return actual_lookahead;
    }
    geometry_msgs::msg::Point pointAtDistance(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2, double distance, geometry_msgs::msg::Point circle_center)
    {

        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;

        double A = dx * dx + dy * dy;
        double B = 2 * (dx * (p1.x - circle_center.x) + dy * (p1.y - circle_center.y));
        double C = (p1.x - circle_center.x) * (p1.x - circle_center.x) + (p1.y - circle_center.y) * (p1.y - circle_center.y) - distance * distance;

        double discriminant = B * B - 4 * A * C;
        geometry_msgs::msg::Point intersection;
        intersection.x = p2.x;
        intersection.y = p2.y;
        if (discriminant < 0)
        {
            // No intersection
        }
        else
        {
            // t1 and t2 are the parameters to be used in the parametric line equations.
            double t1 = (-B + std::sqrt(discriminant)) / (2 * A);
            double t2 = (-B - std::sqrt(discriminant)) / (2 * A);
            if (t1 >= 0 && t1 <= 1)
            {
                intersection.x = (p1.x + t1 * dx);
                intersection.y = (p1.y + t1 * dy);
            }

            if (t2 >= 0 && t2 <= 1)
            {
                intersection.x = (p1.x + t2 * dx);
                intersection.y = (p1.y + t2 * dy);
            }
        }

        return intersection;
    }

    void waypointCallback(const geometry_msgs::msg::PoseArray &msg)
    {
        target_pose_arr.poses.clear();
        pursuit_goal.header.frame_id = tf_frame_id;
        pursuit_goal.texture.header.frame_id = tf_frame_id;
        pursuit_goal.header.stamp = this->now();
        pursuit_goal.ns = "pursuit_goal";
        pursuit_goal.id = 0;
        pursuit_goal.scale.x = 1.2;
        pursuit_goal.scale.y = 0.6;
        pursuit_goal.scale.z = 0.8;
        // https://github.com/jkk-research/colors
        // md_amber_500 -- 1.00 0.76 0.03
        pursuit_goal.color.r = 1.00;
        pursuit_goal.color.g = 0.76;
        pursuit_goal.color.b = 0.03;
        pursuit_goal.color.a = 1.0;
        pursuit_goal.type = visualization_msgs::msg::Marker::CYLINDER;
        pursuit_goal.action = visualization_msgs::msg::Marker::MODIFY;
        pursuit_closest.header.frame_id = tf_frame_id;
        pursuit_closest.texture.header.frame_id = tf_frame_id;
        pursuit_closest.header.stamp = this->now();
        pursuit_closest.ns = "pursuit_closest";
        pursuit_closest.id = 1;
        pursuit_closest.scale.x = 1.2;
        pursuit_closest.scale.y = 0.6;
        pursuit_closest.scale.z = 2.8;
        // https://github.com/jkk-research/colors
        // md_pink_500 -- 0.91 0.12 0.39
        pursuit_closest.color.r = 0.91;
        pursuit_closest.color.g = 0.12;
        pursuit_closest.color.b = 0.39;
        pursuit_closest.color.a = 1.0;
        pursuit_closest.type = visualization_msgs::msg::Marker::CYLINDER;
        pursuit_closest.action = visualization_msgs::msg::Marker::MODIFY;
        cross_track_marker.header.frame_id = tf_frame_id;
        cross_track_marker.texture.header.frame_id = tf_frame_id;
        cross_track_marker.header.stamp = this->now();
        cross_track_marker.ns = "cross_track_marker";
        cross_track_marker.id = 2;
        cross_track_marker.scale.x = 0.8;
        cross_track_marker.scale.y = 0.8;
        cross_track_marker.scale.z = 1.8;
        // https://github.com/jkk-research/colors
        // md_amber_500 -- 1.00 0.76 0.03
        cross_track_marker.color.r = 1.00;
        cross_track_marker.color.g = 0.76;
        cross_track_marker.color.b = 0.03;
        cross_track_marker.color.a = 1.0;
        cross_track_marker.pose.position.z = 1.8;
        cross_track_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        cross_track_marker.action = visualization_msgs::msg::Marker::MODIFY;
        int target_waypoint = 0;                 // target waypoint to be published
        int first_wp = 0;                        // first waypoint in the trajectory
        int last_wp = int(msg.poses.size() - 1); // last waypoint in the trajectory
        // determine whether trajectory is linear (loop closure false) circular (loop closure true).
        // loop closure is true, when the first and last waypoint are closer than 4.0 meters
        if (distanceFromWayPoint(msg.poses[first_wp], msg.poses[last_wp]) < 4.0)
        {
            traj_closed_loop = true;
        }
        else
        {
            traj_closed_loop = false;
        }
        // RCLCPP_INFO_STREAM(this->get_logger(), "traj_closed_loop: " << traj_closed_loop);
        // find the closest waypoint where distanceFromWayPoint(msg.poses[i], current_pose) is the smallest
        double smallest_curr_distance = 100000000; // std::numeric_limits<double>::max();
        // in a circular loop trajectory, jump to the beginning of the trajectory when the end is reached
        if (traj_closed_loop == true)
        {
            if (closest_waypoint > last_wp - 10)
            {
                closest_waypoint = first_wp;
            }
        }
        // search for the closest waypoint within a search range (search_start, search_end)
        int search_start = closest_waypoint - 10;
        int search_end = closest_waypoint + 30;
        if (search_start < first_wp)
        {
            search_start = first_wp;
        }
        if (search_end > last_wp)
        {
            search_end = last_wp;
        }
        // if got a reinit signal, search in the whole trajectory
        if (reinit == true)
        {
            search_start = first_wp;
            search_end = last_wp;
            reinit = false;
        }
        for (int i = search_start; i <= search_end; i++)
        {
            if (smallest_curr_distance > distanceFromWayPoint(msg.poses[i], current_pose))
            {
                closest_waypoint = i;
                smallest_curr_distance = distanceFromWayPoint(msg.poses[i], current_pose);
            }
        }

        // calculate the average distance
        if (average_distance_count == 0)
        {
            average_distance = smallest_curr_distance;
        }
        else
        {
            average_distance = (average_distance * average_distance_count + smallest_curr_distance) / (average_distance_count + 1);
        }
        // calculate the maximum distance
        if (maximum_distance < smallest_curr_distance)
        {
            // only update the maximum distance if current distance makes sense (less than 100 m)
            if (smallest_curr_distance < 100.0)
            {
                maximum_distance = smallest_curr_distance;
            }
        }
        average_distance_count += 1;
        metrics_arr.data[common_wpt::CUR_LAT_DIST_ABS] = smallest_curr_distance;
        metrics_arr.data[common_wpt::CUR_WAYPOINT_ID] = closest_waypoint;
        metrics_arr.data[common_wpt::AVG_LAT_DISTANCE] = average_distance;
        metrics_arr.data[common_wpt::MAX_LAT_DISTANCE] = maximum_distance;
        // calculate the adaptive lookahead distance
        double lookahead_actual = calcLookahead(speed_msg.data);

        for (int i = closest_waypoint; i <= last_wp; i++)
        {
            // RCLCPP_INFO_STREAM(this->get_logger(), "distanceFromWayPoint: " << distanceFromWayPoint(msg.poses[i], current_pose));
            if (distanceFromWayPoint(msg.poses[i], current_pose) > lookahead_actual)
            {
                if (distanceFromWayPoint(msg.poses[i], current_pose) < lookahead_actual + 10.0)
                {
                    target_waypoint = i;
                    metrics_arr.data[common_wpt::TRG_WAYPOINT_ID] = target_waypoint;
                    metrics_arr.data[common_wpt::TRG_WAY_LON_DIST] = distanceFromWayPoint(msg.poses[i], current_pose);
                }
                break;
            }
        }
        // calculate the waypoint from which to interpolate the lookahead_actual distance
        int interpol_waypoint = closest_waypoint;
        if (interpolate_waypoints)
        {
            for (int i = closest_waypoint; i <= last_wp; i++)
            {
                if (distanceFromWayPoint(msg.poses[i], current_pose) > lookahead_actual - 10.0)
                {
                    if (distanceFromWayPoint(msg.poses[i], current_pose) < lookahead_actual)
                    {
                        interpol_waypoint = i;
                    }
                    break;
                }
            }
        }

        metrics_arr.data[common_wpt::TRG_WAYPOINT_ID] = target_waypoint;

        // RCLCPP_INFO_STREAM(this->get_logger(), "Closest waypoint[" << closest_waypoint << "] " << std::fixed << std::setprecision(1) << metrics_arr.data[common_wpt::CUR_LAT_DIST_ABS] << " m away");
        // RCLCPP_INFO_STREAM(this->get_logger(), "Target waypoint[" << target_waypoint << "] " << std::fixed << std::setprecision(1) << metrics_arr.data[common_wpt::TRG_WAY_LON_DIST] << " m away");
        // if the lookahead distance far away
        if (metrics_arr.data[common_wpt::TRG_WAY_LON_DIST] > (lookahead_max + 2.0))
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Target is " << std::fixed << std::setprecision(1) << metrics_arr.data[common_wpt::TRG_WAY_LON_DIST] << " m away");
        }
        // if the target waypoint is too close
        if (metrics_arr.data[common_wpt::TRG_WAY_LON_DIST] < lookahead_min)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Tagret too close: " << std::fixed << std::setprecision(1) << metrics_arr.data[common_wpt::TRG_WAY_LON_DIST] << " m away");
        }
        // check if last waypoint is reached
        if (target_waypoint >= last_wp)
        {
            if (last_waypoint_reached == false)
            {
                last_waypoint_reached_time = this->now();
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Last waypoint reached " << std::fixed << std::setprecision(1) << (last_waypoint_reached_time - this->now()).nanoseconds() / -1e9 << "s ago");
            last_waypoint_reached = true;
        }
        if (interpolate_waypoints)
        {
            // the order should be (distances from the current pose): interpol_waypoint < lookahead < target_waypoint 
            // if this is not the case, do not use the interpolated waypoint
            if (distanceFromWayPoint(msg.poses[interpol_waypoint], current_pose) > lookahead_actual or distanceFromWayPoint(msg.poses[target_waypoint], current_pose) < lookahead_actual)
            {
                // this case do not use interpolated, use the target waypoint
                pursuit_goal.pose.position = msg.poses[target_waypoint].position;
            }
            else
            {
                // calculate the inteprolated lookahead_actual disatance between the two waypoints (interpol_waypoint, target_waypoint)
                geometry_msgs::msg::Point interpolated_pose = pointAtDistance(msg.poses[interpol_waypoint].position, msg.poses[target_waypoint].position, lookahead_actual, current_pose.position);
                pursuit_goal.pose.position = interpolated_pose;
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "I: " << distanceFromWayPoint(msg.poses[interpol_waypoint], current_pose) << "m L: " << lookahead_actual << "m T:"<< distanceFromWayPoint(msg.poses[target_waypoint], current_pose) << "m");
        }
        else
        {
            pursuit_goal.pose.position = msg.poses[target_waypoint].position;
        }
        pursuit_goal.pose.orientation = msg.poses[target_waypoint].orientation;
        pursuit_closest.pose.position = msg.poses[closest_waypoint].position;
        pursuit_closest.pose.orientation = msg.poses[closest_waypoint].orientation;
        geometry_msgs::msg::Pose pose_global; // a pose to put in the target_pose array
        pose_global.position = msg.poses[target_waypoint].position;
        pose_global.orientation = msg.poses[target_waypoint].orientation;
        // doTransform target_pose[0] to local frame
        geometry_msgs::msg::Pose target_pose_local, pursuit_goal_local, pursuit_closest_local;
        tf2::doTransform(pose_global, target_pose_local, transformInverse);
        tf2::doTransform(pursuit_goal.pose, pursuit_goal_local, transformInverse);
        tf2::doTransform(pursuit_closest.pose, pursuit_closest_local, transformInverse);
        pursuit_goal.pose.position = pursuit_goal_local.position;
        pursuit_goal.pose.orientation = pursuit_goal_local.orientation;
        pursuit_closest.pose.position = pursuit_closest_local.position;
        pursuit_closest.pose.orientation = pursuit_closest_local.orientation;
        target_pose_arr.header.stamp = this->now();
        target_pose_arr.poses.push_back(target_pose_local);
        double local_cw_roll, local_cw_pitch, local_cw_yaw, smallest_curr_signed_dist;
        tf2::Quaternion q_local(
            pursuit_closest_local.orientation.x,
            pursuit_closest_local.orientation.y,
            pursuit_closest_local.orientation.z,
            pursuit_closest_local.orientation.w);
        tf2::Matrix3x3 m(q_local);
        m.getRPY(local_cw_roll, local_cw_pitch, local_cw_yaw);
        if (local_cw_yaw > 1.309 or local_cw_yaw < -1.309) // 75 deg = 1.309 rad
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Current waypoint orientation is more than 75 deg [" << std::fixed << std::setprecision(1) << local_cw_yaw * 180 / M_PI << "deg]");
        }
        if (pursuit_closest_local.position.y < 0)
        {
            smallest_curr_signed_dist = smallest_curr_distance * -1.0;
        }
        else
        {
            smallest_curr_signed_dist = smallest_curr_distance;
        }
        metrics_arr.data[common_wpt::CUR_LAT_DIST_SIGNED] = smallest_curr_signed_dist;
        std::stringstream stream;
        stream << std::fixed << std::setprecision(1) << smallest_curr_signed_dist << " m";
        cross_track_marker.text = stream.str();
        pursuit_vizu_arr.markers[0] = pursuit_goal;
        pursuit_vizu_arr.markers[1] = pursuit_closest;
        pursuit_vizu_arr.markers[2] = cross_track_marker;
        // RCLCPP_INFO_STREAM(this->get_logger(), "transformInv: " << transformInverse.transform.translation.x << ", " << transformInverse.transform.translation.y << ", " << transformInverse.transform.translation.z);
    }
    void speedCallback(const std_msgs::msg::Float32MultiArray &msg)
    {
        // speed data from the target waypoint
        if (static_speed_enabled)
        {
            speed_msg.data = static_speed;
        }
        else
        {
            speed_msg.data = msg.data[metrics_arr.data[common_wpt::CUR_WAYPOINT_ID]];
        }
        // RCLCPP_INFO_STREAM(this->get_logger(), "Target speed:" << speed_msg.data << " m/s");
        // stop at the end of the path (if the trajectory is not a circular loop)
        if (traj_closed_loop == false)
        {
            if (last_waypoint_reached == true)
            {
                if ((last_waypoint_reached_time - this->now()).nanoseconds() / -1e9 > 5.0)
                {
                    speed_msg.data = 0.0;
                    RCLCPP_INFO_STREAM(this->get_logger(), "STOP: last waypoint reached at more than 5s ago");
                }
            }
        }
    }

    // get tf2 transform from map to base_link
    void getTransform()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            // TODO: better way?
            transformStamped = tf_buffer_->lookupTransform(tf_child_frame_id, tf_frame_id, tf2::TimePointZero);
            transformInverse = tf_buffer_->lookupTransform(tf_frame_id, tf_child_frame_id, tf2::TimePointZero);
        }

        catch (const tf2::TransformException &ex)
        {
            // RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }
        current_pose.position.x = transformStamped.transform.translation.x;
        current_pose.position.y = transformStamped.transform.translation.y;
        current_pose.position.z = transformStamped.transform.translation.z;
        current_pose.orientation.x = transformStamped.transform.rotation.x;
        current_pose.orientation.y = transformStamped.transform.rotation.y;
        current_pose.orientation.z = transformStamped.transform.rotation.z;
        current_pose.orientation.w = transformStamped.transform.rotation.w;
        // RCLCPP_INFO_STREAM(this->get_logger(), "current_pose: " << current_pose.position.x << ", " << current_pose.position.y << ", " << current_pose.position.z);
    }

    void reinitCallback(const std_msgs::msg::Bool &msg)
    {
        reinit = msg.data;
    }

    void timer_callback()
    {
        // RCLCPP_INFO(this->get_logger(), "timer");
        getTransform();
        goal_pub_->publish(pursuit_vizu_arr);
        target_pub_->publish(target_pose_arr);
        speed_pub_->publish(speed_msg);
        metrics_pub_->publish(metrics_arr);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr metrics_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_w_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_s_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_reinit_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std_msgs::msg::Float32MultiArray metrics_arr; // array of metrics (eg. current lateral distance, average lateral distance, current waypoint ID etc)
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    // parameters
    std::string waypoint_topic = "waypointarray";
    double lookahead_min = 8.5; // eg. Lexus3 front from base_link: 2.789 + 1.08 = 3.869
    double lookahead_max = lookahead_min + 15.0;
    double mps_alpha = 3.0;   // (3*3.6 = 10.8 km/h)
    double mps_beta = 5.0;    // (5*3.6 = 18 km/h)
    int closest_waypoint = 0; // closest waypoint to the current pose
    float average_distance = 0.0, maximum_distance = 0.0;
    int average_distance_count = 0;
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::TransformStamped transformInverse;
    rclcpp::Time last_waypoint_reached_time;
    bool last_waypoint_reached = false;
    bool static_speed_enabled = false;
    bool traj_closed_loop = false; // Trajectory loop closure bool, if the trajectory is linear/open loop: false, if circular/cloded loop: true
    bool reinit = true, interpolate_waypoints = false;
    double static_speed; // value of static speed in m/s
    visualization_msgs::msg::Marker pursuit_goal, pursuit_closest, cross_track_marker;
    visualization_msgs::msg::MarkerArray pursuit_vizu_arr;
    geometry_msgs::msg::PoseArray target_pose_arr;
    std::string tf_child_frame_id, tf_frame_id;
    std_msgs::msg::Float32 speed_msg;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointToTarget>());
    rclcpp::shutdown();
    return 0;
}