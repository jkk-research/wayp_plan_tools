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
        }
        return result;
    }
    WaypointToTarget() : Node("waypoint_to_target_node")
    {
        metrics_arr.data.resize(common_wpt::NOT_USED_YET); // initialize the metrics array
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

        if (waypoint_topic == "")
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "waypoint_topic is not set using default value: /lexus3/waypointarray");
            waypoint_topic = "/lexus3/waypointarray";
        }

        goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lexus3/pursuittarget_marker", 10);
        speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("lexus3/pursuitspeedtarget", 10);
        target_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("lexus3/targetpoints", 10);
        metrics_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("lexus3/metrics_wayp", 10);
        sub_w_ = this->create_subscription<geometry_msgs::msg::PoseArray>(waypoint_topic, 10, std::bind(&WaypointToTarget::waypointCallback, this, _1));
        sub_s_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("lexus3/waypointarray_speeds", 10, std::bind(&WaypointToTarget::speedCallback, this, _1));
        sub_reinit_ = this->create_subscription<std_msgs::msg::Bool>("/lexus3/control_reinit", 10, std::bind(&WaypointToTarget::reinitCallback, this, _1));
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

    void waypointCallback(const geometry_msgs::msg::PoseArray &msg)
    {
        target_pose_arr.poses.clear();
        pursuit_goal.header.frame_id = "lexus3/base_link";
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
        int search_end = closest_waypoint + 10;
        if (search_start < first_wp)
        {
            search_start = first_wp;
        }
        if (search_end > last_wp)
        {
            search_end = last_wp;
        }
        // if got a reinit signal, search in the whole trajectory
        if(reinit == true)
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
        average_distance_count += 1;
        metrics_arr.data[common_wpt::CUR_LAT_DIST_ABS] = smallest_curr_distance;
        metrics_arr.data[common_wpt::CUR_WAYPOINT_ID] = closest_waypoint;
        metrics_arr.data[common_wpt::AVG_LAT_DISTANCE] = average_distance;

        metrics_arr.data[common_wpt::CUR_LAT_DIST_SIGNED] = msg.poses[closest_waypoint].position.x; 

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
        pursuit_goal.pose.position = msg.poses[target_waypoint].position;
        pursuit_goal.pose.orientation = msg.poses[target_waypoint].orientation;
        geometry_msgs::msg::Pose pose_global; // a pose to put in the target_pose array
        pose_global.position = msg.poses[target_waypoint].position;
        pose_global.orientation = msg.poses[target_waypoint].orientation;
        // doTransform target_pose[0] to local frame
        geometry_msgs::msg::Pose target_pose_local, pursuit_goal_local;
        tf2::doTransform(pose_global, target_pose_local, transformInverse);
        tf2::doTransform(pursuit_goal.pose, pursuit_goal_local, transformInverse);
        pursuit_goal.pose.position = pursuit_goal_local.position;
        pursuit_goal.pose.orientation = pursuit_goal_local.orientation;
        target_pose_arr.poses.push_back(target_pose_local);
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

    // get tf2 transform from map to lexus3/base_link
    void getTransform()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            // TODO: better way?
            transformStamped = tf_buffer_->lookupTransform("map", "lexus3/base_link", tf2::TimePointZero);
            transformInverse = tf_buffer_->lookupTransform("lexus3/base_link", "map", tf2::TimePointZero);
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
        goal_pub_->publish(pursuit_goal);
        target_pub_->publish(target_pose_arr);
        speed_pub_->publish(speed_msg);
        metrics_pub_->publish(metrics_arr);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_;
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
    std::string waypoint_topic = "lexus3/waypointarray";
    double lookahead_min = 8.5; // Lexus3 front from base_link: 2.789 + 1.08 = 3.869
    double lookahead_max = lookahead_min + 15.0;
    double mps_alpha = 3.0;   // (3*3.6 = 10.8 km/h)
    double mps_beta = 5.0;    // (5*3.6 = 18 km/h)
    int closest_waypoint = 0; // closest waypoint to the current pose
    float average_distance = 0.0;
    int average_distance_count = 0;
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::TransformStamped transformInverse;
    rclcpp::Time last_waypoint_reached_time;
    bool last_waypoint_reached = false;
    bool static_speed_enabled = false;
    bool traj_closed_loop = false; // Trajectory loop closure bool, if the trajectory is linear/open loop: false, if circular/cloded loop: true
    bool reinit = true;
    double static_speed;           // value of static speed in m/s
    visualization_msgs::msg::Marker pursuit_goal;
    geometry_msgs::msg::PoseArray target_pose_arr;
    std_msgs::msg::Float32 speed_msg;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointToTarget>());
    rclcpp::shutdown();
    return 0;
}