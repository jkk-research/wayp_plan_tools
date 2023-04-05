#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
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

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class WaypointToTarget : public rclcpp::Node
{
public:
    WaypointToTarget() : Node("waypoint_to_target_node")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Call timer_callback function 20 Hz, 50 milliseconds 
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&WaypointToTarget::timer_callback, this));

        this->declare_parameter<std::string>("waypoint_topic", "");
        // this->get_parameter("waypoint_topic", waypoint_topic);

        goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lexus3/pursuittarget_marker", 10);
        speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("lexus3/pursuitspeedtarget", 10);
        target_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("lexus3/targetpoints", 10);   
        sub_w_ = this->create_subscription<geometry_msgs::msg::PoseArray>(waypoint_topic, 10, std::bind(&WaypointToTarget::waypointCallback, this, _1));
        sub_s_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("lexus3/waypointarray_speeds", 10, std::bind(&WaypointToTarget::speedCallback, this, _1));
        RCLCPP_INFO_STREAM(this->get_logger(), "waypoint_to_target node started");
    }

private:
    double distanceFromWayPoint(const geometry_msgs::msg::Pose &waypoint, const geometry_msgs::msg::Pose &pose_curr)
    {
        double dx = waypoint.position.x - pose_curr.position.x;
        double dy = waypoint.position.y - pose_curr.position.y;
        return sqrt(dx * dx + dy * dy);
    }

    void waypointCallback(const geometry_msgs::msg::PoseArray &msg)
    {
        visualization_msgs::msg::Marker pursuit_goal;
        geometry_msgs::msg::PoseArray target_pose_arr;
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

        // choose waypoint(s) closest to lookahead distance
        int closest_waypoint = already_visited_waypoint;
        // get the first waypoint which is at least lookahead_distance_min away from current pose
        for (int i = already_visited_waypoint; i <= int(msg.poses.size()); i++)
        {
            // RCLCPP_INFO_STREAM(this->get_logger(), "distanceFromWayPoint: " << distanceFromWayPoint(msg.poses[i], current_pose));
            if (distanceFromWayPoint(msg.poses[i], current_pose) > lookahead_distance_min)
            {
                if (distanceFromWayPoint(msg.poses[i], current_pose) < lookahead_distance_max)
                {
                    closest_waypoint = i;
                }
                break;
            }
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Closest waypoint[" << closest_waypoint << "] " << std::fixed << std::setprecision(1) << distanceFromWayPoint(msg.poses[closest_waypoint], current_pose) << " m away");
        // if the lookahead distance far away
        if (distanceFromWayPoint(msg.poses[closest_waypoint], current_pose) > (lookahead_distance_max + 2.0))
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Closest waypoint is " << std::fixed << std::setprecision(1) << distanceFromWayPoint(msg.poses[closest_waypoint], current_pose) << " m away");
        }
        // if the closest waypoint is too close
        if (distanceFromWayPoint(msg.poses[closest_waypoint], current_pose) < lookahead_distance_min)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Waypoint too close: " << std::fixed << std::setprecision(1) << distanceFromWayPoint(msg.poses[closest_waypoint], current_pose) << " m away");
        }
        // if last waypoint is reached, stop
        if (closest_waypoint >= int(msg.poses.size() - 1))
        {
            if (last_waypoint_reached == false)
            {
                last_waypoint_reached_time = this->now();
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Last waypoint reached at " << std::fixed << std::setprecision(1) << (last_waypoint_reached_time - this->now()).nanoseconds() / -1e9 << "s ago");
            last_waypoint_reached = true;
        }
        already_visited_waypoint = closest_waypoint;
        pursuit_goal.pose.position = msg.poses[closest_waypoint].position;
        pursuit_goal.pose.orientation = msg.poses[closest_waypoint].orientation;
        geometry_msgs::msg::Pose pose_global; // a pose to put in the target_pose array
        pose_global.position = msg.poses[closest_waypoint].position;
        pose_global.orientation = msg.poses[closest_waypoint].orientation;
        // doTransform target_pose[0] to local frame
        geometry_msgs::msg::Pose target_pose_local, pursuit_goal_local;
        tf2::doTransform(pose_global, target_pose_local, transformInverse); 
        tf2::doTransform(pursuit_goal.pose, pursuit_goal_local, transformInverse); 
        pursuit_goal.pose.position = pursuit_goal_local.position;
        pursuit_goal.pose.orientation = pursuit_goal_local.orientation;        
        target_pose_arr.poses.push_back(target_pose_local);
        // RCLCPP_INFO_STREAM(this->get_logger(), "transformInv: " << transformInverse.transform.translation.x << ", " << transformInverse.transform.translation.y << ", " << transformInverse.transform.translation.z);
        goal_pub_->publish(pursuit_goal);
        target_pub_->publish(target_pose_arr);
    }
    void speedCallback(const std_msgs::msg::Float32MultiArray &msg)
    {
        std_msgs::msg::Float32 speed_msg;
        speed_msg.data = msg.data[already_visited_waypoint];
        // RCLCPP_INFO_STREAM(this->get_logger(), "Target speed:" << speed_msg.data << " m/s");
        //  stop at the end of the path
        if (last_waypoint_reached == true)
        {
            if ((last_waypoint_reached_time - this->now()).nanoseconds() / -1e9 > 5.0)
            {
                speed_msg.data = 0.0;
                RCLCPP_INFO_STREAM(this->get_logger(), "STOP: last waypoint reached at more than 5s ago");
            }
        }
        speed_pub_->publish(speed_msg);
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

    void timer_callback()
    {
        // RCLCPP_INFO(this->get_logger(), "timer");
        getTransform();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr target_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_w_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_s_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    int already_visited_waypoint = 0;
    // parameters
    std::string waypoint_topic = "lexus3/waypointarray";
    double lookahead_distance_min = 8.5; // Lexus3 front from base_link: 2.789 + 1.08 = 3.869
    double lookahead_distance_max = lookahead_distance_min + 15.0;
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::TransformStamped transformInverse;
    rclcpp::Time last_waypoint_reached_time;
    bool last_waypoint_reached = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointToTarget>());
    rclcpp::shutdown();
    return 0;
}