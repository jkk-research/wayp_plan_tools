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

rclcpp::Node::SharedPtr node;
int already_visited_waypoint = 0;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub_;

// parameters
std::string waypoint_topic = "/lexus3/waypointarray";
int lookahead_distance = 3.5;
geometry_msgs::msg::Pose current_pose;


double distanceFromWayPoint(const geometry_msgs::msg::Pose &waypoint, const geometry_msgs::msg::Pose &pose_curr)
{
    double dx = waypoint.position.x - pose_curr.position.x;
    double dy = waypoint.position.y - pose_curr.position.y;
    return sqrt(dx * dx + dy * dy);
}

void waypointCallback(const geometry_msgs::msg::PoseArray &msg)
{
    visualization_msgs::msg::Marker pursuit_goal;
    pursuit_goal.header.frame_id = "/map";
    pursuit_goal.header.stamp = node->now();
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
    // get the first waypoint which is at least lookahead_distance away from current pose
    for (int i = already_visited_waypoint; i < int(msg.poses.size()); i++)
    {
        //RCLCPP_INFO_STREAM(node->get_logger(), "distanceFromWayPoint: " << distanceFromWayPoint(msg.poses[i], current_pose));
        if (distanceFromWayPoint(msg.poses[i], current_pose) < lookahead_distance)
        {
            closest_waypoint = i;
            break;
        }
    } 
    // if the lookahead distance further away than 20 meters
    if (distanceFromWayPoint(msg.poses[closest_waypoint], current_pose) > 20.0){
        RCLCPP_WARN_STREAM(node->get_logger(), "Closest waypoint is " << distanceFromWayPoint(msg.poses[closest_waypoint], current_pose) << " m away");
    }
    already_visited_waypoint = closest_waypoint;
    //RCLCPP_INFO_STREAM(node->get_logger(), "closest_waypoint id: " << closest_waypoint);

    pursuit_goal.pose.position = msg.poses[closest_waypoint].position;
    pursuit_goal.pose.orientation = msg.poses[closest_waypoint].orientation;
    goal_pub_->publish(pursuit_goal);
}
void speedCallback(const std_msgs::msg::Float32MultiArray &msg)
{
    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = msg.data[already_visited_waypoint];
    speed_pub_->publish(speed_msg);
}

// get tf2 transform from map to lexus3/base_link
void getTransform()
{
    tf2_ros::Buffer tfBuffer(node->get_clock());
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("map", "lexus3/base_link", rclcpp::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(node->get_logger(), "Could not get transform: %s", ex.what());
    }
    current_pose.position.x = transformStamped.transform.translation.x;
    current_pose.position.y = transformStamped.transform.translation.y;
    current_pose.position.z = transformStamped.transform.translation.z;
    current_pose.orientation.x = transformStamped.transform.rotation.x;
    current_pose.orientation.y = transformStamped.transform.rotation.y;
    current_pose.orientation.z = transformStamped.transform.rotation.z;
    current_pose.orientation.w = transformStamped.transform.rotation.w;
}


int main(int argc, char **argv)
{
    current_pose.position.x = 35.0;
    current_pose.position.y = 70.0;

    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("waypoint_to_target_node");

    node->declare_parameter<std::string>("waypoint_topic", "");
    // node->get_parameter("waypoint_topic", waypoint_topic);

    goal_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("/lexus3/pursuitgoal", 10);
    speed_pub_ = node->create_publisher<std_msgs::msg::Float32>("/lexus3/pursuitspeedtarget", 10);

    auto sub_w_ = node->create_subscription<geometry_msgs::msg::PoseArray>(waypoint_topic, 10, waypointCallback);
    auto sub_s_ = node->create_subscription<std_msgs::msg::Float32MultiArray>("/lexus3/waypointarray_speeds", 10, speedCallback);
    // Call on_timer function every second
    auto timer_ = node->create_wall_timer(std::chrono::milliseconds(1000), getTransform);
    RCLCPP_INFO_STREAM(node->get_logger(), "pure_pursuit_node started: ");
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}