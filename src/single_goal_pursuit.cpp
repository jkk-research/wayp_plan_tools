#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

rclcpp::Node::SharedPtr node;
std::string waypoint_topic = "/lexus3/pursuitgoal";
geometry_msgs::msg::Twist pursuit_vel;

// parameters
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr goal_pub_;
float wheelbase = 2.789; // Lexus3

// pure pursuit steering angle calc
float calcPursuitAngle(float goal_x, float goal_y)
{
  float alpha = atan2(goal_y, goal_x);
  float lookahead_distance = sqrt(pow(goal_x, 2) + pow(goal_y, 2));
  float steering_angle = atan2(2.0 * wheelbase * sin(alpha) / (lookahead_distance), 1);
  return steering_angle;
}

void speedCallback(const std_msgs::msg::Float32 &msg)
{
  pursuit_vel.linear.x = msg.data;
}

void waypointCallback(const geometry_msgs::msg::PoseArray &msg)
{
  pursuit_vel.angular.z = calcPursuitAngle(msg.poses[0].position.x, msg.poses[0].position.y);
  goal_pub_->publish(pursuit_vel);
}

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("pure_pursuit_node");

  node->declare_parameter<std::string>("waypoint_topic", "");
  // node->get_parameter("waypoint_topic", waypoint_topic);

  goal_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/lexus3/cmd_vel", 10);

  auto sub_w_ = node->create_subscription<geometry_msgs::msg::PoseArray>(waypoint_topic, 10, waypointCallback);
  auto sub_s_ = node->create_subscription<std_msgs::msg::Float32>("/lexus3/pursuitspeedtarget", 10, speedCallback);
  
  RCLCPP_INFO_STREAM(node->get_logger(), "pure_pursuit_node started: ");
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}