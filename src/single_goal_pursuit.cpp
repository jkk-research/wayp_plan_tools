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

std::string waypoint_topic = "/lexus3/pursuitgoal";
geometry_msgs::msg::Twist pursuit_vel;

using std::placeholders::_1;

class SingleGoalPursuit : public rclcpp::Node
{
public:
  SingleGoalPursuit() : Node("pure_pursuit_node")
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "pure_pursuit_node started: ");
    this->declare_parameter<std::string>("waypoint_topic", "");
    this->get_parameter("waypoint_topic", waypoint_topic);

    goal_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/lexus3/cmd_vel", 10);
    sub_w_ = this->create_subscription<geometry_msgs::msg::PoseArray>(waypoint_topic, 10, std::bind(&SingleGoalPursuit::waypointCallback, this, _1));
    sub_s_ = this->create_subscription<std_msgs::msg::Float32>("/lexus3/pursuitspeedtarget", 10, std::bind(&SingleGoalPursuit::speedCallback, this, _1));
  }

private:
  // pure pursuit steering angle calc
  float calcPursuitAngle(float goal_x, float goal_y) const
  {
    float alpha = atan2(goal_y, goal_x);
    float lookahead_distance = sqrt(pow(goal_x, 2) + pow(goal_y, 2));
    float steering_angle = atan2(2.0 * wheelbase * sin(alpha) / (lookahead_distance), 1);
    steering_angle *= 14.8; // TODO: ratio to param
    return steering_angle;
  }

  void speedCallback(const std_msgs::msg::Float32 &msg) const
  {
    pursuit_vel.linear.x = msg.data;
  }

  void waypointCallback(const geometry_msgs::msg::PoseArray &msg) const
  {
    pursuit_vel.angular.z = calcPursuitAngle(msg.poses[0].position.x, msg.poses[0].position.y);
    goal_pub_->publish(pursuit_vel);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_w_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_s_;
  // parameters
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr goal_pub_;
  float wheelbase = 2.789; // Lexus3
};


int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SingleGoalPursuit>());
  rclcpp::shutdown();
  return 0;
}