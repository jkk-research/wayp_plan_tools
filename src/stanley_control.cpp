#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

std::string waypoint_topic = "/lexus3/pursuitgoal";
geometry_msgs::msg::Twist pursuit_vel;

using namespace std::chrono_literals;
using std::placeholders::_1;

class StanleyControl : public rclcpp::Node
{
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
      if (param.get_name() == "waypoint_topic")
      {
        waypoint_topic = param.as_string();
        sub_w_ = this->create_subscription<geometry_msgs::msg::PoseArray>(waypoint_topic, 10, std::bind(&StanleyControl::waypointCallback, this, _1));
      }
      if (param.get_name() == "wheelbase")
      {
        wheelbase = param.as_double();
      }
    }
    return result;
  }

public:
  StanleyControl() : Node("stanley_control_node")
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "stanley_control_node started: ");
    this->declare_parameter<std::string>("waypoint_topic", "");
    this->declare_parameter<float>("wheelbase", wheelbase);
    this->get_parameter("waypoint_topic", waypoint_topic);
    this->get_parameter("wheelbase", wheelbase);

    goal_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/lexus3/cmd_vel", 10);
    reinit_pub_ = this->create_publisher<std_msgs::msg::Bool>("/lexus3/control_reinit", 10);
    sub_w_ = this->create_subscription<geometry_msgs::msg::PoseArray>(waypoint_topic, 10, std::bind(&StanleyControl::waypointCallback, this, _1));
    sub_s_ = this->create_subscription<std_msgs::msg::Float32>("/lexus3/pursuitspeedtarget", 10, std::bind(&StanleyControl::speedCallback, this, _1));
    timer_ = this->create_wall_timer(50ms, std::bind(&StanleyControl::timerLoop, this));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&StanleyControl::parametersCallback, this, std::placeholders::_1));
    std::this_thread::sleep_for(600ms);
    reinitControl();
  }

private:
  // TODO:
  // heading error
  // cross-track error
  float calcStanley(float goal_x, float goal_y, float goal_yaw, float current_yaw) const
  {
    float alpha = atan2(goal_y, goal_x);
    float lookahead_distance = sqrt(pow(goal_x, 2) + pow(goal_y, 2));
    float steering_angle = atan2(2.0 * wheelbase * sin(alpha) / (lookahead_distance), 1);
    return steering_angle;
  }

  void speedCallback(const std_msgs::msg::Float32 &msg) const
  {
    pursuit_vel.linear.x = msg.data;
  }

  void waypointCallback(const geometry_msgs::msg::PoseArray &msg) const
  {
    pursuit_vel.angular.z = calcStanley(msg.poses[0].position.x, msg.poses[0].position.y, 0, 0);
  }
  void timerLoop()
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "timer");
    goal_pub_->publish(pursuit_vel);
  }
  void reinitControl()
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "reinitControl");
    std_msgs::msg::Bool reinit;
    reinit.data = true;
    reinit_pub_->publish(reinit);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_w_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_s_;
  // parameters
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reinit_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  float wheelbase = 2.789; // Lexus3
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StanleyControl>());
  rclcpp::shutdown();
  return 0;
}