#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "wayp_plan_tools/common.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class EchoMetrics : public rclcpp::Node
{

public:
  EchoMetrics() : Node("echo_metrics_node")
  {
    //RCLCPP_INFO_STREAM(this->get_logger(), "echo_metrics_node started: ");

    metrics_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("metrics_wayp", 10, std::bind(&EchoMetrics::MetricsCallback, this, _1));   
  }

private:
  void MetricsCallback(const std_msgs::msg::Float32MultiArray &msg) const
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Max lateral distance: " << std::setprecision(2) << msg.data[common_wpt::MAX_LAT_DISTANCE] << " m");
    RCLCPP_INFO_STREAM(this->get_logger(), "Avg lateral distance: " << std::setprecision(2) << msg.data[common_wpt::AVG_LAT_DISTANCE] << " m");
    rclcpp::shutdown();
  }
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr metrics_sub_;
};

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EchoMetrics>());
  rclcpp::shutdown();
  return 0;
}