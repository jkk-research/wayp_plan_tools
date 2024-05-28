#include "rclcpp/rclcpp.hpp"

// TODO: plan code structure

class ObstacleAvoidanceTrapezoid : public rclcpp::Node
{
   public:
    ObstacleAvoidanceTrapezoid()
        : Node("obstacle_avoidance_trapezoid")
    {
        RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Trapezoid Node has been started.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared< ObstacleAvoidanceTrapezoid >());
    rclcpp::shutdown();
    return 0;
}