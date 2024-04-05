#include "rclcpp/rclcpp.hpp"


// TODO: add to CMakeLists.txt
// TODO: add launch file
// TODO: plan code structure


class ObstacleAvoidanceTrapezoid : public rclcpp::Node
{

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidanceTrapezoid>());
    rclcpp::shutdown();
    return 0;
}