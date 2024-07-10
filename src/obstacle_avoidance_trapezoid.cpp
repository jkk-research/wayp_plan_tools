#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <cmath> // For std::sqrt and std::pow

using namespace std::chrono_literals;
using std::placeholders::_1;


// TODO: plan code structure

class ObstacleAvoidanceTrapezoid : public rclcpp::Node
{
   public:
    ObstacleAvoidanceTrapezoid()
        : Node("obstacle_avoidance_trapezoid")
    {

        lane_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("waypointarray",10 ,std::bind(&ObstacleAvoidanceTrapezoid::lane_callback,this, std::placeholders::_1));
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 10, std::bind(&ObstacleAvoidanceTrapezoid::current_pose_callback, this, std::placeholders::_1));
        marker_array_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("clustered_marker", 10, std::bind(&ObstacleAvoidanceTrapezoid::marker_array_callback, this, std::placeholders::_1));
    }
    private:
    void lane_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        auto num_poses = msg->poses.size(); // This line gets the number of elements in PoseArray
        RCLCPP_INFO(this->get_logger(), "Number of poses received: %zu", num_poses);
    }
    void current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Current pose: x=%f, y=%f", msg->pose.position.x, msg->pose.position.y);
    }
    
    void marker_array_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        auto num_markers = msg->markers.size();
        RCLCPP_INFO(this->get_logger(), "Number of markers received: %zu", num_markers);
    }

    int find_closest_waypoint(double current_x, double current_y, const geometry_msgs::msg::PoseArray& waypoints)
    {
        double min_distance = std::numeric_limits<double>::max();
        int closest_waypoint_index = -1;

        for (size_t i = 0; i < waypoints.poses.size(); ++i) {
            double waypoint_x = waypoints.poses[i].position.x;
            double waypoint_y = waypoints.poses[i].position.y;

            double distance = std::sqrt(std::pow(waypoint_x - current_x, 2) + std::pow(waypoint_y - current_y, 2));

            if (distance < min_distance) {
                min_distance = distance;
                closest_waypoint_index = static_cast<int>(i);
                }
            }

        return closest_waypoint_index;
    }

    double line_length(double x1, double x2, double y1, double y2)
        {
            return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
        }


    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr lane_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared< ObstacleAvoidanceTrapezoid >());
    rclcpp::shutdown();
    return 0;
}