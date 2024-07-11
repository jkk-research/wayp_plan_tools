#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>


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


    geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
    geometry_msgs::msg::PoseArray::SharedPtr waypoints_;
    int waypoints_size, closest_waypoint_index;
    int lookahead_distance_ = 10.0;

    void lane_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        int waypoints_size = msg->poses.size(); // This line gets the number of elements in PoseArray
        waypoints_= msg;
        
    }
    void current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_= msg;
        if (waypoints_) {
        int closest_waypoint_index = find_closest_waypoint(current_pose_->pose.position.x, current_pose_->pose.position.y, *waypoints_);
        }
    }
    
    void marker_array_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    auto clock = this->get_clock();
    tf2_ros::Buffer tfBuffer(clock);
    tf2_ros::TransformListener tfListener(tfBuffer);

    auto num_markers = msg->markers.size();
    for (size_t i = 0; i < num_markers; ++i) {
        // Only transform markers where ns is "hull"
        if (msg->markers[i].ns == "hull") {
            try {
                geometry_msgs::msg::TransformStamped transformStamped = tfBuffer.lookupTransform("new_frame", msg->markers[i].header.frame_id, tf2::TimePointZero);

                for (auto& point : msg->markers[i].points) {
                    // Convert Point to PointStamped
                    geometry_msgs::msg::PointStamped point_in, point_out;
                    point_in.header = msg->markers[i].header;
                    point_in.point = point;

                    // Transform PointStamped
                    tf2::doTransform(point_in, point_out, transformStamped);

                    // Convert PointStamped back to Point
                    point = point_out.point;
                }
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "%s", ex.what());
                continue;
            }
        }
    }

    if (waypoints_size - closest_waypoint_index < lookahead_distance_) {
        lookahead_distance_ = waypoints_size - closest_waypoint_index;
    }

    std::vector<int> close_waypoints;
    for (int i = closest_waypoint_index; i < closest_waypoint_index + lookahead_distance_; ++i) {
        if (i < waypoints_->poses.size() - 1) {
            double distance = line_length(
                waypoints_->poses[i].position.x, waypoints_->poses[i+1].position.x,
                waypoints_->poses[i].position.y, waypoints_->poses[i+1].position.y
            );
            if (distance < 2.0) {
                close_waypoints.push_back(i);
            }
        }
    }



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