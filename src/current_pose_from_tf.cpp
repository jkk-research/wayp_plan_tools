// current_pose_from_tf
// Publishes a current_pose topic based on a transform listener (e.g. map -> base_link)

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class CurrentPoseFromTf : public rclcpp::Node
{
public:
    CurrentPoseFromTf() : Node("current_pose_from_tf_node")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Call timer_callback function 50 milliseconds
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CurrentPoseFromTf::timer_callback, this));
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("gamma/current_pose", 10);
        RCLCPP_INFO_STREAM(this->get_logger(), "current_pose_from_tf node started");
    }

private:
    // get tf2 transform from map to lexus3/base_link
    void getTransform()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_->lookupTransform(frame_id_, child_frame_id_, tf2::TimePointZero);
        }

        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }
        current_pose.pose.position.x = transformStamped.transform.translation.x;
        current_pose.pose.position.y = transformStamped.transform.translation.y;
        current_pose.pose.position.z = transformStamped.transform.translation.z;
        current_pose.pose.orientation.x = transformStamped.transform.rotation.x;
        current_pose.pose.orientation.y = transformStamped.transform.rotation.y;
        current_pose.pose.orientation.z = transformStamped.transform.rotation.z;
        current_pose.pose.orientation.w = transformStamped.transform.rotation.w;
        current_pose.header.frame_id = "map_gamma";
        current_pose.header.stamp = this->get_clock()->now();
        //RCLCPP_INFO_STREAM(this->get_logger(), "current_pose: " << std::fixed << std::setprecision(2) << current_pose.position.x << ", " << current_pose.position.y);
        pose_pub_->publish(current_pose);
    }

    void timer_callback()
    {
        // RCLCPP_INFO(this->get_logger(), "timer");
        getTransform();
    }
 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    geometry_msgs::msg::PoseStamped current_pose;
    // TODO: parameters 
    std::string frame_id_ = "map_gamma";
    std::string child_frame_id_ = "base_link";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurrentPoseFromTf>());
    rclcpp::shutdown();
    return 0;
}