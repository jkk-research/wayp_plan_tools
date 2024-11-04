#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudFilter : public rclcpp::Node {
public:
    PointCloudFilter(float x_threshold)
        : Node("point_cloud_filter"), x_threshold_(x_threshold) {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/nonground", 10, std::bind(&PointCloudFilter::listener_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_nonground", 10);
    }

private:
    void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert PointCloud2 message to PCL data
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Filter points based on x_threshold
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : cloud->points) {
            if (point.x > x_threshold_) {
                filtered_cloud->points.push_back(point);
            }
        }

        // Convert PCL data back to PointCloud2 message
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header = msg->header;
        filtered_msg.header.frame_id = "gamma/ouster_link/ouster"; 

        // Publish the filtered point cloud
        publisher_->publish(filtered_msg);
    }

    float x_threshold_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    float x_threshold = 1.7;  // Default threshold is 1.0
    auto node = std::make_shared<PointCloudFilter>(x_threshold);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}