/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Based on Autoware waypoint saver, but here no Autoware message is used

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


rclcpp::Node::SharedPtr node;
std::string file_dir, file_name;
double interval_ = 0.4;
geometry_msgs::msg::Pose previous_pose;
geometry_msgs::msg::Pose current_tf;
bool topic_based_saving; // topic or transform (tf) based saving

void poseCurrentCallback(const geometry_msgs::msg::PoseStamped &current_pose)
{
    RCLCPP_INFO_STREAM(node->get_logger(), "X : " << current_pose.pose.position.x);
    /*
    double distance = sqrt(pow((current_pose.position.x - previous_pose.position.x), 2) +
                           pow((current_pose.position.y - previous_pose.position.y), 2));

    std::ofstream ofs(file_name.c_str(), std::ios::app);
    tf2::Quaternion q(
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // if car moves [interval] meter
    if (distance > interval_)
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "X Y yaw: " << current_pose.position.x << " " << current_pose.position.y << " " << yaw);
        ofs << std::fixed << std::setprecision(4) << current_pose.position.x << "," << current_pose.position.y << ","  << current_pose.position.z << "," << yaw << ",0,0" << std::endl;
        previous_pose = current_pose;
    }
    */
}

// get tf2 transform from map to lexus3/base_link
void getTransform()
{
    tf2_ros::Buffer tfBuffer(node->get_clock());
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("map", "lexus3/base_link", rclcpp::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(node->get_logger(), "Could not get transform: %s", ex.what());
    }
    current_tf.position.x = transformStamped.transform.translation.x;
    current_tf.position.y = transformStamped.transform.translation.y;
    current_tf.position.z = transformStamped.transform.translation.z;
    current_tf.orientation.x = transformStamped.transform.rotation.x;
    current_tf.orientation.y = transformStamped.transform.rotation.y;
    current_tf.orientation.z = transformStamped.transform.rotation.z;
    current_tf.orientation.w = transformStamped.transform.rotation.w;


    double distance = sqrt(pow((current_tf.position.x - previous_pose.position.x), 2) +
                           pow((current_tf.position.y - previous_pose.position.y), 2));

    std::ofstream ofs(file_name.c_str(), std::ios::app);
    tf2::Quaternion q(
        current_tf.orientation.x,
        current_tf.orientation.y,
        current_tf.orientation.z,
        current_tf.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    RCLCPP_INFO_STREAM(node->get_logger(), "X Y yaw: " << current_tf.position.x << " " << current_tf.position.y << " " << yaw);
    
    // if car moves [interval] meter
    if (distance > interval_)
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "X Y yaw: " << current_tf.position.x << " " << current_tf.position.y << " " << yaw);
        ofs << std::fixed << std::setprecision(4) << current_tf.position.x << "," << current_tf.position.y << ","  << current_tf.position.z << "," << yaw << ",0,0" << std::endl;
        previous_pose = current_tf;
    }


}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("waypoint_saver_node");

    node->declare_parameter<std::string>("file_dir", "");
    node->declare_parameter<std::string>("file_name", "");
    node->declare_parameter<std::string>("pose_topic", "");
    node->get_parameter("file_dir", file_dir);
    node->get_parameter("file_name", file_name);

    if(file_name.empty()){
        file_name = "waypoint01.csv";  
    }
    std::ofstream ofs((file_dir + "/" + file_name).c_str(), std::ios::app);
    // mps = m/s metre per second ROS 2 standard SI unit for velocity
    ofs << "x,y,z,yaw,mps,change_flag" << std::endl;
    topic_based_saving = true; // set (TODO: param)
    if(topic_based_saving){
        RCLCPP_INFO_STREAM(node->get_logger(), "Save to " << file_dir << "/" << file_name << " source: pose topic");
        auto sub_current_pose = node->create_subscription<geometry_msgs::msg::PoseStamped>("/current_pose", 10, poseCurrentCallback);
    }
    else{
        RCLCPP_INFO_STREAM(node->get_logger(), "Save to " << file_dir << "/" << file_name << " source: tf");
        // Call on_timer function every second
        auto timer_ = node->create_wall_timer(std::chrono::milliseconds(1000), getTransform);
    }
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();

  return 0;
}
