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
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
std::ofstream ofs;

class WaypointSaver : public rclcpp::Node
{
    geometry_msgs::msg::Pose previous_pose;
    float speed_mps;
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param : parameters)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
            if (param.get_name() == "tf_frame_id")
            {
                tf_frame_id = param.as_string();
            }
            if (param.get_name() == "tf_child_frame_id")
            {
                tf_child_frame_id = param.as_string();
            }
        }
        return result;
    }
public:
    WaypointSaver() : Node("waypoint_saver_node")
    {
        speed_mps = -1.0;
        this->declare_parameter<std::string>("file_dir", "");
        this->declare_parameter<std::string>("file_name", "");
        this->declare_parameter<std::string>("pose_topic", "");
        this->declare_parameter<std::string>("pose_topic_type", "");
        this->declare_parameter<bool>("topic_based_saving", false);
        this->declare_parameter<std::string>("tf_frame_id", "map");
        this->declare_parameter<std::string>("tf_child_frame_id", "base_link");
        this->get_parameter("file_dir", file_dir);
        this->get_parameter("file_name", file_name);
        this->get_parameter("pose_topic", pose_topic);
        this->get_parameter("pose_topic_type", pose_topic_type);
        this->get_parameter("topic_based_saving", topic_based_saving);
        this->get_parameter("tf_frame_id", tf_frame_id);
        this->get_parameter("tf_child_frame_id", tf_child_frame_id);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        if (file_name.empty())
        {
            file_name = "waypoint01.csv";
        }
        ofs.open((file_dir + "/" + file_name).c_str(), std::ios::app);
        // mps = m/s metre per second ROS 2 standard SI unit for velocity
        ofs << "x,y,z,yaw,mps,change_flag" << std::endl;
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&WaypointSaver::parametersCallback, this, std::placeholders::_1));
        if (topic_based_saving)
        {
            if(pose_topic_type.compare("PoseStamped") == 0)
            {
                sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&WaypointSaver::poseStampedCallback, this, _1));
            }
            else if(pose_topic_type.compare("Odometry") == 0)
            {
                sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(pose_topic, 10, std::bind(&WaypointSaver::odometryCallback, this, _1));
            }
            else if(pose_topic_type.compare("PoseArray") == 0)
            {
                sub_pose_array_ = this->create_subscription<geometry_msgs::msg::PoseArray>(pose_topic, 10, std::bind(&WaypointSaver::poseArrayCallback, this, _1));
            }            
            else
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Topic type not supported"); // TODO: PoseWithCovarianceStamped, PoseWithCovariance, Pose
                return;
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Save to " << file_dir << "/" << file_name << " source: pose topic");
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Save to " << file_dir << "/" << file_name << " source: tf");
            RCLCPP_INFO_STREAM(this->get_logger(), "tf_frame_id: " << tf_frame_id << " tf_child_frame_id: " << tf_child_frame_id);
            // Call on_timer function every 100 milliseconds
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&WaypointSaver::getTransform, this));
        }
        sub_vehicle_speed_ = this->create_subscription<std_msgs::msg::Float32>("vehicle_speed_kmph", 10, std::bind(&WaypointSaver::vehicleSpeedCallback, this, _1));
    }

private:

    void vehicleSpeedCallback(const std_msgs::msg::Float32 msg) 
    {
        speed_mps = msg.data / 3.6;
        //RCLCPP_INFO_STREAM(this->get_logger(), "Speed: " << msg.data << " km/h");
    }

    void poseStampedCallback(const geometry_msgs::msg::PoseStamped &current_pose)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), "X : " << current_pose.pose.position.x);
        double distance = sqrt(pow((current_pose.pose.position.x - previous_pose.position.x), 2) +
                               pow((current_pose.pose.position.y - previous_pose.position.y), 2));

        //std::ofstream ofs((file_dir + "/" + file_name).c_str(), std::ios::app);
        tf2::Quaternion q(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // if car moves [interval] meter
        if (distance >= interval_)
        {
            //RCLCPP_INFO_STREAM(this->get_logger(), "distance: " << std::fixed << std::setprecision(2) << distance);
            RCLCPP_INFO_STREAM(this->get_logger(), "X Y yaw: " << std::fixed << std::setprecision(2) << current_pose.pose.position.x << " " << current_pose.pose.position.y << " " << yaw);
            ofs << std::fixed << std::setprecision(4) << current_pose.pose.position.x << "," << current_pose.pose.position.y << "," << current_pose.pose.position.z << "," << yaw << "," << speed_mps << ",0" << std::endl;
            // update previous pose from values of current pose
            previous_pose = current_pose.pose;
        }
    }

    void poseArrayCallback(const geometry_msgs::msg::PoseArray &current_pose_arr)
    {
        // get second element of current_pose_arr array and print it



        double distance = sqrt(pow((current_pose_arr.poses[1].position.x - previous_pose.position.x), 2) +
                               pow((current_pose_arr.poses[1].position.y - previous_pose.position.y), 2));

        //std::ofstream ofs((file_dir + "/" + file_name).c_str(), std::ios::app);
        tf2::Quaternion q(
            current_pose_arr.poses[1].orientation.x,
            current_pose_arr.poses[1].orientation.y,
            current_pose_arr.poses[1].orientation.z,
            current_pose_arr.poses[1].orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // if car moves [interval] meter
        if (distance >= interval_)
        {
            //RCLCPP_INFO_STREAM(this->get_logger(), "distance: " << std::fixed << std::setprecision(2) << distance);
            RCLCPP_INFO_STREAM(this->get_logger(), "X Y yaw: " << std::fixed << std::setprecision(2) << current_pose_arr.poses[1].position.x << " " << current_pose_arr.poses[1].position.y << " " << yaw);
            ofs << std::fixed << std::setprecision(4) << current_pose_arr.poses[1].position.x << "," << current_pose_arr.poses[1].position.y << "," << current_pose_arr.poses[1].position.z << "," << yaw << "," << speed_mps << ",0" << std::endl;
            // update previous pose from values of current pose
            previous_pose = current_pose_arr.poses[1];
        }
    }

    void odometryCallback(const nav_msgs::msg::Odometry &current_odom)
    {
        double distance = sqrt(pow((current_odom.pose.pose.position.x - previous_pose.position.x), 2) +
                               pow((current_odom.pose.pose.position.y - previous_pose.position.y), 2));

        tf2::Quaternion q(
            current_odom.pose.pose.orientation.x,
            current_odom.pose.pose.orientation.y,
            current_odom.pose.pose.orientation.z,
            current_odom.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // if car moves [interval] meter
        if (distance >= interval_)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "X Y yaw: " << std::fixed << std::setprecision(2) << current_odom.pose.pose.position.x << " " << current_odom.pose.pose.position.y << " " << yaw);
            ofs << std::fixed << std::setprecision(4) << current_odom.pose.pose.position.x << "," << current_odom.pose.pose.position.y << "," << current_odom.pose.pose.position.z << "," << yaw << "," << speed_mps << ",0" << std::endl;
            // update previous pose from values of current pose
            previous_pose = current_odom.pose.pose;
        }
    }

    // get tf2 transform from map to base_link
    void getTransform()
    {
        //RCLCPP_INFO_STREAM(this->get_logger(), "Speed: " << speed_mps << " m/s");

        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_->lookupTransform(tf_frame_id, tf_child_frame_id, tf2::TimePointZero); 
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }catch(const std::exception &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Error TF");
            return;
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

        // if car moves [interval] meters
        if (distance > interval_)
        {
            tf2::Quaternion q(
                current_tf.orientation.x,
                current_tf.orientation.y,
                current_tf.orientation.z,
                current_tf.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            //RCLCPP_INFO_STREAM(this->get_logger(), "distance: " << std::fixed << std::setprecision(2) << distance);
            RCLCPP_INFO_STREAM(this->get_logger(), "X Y yaw speed: " << std::fixed << std::setprecision(2) << current_tf.position.x << " " << current_tf.position.y << " " << yaw << " " << speed_mps << " "<< file_name.c_str());
            ofs << std::fixed << std::setprecision(4) << current_tf.position.x << "," << current_tf.position.y << "," << current_tf.position.z << "," << yaw << "," << speed_mps << ",0" << std::endl;
            ofs.flush();
            previous_pose = current_tf;
            if (speed_mps < -0.01)
            {
                RCLCPP_WARN_STREAM(this->get_logger(), "Speed is less than 0");
            }
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_pose_array_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_vehicle_speed_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string file_dir, file_name, pose_topic, pose_topic_type, tf_child_frame_id, tf_frame_id;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    double interval_ = 0.4;
    geometry_msgs::msg::Pose current_tf;
    bool topic_based_saving; // topic or transform (tf) based saving    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointSaver>());
    ofs.close();
    rclcpp::shutdown();
    return 0;
}