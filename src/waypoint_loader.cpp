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

// Based on Autoware waypoint loader, but here no Autoware message is used

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <vector>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class WaypointLoader : public rclcpp::Node
{
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
      if (param.get_name() == "speed_marker_unit")
      {
        speed_marker_unit = param.as_string();
      }
      if(param.get_name() == "per_waypoint_display")
      {
        per_waypoint = param.as_int();
      }
    }
    return result;
  }

public:
  WaypointLoader() : Node("waypoint_loader_node")
  {
    this->declare_parameter<std::string>("file_dir", "");
    this->declare_parameter<std::string>("file_name", "");
    this->declare_parameter<std::string>("waypoint_topic", "");
    this->declare_parameter<std::string>("speed_marker_unit", "kmph");
    this->declare_parameter<int>("per_waypoint_display", per_waypoint); // // display speed text marker every Nth waypoint
    this->get_parameter("file_dir", file_dir);
    this->get_parameter("file_name", file_name);
    this->get_parameter("speed_marker_unit", speed_marker_unit);
    this->get_parameter("per_waypoint_display", per_waypoint);
    multi_file_path_.clear();
    /*
    if (file_name.empty())
    {
      file_name = "tmp99.csv";
    }
    if (file_dir.empty())
    {
      file_dir = "/mnt/c/waypoints";
    }
    */
    parseColumns((file_dir + "/" + file_name), &multi_file_path_);
    lane_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("waypointarray", 1);
    speed_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("waypointarray_speeds", 1);
    mark_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypointarray_marker", 1);
    timer_ = this->create_wall_timer(500ms, std::bind(&WaypointLoader::timer_callback, this));
    sub_reinit_ = this->create_subscription<std_msgs::msg::Bool>("control_reinit", 10, std::bind(&WaypointLoader::reinitCallback, this, _1));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&WaypointLoader::parametersCallback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded file: " << file_dir << "/" << file_name << " ");
  }

private:
  void parseColumns(const std::string &line, std::vector<std::string> *columns)
  {
    std::istringstream ss(line);
    std::string column;
    while (std::getline(ss, column, ','))
    {
      while (1)
      {
        auto res = std::find(column.begin(), column.end(), ' ');
        if (res == column.end())
        {
          break;
        }
        column.erase(res);
      }
      if (!column.empty())
      {
        columns->emplace_back(column);
      }
    }
  }

  void parseWaypointForVer3(const std::string &line, const std::vector<std::string> &contents, geometry_msgs::msg::Pose *wp, float *speed)
  {
    std::vector<std::string> columns;
    parseColumns(line, &columns);
    std::unordered_map<std::string, std::string> map;
    for (size_t i = 0; i < contents.size(); i++)
    {
      map[contents.at(i)] = columns.at(i);
    }

    wp->position.x = std::stod(map["x"]);
    wp->position.y = std::stod(map["y"]);
    wp->position.z = std::stod(map["z"]);
    *speed = std::stod(map["mps"]);
    float yaw = std::stod(map["yaw"]);
    tf2::Quaternion toQuaternion;
    toQuaternion.setRPY(0, 0, yaw); // Create this quaternion from roll/pitch/yaw (in radians)
    wp->orientation.w = toQuaternion.w();
    wp->orientation.x = toQuaternion.x();
    wp->orientation.y = toQuaternion.y();
    wp->orientation.z = toQuaternion.z();
    /*
    wp->change_flag = std::stoi(map["change_flag"]);
    wp->wpstate.steering_state = (map.find("steering_flag") != map.end()) ? std::stoi(map["steering_flag"]) : 0;
    wp->wpstate.accel_state = (map.find("accel_flag") != map.end()) ? std::stoi(map["accel_flag"]) : 0;
    wp->wpstate.stop_state = (map.find("stop_flag") != map.end()) ? std::stoi(map["stop_flag"]) : 0;
    wp->wpstate.event_state = (map.find("event_flag") != map.end()) ? std::stoi(map["event_flag"]) : 0;
    */
    RCLCPP_DEBUG_STREAM(this->get_logger(), "X Y yaw: " << wp->position.x << " " << wp->position.y << " "
                                                        << " ");
  }

  //  FileFormat ver3 x,y,z,yaw,mps, first line consists on explanation of values
  //  mps = m/s metre per second ROS 2 standard SI unit for velocity

  void loadWaypointsForVer3(const char *filename, std::vector<geometry_msgs::msg::Pose> *wps, std::vector<float> *speeds)
  {
    std::ifstream ifs(filename);

    if (!ifs)
    {
      return;
    }

    std::string line;
    std::getline(ifs, line); // get first line
    std::vector<std::string> contents;
    parseColumns(line, &contents);

    // std::getline(ifs, line);  // remove second line
    while (std::getline(ifs, line))
    {
      geometry_msgs::msg::Pose wp;
      float speed;
      parseWaypointForVer3(line, contents, &wp, &speed);
      wps->emplace_back(wp);
      speeds->emplace_back(speed);
    }
    // RCLCPP_INFO_STREAM(this->get_logger(), "wps size: " << (*wps).size()  << " ");
  }

  bool verifyFileConsistency(const char *filename)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "verify...");
    std::ifstream ifs(filename);

    if (!ifs)
    {
      return false;
    }
    // TODO:
    return true;
  }

  float mapval(float x, float in_min, float in_max, float out_min, float out_max) const
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  std_msgs::msg::ColorRGBA getColor(float speed_to_color) const
  {
    const float max = 6.0; // max speed m/s
    std_msgs::msg::ColorRGBA color_w;
    // gradient
    // green-blue-red gradient: https://coolors.co/gradient-maker/7fffbb-0088cc-e73666
    float green_r = 127. / 255., green_g = 255. / 255., green_b = 187. / 255.; // #7FFFBB 127, 255, 187
    float red_r = 231. / 255., red_g = 54. / 255., red_b = 102. / 255.;        // #E73666 231, 54, 102
    float blue_r = 0. / 255., blue_g = 136. / 255., blue_b = 204. / 255.;      // #0088CC 0, 136, 204
    if (speed_to_color < 0)
    {
      color_w.r = green_r;
      color_w.g = green_g;
      color_w.b = green_b;
    }
    else if (speed_to_color > max)
    {
      color_w.r = red_r;
      color_w.g = red_g;
      color_w.b = red_b;
    }
    else if (0 < speed_to_color && speed_to_color < max / 2)
    {
      float c0 = mapval(speed_to_color, 0, max / 2, green_r, blue_r);
      float c1 = mapval(speed_to_color, 0, max / 2, green_g, blue_g);
      float c2 = mapval(speed_to_color, 0, max / 2, green_b, blue_b);
      color_w.r = c0;
      color_w.g = c1;
      color_w.b = c2;
    }
    else
    {
      float c0 = mapval(speed_to_color, max / 2, max, blue_r, red_r);
      float c1 = mapval(speed_to_color, max / 2, max, blue_g, red_g);
      float c2 = mapval(speed_to_color, max / 2, max, blue_b, red_b);
      color_w.r = c0;
      color_w.g = c1;
      color_w.b = c2;
    }
    color_w.a = 1.0;
    return color_w;
  }

  void pubLaneWaypoint(const std::string &file_path)
  {
    if (reinit){
      // clear wps_c and speeds_c, these will be filled with new data from file
      wps_c.clear();
      speeds_c.clear();
      if (!verifyFileConsistency(file_path.c_str()))
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "something wrong with lane data...");
        return;
      }

      RCLCPP_INFO_STREAM(this->get_logger(), "lane data is valid. publishing...");
      loadWaypointsForVer3(file_path.c_str(), &wps_c, &speeds_c);
      reinit = false;
    }
    std_msgs::msg::Float32MultiArray speed_array;
    geometry_msgs::msg::PoseArray wp_array;
    visualization_msgs::msg::MarkerArray mark_array;
    // load speeds_c to speed_array
    for (std::vector<float>::iterator it = speeds_c.begin(); it != speeds_c.end(); ++it)
    {
      speed_array.data.push_back(*it);
    }
    int id = 0;
    for (std::vector<geometry_msgs::msg::Pose>::iterator it = wps_c.begin(); it != wps_c.end(); ++it)
    {
      wp_array.poses.push_back(*it);
      visualization_msgs::msg::Marker mark_elem;
      mark_elem.header.frame_id = "/map";
      mark_elem.header.stamp = this->now();
      mark_elem.ns = "waypoints";
      mark_elem.id = id;
      mark_elem.scale.x = 1.0;
      mark_elem.scale.y = 0.4;
      mark_elem.scale.z = 0.6;
      mark_elem.color = getColor(speeds_c[id]);
      mark_elem.type = visualization_msgs::msg::Marker::CUBE;
      mark_elem.action = visualization_msgs::msg::Marker::MODIFY;
      mark_elem.pose = *it;
      mark_array.markers.push_back(mark_elem);
      if (id % per_waypoint == 0)
      {
        visualization_msgs::msg::Marker text_elem;
        text_elem.header.frame_id = "/map";
        text_elem.header.stamp = this->now();
        text_elem.ns = "speed_kmph";
        text_elem.id = id;
        text_elem.scale.x = 0.2 * per_waypoint;
        text_elem.scale.y = 0.1 * per_waypoint;
        text_elem.scale.z = 0.1 * per_waypoint;
        text_elem.color.r = 0.149;
        text_elem.color.g = 0.588;
        text_elem.color.b = 0.537;
        text_elem.color.a = 1.0;
        std::stringstream stream;
        if (speed_marker_unit == "mps") // Meter per second (m/s)
        {
          stream << std::fixed << std::setprecision(1) << speeds_c[id] << " m/s";
        }
        else if (speed_marker_unit == "mph") // Miles per hour (mph)
        {
          stream << std::fixed << std::setprecision(1) << speeds_c[id] * 2.23694 << " mph";
        }
        else // Kilometer per hour (km/h)
        {
          stream << std::fixed << std::setprecision(1) << speeds_c[id] * 3.6 << " km/h";
        }
        text_elem.text = stream.str();
        text_elem.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_elem.action = visualization_msgs::msg::Marker::MODIFY;
        text_elem.pose = *it;
        text_elem.pose.position.z += 0.4;
        mark_array.markers.push_back(text_elem);
      }

      id++;
    }
    lane_pub_->publish(wp_array);
    speed_pub_->publish(speed_array);
    mark_pub_->publish(mark_array);
  }

  void reinitCallback(const std_msgs::msg::Bool &msg)
  {
      reinit = msg.data;
  }

  void timer_callback()
  {
    // RCLCPP_INFO(this->get_logger(), "timer");
    for (const auto &el : multi_file_path_)
    {
      pubLaneWaypoint(el);
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr lane_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mark_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_reinit_;  
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  std::vector<std::string> multi_file_path_;
  std::string file_dir, file_name, speed_marker_unit;
  double interval_ = 0.4;
  bool topic_based_saving; // topic or transform (tf) based saving
  bool reinit = true;
  geometry_msgs::msg::PoseArray lane_array;
  std::vector<geometry_msgs::msg::Pose> wps_c;
  unsigned int per_waypoint = 5; // display speed text marker every 5th waypoint
  std::vector<float> speeds_c;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointLoader>());
  rclcpp::shutdown();
  return 0;
}