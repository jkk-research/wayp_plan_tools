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
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

rclcpp::Node::SharedPtr node;
std::string file_dir, file_name;
std::vector<std::string> multi_file_path_;
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr lane_pub_;
rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_pub_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mark_pub_;

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
  toQuaternion.setRPY(0, 0, yaw);  // Create this quaternion from roll/pitch/yaw (in radians)
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
  RCLCPP_DEBUG_STREAM(node->get_logger(), "X Y yaw: " << wp->position.x << " " << wp->position.y << " " << " ");
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
  //RCLCPP_INFO_STREAM(node->get_logger(), "wps size: " << (*wps).size()  << " ");
}

bool verifyFileConsistency(const char *filename)
{
  //RCLCPP_INFO_STREAM(node->get_logger(), "verify...");
  std::ifstream ifs(filename);

  if (!ifs)
  {
    return false;
  }
  // TODO:
  return true;
}

float mapval(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

std_msgs::msg::ColorRGBA getColor(float speed_to_color){
  const float max = 6.0; // max speed m/s
  std_msgs::msg::ColorRGBA color_w;
  // gradient
  // green-blue-red gradient: https://coolors.co/gradient-maker/7fffbb-0088cc-e73666
  float green_r = 127./255., green_g = 255./255., green_b = 187./255.; // #7FFFBB 127, 255, 187
  float red_r = 231./255., red_g =54./255., red_b =102./255.; // #E73666 231, 54, 102
  float blue_r = 0./255., blue_g =136./255., blue_b =204./255.; // #0088CC 0, 136, 204
  if(speed_to_color < 0){
      color_w.r = green_r; color_w.g = green_g; color_w.b = green_b;
  }
  else if(speed_to_color > max){
      color_w.r = red_r; color_w.g = red_g; color_w.b = red_b;
  }
  else if( 0 < speed_to_color && speed_to_color < max / 2){
      float c0 = mapval(speed_to_color, 0, max / 2, green_r, blue_r);
      float c1 = mapval(speed_to_color, 0, max / 2, green_g, blue_g);
      float c2 = mapval(speed_to_color, 0, max / 2, green_b, blue_b);
      color_w.r = c0; color_w.g = c1; color_w.b = c2;
  }
  else{
      float c0 = mapval(speed_to_color, max / 2, max, blue_r, red_r);
      float c1 = mapval(speed_to_color, max / 2, max, blue_g, red_g);
      float c2 = mapval(speed_to_color, max / 2, max, blue_b, red_b);   
      color_w.r = c0; color_w.g = c1; color_w.b = c2;
  }
  color_w.a = 1.0;
  return color_w;
}



void pubLaneWaypoint(const std::string &file_path)
{
  if (!verifyFileConsistency(file_path.c_str()))
  {
    RCLCPP_DEBUG_STREAM(node->get_logger(), "lane data is something wrong...");
    return;
  }

  RCLCPP_DEBUG_STREAM(node->get_logger(), "lane data is valid. publishing...");
  std::vector<geometry_msgs::msg::Pose> wps;
  std::vector<float> speeds;
  geometry_msgs::msg::PoseArray wp_array;
  visualization_msgs::msg::MarkerArray mark_array;
  loadWaypointsForVer3(file_path.c_str(), &wps, &speeds);
  int id = 0;
  for (std::vector<geometry_msgs::msg::Pose>::iterator it = wps.begin() ; it != wps.end(); ++it){
    wp_array.poses.push_back(*it);
    visualization_msgs::msg::Marker mark_elem;
    mark_elem.header.frame_id = "/map";
    mark_elem.header.stamp = node->now();
    mark_elem.ns = "waypoints";
    mark_elem.id = id; 
    mark_elem.scale.x = 1.0; mark_elem.scale.y = 0.4; mark_elem.scale.z = 0.6;
    mark_elem.color = getColor(speeds[id]);
    mark_elem.type = visualization_msgs::msg::Marker::CUBE;
    mark_elem.action = visualization_msgs::msg::Marker::ADD;
    mark_elem.pose = *it;
    mark_array.markers.push_back(mark_elem);
    if (id % 5 == 0)
    {
      visualization_msgs::msg::Marker text_elem;
      text_elem.header.frame_id = "/map";
      text_elem.header.stamp = node->now();
      text_elem.ns = "speed_kmph";
      text_elem.id = id; 
      text_elem.scale.x = 1.0; text_elem.scale.y = 0.4; text_elem.scale.z = 0.6;
      text_elem.color.r = 0.149; text_elem.color.g = 0.588; text_elem.color.b = 0.537; text_elem.color.a = 1.0;
      std::stringstream stream;
      stream << std::fixed << std::setprecision(1) << speeds[id]*3.6 << " km/h";
      text_elem.text = stream.str();
      text_elem.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_elem.action = visualization_msgs::msg::Marker::ADD;
      text_elem.pose = *it;
      text_elem.pose.position.z += 0.4; 
      mark_array.markers.push_back(text_elem);
      
    }
    

    id++;
  }
  lane_pub_->publish(wp_array);
  mark_pub_->publish(mark_array);

}


void createLaneArray(const std::vector<std::string> &paths, geometry_msgs::msg::PoseArray *lane_array)
{
  for (const auto &el : paths){
    pubLaneWaypoint(el);
  }
}


int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("waypoint_loader_node");

  node->declare_parameter<std::string>("file_dir", "");
  node->declare_parameter<std::string>("file_name", "");
  node->declare_parameter<std::string>("waypoint_topic", "");
  node->get_parameter("file_dir", file_dir);
  node->get_parameter("file_name", file_name);
  multi_file_path_.clear();
  parseColumns((file_dir + "/" + file_name), &multi_file_path_);
  geometry_msgs::msg::PoseArray lane_array; 
  lane_pub_ = node->create_publisher<geometry_msgs::msg::PoseArray>("/lexus3/waypointarray", 10);
  speed_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("/lexus3/waypointarray_speeds", 10); // TODO:
  mark_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/lexus3/waypointarray_marker", 10);
  rclcpp::Rate loop_rate(10);
  RCLCPP_INFO_STREAM(node->get_logger(), "Loaded file: " << file_dir << "/" << file_name << " ");
  while (rclcpp::ok())
  {
    createLaneArray(multi_file_path_, &lane_array);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}