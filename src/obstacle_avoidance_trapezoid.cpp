#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/float32.hpp>




#include <cmath> // For std::sqrt and std::pow
#include <vector>
#include <utility> // For std::pair

using namespace std::chrono_literals;
using std::placeholders::_1;


// TODO: plan code structure

class ObstacleAvoidanceTrapezoid : public rclcpp::Node
{
     rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector< rclcpp::Parameter > &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param : parameters)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
            if (param.get_name() == "detour_length_ ")
            {
                detour_length_ = param.as_double();
            }
            if (param.get_name() == "avoid_detour_length")
            {
                avoid_detour_length = param.as_double();
            }
            if (param.get_name() == "return_length_")
            {
                return_length_ = param.as_double();
            }
            if (param.get_name() == "avoid_return_length")
            {
                avoid_return_length = param.as_double();
            }
            if (param.get_name() == "offset_distance_")
            {
                offset_distance_ = param.as_double();
            }
            if (param.get_name() == "avoidance_direction")
            {
                avoidance_direction = param.as_string();
            }
            if (param.get_name() == "lookahead_distance_")
            {
                lookahead_distance_ = param.as_double();
            }
            if (param.get_name() == "sensitivity")
            {
                sensitivity = param.as_double();
            }
            if (param.get_name() == "min_distance_treshold")
            {
                min_distance_treshold = param.as_double();
            }
            if (param.get_name() == "waypoint_topic")
            {
                waypoint_topic = param.as_string();
            }
            if (param.get_name() == "pose_topic")
            {
                pose_topic = param.as_string();
            }
            if (param.get_name() == "obstacle_topic")
            {
                obstacle_topic = param.as_string();
            }
            if (param.get_name() == "lidar_frame")
            {
                lidar_frame = param.as_string();
            }
            
        }
        return result;
    }

   public:
    ObstacleAvoidanceTrapezoid()
        : Node("obstacle_avoidance_trapezoid")
    {
       
        this->declare_parameter("detour_length_", detour_length_);
        this->declare_parameter("avoid_detour_length", avoid_detour_length);
        this->declare_parameter("return_length_", return_length_);
        this->declare_parameter("avoid_return_length", avoid_return_length);
        this->declare_parameter("offset_distance_", offset_distance_);
        this->declare_parameter("avoidance_direction", avoidance_direction);
        this->declare_parameter("lookahead_distance_", lookahead_distance_);
        this->declare_parameter("sensitivity", sensitivity);
        this->declare_parameter("min_distance_treshold", min_distance_treshold);
        this->declare_parameter("waypoint_topic", "waypointarray"); //default
        this->declare_parameter("pose_topic", "rotated_pose"); //default
        this->declare_parameter("obstacle_topic", "clustered_marker"); //default
        this->declare_parameter("lidar_frame", "lexus3/os_center_a_laser_data_frame"); //default
        

        this->get_parameter("detour_length_", detour_length_);
        this->get_parameter("avoid_detour_length", avoid_detour_length);
        this->get_parameter("return_length_", return_length_);
        this->get_parameter("avoid_return_length", avoid_return_length);    
        this->get_parameter("offset_distance_", offset_distance_);
        this->get_parameter("avoidance_direction", avoidance_direction);
        this->get_parameter("lookahead_distance_", lookahead_distance_);
        this->get_parameter("sensitivity", sensitivity);
        this->get_parameter("min_distance_treshold", min_distance_treshold);
        this->get_parameter("waypoint_topic", waypoint_topic);
        this->get_parameter("pose_topic", pose_topic);
        this->get_parameter("obstacle_topic", obstacle_topic);
        this->get_parameter("lidar_frame", lidar_frame);
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ObstacleAvoidanceTrapezoid::parametersCallback, this, _1));


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        lane_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(waypoint_topic,10 ,std::bind(&ObstacleAvoidanceTrapezoid::lane_callback,this, std::placeholders::_1));
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&ObstacleAvoidanceTrapezoid::current_pose_callback, this, std::placeholders::_1));
        marker_array_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(obstacle_topic, 10, std::bind(&ObstacleAvoidanceTrapezoid::marker_array_callback, this, std::placeholders::_1));
        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_avoidance_waypoint_markers", 10);
        pose_array_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("obstacle_avoidance_pose_array_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ObstacleAvoidanceTrapezoid::timer_callback, this));
    }
    private:

    std::string waypoint_topic, pose_topic, obstacle_topic, lidar_frame;
    geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
    geometry_msgs::msg::PoseArray::SharedPtr waypoints_;
    geometry_msgs::msg::PoseArray::SharedPtr modified_waypoints;
    visualization_msgs::msg::MarkerArray::SharedPtr objects_;
    int waypoints_size = 0;
    int closest_waypoint_index ;
    int lookahead_distance_ = 50.0;
    int start_index = -1, end_index = -1;
    int avoidance_start_index, avoidance_end_index;
    int first_index, last_index;
    double distance = 0.0;
    
    visualization_msgs::msg::MarkerArray::SharedPtr msg_;

    // Trapezoid parameters   TODO: Check if all needed
    double detour_length_ = 5.0;
    double avoid_detour_length = 15.0;
    double detour_= detour_length_ + avoid_detour_length;
    double avoid_return_length = 10.0;
    double return_length_ = 6.0;
    double return_ = return_length_ + avoid_return_length;
    double avoid_length = avoid_detour_length + avoid_return_length;
    double offset_distance_ = 6.0;
    std::string avoidance_direction = "left";
    double min_distance_treshold = 5.0;
    double sensitivity = 3.0;

    double actual_len_of_avoid = 0.0;
    bool is_calculated = false;
    bool is_trapezoid = false;
    bool first_run = true;
    
    std::vector<int> closest_waypoint_index_m;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::map<int, int> index_counts;
    std::vector<int> repeated_indices;
    std::unordered_map<int, int> waypoint_frame_counts;

public:
    

    void timer_callback()
    {

        if (waypoints_ != nullptr && current_pose_ != nullptr) 
        {
            int closest_waypoint_index = find_closest_waypoint(current_pose_->pose.position.x, current_pose_->pose.position.y, *waypoints_);
            //RCLCPP_INFO(this->get_logger(), "waypoints_size: %d , Current pose: (%f, %f),closest_waypoint_index: %d", waypoints_size, current_pose_->pose.position.x, current_pose_->pose.position.y,closest_waypoint_index);
            
            if (objects_ != nullptr)
            {

                for (auto& marker : objects_->markers)
                {
                    for (auto& point : marker.points)
                    {
                        try
                        {
                            geometry_msgs::msg::PointStamped point_in;
                            point_in.point = point;
                            point_in.header.frame_id = marker.header.frame_id;

                            //RCLCPP_INFO(this->get_logger(), "Number of markers: %zu", objects_->markers.size());
                            if (!tf_buffer_->canTransform("map", lidar_frame, tf2::TimePointZero, std::chrono::seconds(1)))
                                {
                                    RCLCPP_WARN(this->get_logger(), "Waiting for transform timed out");
                                    continue;
                                }
                            else
                            {
                                geometry_msgs::msg::TransformStamped transformStamped = 
                                tf_buffer_->lookupTransform("map", point_in.header.frame_id, tf2::TimePointZero);

                                std::vector<geometry_msgs::msg::PointStamped::SharedPtr> points_out;

                                //geometry_msgs::msg::PointStamped point_out;
                                auto point_out = std::make_shared<geometry_msgs::msg::PointStamped>();
                                tf2::doTransform(point_in, *point_out, transformStamped);

                                points_out.push_back(point_out);
                                if (!is_calculated)
                                {
                                    std::tie(first_index, last_index) = processFrame(points_out, closest_waypoint_index, waypoints_);
                                }

                                if (first_index != -1 && last_index != -1 && first_index != last_index && first_index < last_index) 
                                {
                                    is_calculated = true;
                                }   

                                if (is_calculated)
                                {                                  
                                    std::tie(start_index, end_index ) = get_start_end_index(first_index, last_index);

                                    if (closest_waypoint_index > end_index)
                                    {

                                        waypoint_frame_counts.clear();
                                        // Reset variables
                                        is_calculated = false;
                                        first_index = -1;
                                        last_index = -1;
                                        start_index = -1;
                                        end_index = -1;
                                        is_trapezoid = false;
                                        actual_len_of_avoid = 0.0;
                                        first_run = true;
                                        
                                    }
                                    else
                                    {    
                                        if (is_calculated && !is_trapezoid && start_index != end_index && start_index < end_index)
                                        {
                                            for (int i = start_index; i <= end_index; ++i)
                                            {
                                                double x1 = waypoints_->poses[i].position.x;
                                                double y1 = waypoints_->poses[i].position.y;
                                                double x2, y2;

                                                // Calculate new waypoint positions and orientation
                                                if (i < end_index) // Check if next waypoint exists
                                                {
                                                    x2 = waypoints_->poses[i+1].position.x;
                                                    y2 = waypoints_->poses[i+1].position.y;
                                                }
                                                else
                                                {
                                                    x2 = x1;
                                                    y2 = y1;
                                                }
                                                
                                                

                                                double orientation = compute_orientation(x1, y1, x2, y2);
                                                actual_len_of_avoid += line_length(x1, x2, y1, y2);

                                                double new_x, new_y;

                                                if (actual_len_of_avoid < detour_length_) 
                                                {
                                                    double distance = offset_distance_ * (actual_len_of_avoid / detour_length_);
                                                    RCLCPP_INFO(this->get_logger(), " distance_first: %f", distance);
                                                    if (avoidance_direction == "left") 
                                                    {
                                                        new_x = x1 + distance * std::cos(orientation + M_PI / 2);
                                                        new_y = y1 + distance * std::sin(orientation + M_PI / 2);
                                                    }   
                                                    else 
                                                    {
                                                        new_x = x1 + distance * std::cos(orientation - M_PI / 2);
                                                        new_y = y1 + distance * std::sin(orientation - M_PI / 2);
                                                    }
                                                    

                                                }
                                                else if (actual_len_of_avoid < detour_length_ + avoid_length) 
                                                {
                                                    double distance = offset_distance_;
                                                    RCLCPP_INFO(this->get_logger(), " distance_second: %f", distance);
                                                    if (avoidance_direction == "left") 
                                                    {
                                                        new_x = x1 + distance * std::cos(orientation + M_PI / 2);
                                                        new_y = y1 + distance * std::sin(orientation + M_PI / 2);
                                                    }   
                                                    else 
                                                    {
                                                        new_x = x1 + distance * std::cos(orientation - M_PI / 2);
                                                        new_y = y1 + distance * std::sin(orientation - M_PI / 2);
                                                    }
                                                
                                                }
                                                else if (actual_len_of_avoid < detour_length_ + avoid_length + return_length_ && actual_len_of_avoid > detour_length_ + avoid_length) 
                                                {
                                                    double distance = offset_distance_ * (1 - (actual_len_of_avoid - detour_length_ - avoid_length) / return_length_);
                                                    RCLCPP_INFO(this->get_logger(), " distance_third: %f", distance);
                                                    if (avoidance_direction == "left") 
                                                    {
                                                        new_x = x1 + distance * std::cos(orientation + M_PI / 2);
                                                        new_y = y1 + distance * std::sin(orientation + M_PI / 2);
                                                    }   
                                                    else 
                                                    {
                                                        new_x = x1 + distance * std::cos(orientation - M_PI / 2);
                                                        new_y = y1 + distance * std::sin(orientation - M_PI / 2);
                                                    }
                                                }
                                                else
                                                {
                                                    new_x = x1;
                                                    new_y = y1;
                                                }

                                                waypoints_->poses[i].position.x = new_x;
                                                waypoints_->poses[i].position.y = new_y;

                                               

                                                for (int i = start_index; i < end_index-1; ++i)
                                                //for (size_t i = 0; i < waypoints_->poses.size(); ++i)
                                                {
                                                    double next_x = waypoints_->poses[i+1].position.x;
                                                    double next_y = waypoints_->poses[i+1].position.y;
                                                    double new_x = waypoints_->poses[i].position.x;
                                                    double new_y = waypoints_->poses[i].position.y;
                                                    double new_orientation = compute_orientation(new_x, new_y, next_x, next_y);

                                                    // Set the new orientation
                                                    double half_yaw = new_orientation * 0.5; ;
                                                    waypoints_->poses[i].orientation.w = std::cos(half_yaw);
                                                    waypoints_->poses[i].orientation.z = std::sin(half_yaw);
                                                    waypoints_->poses[i].orientation.x = 0.0;
                                                    waypoints_->poses[i].orientation.y = 0.0;
                                                }

                                                is_trapezoid = true;
                                            }
                                        }
                                    
                                    first_run = false;
                                    }    
                             
                                }
                                                                                                                       
                                //RCLCPP_INFO(this->get_logger(), "closest_waypoint_index: %d, start_index: %d, end_index: %d,first_index: %d, last_index: %d, is_calculated: %d, is_trapezoid: %d, first_run: %d ",  closest_waypoint_index, start_index, end_index,first_index,end_index, is_calculated,is_trapezoid, first_run);
                                
                                publishMarkers(closest_waypoint_index,start_index,end_index,first_index,last_index, waypoints_);
                                RCLCPP_INFO(this->get_logger(), "Size of waypoints_frame_count: %zu, is_calculated: %d",waypoint_frame_counts.size(), is_calculated);
                                
                            }
                        }
                        catch (tf2::TransformException &ex)
                        {
                            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
                        }
                    }
                    

                }
            }    
            else
            {
                RCLCPP_INFO(this->get_logger(), "No markers received");
            }
                
        }
    }

    void lane_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        

        waypoints_size = msg->poses.size(); // This line gets the number of elements in PoseArray
        if (first_run)
        {
            waypoints_ = msg;
        }
        
        if (waypoints_size < 2) {
            RCLCPP_WARN(this->get_logger(), "Not enough waypoints received");
            return;
        }
        
    }


    void current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_= msg;
    }
    
    void marker_array_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        objects_= msg;    
    }

    int find_closest_waypoint(double current_x, double current_y, const geometry_msgs::msg::PoseArray& waypoints)
    {
        double min_distance = std::numeric_limits<double>::max();
        int closest_waypoint_index = -1;

        for (size_t i = 0; i < waypoints.poses.size(); ++i) 
        {
            double waypoint_x = waypoints.poses[i].position.x;
            double waypoint_y = waypoints.poses[i].position.y;

            double distance = std::sqrt(std::pow(waypoint_x - current_x, 2) + std::pow(waypoint_y - current_y, 2));

            if (distance < min_distance) 
            {
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

    double compute_orientation(double x1, double y1, double x2, double y2)
        {
            // Calculate the difference in coordinates
            double delta_x = x2 - x1;
            double delta_y = y2 - y1;
            
            // Calculate the orientation
            double orientation = std::atan2(delta_y, delta_x);
            
            return orientation;
        }

    std::pair<int, int> processFrame(const std::vector<geometry_msgs::msg::PointStamped::SharedPtr>& points_out, int closest_waypoint_index, geometry_msgs::msg::PoseArray::SharedPtr waypoints_)
    {
        int first_index = -1;
        int last_index  = -1;
        
      
        
        //RCLCPP_INFO(this->get_logger(), "closest_waypoint_index: %d, first_index: %d, last_index: %d, is_calculated: %d ",  closest_waypoint_index, first_index, last_index, is_calculated, waypoints_size);
         
        
            std::unordered_set<int> waypoints_in_current_frame;
            
            
                for (const auto& point_out : points_out)
                {

                    double point_x = point_out->point.x;
                    double point_y = point_out->point.y;
                    

                    double min_distance = std::numeric_limits<double>::max();
                    int closest_waypoint_index_m = -1;
                    for (int i = closest_waypoint_index; i < closest_waypoint_index + lookahead_distance_; ++i)
                    {
                        double distance = std::sqrt(std::pow(waypoints_->poses[i].position.x - point_x, 2) +
                                                    std::pow(waypoints_->poses[i].position.y - point_y, 2));
                        if (distance < min_distance) 
                        {
                            min_distance = distance;
                            closest_waypoint_index_m = i;
                        }
                    }
                    if (closest_waypoint_index_m != -1 && min_distance < min_distance_treshold) 
                    {
                        //RCLCPP_INFO(this->get_logger(), "Closest waypoint to a marker point is %d with a distance of %f and current_pose is: %d", closest_waypoint_index_m, min_distance,closest_waypoint_index);
                        waypoints_in_current_frame.insert(closest_waypoint_index_m);
                    }
                    
                }
                //RCLCPP_INFO(this->get_logger(), "Size of waypoints_in_current_frame: %zu", waypoints_in_current_frame.size());
            
            for (auto& waypoint : waypoints_in_current_frame) 
            {
                waypoint_frame_counts[waypoint]++;
                //RCLCPP_INFO(this->get_logger(), "Size of waypoints_frame_count: %zu", waypoint_frame_counts.size());
            }

            if (waypoint_frame_counts.size() > 3) 
            {
                for (auto& pair : waypoint_frame_counts) 
                {
                   
                    if (pair.second >= sensitivity) 
                    {
                        if (first_index == -1 || pair.first < first_index) 
                        {
                            first_index = pair.first;
                        }
                        if (last_index == -1 || pair.first > last_index) 
                        {
                            last_index = pair.first;
                        }
                    }
                }
            }
            
        return std::make_pair (first_index, last_index);
    }            

    std::pair<int, int> get_start_end_index(size_t first_index,size_t last_index)
    {
        int start_index = -1;
        int end_index = -1;

        double first_x = waypoints_->poses[first_index].position.x;
        double first_y = waypoints_->poses[first_index].position.y;
        double last_x = waypoints_->poses[last_index].position.x;
        double last_y = waypoints_->poses[last_index].position.y;

        double initial_orientation = compute_orientation(first_x, first_y, last_x, last_y);
    
        start_index = find_closest_waypoint(first_x - detour_ * std::cos(initial_orientation), first_y - detour_ * std::sin(initial_orientation), *waypoints_);
        end_index = find_closest_waypoint(first_x + return_ * std::cos(initial_orientation), first_y + return_ * std::sin(initial_orientation), *waypoints_);
        int avoidance_start_index = find_closest_waypoint(first_x - avoid_detour_length * std::cos(initial_orientation), first_y - avoid_detour_length * std::sin(initial_orientation), *waypoints_);
        int avoidance_end_index = find_closest_waypoint(first_x + avoid_return_length * std::cos(initial_orientation),first_y + avoid_return_length + return_ * std::sin(initial_orientation), *waypoints_);

        return std::make_pair(start_index, end_index);
    }
          
    void publishMarkers(size_t closest_waypoint_index, size_t start_index, size_t end_index ,size_t first_index,size_t last_index, geometry_msgs::msg::PoseArray::SharedPtr waypoints_)
    {
        auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
        auto pose_array = std::make_shared<geometry_msgs::msg::PoseArray>();
        if (waypoints_ != nullptr) 
        {

            pose_array->header.frame_id = "map";
            pose_array->header.stamp = this->now();
            // Iterate over the waypoints
            for (size_t i = 0; i < waypoints_->poses.size(); ++i) 
            {
                // Create a Marker for the current waypoint
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = this->now();
                marker.ns = "waypoints";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::ARROW;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose = waypoints_->poses[i];

                if (i == closest_waypoint_index)
                {
                    // Set a different scale and color for the closest waypoint
                    marker.scale.x = 2.0;
                    marker.scale.y = 2.0;
                    marker.scale.z = 2.0;
                    marker.color.a = 1.0;
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                }
                else if (i==start_index)
                {
                    marker.scale.x = 1.5;
                    marker.scale.y = 1.5;
                    marker.scale.z = 1.5;
                    marker.color.a = 1.0;
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                }
                else if (i==end_index)
                {
                    marker.scale.x = 1.5;
                    marker.scale.y = 1.5;
                    marker.scale.z = 1.5;
                    marker.color.a = 1.0;
                    marker.color.r = 0.0;
                    marker.color.g = 0.5;
                    marker.color.b = 1.0;
                }
                else if (i==first_index)
                {
                    marker.scale.x = 1.5;
                    marker.scale.y = 1.5;
                    marker.scale.z = 1.5;
                    marker.color.a = 1.0;
                    marker.color.r = 0.5;
                    marker.color.g = 0.5;
                    marker.color.b = 1.0;
                }
                else if (i==last_index)
                {
                    marker.scale.x = 1.5;
                    marker.scale.y = 1.5;
                    marker.scale.z = 1.5;
                    marker.color.a = 1.0;
                    marker.color.r = 0.0;
                    marker.color.g = 0.5;
                    marker.color.b = 0.5;
                }
                else if (i==closest_waypoint_index + lookahead_distance_)
                {
                    marker.scale.x = 1.5;
                    marker.scale.y = 1.5;
                    marker.scale.z = 1.5;
                    marker.color.a = 1.0;
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                }

                else
                {
                    marker.scale.x = 0.7;
                    marker.scale.y = 0.7;
                    marker.scale.z = 0.7;
                    marker.color.a = 1.0;
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                }
                
                // Add the Marker to the MarkerArray
                marker_array->markers.push_back(marker);
                pose_array->poses.push_back(waypoints_->poses[i]);

            }
        }
        // Publish the MarkerArray
        marker_pub->publish(*marker_array);
        pose_array_pub->publish(*pose_array);
    }
    

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr lane_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared< ObstacleAvoidanceTrapezoid >());
    rclcpp::shutdown();
    return 0;
}