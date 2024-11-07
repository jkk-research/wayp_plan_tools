#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>




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
           else if (param.get_name() == "avoid_detour_length")
            {
                avoid_detour_length = param.as_double();
            }
           else if (param.get_name() == "return_length_")
            {
                return_length_ = param.as_double();
            }
            else if (param.get_name() == "avoid_return_length")
            {
                avoid_return_length = param.as_double();
            }
            else if (param.get_name() == "offset_distance_")
            {
                offset_distance_ = param.as_double();
            }
            else if (param.get_name() == "avoidance_direction")
            {
                avoidance_direction = param.as_string();
            }
            else if (param.get_name() == "lookahead_distance_")
            {
                lookahead_distance_ = param.as_double();
            }
            else if (param.get_name() == "sensitivity")
            {
                sensitivity = param.as_double();
            }
            else if (param.get_name() == "min_distance_treshold")
            {
                min_distance_treshold = param.as_double();
            }
            else if (param.get_name() == "waypoint_topic")
            {
                waypoint_topic = param.as_string();
            }
            else if (param.get_name() == "pose_topic")
            {
                pose_topic = param.as_string();
            }
            else if (param.get_name() == "obstacle_topic")
            {
                obstacle_topic = param.as_string();
            }
            else if (param.get_name() == "lidar_frame")
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
        this->declare_parameter<bool>("odometry_topic", true); //default
        

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
        this->get_parameter("odometry_topic", odometry_topic);
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ObstacleAvoidanceTrapezoid::parametersCallback, this, _1));


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        lane_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(waypoint_topic,10 ,std::bind(&ObstacleAvoidanceTrapezoid::lane_callback,this, std::placeholders::_1));
       
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&ObstacleAvoidanceTrapezoid::current_pose_callback, this, std::placeholders::_1));
        
        marker_array_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(obstacle_topic, 10, std::bind(&ObstacleAvoidanceTrapezoid::marker_array_callback, this, std::placeholders::_1));
        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_avoidance_waypoint_markers", 10);
        pose_array_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("obstacle_avoidance_pose_array_topic", 10);
        debug_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("debug_markers", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ObstacleAvoidanceTrapezoid::timer_callback, this));
    }
    private:

    std::string waypoint_topic, pose_topic, obstacle_topic, lidar_frame;
    geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
    geometry_msgs::msg::PoseArray::SharedPtr waypoints_;
    geometry_msgs::msg::PoseArray::SharedPtr modified_waypoints;
    visualization_msgs::msg::MarkerArray::SharedPtr objects_;
    geometry_msgs::msg::PoseArray::SharedPtr previous_waypoints_;


    
    int waypoints_size = 0;
    int closest_waypoint_index ;
    int lookahead_distance_index;
    int lookahead_distance_= 50 ;
    int start_index = -1, end_index = -1;
    int avoidance_start_index = -1 , avoidance_end_index= -1;
    int first_index = -1, last_index = -1;
    double distance = 0.0;
    
    visualization_msgs::msg::MarkerArray::SharedPtr msg_;

    // Trapezoid parameters   TODO: Check if all needed
    double detour_length_ ;
    double avoid_detour_length ;
    double avoid_return_length ;
    double return_length_ ;

    double distance_first_start; 
    double distance_first_avoidance_end;
    double distance_first_avoidance_start;
    double distance_first_end;
   
    double offset_distance_ ;
    std::string avoidance_direction = "left";
    double min_distance_treshold ;
    double sensitivity ;

    double actual_len_of_avoid = 0.0;
    bool is_calculated = false;
    bool is_trapezoid = false;
    bool first_run = true;
    bool is_avoiding = false;
    bool odometry_topic = false; //default
    bool new_lap;
    
    std::vector<int> closest_waypoint_index_m;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::map<int, int> index_counts;
    std::vector<int> repeated_indices;
    std::unordered_map<int, int> waypoint_frame_counts;
    std::vector<int> sensitive_waypoints;
    std::vector<geometry_msgs::msg::PointStamped::SharedPtr> points_out;


public:
    

    void timer_callback()
    {

        if (waypoints_ != nullptr && current_pose_ != nullptr) 
        {
            int closest_waypoint_index = find_closest_waypoint(current_pose_->pose.position.x, current_pose_->pose.position.y, *waypoints_);

            int lookahead_distance_index = closest_waypoint_index + lookahead_distance_;
            lookahead_distance_index = lookahead_distance_index % waypoints_size;

        

            //publishMarkers(waypoints_);
            publishDebugMarkers(closest_waypoint_index, start_index, end_index, first_index, last_index, waypoints_,lookahead_distance_index);
            //RCLCPP_INFO(this->get_logger(), "start_index: %d, end_index: %d, first_index %d,last_index %d,avoidence_start_index: %d, avoidence_end_index: %d , is_calculated: %d, closest_waypoint_index:%d,is_first_run %s, Is avoiding: %s" , start_index, end_index,first_index,last_index, avoidance_start_index,avoidance_end_index , is_calculated, closest_waypoint_index, first_run ? "true" : "false" ,is_avoiding ? "true" : "false" );
        
            //print out sensitive waypoints
            for (auto& waypoint : sensitive_waypoints)
            {
                RCLCPP_INFO(this->get_logger(), "Sensitive waypoint: %d", waypoint);
            }
            
          
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

                            if (!tf_buffer_->canTransform("map", lidar_frame, tf2::TimePointZero, std::chrono::seconds(1)))
                                {
                                    RCLCPP_WARN(this->get_logger(), "Waiting for transform timed out");
                                    continue;
                                }
                            else
                            {
                                geometry_msgs::msg::TransformStamped transformStamped = 
                                tf_buffer_->lookupTransform("map", point_in.header.frame_id, tf2::TimePointZero);

                                // std::vector<geometry_msgs::msg::PointStamped::SharedPtr> points_out;

                                //geometry_msgs::msg::PointStamped point_out;
                                auto point_out = std::make_shared<geometry_msgs::msg::PointStamped>();
                                tf2::doTransform(point_in, *point_out, transformStamped);

                                points_out.push_back(point_out);                             
                              
                                
                                
                                                                
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



            
             //RCLCPP_INFO(this->get_logger(), "start_index: %d, end_index: %d, closest_waypoint_index:%d,lookahead:%d,waypoints size:%d" , start_index, end_index, closest_waypoint_index,lookahead_distance_index,waypoints_size);
        

            if (!is_calculated && !is_avoiding)
            {
                //RCLCPP_INFO(this->get_logger(), "IS CALCULATED FALSE , IS AVOIDING FALSE first_index: %d, last_index: %d, start_index: %d, end_index: %d", first_index, last_index, start_index, end_index);
                std::tie(first_index, last_index) = processFrame(points_out, closest_waypoint_index,lookahead_distance_index, waypoints_);
                if (first_index != -1 && last_index != -1)
                {
                    is_calculated = true;
                }                                    
            }


            else if (is_calculated && !is_avoiding)
            {      
                
                //RCLCPP_INFO(this->get_logger(), "IS CALCULATED TRUE , IS AVOIDING FALSE first_index: %d, last_index: %d, start_index: %d, end_index: %d", first_index, last_index, start_index, end_index);
                std::tie(start_index, end_index,avoidance_start_index,avoidance_end_index ) = get_start_end_index(first_index, last_index);
                
                if (start_index != -1 && end_index != -1)
                {
                    is_avoiding = true;
                }

            }

            else if (is_calculated && is_avoiding)
            {
                //RCLCPP_INFO(this->get_logger(), "IS CALCULATED TRUE , IS AVOIDING TRUE first_index: %d, last_index: %d, start_index: %d, end_index: %d", first_index, last_index, start_index, end_index);
                if (!is_trapezoid && start_index != -1 && end_index != -1)
                {
                    RCLCPP_INFO(this->get_logger(), "tervezes, closest_waypoint_index: %d, start_index: %d, end_index: %d", closest_waypoint_index, start_index, end_index);
                    double distance_first_start = distanceBetweenPoints(waypoints_->poses[first_index], waypoints_->poses[start_index]);
                    double distance_first_avoidance_end = distanceBetweenPoints(waypoints_->poses[first_index], waypoints_->poses[avoidance_end_index]);
                    double distance_first_avoidance_start = distanceBetweenPoints(waypoints_->poses[first_index], waypoints_->poses[avoidance_start_index]);
                    double distance_first_end = distanceBetweenPoints(waypoints_->poses[first_index], waypoints_->poses[end_index]);
                    calculateTrapezoid(waypoints_, closest_waypoint_index,start_index, end_index, detour_length_, return_length_, offset_distance_, avoidance_direction, distance_first_start, distance_first_avoidance_end, distance_first_avoidance_start, distance_first_end);
                    is_trapezoid = true;
                    
                }
                first_run = false;
        
                
        
                if (!is_trapezoid)
                {
                    RCLCPP_INFO(this->get_logger(), "PARAMETER SET is NOT avaible");
                    calculateTrapezoid(waypoints_, closest_waypoint_index,start_index, end_index, detour_length_, return_length_, offset_distance_, avoidance_direction, distance_first_start, distance_first_avoidance_end, distance_first_avoidance_start, distance_first_end);
                    
                }
            }    


            if (is_calculated && is_avoiding && start_index < end_index && closest_waypoint_index > end_index && closest_waypoint_index < lookahead_distance_index) 
            {
                RCLCPP_INFO(this->get_logger(), "Resetting variables,AAAAAAAAAAAAA, closest_waypoint_index: %d, start_index: %d, end_index: %d", closest_waypoint_index, start_index, end_index);
                waypoint_frame_counts.clear();
                sensitive_waypoints.clear();
                // Reset variables
                is_calculated = false;
                first_index = -1;
                last_index = -1;
                start_index = -1;
                end_index = -1;
                avoidance_start_index = -1;
                avoidance_end_index = -1;
                is_trapezoid = false;
                actual_len_of_avoid = 0.0;
                first_run = true;
                is_avoiding = false;
            }

            else if (is_calculated && is_avoiding && start_index < end_index && closest_waypoint_index > end_index && closest_waypoint_index > lookahead_distance_index && end_index > lookahead_distance_index )
            {
                RCLCPP_INFO(this->get_logger(), "Resetting variables,CCCCC, closest_waypoint_index: %d, start_index: %d, end_index: %d", closest_waypoint_index, start_index, end_index);
                waypoint_frame_counts.clear();
                sensitive_waypoints.clear();
                // Reset variables
                is_calculated = false;
                first_index = -1;
                last_index = -1;
                start_index = -1;
                end_index = -1;
                avoidance_start_index = -1;
                avoidance_end_index = -1;
                is_trapezoid = false;
                actual_len_of_avoid = 0.0;
                first_run = true;
                is_avoiding = false;
            }    

            // else if (is_calculated && is_avoiding && start_index > end_index && closest_waypoint_index - waypoints_size > end_index && closest_waypoint_index - waypoints_size < start_index && closest_waypoint_index > lookahead_distance_index)
            // {
            //     RCLCPP_INFO(this->get_logger(), "Resetting variables,BBBBBBBBBBBBBB, closest_waypoint_index: %d, start_index: %d, end_index: %d", closest_waypoint_index, start_index, end_index);
            //     waypoint_frame_counts.clear();
            //     sensitive_waypoints.clear();
            //     // Reset variables
            //     is_calculated = false;
            //     first_index = -1;
            //     last_index = -1;
            //     start_index = -1;
            //     end_index = -1;
            //     avoidance_start_index = -1;
            //     avoidance_end_index = -1;
            //     is_trapezoid = false;
            //     actual_len_of_avoid = 0.0;
            //     first_run = true;
            //     is_avoiding = false;
            // }
    
                    
            // }                                           
                                                                                                                                    
            publishMarkers(waypoints_);
            // publishDebugMarkers(closest_waypoint_index, start_index, end_index, first_index, last_index, waypoints_,lookahead_distance_index);
            // RCLCPP_INFO(this->get_logger(), "start_index: %d, end_index: %d, first_index %d,last_index %d,avoidence_start_index: %d, avoidence_end_index: %d , is_calculated: %d, closest_waypoint_index:%d, lookahead_distance_index:%d, closest_waypoint_index - waypoints_size:%d, is_first_run %s, Is avoiding: %s" , start_index, end_index,first_index,last_index, avoidance_start_index,avoidance_end_index , is_calculated, closest_waypoint_index,lookahead_distance_index,closest_waypoint_index - waypoints_size, first_run ? "true" : "false" ,is_avoiding ? "true" : "false" );
            

                
        }

    }

    void lane_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        

        waypoints_size = msg->poses.size(); // This line gets the number of elements in PoseArray
        
        if (first_run==true)
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

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat)
        {
            tf2::Quaternion q(
                quat.x,
                quat.y,
                quat.z,
                quat.w
            );
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            return yaw;
        }

    


    std::pair<int, int> processFrame(const std::vector<geometry_msgs::msg::PointStamped::SharedPtr>& points_out, int closest_waypoint_index, int lookahead_distance_index, geometry_msgs::msg::PoseArray::SharedPtr waypoints_)
    {
        int first_index = -1;
        int last_index  = -1;
        

        waypoints_size = waypoints_->poses.size(); 

        std::unordered_set<int> waypoints_in_current_frame;
        
        
        for (const auto& point_out : points_out)
        {

            double point_x = point_out->point.x;
            double point_y = point_out->point.y;
            

           if   (closest_waypoint_index < lookahead_distance_index)
            {
                for (int i = closest_waypoint_index; i < lookahead_distance_index ; ++i)
                {
                    double distance = std::sqrt(std::pow(waypoints_->poses[i].position.x - point_x, 2) +
                                                std::pow(waypoints_->poses[i].position.y - point_y, 2));
                    if (distance < min_distance_treshold) 
                    {
                        waypoints_in_current_frame.insert(i);
                    }
                }
            }
            else if (closest_waypoint_index > lookahead_distance_index)
            {
               
               int l = lookahead_distance_index + waypoints_size;
                for (int i = closest_waypoint_index; i < l ; ++i)
                 {
                      double distance = std::sqrt(std::pow(waypoints_->poses[i - waypoints_size].position.x - point_x, 2) +
                                                  std::pow(waypoints_->poses[i - waypoints_size].position.y - point_y, 2));
                      if (distance < min_distance_treshold) 
                      {
                            waypoints_in_current_frame.insert(i % waypoints_size);
                      }
                 }
            }   

        }
      
        
        for (auto& waypoint : waypoints_in_current_frame) 
        {
            waypoint_frame_counts[waypoint]++;
            
        }

        for (const auto& waypoint : waypoint_frame_counts) 
        {
            //RCLCPP_INFO(this->get_logger(), "Waypoint index: %d, Count: %d", waypoint.first, waypoint.second);
            if (waypoint.second > sensitivity) 
            {
                if (std::find(sensitive_waypoints.begin(), sensitive_waypoints.end(), waypoint.first) == sensitive_waypoints.end()) 
                {
                    sensitive_waypoints.push_back(waypoint.first);
                }
            }

        }
        

        if (sensitive_waypoints.size() > 0) 
        {
            std::sort(sensitive_waypoints.begin(), sensitive_waypoints.end());

            last_index = sensitive_waypoints[0];
            for (size_t i = 0; i < sensitive_waypoints.size() - 1; ++i) 
            {
                if (sensitive_waypoints[i + 1] - sensitive_waypoints[i] > 1) 
                {
                    break;
                }
                last_index = sensitive_waypoints[i + 1];
            }

            first_index = *sensitive_waypoints.begin();
        }
        
        return std::make_pair (first_index, last_index);
    }       

    std::tuple<int, int,int,int> get_start_end_index(size_t first_index,size_t last_index)
    {   
       // RCLCPP_INFO(this->get_logger(), "detour_length: %f, return_length: %f,detour_: %f, return_: %f,avoid_detour_length %f", detour_length_, return_length_, detour_, return_, avoid_detour_length);
        
        int start_index = -1;
        int end_index = -1;

        double detour_= detour_length_ + avoid_detour_length;
        double return_ = return_length_ + avoid_return_length;

        double first_x = waypoints_->poses[first_index].position.x;
        double first_y = waypoints_->poses[first_index].position.y;
        double last_x = waypoints_->poses[last_index].position.x;
        double last_y = waypoints_->poses[last_index].position.y;

        // if first and last index are the same, initial orientation is the waypoints orientation else it is the orientation between the first and last index
        double initial_orientation;

        if (first_index == last_index)
        {
            initial_orientation = getYawFromQuaternion(waypoints_->poses[first_index].orientation);
        }
        else
        {
            initial_orientation = compute_orientation(first_x, first_y, last_x, last_y);
        }

        //print out the initial orientation and the first and last index
        RCLCPP_INFO(this->get_logger(), "initial_orientation: %f, first_index: %d, last_index: %d", initial_orientation, first_index, last_index);
    
        start_index = find_closest_waypoint(first_x - detour_ * std::cos(initial_orientation), first_y - detour_ * std::sin(initial_orientation), *waypoints_);
        end_index = find_closest_waypoint(first_x + return_  * std::cos(initial_orientation), first_y + return_ * std::sin(initial_orientation), *waypoints_);
        int avoidance_start_index = find_closest_waypoint(first_x - avoid_detour_length * std::cos(initial_orientation), first_y - avoid_detour_length * std::sin(initial_orientation), *waypoints_);
        int avoidance_end_index = find_closest_waypoint(first_x +  avoid_return_length  * std::cos(initial_orientation),first_y  + avoid_return_length  * std::sin(initial_orientation), *waypoints_);

        //If end_index is less than start_index, swap them

        // if (end_index < start_index)
        // {
        //     std::swap(end_index, start_index);
        // }

        //check the the distancetwo point and the closer to the closest waypoint is the start point

        

        return std::make_tuple(start_index, end_index,avoidance_start_index,avoidance_end_index);
    }


    void calculateTrapezoid(geometry_msgs::msg::PoseArray::SharedPtr waypoints_, int closest_waypoint_index, int start_index, int end_index, double detour_length_, double return_length_, double offset_distance_, std::string avoidance_direction, double distance_first_start, double distance_first_avoidance_end, double distance_first_avoidance_start, double distance_first_end) 
    {

        double avoid_length = avoid_detour_length + avoid_return_length;        

        for (int i = start_index; i <= end_index; ++i)
        {

             

            if (i >= waypoints_size)
            {
                i = 0;
            }

            //print out the i value
            RCLCPP_INFO(this->get_logger(), "i: %d", i);



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

            if (actual_len_of_avoid < distance_first_start - distance_first_avoidance_start) 
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
            else if (actual_len_of_avoid > distance_first_start - distance_first_avoidance_start && actual_len_of_avoid < distance_first_start + distance_first_avoidance_end) 
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
            else if (actual_len_of_avoid > distance_first_start + distance_first_avoidance_end && actual_len_of_avoid < distance_first_start + distance_first_end) 
            {
                double distance = offset_distance_ * (-1) * ((actual_len_of_avoid - detour_length_ - avoid_length - return_length_) / return_length_);
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

            

            for (int i = start_index; i < end_index; ++i)
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
        }
    }  

    double distanceBetweenPoints(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2) 
    {
    return std::sqrt(std::pow(p2.position.x - p1.position.x, 2) + std::pow(p2.position.y - p1.position.y, 2));
    }  

          
    void publishMarkers( geometry_msgs::msg::PoseArray::SharedPtr waypoints_)
    {       
            geometry_msgs::msg::PoseArray::SharedPtr previous_waypoints_;

            
            
            if (previous_waypoints_ == nullptr || waypoints_->poses != previous_waypoints_->poses)
            {
                //RCLCPP_INFO(this->get_logger(), "Publishing markers");
                previous_waypoints_ = waypoints_;

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
                        marker.scale.x = 0.35;
                        marker.scale.y = 0.35;
                        marker.scale.z = 0.35;
                        marker.color.a = 1.0;
                        marker.color.r = 0.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;
                    
                        
                        // Add the Marker to the MarkerArray
                        marker_array->markers.push_back(marker);
                        pose_array->poses.push_back(waypoints_->poses[i]);
                    }

                }
                // Publish the MarkerArray
                marker_pub->publish(*marker_array);
                pose_array_pub->publish(*pose_array);
            }

            
    }

    void publishDebugMarkers (size_t closest_waypoint_index, size_t start_index, size_t end_index ,size_t first_index,size_t last_index, geometry_msgs::msg::PoseArray::SharedPtr waypoints_,int lookahead_distance_index)
    {
        static size_t prev_closest_waypoint_index = -1;
        static size_t prev_start_index = -1;
        static size_t prev_end_index = -1;
        static size_t prev_first_index = -1;
        static size_t prev_last_index = -1;

        if (closest_waypoint_index != prev_closest_waypoint_index || start_index != prev_start_index || end_index != prev_end_index || first_index != prev_first_index || last_index != prev_last_index) 
        {

            auto debug_marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
            if (waypoints_ != nullptr) 
            {
                
                // Create a Marker for the current waypoint
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = this->now();
                marker.ns = "waypoints";
                marker.id = 0;
                marker.type = visualization_msgs::msg::Marker::ARROW;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose = waypoints_->poses[closest_waypoint_index];
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                marker.scale.x = 1.5;
                marker.scale.y = 1.1;
                marker.scale.z = 1.1;
                debug_marker_array->markers.push_back(marker);

                // Create a Marker for the closest waypoint + lookahead distance
                visualization_msgs::msg::Marker lookahead_marker;
                lookahead_marker.header.frame_id = "map";
                lookahead_marker.header.stamp = this->now();
                lookahead_marker.ns = "waypoints";
                lookahead_marker.id = 5;
                lookahead_marker.type = visualization_msgs::msg::Marker::ARROW;
                lookahead_marker.action = visualization_msgs::msg::Marker::ADD;
                lookahead_marker.pose = waypoints_->poses[lookahead_distance_index];
                lookahead_marker.color.r = 1.0;
                lookahead_marker.color.g = 1.0;
                lookahead_marker.color.b = 0.0;
                lookahead_marker.color.a = 1.0;
                lookahead_marker.scale.x = 1.5;
                lookahead_marker.scale.y = 1.1;
                lookahead_marker.scale.z = 1.1;
                debug_marker_array->markers.push_back(lookahead_marker);

                if (start_index != -1 && end_index != -1 ) 
                {
                    // Create a Marker for the start waypoint
                    visualization_msgs::msg::Marker start_marker;
                    start_marker.header.frame_id = "map";
                    start_marker.header.stamp = this->now();
                    start_marker.ns = "waypoints";
                    start_marker.id = 1;
                    start_marker.type = visualization_msgs::msg::Marker::ARROW;
                    start_marker.action = visualization_msgs::msg::Marker::ADD;
                    start_marker.pose = waypoints_->poses[start_index];
                    start_marker.color.r = 0.0;
                    start_marker.color.g = 1.0;
                    start_marker.color.b = 0.0;
                    start_marker.color.a = 1.0;
                    start_marker.scale.x = 1.5;
                    start_marker.scale.y = 1.1;
                    start_marker.scale.z = 1.1;
                    debug_marker_array->markers.push_back(start_marker);

                    // Create a Marker for the end waypoint
                    visualization_msgs::msg::Marker end_marker;
                    end_marker.header.frame_id = "map";
                    end_marker.header.stamp = this->now();
                    end_marker.ns = "waypoints";
                    end_marker.id = 2;
                    end_marker.type = visualization_msgs::msg::Marker::ARROW;
                    end_marker.action = visualization_msgs::msg::Marker::ADD;
                    end_marker.pose = waypoints_->poses[end_index];
                    end_marker.color.r = 0.0;
                    end_marker.color.g = 0.0;
                    end_marker.color.b = 1.0;
                    end_marker.color.a = 1.0;
                    end_marker.scale.x = 1.5;
                    end_marker.scale.y = 1.1;
                    end_marker.scale.z = 1.1;
                    debug_marker_array->markers.push_back(end_marker);
                }
                if (first_index != -1 && last_index != -1)
                {
                    // Create a Marker for the first waypoint
                    visualization_msgs::msg::Marker first_marker;
                    first_marker.header.frame_id = "map";
                    first_marker.header.stamp = this->now();
                    first_marker.ns = "waypoints";
                    first_marker.id = 3;
                    first_marker.type = visualization_msgs::msg::Marker::ARROW;
                    first_marker.action = visualization_msgs::msg::Marker::ADD;
                    first_marker.pose = waypoints_->poses[first_index];
                    first_marker.color.r = 0.5;
                    first_marker.color.g = 0.5;
                    first_marker.color.b = 1.0;
                    first_marker.color.a = 1.0;
                    first_marker.scale.x = 1.5;
                    first_marker.scale.y = 1.1;
                    first_marker.scale.z = 1.1;
                    debug_marker_array->markers.push_back(first_marker);

                    // Create a Marker for the last waypoint
                    visualization_msgs::msg::Marker last_marker;
                    last_marker.header.frame_id = "map";
                    last_marker.header.stamp = this->now();
                    last_marker.ns = "waypoints";
                    last_marker.id = 4;
                    last_marker.type = visualization_msgs::msg::Marker::ARROW;
                    last_marker.action = visualization_msgs::msg::Marker::ADD;
                    last_marker.pose = waypoints_->poses[last_index];
                    last_marker.color.r = 0.0;
                    last_marker.color.g = 0.5;
                    last_marker.color.b = 0.5;
                    last_marker.color.a = 1.0;
                    last_marker.scale.x = 1.5;
                    last_marker.scale.y = 1.1;
                    last_marker.scale.z = 1.1;
                    debug_marker_array->markers.push_back(last_marker);

                }
                if (avoidance_end_index != -1 && avoidance_start_index != -1)
                {
                    // Create a Marker for the avoidance start waypoint
                    visualization_msgs::msg::Marker avoidance_start_marker;
                    avoidance_start_marker.header.frame_id = "map";
                    avoidance_start_marker.header.stamp = this->now();
                    avoidance_start_marker.ns = "waypoints";
                    avoidance_start_marker.id = 7;
                    avoidance_start_marker.type = visualization_msgs::msg::Marker::ARROW;
                    avoidance_start_marker.action = visualization_msgs::msg::Marker::ADD;
                    avoidance_start_marker.pose = waypoints_->poses[avoidance_start_index];
                    avoidance_start_marker.color.r = 0.5;
                    avoidance_start_marker.color.g = 0.5;
                    avoidance_start_marker.color.b = 1.0;
                    avoidance_start_marker.color.a = 1.0;
                    avoidance_start_marker.scale.x = 1.5;
                    avoidance_start_marker.scale.y = 1.1;
                    avoidance_start_marker.scale.z = 1.1;
                    debug_marker_array->markers.push_back(avoidance_start_marker);

                    // Create a Marker for the avoidance end waypoint
                    visualization_msgs::msg::Marker avoidance_end_marker;
                    avoidance_end_marker.header.frame_id = "map";
                    avoidance_end_marker.header.stamp = this->now();
                    avoidance_end_marker.ns = "waypoints";
                    avoidance_end_marker.id = 8;
                    avoidance_end_marker.type = visualization_msgs::msg::Marker::ARROW;
                    avoidance_end_marker.action = visualization_msgs::msg::Marker::ADD;
                    avoidance_end_marker.pose = waypoints_->poses[avoidance_end_index];
                    avoidance_end_marker.color.r = 0.0;
                    avoidance_end_marker.color.g = 0.5;
                    avoidance_end_marker.color.b = 0.5;
                    avoidance_end_marker.color.a = 1.0;
                    avoidance_end_marker.scale.x = 1.5;
                    avoidance_end_marker.scale.y = 1.1;
                    avoidance_end_marker.scale.z = 1.1;
                    debug_marker_array->markers.push_back(avoidance_end_marker);
                }

                    // Create a Marker for the sensitive waypoints
                for (size_t i = 0; i < sensitive_waypoints.size(); ++i) 
                {
                    visualization_msgs::msg::Marker sensitive_marker;
                    sensitive_marker.header.frame_id = "map";
                    sensitive_marker.header.stamp = this->now();
                    sensitive_marker.ns = "waypoints";
                    sensitive_marker.id = 6 + i;
                    sensitive_marker.type = visualization_msgs::msg::Marker::ARROW;
                    sensitive_marker.action = visualization_msgs::msg::Marker::ADD;
                    sensitive_marker.pose = waypoints_->poses[sensitive_waypoints[i]];
                    sensitive_marker.color.r = 1.0;
                    sensitive_marker.color.g = 0.5;
                    sensitive_marker.color.b = 0.0;
                    sensitive_marker.color.a = 1.0;
                    sensitive_marker.scale.x = 1.5;
                    sensitive_marker.scale.y = 1.1;
                    sensitive_marker.scale.z = 1.1;
                    debug_marker_array->markers.push_back(sensitive_marker);
                }
                debug_marker_pub->publish(*debug_marker_array);
            }
            prev_closest_waypoint_index = closest_waypoint_index;
            prev_start_index = start_index;
            prev_end_index = end_index;
            prev_first_index = first_index;
            prev_last_index = last_index;
            
        }
    }


    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr lane_sub_;   
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared< ObstacleAvoidanceTrapezoid >());
    rclcpp::shutdown();
    return 0;
}