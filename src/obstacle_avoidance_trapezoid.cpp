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

        this->get_parameter("detour_length_", detour_length_);
        this->get_parameter("avoid_detour_length", avoid_detour_length);
        this->get_parameter("return_length_", return_length_);
        this->get_parameter("avoid_return_length", avoid_return_length);    
        this->get_parameter("offset_distance_", offset_distance_);
        this->get_parameter("avoidance_direction", avoidance_direction);
        this->get_parameter("lookahead_distance_", lookahead_distance_);
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ObstacleAvoidanceTrapezoid::parametersCallback, this, _1));



        lane_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("sim1/waypointarray",10 ,std::bind(&ObstacleAvoidanceTrapezoid::lane_callback,this, std::placeholders::_1));
        //current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 10, std::bind(&ObstacleAvoidanceTrapezoid::current_pose_callback, this, std::placeholders::_1));
        marker_array_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("clustered_marker", 10, std::bind(&ObstacleAvoidanceTrapezoid::marker_array_callback, this, std::placeholders::_1));
        closest_waypoint_sub_ = this->create_subscription<std_msgs::msg::Float32>("closest_waypoint", 10, std::bind(&ObstacleAvoidanceTrapezoid::closest_waypoint_callback, this, std::placeholders::_1));
        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("avoidance_waypoint_markers", 10);
    }
    private:


    geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
    geometry_msgs::msg::PoseArray::SharedPtr waypoints_;
    int waypoints_size = 0;
    int closest_waypoint_index = 0;
    int lookahead_distance_ = 20.0;
    int start_index, end_index, avoidance_start_index, avoidance_end_index;
    int first_index, last_index;
    double distance = 0.0;
    


    // Trapezoid parameters   TODO: Check if all needed
    double detour_length_ = 2.0;
    double avoid_detour_length = 5.0;
    double detour_= detour_length_ + avoid_detour_length;
    double avoid_return_length = 10.0;
    double return_length_ = 2.0;
    double return_ = return_length_ + avoid_return_length;
    double avoid_length = avoid_detour_length + avoid_return_length;
    double offset_distance_ = 2.0;
    std::string avoidance_direction = "left";
    double actual_len_of_avoid = 0.0;
    bool is_calculated = false;
    bool first_run = true;
    bool is_avoiding = false;
    std::vector<int> closest_waypoint_index_m;


    std::map<int, int> index_counts;
    std::vector<int> repeated_indices;
    std::unordered_map<int, int> waypoint_frame_counts;

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

    void closest_waypoint_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        closest_waypoint_index = static_cast<int>(msg->data);        
    }


    // void current_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    // {
    //     current_pose_= msg;
    //     if (waypoints_) {
    //     int closest_waypoint_index = find_closest_waypoint(current_pose_->pose.position.x, current_pose_->pose.position.y, *waypoints_);
    //     }
    // }
    
    void marker_array_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
   
    int lookahead_distance_ = 20.0;
    if (waypoints_ != nullptr)
    {
    if (waypoints_size - closest_waypoint_index < lookahead_distance_) 
        {
            lookahead_distance_ = waypoints_size - closest_waypoint_index;
        }
    }    

    
    
    std::tie(first_index, last_index) = processFrame(msg);
    processtrapezoid(first_index, last_index); 
       
    publishMarkers();

    RCLCPP_INFO(this->get_logger(), "waypoints_size: %d, closest_waypoint_index: %d, lookahead_distance_: %d", waypoints_size, closest_waypoint_index, lookahead_distance_);
    
  
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

    std::pair<int, int> processFrame(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        int first_index = -1;
        int last_index = -1;
        

        if (waypoints_ != nullptr) 
        {
            std::unordered_set<int> waypoints_in_current_frame;
            for (auto& marker : msg->markers) 
            {
                for (auto& point : marker.points) 
                {
                    double min_distance = std::numeric_limits<double>::max();
                    int closest_waypoint_index_m = -1;
                    for (int i = closest_waypoint_index; i < closest_waypoint_index + lookahead_distance_; ++i)
                    {
                        double distance = std::sqrt(std::pow(waypoints_->poses[i].position.x - point.x, 2) +
                                                    std::pow(waypoints_->poses[i].position.y - point.y, 2));
                        if (distance < min_distance) 
                        {
                            min_distance = distance;
                            closest_waypoint_index_m = i;
                        }
                    }
                    if (closest_waypoint_index_m != -1 && min_distance < 5.0 ) 
                    {
                        //RCLCPP_INFO(this->get_logger(), "Closest waypoint to a marker point is %d with a distance of %f and current_pose is: %d", closest_waypoint_index_m, min_distance,closest_waypoint_index);
                        waypoints_in_current_frame.insert(closest_waypoint_index_m);
                    }
                }
            }
            for (auto& waypoint : waypoints_in_current_frame) 
            {
                waypoint_frame_counts[waypoint]++;
            }
            for (auto& pair : waypoint_frame_counts) 
            {
                RCLCPP_INFO(this->get_logger(), "Waypoint %d frame count: %d", pair.first, pair.second);
                if (pair.second >= 3) 
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
            return std::make_pair(first_index, last_index);
        }
    }        

        void processtrapezoid(int first_index, int last_index)  
        {
            if (!is_calculated)
            {
                //if (first_index != -1 && last_index != -1) 
                //{

                    double first_x = waypoints_->poses[first_index].position.x;
                    double first_y = waypoints_->poses[first_index].position.y;
                    double last_x = waypoints_->poses[last_index].position.x;
                    double last_y = waypoints_->poses[last_index].position.y;

                    double initial_orientation = compute_orientation(first_x, first_y, last_x, last_y);
                
                    int start_index = find_closest_waypoint(first_x - detour_ * std::cos(initial_orientation), first_y - detour_ * std::sin(initial_orientation), *waypoints_);
                    int end_index = find_closest_waypoint(last_x + return_ * std::cos(initial_orientation), last_y + return_ * std::sin(initial_orientation), *waypoints_);
                    int avoidance_start_index = find_closest_waypoint(first_x - avoid_detour_length * std::cos(initial_orientation), first_y - avoid_detour_length * std::sin(initial_orientation), *waypoints_);
                    int avoidance_end_index = find_closest_waypoint(last_x + avoid_return_length * std::cos(initial_orientation), avoid_return_length + return_ * std::sin(initial_orientation), *waypoints_);

                    RCLCPP_INFO(this->get_logger(), "start_index: %d, end_index: %d, avoidance_start_index: %d, avoidance_end_index: %d", 
                    start_index, end_index, avoidance_start_index, avoidance_end_index);

                    double distance = 0.0;

                    for (size_t i = start_index ; i < end_index+1; ++i) 
                    {
                        double x1 = waypoints_->poses[i].position.x;
                        double y1 = waypoints_->poses[i].position.y;
                        double x2 = waypoints_->poses[i+1].position.x;
                        double y2 = waypoints_->poses[i+1].position.y;

                        

                        actual_len_of_avoid += line_length(x1, x2, y1, y2);
                        double orientation = compute_orientation(x1, y1, x2, y2);

                        
                        double new_x, new_y;

                        if (actual_len_of_avoid < detour_length_) 
                        {
                            double distance = offset_distance_ * (actual_len_of_avoid / detour_length_);
                            RCLCPP_INFO(this->get_logger(), " distance: %f", distance);
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
                            waypoints_->poses[i].position.x = new_x;
                            waypoints_->poses[i].position.y = new_y;

                        }
                        else if (actual_len_of_avoid < detour_length_ + avoid_length) 
                        {
                            double distance = offset_distance_;
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
                            waypoints_->poses[i].position.x = new_x;
                            waypoints_->poses[i].position.y = new_y;

                        }
                        else if (actual_len_of_avoid < detour_length_ + avoid_length + return_length_) 
                        {
                            double distance = offset_distance_ * (1 - (actual_len_of_avoid - detour_length_ - avoid_length) / return_length_);
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
                            waypoints_->poses[i].position.x = new_x;
                            waypoints_->poses[i].position.y = new_y;

                        }
                        

                        RCLCPP_INFO(this->get_logger(), " x,y values: %f,%f,%f,%f , actual_len_of_avoid: %f, orientation: %f",x1,x2,y1,y2, actual_len_of_avoid,orientation);

                        // // Set the new position
                        waypoints_->poses[i].position.x = new_x;
                        waypoints_->poses[i].position.y = new_y;

                        // Compute and set the new orientation
                        
                        double new_orientation = compute_orientation(x1, y1, new_x, new_y);

                        // Set the new orientation
                        double half_yaw = new_orientation * 0.5;
                        waypoints_->poses[i].orientation.w = std::cos(half_yaw);
                        waypoints_->poses[i].orientation.z = std::sin(half_yaw);
                        waypoints_->poses[i].orientation.x = 0.0;
                        waypoints_->poses[i].orientation.y = 0.0;
                        is_calculated = true;

                    }
                //}
            }

            first_run = false;

            
            

        }
        void publishMarkers()
        {
            auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
            if (waypoints_ != nullptr) 
            {
                // Iterate over the waypoints
                for (size_t i = 0; i < waypoints_->poses.size(); ++i) 
                {
                    // Create a Marker for the current waypoint
                    visualization_msgs::msg::Marker marker;
                    marker.header.frame_id = "map";
                    marker.header.stamp = this->now();
                    marker.ns = "waypoints";
                    marker.id = i;
                    marker.type = visualization_msgs::msg::Marker::SPHERE;
                    marker.action = visualization_msgs::msg::Marker::ADD;
                    marker.pose = waypoints_->poses[i];
                    marker.scale.x = 0.2;
                    marker.scale.y = 0.2;
                    marker.scale.z = 0.2;
                    marker.color.a = 1.0;
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;

                    // Add the Marker to the MarkerArray
                    marker_array->markers.push_back(marker);
                }
            }
            // Publish the MarkerArray
            marker_pub->publish(*marker_array);
        }
        


    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr lane_sub_;
    //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr closest_waypoint_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared< ObstacleAvoidanceTrapezoid >());
    rclcpp::shutdown();
    return 0;
}