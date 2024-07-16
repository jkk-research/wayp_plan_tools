#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>




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



        lane_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("waypointarray",10 ,std::bind(&ObstacleAvoidanceTrapezoid::lane_callback,this, std::placeholders::_1));
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 10, std::bind(&ObstacleAvoidanceTrapezoid::current_pose_callback, this, std::placeholders::_1));
        marker_array_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("clustered_marker", 10, std::bind(&ObstacleAvoidanceTrapezoid::marker_array_callback, this, std::placeholders::_1));
    }
    private:


    geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
    geometry_msgs::msg::PoseArray::SharedPtr waypoints_;
    int waypoints_size, closest_waypoint_index;
    int lookahead_distance_ = 10.0;
    int start_index, end_index, avoidance_start_index, avoidance_end_index;

    // Trapezoid parameters   TODO: Check if all needed
    double detour_length_ = 10.0;
    double avoid_detour_length = 2.0;
    double detour_= detour_length_ + avoid_detour_length;
    double avoid_return_length = 8.0;
    double return_length_ = 2.0;
    double return_ = return_length_ + avoid_return_length;
    double avoid_length = avoid_detour_length + avoid_return_length;
    double offset_distance_ = 2.0;
    std::string avoidance_direction = "left";
    double actual_len_of_avoid = 0.0;

    std::map<int, int> index_counts;
    std::vector<int> repeated_indices;

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

                 // Increment count for this index
                index_counts[i]++;

                // If this index appears more than 3 times, store it in the vector
                if (index_counts[i] > 3) {
                    // Check if this index is already in the vector
                    if (std::find(repeated_indices.begin(), repeated_indices.end(), i) == repeated_indices.end()) {
                        repeated_indices.push_back(i);
                        // RCLCPP_INFO(this->get_logger(), "Index %d appears more than 3 times", i);
                    }
                }
            }
        }
    }

    if (!repeated_indices.empty()) {
        int first_index = repeated_indices.front();
        int last_index = repeated_indices.back();

        double first_x = waypoints_->poses[first_index].position.x;
        double first_y = waypoints_->poses[first_index].position.y;
        double last_x = waypoints_->poses[last_index].position.x;
        double last_y = waypoints_->poses[last_index].position.y;

        double initial_orientation = compute_orientation(first_x, first_y, last_x, last_y);
       
        int start_index = find_closest_waypoint(first_x - detour_ * std::cos(initial_orientation), first_y - detour_ * std::sin(initial_orientation), *waypoints_);
        int end_index = find_closest_waypoint(last_x + return_ * std::cos(initial_orientation), last_y + return_ * std::sin(initial_orientation), *waypoints_);
        int avoidance_start_index = find_closest_waypoint(first_x - avoid_detour_length * std::cos(initial_orientation), first_y - avoid_detour_length * std::sin(initial_orientation), *waypoints_);
        int avoidance_end_index = find_closest_waypoint(last_x + avoid_return_length * std::cos(initial_orientation), avoid_return_length + return_ * std::sin(initial_orientation), *waypoints_);

        // RCLCPP_INFO(this->get_logger(), "Line length between first and last repeated index: %f", line_length);
        }  

    for (size_t i = start_index ; i < end_index+1; ++i) 
    {
        double x1 = waypoints_->poses[i].position.x;
        double y1 = waypoints_->poses[i].position.y;
        double x2 = waypoints_->poses[i+1].position.x;
        double y2 = waypoints_->poses[i+1].position.y;

        double distance = 0.0;

        actual_len_of_avoid += line_length(x1, x2, y1, y2);
        double orientation = compute_orientation(x1, y1, x2, y2);

        if (actual_len_of_avoid < detour_length_) 
        {
           double distance = offset_distance_ * (actual_len_of_avoid / detour_length_);
        }
        else if (actual_len_of_avoid < detour_length_ + avoid_length) 
        {
           double distance = offset_distance_;
        }
        else if (actual_len_of_avoid < detour_length_ + avoid_length + return_length_) 
        {
           double distance = offset_distance_ * (1 - (actual_len_of_avoid - detour_length_ - avoid_length) / return_length_);
        }
        

        double new_x, new_y;
        if (avoidance_direction == "left") {
            new_x = x1 + distance * std::cos(orientation + M_PI / 2);
            new_y = y1 + distance * std::sin(orientation + M_PI / 2);
        } else {
            new_x = x1 + distance * std::cos(orientation - M_PI / 2);
            new_y = y1 + distance * std::sin(orientation - M_PI / 2);
        }

        // Set the new position
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

    double compute_orientation(double x1, double y1, double x2, double y2)
        {
            // Calculate the difference in coordinates
            double delta_x = x2 - x1;
            double delta_y = y2 - y1;
            
            // Calculate the orientation
            double orientation = std::atan2(delta_y, delta_x);
            
            return orientation;
        }


    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr lane_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_sub_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared< ObstacleAvoidanceTrapezoid >());
    rclcpp::shutdown();
    return 0;
}