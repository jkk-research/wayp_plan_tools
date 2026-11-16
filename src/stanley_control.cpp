// Stanley lateral controller
//
// Implementation of the steering control law of
//   S. Thrun et al., "Stanley: The Robot that Won the DARPA Grand Challenge",
//   Journal of Field Robotics 23(9), pp. 661-692, 2006, Section 9.2 (Eq. 7)
//   https://robots.stanford.edu/papers/thrun.stanley05.pdf
//
//   delta(t) = psi(t) + arctan( k * x(t) / u(t) )
//
//   psi(t): orientation of the nearest path segment relative to the vehicle's own orientation
//   x(t):   cross-track error, lateral distance of the center of the FRONT axle from the
//           nearest point of the trajectory (signed, positive if the path is on the left)
//   u(t):   vehicle speed
//   k:      gain (rate of the exponential convergence: x(t) ~ x(0) * exp(-k t))
//
// Practical additions (not part of Eq. 7, but needed on a real vehicle):
//   - k_soft is added to the speed in the denominator so the law is defined at u = 0
//     (set it to 0.0 for the pure law from the paper)
//   - the steering command is saturated to [-max_steer, max_steer]
//
// Input:  global waypoint array (geometry_msgs/PoseArray in the map frame),
//         the vehicle pose from TF (map -> base_link, base_link is the rear axle center),
//         the target speed (pursuitspeedtarget)
// Output: geometry_msgs/Twist, linear.x = speed, angular.z = steering angle [rad]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class StanleyControl : public rclcpp::Node
{
public:
  StanleyControl() : Node("stanley_control_node")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor_slider;
    rcl_interfaces::msg::FloatingPointRange range;
    descriptor_slider.read_only = false;
    descriptor_slider.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
    descriptor_slider.description = "Slider parameter";
    range.set__from_value(0.0).set__to_value(5.0);
    descriptor_slider.floating_point_range = {range};

    this->declare_parameter<std::string>("waypoint_topic", waypoint_topic);
    this->declare_parameter<std::string>("cmd_topic", cmd_topic);
    this->declare_parameter<std::string>("map_frame", map_frame);
    this->declare_parameter<std::string>("base_frame", base_frame);
    this->declare_parameter<double>("wheelbase", wheelbase);
    this->declare_parameter<double>("cross_track_err_gain", cross_track_err_gain, descriptor_slider);
    this->declare_parameter<double>("heading_err_gain", heading_err_gain, descriptor_slider);
    this->declare_parameter<double>("k_soft", k_soft);
    this->declare_parameter<double>("max_steer", max_steer);
    this->get_parameter("waypoint_topic", waypoint_topic);
    this->get_parameter("cmd_topic", cmd_topic);
    this->get_parameter("map_frame", map_frame);
    this->get_parameter("base_frame", base_frame);
    this->get_parameter("wheelbase", wheelbase);
    this->get_parameter("cross_track_err_gain", cross_track_err_gain);
    this->get_parameter("heading_err_gain", heading_err_gain);
    this->get_parameter("k_soft", k_soft);
    this->get_parameter("max_steer", max_steer);
    if (waypoint_topic.empty())
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "waypoint_topic is not set, using default value: waypointarray");
      waypoint_topic = "waypointarray";
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    reinit_pub_ = this->create_publisher<std_msgs::msg::Bool>("control_reinit", 10);
    deb_c_pub_ = this->create_publisher<std_msgs::msg::Float32>("debug_cross_track_err", 10);
    deb_h_pub_ = this->create_publisher<std_msgs::msg::Float32>("debug_heading_err", 10);
    deb_s_pub_ = this->create_publisher<std_msgs::msg::Float32>("debug_steering_angle", 10);
    sub_w_ = this->create_subscription<geometry_msgs::msg::PoseArray>(waypoint_topic, 10, std::bind(&StanleyControl::waypointCallback, this, _1));
    sub_s_ = this->create_subscription<std_msgs::msg::Float32>("pursuitspeedtarget", 10, std::bind(&StanleyControl::speedCallback, this, _1));
    // 20 Hz, the same rate as in the paper
    timer_ = this->create_wall_timer(50ms, std::bind(&StanleyControl::timerLoop, this));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&StanleyControl::parametersCallback, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "stanley_control_node started, waypoint_topic: " << waypoint_topic << " cmd_topic: " << cmd_topic);
    std::this_thread::sleep_for(600ms);
    reinitControl();
  }

private:
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
      const std::string &name = param.get_name();
      RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << name << ": " << param.value_to_string());
      if (name == "waypoint_topic")
      {
        waypoint_topic = param.as_string();
        sub_w_ = this->create_subscription<geometry_msgs::msg::PoseArray>(waypoint_topic, 10, std::bind(&StanleyControl::waypointCallback, this, _1));
        path_initialized = false;
      }
      else if (name == "wheelbase")
        wheelbase = param.as_double();
      else if (name == "cross_track_err_gain")
        cross_track_err_gain = param.as_double();
      else if (name == "heading_err_gain")
        heading_err_gain = param.as_double();
      else if (name == "k_soft")
        k_soft = param.as_double();
      else if (name == "max_steer")
        max_steer = param.as_double();
    }
    return result;
  }

  static double normalizeAngle(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q_msg)
  {
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  // Result of projecting the front axle point onto a path segment
  struct Projection
  {
    double dist_sq = std::numeric_limits<double>::max();
    double signed_dist = 0.0; // positive if the point is on the left of the path
    double seg_yaw = 0.0;     // orientation of the segment in the map frame
  };

  // Project point (px, py) onto segment i -> i+1 of the path
  Projection projectOnSegment(size_t i, double px, double py) const
  {
    Projection p;
    const auto &a = path_.poses[i].position;
    const auto &b = path_.poses[(i + 1) % path_.poses.size()].position;
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double len_sq = dx * dx + dy * dy;
    if (len_sq < 1e-9)
    {
      // degenerate segment (duplicate waypoints): use the waypoint itself and its orientation
      double ex = px - a.x, ey = py - a.y;
      p.seg_yaw = yawFromQuaternion(path_.poses[i].orientation);
      p.dist_sq = ex * ex + ey * ey;
      p.signed_dist = std::cos(p.seg_yaw) * ey - std::sin(p.seg_yaw) * ex;
      return p;
    }
    double t = ((px - a.x) * dx + (py - a.y) * dy) / len_sq;
    t = std::clamp(t, 0.0, 1.0);
    double cx = a.x + t * dx, cy = a.y + t * dy;
    double ex = px - cx, ey = py - cy;
    double len = std::sqrt(len_sq);
    p.dist_sq = ex * ex + ey * ey;
    // the sign comes from the cross product of the segment direction and the error vector
    p.signed_dist = (dx * ey - dy * ex) / len;
    p.seg_yaw = std::atan2(dy, dx);
    return p;
  }

  // Search for the nearest path segment to the front axle point
  // A window around the previous nearest segment is searched, the whole path only at (re)initialization
  bool findNearestSegment(double px, double py, Projection &best)
  {
    const size_t n = path_.poses.size();
    if (n < 2)
      return false;
    const size_t seg_count = closed_loop_ ? n : n - 1;
    auto search = [&](long from, long to) {
      Projection best_local;
      size_t best_idx = nearest_idx_;
      for (long j = from; j <= to; j++)
      {
        long idx = j;
        if (closed_loop_)
          idx = ((j % long(seg_count)) + long(seg_count)) % long(seg_count);
        else if (j < 0 || j >= long(seg_count))
          continue;
        Projection p = projectOnSegment(size_t(idx), px, py);
        if (p.dist_sq < best_local.dist_sq)
        {
          best_local = p;
          best_idx = size_t(idx);
        }
      }
      return std::make_pair(best_local, best_idx);
    };
    std::pair<Projection, size_t> res;
    if (path_initialized)
    {
      res = search(long(nearest_idx_) - search_back, long(nearest_idx_) + search_forward);
    }
    // full search at start or when the local search result is far away (e.g. relocalization)
    if (!path_initialized || res.first.dist_sq > reinit_distance * reinit_distance)
    {
      res = search(0, long(seg_count) - 1);
      path_initialized = true;
    }
    best = res.first;
    nearest_idx_ = res.second;
    return best.dist_sq < std::numeric_limits<double>::max();
  }

  bool getVehiclePose(double &x, double &y, double &yaw)
  {
    geometry_msgs::msg::TransformStamped tf;
    try
    {
      tf = tf_buffer_->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Could not get transform " << map_frame << " -> " << base_frame << ": " << ex.what());
      return false;
    }
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    yaw = yawFromQuaternion(tf.transform.rotation);
    return true;
  }

  // Stanley steering law, Thrun et al. 2006, Eq. 7
  double calcStanley(double heading_error, double cross_track_error, double speed) const
  {
    double u = std::max(0.0, speed); // forward driving only
    double delta = heading_err_gain * heading_error + std::atan2(cross_track_err_gain * cross_track_error, k_soft + u);
    return std::clamp(delta, -max_steer, max_steer);
  }

  void timerLoop()
  {
    if (path_.poses.size() < 2)
    {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No path received on " << waypoint_topic);
      return;
    }
    double veh_x, veh_y, veh_yaw;
    if (!getVehiclePose(veh_x, veh_y, veh_yaw))
      return;
    // reference point: center of the front axle (base_link is the center of the rear axle)
    double front_x = veh_x + wheelbase * std::cos(veh_yaw);
    double front_y = veh_y + wheelbase * std::sin(veh_yaw);

    Projection nearest;
    if (!findNearestSegment(front_x, front_y, nearest))
      return;
    // psi(t): orientation of the nearest path segment relative to the vehicle
    double heading_error = normalizeAngle(nearest.seg_yaw - veh_yaw);
    // x(t): positive if the path is on the left of the front axle -> steer left (positive)
    double cross_track_error = -nearest.signed_dist;
    double steering = calcStanley(heading_error, cross_track_error, cmd_.linear.x);
    cmd_.angular.z = steering;
    cmd_pub_->publish(cmd_);

    std_msgs::msg::Float32 deb;
    deb.data = cross_track_error;
    deb_c_pub_->publish(deb);
    deb.data = heading_error;
    deb_h_pub_->publish(deb);
    deb.data = steering;
    deb_s_pub_->publish(deb);
    RCLCPP_DEBUG_STREAM(this->get_logger(), std::fixed << std::setprecision(3) << "x(t): " << cross_track_error << " m, psi(t): " << heading_error << " rad, delta: " << steering << " rad");
  }

  void speedCallback(const std_msgs::msg::Float32 &msg)
  {
    cmd_.linear.x = msg.data;
  }

  void waypointCallback(const geometry_msgs::msg::PoseArray &msg)
  {
    if (msg.poses.size() != path_.poses.size())
      path_initialized = false; // new path, search the whole path
    path_ = msg;
    // closed loop trajectory, if the first and the last waypoints are close (same as in waypoint_to_target)
    if (path_.poses.size() > 2)
    {
      const auto &f = path_.poses.front().position;
      const auto &l = path_.poses.back().position;
      closed_loop_ = std::hypot(f.x - l.x, f.y - l.y) < 4.0;
    }
    if (nearest_idx_ >= path_.poses.size())
      nearest_idx_ = 0;
  }

  void reinitControl()
  {
    std_msgs::msg::Bool reinit;
    reinit.data = true;
    reinit_pub_->publish(reinit);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_w_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_s_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reinit_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr deb_c_pub_, deb_h_pub_, deb_s_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseArray path_;
  geometry_msgs::msg::Twist cmd_;
  size_t nearest_idx_ = 0;
  bool path_initialized = false;
  bool closed_loop_ = false;
  const long search_back = 10, search_forward = 50;
  const double reinit_distance = 5.0; // [m] full path search if the local search is farther

  // parameters
  std::string waypoint_topic = "waypointarray";
  std::string cmd_topic = "cmd_vel";
  std::string map_frame = "map";
  std::string base_frame = "base_link";
  double wheelbase = 2.789;           // [m] rear axle (base_link) -> front axle
  double cross_track_err_gain = 1.0;  // k in Eq. 7 [1/s]
  double heading_err_gain = 1.0;      // 1.0 in the paper
  double k_soft = 1.0;                // [m/s] softening constant, 0.0 = pure Eq. 7
  double max_steer = 0.6;             // [rad] steering saturation
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StanleyControl>());
  rclcpp::shutdown();
  return 0;
}
