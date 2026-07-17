#include "hyu_local_planner/ros_outputs.hpp"

#include <cmath>
#include <functional>
#include <limits>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace hyu_local_planner
{
namespace
{

double steadyAge(const SteadyTime receive_time)
{
  if (receive_time == SteadyTime{}) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - receive_time).count();
}

Point2 egoToMap(const Point2 & point, const OdomMetadata & odom)
{
  const double cosine = std::cos(odom.yaw);
  const double sine = std::sin(odom.yaw);
  return {
    odom.position.x + cosine * point.x - sine * point.y,
    odom.position.y + sine * point.x + cosine * point.y,
  };
}

double normalizeAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

}

LocalPlannerOutput::LocalPlannerOutput(
  rclcpp::Node & node, const LocalPlannerOutputTopics topics,
  const double max_input_age_sec, const double heartbeat_hz)
: max_input_age_sec_(max_input_age_sec)
{
  const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  waypoints_publisher_ = node.create_publisher<hyu_msgs::msg::WaypointArrayStamped>(
    topics.waypoints, output_qos);
  path_publisher_ = node.create_publisher<nav_msgs::msg::Path>(topics.path, output_qos);
  validity_publisher_ = node.create_publisher<std_msgs::msg::Bool>(topics.validity, output_qos);
  reason_publisher_ = node.create_publisher<std_msgs::msg::String>(topics.reason, output_qos);

  const auto heartbeat_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / heartbeat_hz));
  heartbeat_timer_ = node.create_wall_timer(
    heartbeat_period, std::bind(&LocalPlannerOutput::publishHeartbeat, this));
}

void LocalPlannerOutput::publishPath(
  const BuildResult & result, const Odometry & odom,
  const builtin_interfaces::msg::Time & stamp, const SteadyTime receive_time)
{
  const auto metadata = odomMetadata(odom, receive_time);
  hyu_msgs::msg::WaypointArrayStamped waypoint_array;
  waypoint_array.header.frame_id = "map";
  waypoint_array.header.stamp = stamp;
  nav_msgs::msg::Path path;
  path.header = waypoint_array.header;
  waypoint_array.waypoints.reserve(result.waypoints.size());
  path.poses.reserve(result.waypoints.size());

  for (const auto & source : result.waypoints) {
    const Point2 map_point = egoToMap({source.x, source.y}, metadata);
    const double map_heading = normalizeAngle(metadata.yaw + source.psi);
    hyu_msgs::msg::Waypoint waypoint;
    waypoint.position.x = map_point.x;
    waypoint.position.y = map_point.y;
    waypoint.position.z = 0.0;
    waypoint.speed = source.speed;
    waypoint.suggested_steering = 0.0;
    waypoint.x_m = map_point.x;
    waypoint.y_m = map_point.y;
    waypoint.s_m = source.s;
    waypoint.psi_rad = map_heading;
    waypoint.kappa_radpm = source.kappa;
    waypoint.vx_mps = source.speed;
    waypoint.ax_mps2 = 0.0;
    waypoint_array.waypoints.push_back(waypoint);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = waypoint.position;
    pose.pose.orientation.z = std::sin(0.5 * map_heading);
    pose.pose.orientation.w = std::cos(0.5 * map_heading);
    path.poses.push_back(pose);
  }

  waypoints_publisher_->publish(waypoint_array);
  path_publisher_->publish(path);
  std::lock_guard<std::mutex> lock(mutex_);
  current_valid_ = true;
  last_valid_receive_time_ = receive_time;
  last_failure_reason_.clear();
}

void LocalPlannerOutput::retainUntilStale(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_failure_reason_ = reason;
  if (last_valid_receive_time_ == SteadyTime{}) {
    current_valid_ = false;
  }
}

void LocalPlannerOutput::invalidateImmediately(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_failure_reason_ = reason;
  current_valid_ = false;
}

void LocalPlannerOutput::publishHeartbeat()
{
  std_msgs::msg::Bool message;
  std_msgs::msg::String reason_message;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_valid_ && steadyAge(last_valid_receive_time_) > max_input_age_sec_) {
      current_valid_ = false;
      if (last_failure_reason_.empty()) {
        last_failure_reason_ = "valid path went stale";
      }
    }
    message.data = current_valid_;
    reason_message.data = current_valid_ ? "" : last_failure_reason_;
  }
  validity_publisher_->publish(message);
  reason_publisher_->publish(reason_message);
}

}
