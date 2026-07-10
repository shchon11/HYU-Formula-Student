#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <eufs_msgs/msg/waypoint_array_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "local_planner/local_path_builder.hpp"
#include "local_planner/ros_inputs.hpp"

namespace local_planner
{

struct LocalPlannerOutputTopics
{
  std::string waypoints;
  std::string path;
  std::string validity;
};

class LocalPlannerOutput
{
public:
  LocalPlannerOutput(
    rclcpp::Node & node, LocalPlannerOutputTopics topics,
    double max_input_age_sec, double heartbeat_hz);

  void publishPath(
    const BuildResult & result, const Odometry & odom,
    const builtin_interfaces::msg::Time & stamp, SteadyTime receive_time);
  void invalidate();

private:
  void publishHeartbeat();

  const double max_input_age_sec_;
  rclcpp::Publisher<eufs_msgs::msg::WaypointArrayStamped>::SharedPtr waypoints_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr validity_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  std::mutex mutex_;
  SteadyTime last_valid_receive_time_{};
  bool current_valid_{false};
};

}
