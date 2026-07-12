#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace local_planner
{
namespace test_support
{

using ConeArray = eufs_msgs::msg::ConeArrayWithCovariance;
using Odometry = nav_msgs::msg::Odometry;

struct ShutdownGuard
{
  ~ShutdownGuard() {rclcpp::shutdown();}
};

struct Observations
{
  std::size_t path_count{0U};
  std::vector<bool> validity_values;
};

template<typename Predicate>
bool spinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  Predicate predicate, const std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(5));
    if (predicate()) {
      return true;
    }
  }
  executor.spin_some();
  return predicate();
}

inline ConeArray cones(std::int32_t stamp_sec)
{
  ConeArray message;
  message.header.frame_id = "base_footprint";
  message.header.stamp.sec = stamp_sec;
  for (const double x : {0.0, 2.0, 4.0, 6.0, 8.0}) {
    eufs_msgs::msg::ConeWithCovariance blue;
    blue.point.x = x;
    blue.point.y = 2.0;
    message.blue_cones.push_back(blue);

    eufs_msgs::msg::ConeWithCovariance yellow;
    yellow.point.x = x;
    yellow.point.y = -2.0;
    message.yellow_cones.push_back(yellow);
  }
  return message;
}

inline Odometry odom(std::int32_t stamp_sec)
{
  Odometry message;
  message.header.frame_id = "map";
  message.header.stamp.sec = stamp_sec;
  message.child_frame_id = "base_footprint";
  message.pose.pose.orientation.w = 1.0;
  return message;
}

inline builtin_interfaces::msg::Time currentStamp(rclcpp::Node & node)
{
  const auto now = node.get_clock()->now().nanoseconds();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(now / 1'000'000'000LL);
  stamp.nanosec = static_cast<std::uint32_t>(now % 1'000'000'000LL);
  return stamp;
}

}
}
