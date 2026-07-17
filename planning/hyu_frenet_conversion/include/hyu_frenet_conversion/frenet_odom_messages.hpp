#pragma once

#include <string>
#include <vector>

#include "clcs_frenet_converter.hpp"
#include "hyu_msgs/msg/waypoint_array_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace hyu_frenet_conversion
{

struct FrenetOutputOptions
{
  std::string frame_id;
  bool compatibility_mode{false};
  bool publish_heading_error{true};
  bool publish_frenet_velocity{true};
  bool clear_covariance{true};
};

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);

std::vector<ReferenceWaypoint> toReferenceWaypoints(
  const hyu_msgs::msg::WaypointArrayStamped & msg);

ClcsConversionInput toConversionInput(const nav_msgs::msg::Odometry & odom);

nav_msgs::msg::Odometry buildFrenetOdometry(
  const nav_msgs::msg::Odometry & input,
  const ClcsConversionResult & conversion,
  const FrenetOutputOptions & options);

nav_msgs::msg::Odometry buildNanFrenetOdometry(
  const nav_msgs::msg::Odometry & input,
  const FrenetOutputOptions & options);

std_msgs::msg::Float64MultiArray buildFrenetDebugMessage(
  const ClcsConversionResult & conversion,
  bool projection_valid,
  bool include_timing);

}
