#include "frenet_conversion/frenet_odom_messages.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace frenet_conversion
{
namespace
{

void clearCovariance(nav_msgs::msg::Odometry & odom)
{
  std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
  std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
}

}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

std::vector<ReferenceWaypoint> toReferenceWaypoints(
  const eufs_msgs::msg::WaypointArrayStamped & msg)
{
  std::vector<ReferenceWaypoint> waypoints;
  waypoints.reserve(msg.waypoints.size());
  for (const auto & waypoint : msg.waypoints) {
    // eufs_msgs/Waypoint carries both the geometric pose (position) and the
    // offline-CSV echo fields (x_m/y_m). The HYU global_planner fills both, so
    // x_m/y_m are the primary source. Fall back to position when the CSV fields
    // are left unset (both exactly zero) so any /global_waypoints producer that
    // only populates position still yields a usable reference path.
    double x = waypoint.x_m;
    double y = waypoint.y_m;
    if (x == 0.0 && y == 0.0 &&
      (waypoint.position.x != 0.0 || waypoint.position.y != 0.0))
    {
      x = waypoint.position.x;
      y = waypoint.position.y;
    }
    waypoints.push_back({x, y, waypoint.s_m});
  }
  return waypoints;
}

ClcsConversionInput toConversionInput(const nav_msgs::msg::Odometry & odom)
{
  ClcsConversionInput input;
  input.x = odom.pose.pose.position.x;
  input.y = odom.pose.pose.position.y;
  input.yaw = yawFromQuaternion(odom.pose.pose.orientation);
  input.linear_x = odom.twist.twist.linear.x;
  input.linear_y = odom.twist.twist.linear.y;
  input.yaw_rate = odom.twist.twist.angular.z;
  return input;
}

nav_msgs::msg::Odometry buildFrenetOdometry(
  const nav_msgs::msg::Odometry & input,
  const ClcsConversionResult & conversion,
  const FrenetOutputOptions & options)
{
  nav_msgs::msg::Odometry output = input;
  output.header.frame_id = options.frame_id;
  output.child_frame_id = std::to_string(conversion.segment_index);
  output.pose.pose.position.x = conversion.s;
  output.pose.pose.position.y = conversion.d;
  output.pose.pose.position.z = 0.0;

  if (!options.compatibility_mode && options.publish_heading_error) {
    output.pose.pose.orientation = quaternionFromYaw(conversion.heading_error);
  }

  if (!options.compatibility_mode && options.publish_frenet_velocity) {
    output.twist.twist.linear.x = conversion.v_s;
    output.twist.twist.linear.y = conversion.v_d;
    output.twist.twist.angular.z = conversion.yaw_rate;
  }

  if (!options.compatibility_mode && options.clear_covariance) {
    clearCovariance(output);
  }
  return output;
}

nav_msgs::msg::Odometry buildNanFrenetOdometry(
  const nav_msgs::msg::Odometry & input,
  const FrenetOutputOptions & options)
{
  nav_msgs::msg::Odometry output = input;
  output.header.frame_id = options.frame_id;
  output.child_frame_id = "-1";
  output.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  output.pose.pose.position.y = std::numeric_limits<double>::quiet_NaN();
  output.pose.pose.position.z = 0.0;
  if (!options.compatibility_mode && options.clear_covariance) {
    clearCovariance(output);
  }
  return output;
}

std_msgs::msg::Float64MultiArray buildFrenetDebugMessage(
  const ClcsConversionResult & conversion,
  const bool projection_valid,
  const bool include_timing)
{
  std_msgs::msg::Float64MultiArray debug;
  debug.data = {
    conversion.s,
    conversion.d,
    conversion.reference_yaw,
    conversion.heading_error,
    conversion.v_s,
    conversion.v_d,
    conversion.track_length,
    projection_valid ? 1.0 : 0.0,
    include_timing ? conversion.conversion_time_us : 0.0,
    conversion.clcs_build_time_ms,
    conversion.waypoint_s_max_error,
    static_cast<double>(conversion.path_version),
    conversion.reconstruction_error,
    static_cast<double>(conversion.segment_index)};
  return debug;
}

}
