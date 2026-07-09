#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "eufs_msgs/msg/waypoint_array_stamped.hpp"
#include "rclcpp/time.hpp"

namespace state_machine
{

class GlobalPathReadiness
{
public:
  bool onWaypoints(
    const eufs_msgs::msg::WaypointArrayStamped & msg,
    const rclcpp::Time & current_time);
  void onGraphSlamStatus(const std::string & status);
  void onValidity(bool valid, const rclcpp::Time & current_time, double timeout_sec);
  void refreshValidity(const rclcpp::Time & current_time, double timeout_sec);

  bool ready(const rclcpp::Time & current_time, double timeout_sec) const;
  bool hasFreshValidity(const rclcpp::Time & current_time, double timeout_sec) const;
  bool graphSlamLocalized() const;

  double pathLength() const;
  bool pathValid() const;
  bool hasWaypoints() const;
  const rclcpp::Time & lastWaypointsTime() const;
  const std::string & graphSlamStatus() const;
  std::uint64_t acceptedWaypointGeneration() const;
  std::uint64_t invalidationGeneration() const;
  std::size_t acceptedWaypointCount() const;

private:
  void invalidate();

  bool has_global_waypoints_{false};
  bool has_graph_slam_status_{false};
  bool has_global_path_valid_{false};
  bool global_path_ready_{false};
  bool global_path_valid_{false};

  std::string graph_slam_status_{"unknown"};
  rclcpp::Time last_global_waypoints_time_;
  rclcpp::Time last_global_path_valid_time_;
  std::uint64_t waypoint_message_generation_{0U};
  std::uint64_t accepted_waypoint_generation_{0U};
  std::uint64_t invalidation_generation_{0U};
  std::size_t accepted_waypoint_count_{0U};
  double global_path_length_{0.0};
};

}
