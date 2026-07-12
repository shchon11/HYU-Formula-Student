// wpnt_publisher_node
//
// Slices the next `waypoint_num` points of the global race trajectory ahead of
// the vehicle and republishes them as /path_waypoints (+ /path_waypoints/path).
//
// The window start is chosen by an s-based binary search over the global
// waypoints' s_m values, NOT by parsing the frenet segment index. The frenet
// child_frame_id is used only as a diagnostic cross-check. See
// planning/docs/wpnt_publisher_proposal_v2.md for the rationale.

#pragma once

#include <cstddef>
#include <mutex>
#include <string>

#include "global_planner/global_path_validity_gate.hpp"
#include "global_planner/path_snapshot.hpp"
#include "rclcpp/rclcpp.hpp"

#include "eufs_msgs/msg/waypoint_array_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

namespace wpnt_publisher
{

class WpntPublisher : public rclcpp::Node
{
public:
  WpntPublisher();

private:
  void onGlobalWaypoints(const eufs_msgs::msg::WaypointArrayStamped::SharedPtr msg);
  void onGlobalPathValid(const std_msgs::msg::Bool::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onValidityWatchdog();

  void crossCheckSegment(const std::string & child_frame_id, std::size_t start, std::size_t n);

  std::string global_waypoints_topic_;
  std::string global_path_valid_topic_;
  std::string frenet_odom_topic_;
  std::string path_waypoints_topic_;
  std::string path_topic_;
  int waypoint_num_{50};
  bool closed_loop_{true};
  double closing_duplicate_tolerance_{1.0e-3};
  double global_path_valid_timeout_sec_{0.5};
  bool check_segment_index_{true};

  std::mutex path_mutex_;
  GlobalPathValidityGate validity_gate_{0.5};

  rclcpp::Subscription<eufs_msgs::msg::WaypointArrayStamped>::SharedPtr global_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_path_valid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<eufs_msgs::msg::WaypointArrayStamped>::SharedPtr path_waypoints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr validity_watchdog_timer_;
};

}  // namespace wpnt_publisher
