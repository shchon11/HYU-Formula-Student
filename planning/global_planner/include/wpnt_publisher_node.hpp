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
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "eufs_msgs/msg/waypoint.hpp"
#include "eufs_msgs/msg/waypoint_array_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"

namespace wpnt_publisher
{

// Immutable snapshot of the global trajectory, swapped atomically on each
// /global_waypoints update. Closing duplicate (last point == first point)
// removed so that the unique points can be indexed with wrap-around.
struct PathSnapshot
{
  std::vector<eufs_msgs::msg::Waypoint> waypoints;  // unique, for verbatim output
  std::vector<double> xs;                            // resolved x, parallel
  std::vector<double> ys;                            // resolved y, parallel
  std::vector<double> s;                             // s_m, parallel, strictly increasing
  double track_length{0.0};
  std::string frame_id;
};

class WpntPublisher : public rclcpp::Node
{
public:
  WpntPublisher();

private:
  void onGlobalWaypoints(const eufs_msgs::msg::WaypointArrayStamped::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  nav_msgs::msg::Path buildPath(
    const PathSnapshot & snap, std::size_t start, std::size_t count,
    const std_msgs::msg::Header & header) const;

  // Diagnostic only: never influences the published window.
  void crossCheckSegment(const std::string & child_frame_id, std::size_t start, std::size_t n);

  std::string global_waypoints_topic_;
  std::string frenet_odom_topic_;
  std::string path_waypoints_topic_;
  std::string path_topic_;
  int waypoint_num_{50};
  bool closed_loop_{true};
  double closing_duplicate_tolerance_{1.0e-3};
  bool check_segment_index_{true};

  std::mutex path_mutex_;
  std::shared_ptr<const PathSnapshot> path_;

  rclcpp::Subscription<eufs_msgs::msg::WaypointArrayStamped>::SharedPtr global_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<eufs_msgs::msg::WaypointArrayStamped>::SharedPtr path_waypoints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace wpnt_publisher
