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
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "global_planner/global_path_validity_gate.hpp"
#include "global_planner/path_snapshot.hpp"
#include "global_planner/tmpc_trajectory_builder.hpp"
#include "rclcpp/rclcpp.hpp"

#include "eufs_msgs/msg/waypoint_array_stamped.hpp"
#include "hyu_tmpc_msgs/msg/tum_trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace wpnt_publisher
{

class WpntPublisher : public rclcpp::Node
{
public:
  explicit WpntPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onGlobalWaypoints(const eufs_msgs::msg::WaypointArrayStamped::SharedPtr msg);
  void onGlobalPathValid(const std_msgs::msg::Bool::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onEgoOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onPathSource(const std_msgs::msg::String::SharedPtr msg);
  void onLapCount(const std_msgs::msg::Int32::SharedPtr msg);
  void onValidityWatchdog();
  void onTmpcTimer();

  void crossCheckSegment(const std::string & child_frame_id, std::size_t start, std::size_t n);
  bool inputFresh(const rclcpp::Time & stamp, const rclcpp::Time & current_time) const;

  std::string global_waypoints_topic_;
  std::string global_path_valid_topic_;
  std::string frenet_odom_topic_;
  std::string ego_odom_topic_;
  std::string path_source_topic_;
  std::string lap_count_topic_;
  std::string path_waypoints_topic_;
  std::string path_topic_;
  std::string tmpc_performance_trajectory_topic_;
  std::string tmpc_emergency_trajectory_topic_;
  int waypoint_num_{50};
  bool closed_loop_{true};
  double closing_duplicate_tolerance_{1.0e-3};
  double global_path_valid_timeout_sec_{0.5};
  bool check_segment_index_{true};
  double tmpc_publish_rate_hz_{20.0};
  double tmpc_input_timeout_sec_{0.5};

  std::mutex path_mutex_;
  std::mutex tmpc_cycle_mutex_;
  GlobalPathValidityGate validity_gate_{0.5};
  bool latest_global_snapshot_valid_{false};
  bool has_frenet_odom_{false};
  bool has_ego_odom_{false};
  bool has_path_source_{false};
  bool has_lap_count_{false};
  double latest_frenet_s_m_{0.0};
  double latest_ego_speed_mps_{0.0};
  std::string latest_path_source_;
  std::int32_t latest_lap_count_{0};
  rclcpp::Time latest_frenet_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_ego_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_path_source_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_lap_count_time_{0, 0, RCL_ROS_TIME};
  std::uint32_t trajectory_counter_{0U};
  std::unique_ptr<global_planner::tmpc::TmpcTrajectoryBuilder> tmpc_builder_;

  rclcpp::Subscription<eufs_msgs::msg::WaypointArrayStamped>::SharedPtr global_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_path_valid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_source_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lap_count_sub_;
  rclcpp::Publisher<eufs_msgs::msg::WaypointArrayStamped>::SharedPtr path_waypoints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<hyu_tmpc_msgs::msg::TumTrajectory>::SharedPtr
  tmpc_performance_pub_;
  rclcpp::Publisher<hyu_tmpc_msgs::msg::TumTrajectory>::SharedPtr
  tmpc_emergency_pub_;
  rclcpp::TimerBase::SharedPtr validity_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr tmpc_timer_;
};

}  // namespace wpnt_publisher
