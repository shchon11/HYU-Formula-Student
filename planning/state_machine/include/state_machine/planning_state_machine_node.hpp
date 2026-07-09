#pragma once

#include <cstddef>
#include <string>

#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace state_machine
{

enum class PlanningState
{
  LOCAL,
  GLOBAL,
  STOP
};

enum class PathSource
{
  LOCAL,
  GLOBAL_FULL,
  GLOBAL_FINAL_STOP,
  STOP
};

class PlanningStateMachineNode : public rclcpp::Node
{
public:
  PlanningStateMachineNode();

private:
  void onFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onCones(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);
  void onStopZoneSStart(const std_msgs::msg::Float64::SharedPtr msg);
  void onStopZoneSEnd(const std_msgs::msg::Float64::SharedPtr msg);
  void onStopZoneValid(const std_msgs::msg::Bool::SharedPtr msg);
  void onTimer();
  void updateState();
  void publishOutputs();

  bool shouldEnterGlobal() const;
  bool shouldEnterStop() const;
  bool isFinalPathEndReached() const;
  bool isStoplineDetected() const;
  bool detectStartFinishGate() const;
  bool hasCrossedStartFinishGate() const;
  void updateLapCount();

  bool isFresh(const rclcpp::Time & stamp, double timeout_sec) const;
  bool hasFreshFrenetOdom() const;
  bool hasFreshGlobalWaypoints() const;
  bool hasFreshConeMap() const;
  bool hasFreshStopZone() const;
  bool isGlobalPathReady() const;
  bool isSInStopZone(double s) const;

  PathSource currentPathSource() const;
  std::string stateToString(PlanningState state) const;
  std::string pathSourceToString(PathSource source) const;
  std::string makeDebugString(PathSource source, bool stop_request) const;

  // TODO(haejun): Add typed /global_waypoints callback after the waypoint message is finalized.
  // The callback should update has_global_waypoints_, global_path_ready_,
  // global_path_length_, and last_global_waypoints_time_.

  std::string frenet_odom_topic_;
  std::string global_waypoints_topic_;
  std::string cone_map_topic_;
  std::string stop_zone_s_start_topic_;
  std::string stop_zone_s_end_topic_;
  std::string stop_zone_valid_topic_;

  int target_lap_count_{4};
  int initial_lap_count_{0};
  int final_lap_start_count_{3};
  int lap_count_{0};

  double frenet_odom_timeout_sec_{0.5};
  double global_waypoints_timeout_sec_{2.0};
  double cone_map_timeout_sec_{1.0};
  double stop_zone_timeout_sec_{1.0};
  double final_path_end_threshold_{2.0};
  double stop_zone_s_margin_{0.0};
  double max_abs_d_for_global_{2.0};

  int state_timer_period_ms_{50};
  bool enable_manual_lap_override_{false};

  PlanningState state_{PlanningState::LOCAL};

  double current_s_{0.0};
  double current_d_{0.0};
  double global_path_length_{0.0};
  double stop_zone_s_start_{0.0};
  double stop_zone_s_end_{0.0};
  std::size_t blue_cone_count_{0U};
  std::size_t yellow_cone_count_{0U};
  std::size_t orange_cone_count_{0U};
  std::size_t big_orange_cone_count_{0U};
  std::size_t unknown_cone_count_{0U};

  bool has_frenet_odom_{false};
  bool has_global_waypoints_{false};
  bool has_cone_map_{false};
  bool global_path_ready_{false};
  bool has_stop_zone_s_start_{false};
  bool has_stop_zone_s_end_{false};
  bool has_stop_zone_valid_{false};
  bool stop_zone_valid_{false};

  std::string closest_segment_id_;
  std::string cone_frame_id_;

  rclcpp::Time last_frenet_odom_time_;
  rclcpp::Time last_global_waypoints_time_;
  rclcpp::Time last_cone_map_time_;
  rclcpp::Time last_stop_zone_s_start_time_;
  rclcpp::Time last_stop_zone_s_end_time_;
  rclcpp::Time last_stop_zone_valid_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_odom_sub_;
  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cone_map_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr stop_zone_s_start_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr stop_zone_s_end_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_zone_valid_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_source_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lap_count_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace state_machine
