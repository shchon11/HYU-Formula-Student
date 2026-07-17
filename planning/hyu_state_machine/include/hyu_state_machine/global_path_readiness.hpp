#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "hyu_msgs/msg/waypoint_array_stamped.hpp"
#include "rclcpp/time.hpp"

namespace hyu_state_machine
{

enum class PlanningState
{
  LOCAL,
  GLOBAL,
  STOP
};

struct GlobalEntryConditions
{
  bool global_path_ready{false};
  bool frenet_fresh{false};
  double current_d{0.0};
  double max_abs_d{2.0};
  bool handoff_dwell_ready{false};
};

bool globalEntryAllowed(const GlobalEntryConditions & conditions);
PlanningState transitionForGlobalReadiness(
  PlanningState current_state,
  bool should_enter_stop,
  const GlobalEntryConditions & conditions);

class ContinuousHandoffGate
{
public:
  void observe(bool ready, double receive_time_sec, double timeout_sec);
  void refresh(double current_time_sec, double timeout_sec);
  bool ready(double current_time_sec, double timeout_sec, double dwell_sec) const;
  void reset();

private:
  bool has_value_{false};
  bool value_{false};
  bool dwell_active_{false};
  double last_receive_time_sec_{0.0};
  double dwell_start_time_sec_{0.0};
};

class GlobalPathReadiness
{
public:
  bool onWaypoints(
    const hyu_msgs::msg::WaypointArrayStamped & msg,
    const rclcpp::Time & current_time);
  void onGraphSlamStatus(const std::string & status);
  void onValidity(bool valid, const rclcpp::Time & current_time, double timeout_sec);
  void refreshValidity(const rclcpp::Time & current_time, double timeout_sec);
  void onHandoffReady(bool ready, const rclcpp::Time & current_time, double timeout_sec);
  void refreshHandoff(const rclcpp::Time & current_time, double timeout_sec);

  bool ready(const rclcpp::Time & current_time, double timeout_sec) const;
  bool ready(
    const rclcpp::Time & current_time,
    double timeout_sec,
    bool require_graph_slam_localization) const;
  bool finalPathEndReached(
    const rclcpp::Time & current_time,
    double timeout_sec,
    bool require_graph_slam_localization,
    bool has_frenet_odom,
    double current_s,
    double final_path_end_threshold) const;
  std::string readinessReason(
    const rclcpp::Time & current_time,
    double timeout_sec,
    bool require_graph_slam_localization) const;
  bool hasFreshValidity(const rclcpp::Time & current_time, double timeout_sec) const;
  bool hasFreshHandoff(const rclcpp::Time & current_time, double timeout_sec) const;
  bool handoffDwellReady(
    const rclcpp::Time & current_time,
    double timeout_sec,
    double dwell_sec) const;
  bool graphSlamLocalized() const;

  double pathLength() const;
  bool pathValid() const;
  bool hasHandoff() const;
  bool handoffReady() const;
  bool hasWaypoints() const;
  const rclcpp::Time & lastWaypointsTime() const;
  const rclcpp::Time & lastHandoffTime() const;
  const std::string & graphSlamStatus() const;
  std::uint64_t acceptedWaypointGeneration() const;
  std::uint64_t invalidationGeneration() const;
  std::size_t acceptedWaypointCount() const;

private:
  void invalidate(const std::string & reason);

  bool has_global_waypoints_{false};
  bool has_graph_slam_status_{false};
  bool has_global_path_valid_{false};
  bool has_global_handoff_ready_{false};
  bool global_path_ready_{false};
  bool global_path_valid_{false};
  bool global_handoff_ready_{false};

  std::string graph_slam_status_{"unknown"};
  std::string last_readiness_loss_reason_;
  rclcpp::Time last_global_waypoints_time_;
  rclcpp::Time last_global_path_valid_time_;
  rclcpp::Time last_global_handoff_ready_time_;
  std::uint64_t waypoint_message_generation_{0U};
  std::uint64_t accepted_waypoint_generation_{0U};
  std::uint64_t invalidation_generation_{0U};
  std::size_t accepted_waypoint_count_{0U};
  double global_path_length_{0.0};
  ContinuousHandoffGate handoff_gate_;
};

}
