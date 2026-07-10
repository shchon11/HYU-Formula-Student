#include "state_machine/global_path_readiness.hpp"

#include <cmath>

namespace state_machine
{

bool globalEntryAllowed(const GlobalEntryConditions & conditions)
{
  return conditions.global_path_ready &&
         conditions.frenet_fresh &&
         std::isfinite(conditions.current_d) &&
         std::isfinite(conditions.max_abs_d) &&
         std::abs(conditions.current_d) <= conditions.max_abs_d &&
         conditions.handoff_dwell_ready;
}

PlanningState transitionForGlobalReadiness(
  PlanningState current_state,
  bool should_enter_stop,
  const GlobalEntryConditions & conditions)
{
  if (current_state == PlanningState::STOP) {
    return PlanningState::STOP;
  }

  if (current_state == PlanningState::GLOBAL) {
    if (should_enter_stop) {
      return PlanningState::STOP;
    }
    return conditions.global_path_ready ? PlanningState::GLOBAL : PlanningState::LOCAL;
  }

  if (current_state == PlanningState::LOCAL) {
    return globalEntryAllowed(conditions) ? PlanningState::GLOBAL : PlanningState::LOCAL;
  }

  return current_state;
}

void ContinuousHandoffGate::observe(
  bool ready, double receive_time_sec, double timeout_sec)
{
  const bool receive_time_valid = std::isfinite(receive_time_sec);
  const bool previous_fresh = has_value_ && value_ && receive_time_valid &&
    receive_time_sec >= last_receive_time_sec_ &&
    receive_time_sec - last_receive_time_sec_ <= timeout_sec;

  has_value_ = receive_time_valid;
  value_ = ready && receive_time_valid;
  last_receive_time_sec_ = receive_time_sec;

  if (!value_) {
    dwell_active_ = false;
    return;
  }

  if (!previous_fresh) {
    dwell_active_ = true;
    dwell_start_time_sec_ = receive_time_sec;
  }
}

void ContinuousHandoffGate::refresh(double current_time_sec, double timeout_sec)
{
  if (!has_value_ || !value_) {
    return;
  }

  const double age = current_time_sec - last_receive_time_sec_;
  if (!std::isfinite(age) || age < 0.0 || age > timeout_sec) {
    reset();
  }
}

bool ContinuousHandoffGate::ready(
  double current_time_sec, double timeout_sec, double dwell_sec) const
{
  const double receive_age = current_time_sec - last_receive_time_sec_;
  const double dwell_age = current_time_sec - dwell_start_time_sec_;
  return has_value_ && value_ && dwell_active_ &&
         std::isfinite(receive_age) && receive_age >= 0.0 && receive_age <= timeout_sec &&
         std::isfinite(dwell_age) && dwell_age + 1.0e-9 >= dwell_sec;
}

void ContinuousHandoffGate::reset()
{
  has_value_ = false;
  value_ = false;
  dwell_active_ = false;
}

bool GlobalPathReadiness::onWaypoints(
  const eufs_msgs::msg::WaypointArrayStamped & msg,
  const rclcpp::Time & current_time)
{
  ++waypoint_message_generation_;

  if (msg.waypoints.empty()) {
    return false;
  }

  has_global_waypoints_ = true;
  global_path_ready_ = true;
  accepted_waypoint_generation_ = waypoint_message_generation_;
  accepted_waypoint_count_ = msg.waypoints.size();
  last_global_waypoints_time_ = current_time;
  global_path_length_ = msg.waypoints.back().s_m;
  return true;
}

void GlobalPathReadiness::onGraphSlamStatus(const std::string & status)
{
  graph_slam_status_ = status;
  has_graph_slam_status_ = true;
}

void GlobalPathReadiness::onValidity(
  bool valid,
  const rclcpp::Time & current_time,
  double timeout_sec)
{
  refreshValidity(current_time, timeout_sec);
  has_global_path_valid_ = true;
  global_path_valid_ = valid;
  last_global_path_valid_time_ = current_time;

  if (!global_path_valid_) {
    invalidate();
  }
}

void GlobalPathReadiness::refreshValidity(
  const rclcpp::Time & current_time,
  double timeout_sec)
{
  if (!has_global_path_valid_ || !global_path_valid_) {
    return;
  }

  if ((current_time - last_global_path_valid_time_).seconds() <= timeout_sec) {
    return;
  }

  global_path_valid_ = false;
  invalidate();
}

void GlobalPathReadiness::onHandoffReady(
  bool ready,
  const rclcpp::Time & current_time,
  double timeout_sec)
{
  has_global_handoff_ready_ = true;
  global_handoff_ready_ = ready;
  last_global_handoff_ready_time_ = current_time;
  handoff_gate_.observe(ready, current_time.seconds(), timeout_sec);
}

void GlobalPathReadiness::refreshHandoff(
  const rclcpp::Time & current_time,
  double timeout_sec)
{
  handoff_gate_.refresh(current_time.seconds(), timeout_sec);
}

bool GlobalPathReadiness::ready(
  const rclcpp::Time & current_time,
  double timeout_sec) const
{
  return has_global_waypoints_ &&
         global_path_ready_ &&
         accepted_waypoint_count_ > 0U &&
         accepted_waypoint_generation_ > invalidation_generation_ &&
         hasFreshValidity(current_time, timeout_sec) &&
         graphSlamLocalized();
}

bool GlobalPathReadiness::finalPathEndReached(
  const rclcpp::Time & current_time,
  double timeout_sec,
  bool has_frenet_odom,
  double current_s,
  double final_path_end_threshold) const
{
  if (!has_frenet_odom || !ready(current_time, timeout_sec)) {
    return false;
  }

  return pathLength() - current_s < final_path_end_threshold;
}

bool GlobalPathReadiness::hasFreshValidity(
  const rclcpp::Time & current_time,
  double timeout_sec) const
{
  return has_global_path_valid_ &&
         global_path_valid_ &&
         (current_time - last_global_path_valid_time_).seconds() <= timeout_sec;
}

bool GlobalPathReadiness::hasFreshHandoff(
  const rclcpp::Time & current_time,
  double timeout_sec) const
{
  return has_global_handoff_ready_ &&
         (current_time - last_global_handoff_ready_time_).seconds() <= timeout_sec;
}

bool GlobalPathReadiness::handoffDwellReady(
  const rclcpp::Time & current_time,
  double timeout_sec,
  double dwell_sec) const
{
  return handoff_gate_.ready(current_time.seconds(), timeout_sec, dwell_sec);
}

bool GlobalPathReadiness::graphSlamLocalized() const
{
  return has_graph_slam_status_ && graph_slam_status_ == "localization";
}

double GlobalPathReadiness::pathLength() const
{
  return global_path_length_;
}

bool GlobalPathReadiness::pathValid() const
{
  return global_path_valid_;
}

bool GlobalPathReadiness::hasHandoff() const
{
  return has_global_handoff_ready_;
}

bool GlobalPathReadiness::handoffReady() const
{
  return global_handoff_ready_;
}

bool GlobalPathReadiness::hasWaypoints() const
{
  return has_global_waypoints_;
}

const rclcpp::Time & GlobalPathReadiness::lastWaypointsTime() const
{
  return last_global_waypoints_time_;
}

const rclcpp::Time & GlobalPathReadiness::lastHandoffTime() const
{
  return last_global_handoff_ready_time_;
}

const std::string & GlobalPathReadiness::graphSlamStatus() const
{
  return graph_slam_status_;
}

std::uint64_t GlobalPathReadiness::acceptedWaypointGeneration() const
{
  return accepted_waypoint_generation_;
}

std::uint64_t GlobalPathReadiness::invalidationGeneration() const
{
  return invalidation_generation_;
}

std::size_t GlobalPathReadiness::acceptedWaypointCount() const
{
  return accepted_waypoint_count_;
}

void GlobalPathReadiness::invalidate()
{
  invalidation_generation_ = waypoint_message_generation_;
  has_global_waypoints_ = false;
  global_path_ready_ = false;
  global_path_length_ = 0.0;
  accepted_waypoint_count_ = 0U;
}

}
