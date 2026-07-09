#include "frenet_conversion/global_path_validity_state.hpp"

namespace frenet_conversion
{

GlobalPathValidityState::GlobalPathValidityState(const double timeout_sec)
{
  setTimeout(timeout_sec);
}

void GlobalPathValidityState::setTimeout(const double timeout_sec)
{
  timeout_sec_ = timeout_sec;
}

GlobalPathInvalidation GlobalPathValidityState::invalidate(
  const rclcpp::Time & now,
  const std::string & reason)
{
  ++invalidation_generation_;
  latest_invalidation_time_ = now;
  global_path_valid_ = false;
  has_valid_heartbeat_ = false;
  accepted_waypoint_generation_ = 0U;
  return {true, invalidation_generation_, reason};
}

GlobalPathInvalidation GlobalPathValidityState::invalidateIfStale(const rclcpp::Time & now)
{
  if (!hasStaleHeartbeat(now)) {
    return {};
  }
  return invalidate(now, "global path validity heartbeat is stale");
}

void GlobalPathValidityState::recordValidHeartbeat(const rclcpp::Time & now)
{
  global_path_valid_ = true;
  has_valid_heartbeat_ = true;
  last_valid_heartbeat_time_ = now;
}

bool GlobalPathValidityState::hasFreshValidHeartbeat(const rclcpp::Time & now) const
{
  if (!global_path_valid_ || !has_valid_heartbeat_) {
    return false;
  }
  const auto age = now - last_valid_heartbeat_time_;
  return age.seconds() <= timeout_sec_;
}

bool GlobalPathValidityState::isCurrentGeneration(const std::uint64_t generation) const
{
  return generation == invalidation_generation_;
}

void GlobalPathValidityState::acceptWaypoints()
{
  accepted_waypoint_generation_ = invalidation_generation_;
}

bool GlobalPathValidityState::hasAcceptedWaypointsForCurrentGeneration() const
{
  return accepted_waypoint_generation_ == invalidation_generation_;
}

bool GlobalPathValidityState::hasStaleHeartbeat(const rclcpp::Time & now) const
{
  if (!global_path_valid_ || !has_valid_heartbeat_) {
    return false;
  }
  return (now - last_valid_heartbeat_time_).seconds() > timeout_sec_;
}

}
