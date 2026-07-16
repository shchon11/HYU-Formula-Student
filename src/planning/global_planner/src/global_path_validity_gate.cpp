#include "global_planner/global_path_validity_gate.hpp"

#include <cmath>

namespace wpnt_publisher
{

GlobalPathValidityGate::GlobalPathValidityGate(double timeout_sec)
: timeout_sec_(timeout_sec)
{
}

void GlobalPathValidityGate::setTimeout(double timeout_sec)
{
  timeout_sec_ = timeout_sec;
}

void GlobalPathValidityGate::onSnapshot(
  std::shared_ptr<const PathSnapshot> snapshot,
  const rclcpp::Time & current_time)
{
  if (has_global_path_valid_ && global_path_valid_ && !validityFresh(current_time)) {
    invalidate();
  }

  if (validityFresh(current_time)) {
    path_ = std::move(snapshot);
    pending_path_.reset();
    pending_path_generation_ = 0U;
  } else {
    pending_path_ = std::move(snapshot);
    pending_path_generation_ = invalidation_generation_;
    path_.reset();
  }
}

ValidityEventResult GlobalPathValidityGate::onValidity(
  bool valid,
  const rclcpp::Time & current_time)
{
  ValidityEventResult result;
  if (!valid) {
    has_global_path_valid_ = true;
    last_global_path_valid_time_ = current_time;
    result.invalidated = invalidate();
    return result;
  }

  if (has_global_path_valid_ && global_path_valid_ && !validityFresh(current_time)) {
    result.invalidated = invalidate();
    result.stale_invalidated = result.invalidated;
  }
  has_global_path_valid_ = true;
  global_path_valid_ = true;
  last_global_path_valid_time_ = current_time;
  result.activated = activatePending();
  return result;
}

bool GlobalPathValidityGate::invalidateIfStale(const rclcpp::Time & current_time)
{
  if (has_global_path_valid_ && global_path_valid_ && !validityFresh(current_time)) {
    return invalidate();
  }
  return false;
}

OdomGateResult GlobalPathValidityGate::readForOdom(const rclcpp::Time & current_time)
{
  if (has_global_path_valid_ && global_path_valid_ && !validityFresh(current_time)) {
    invalidate();
  }

  OdomGateResult result;
  if (validityFresh(current_time)) {
    activatePending();
    result.snapshot = path_;
  }
  result.has_validity = has_global_path_valid_;
  result.validity_true = global_path_valid_;
  return result;
}

bool GlobalPathValidityGate::validityFresh(const rclcpp::Time & current_time) const
{
  if (!has_global_path_valid_ || !global_path_valid_) {
    return false;
  }
  const double age_sec = (current_time - last_global_path_valid_time_).seconds();
  return std::isfinite(age_sec) && age_sec >= 0.0 && age_sec <= timeout_sec_;
}

bool GlobalPathValidityGate::invalidate()
{
  const bool had_cached_path = static_cast<bool>(path_) || static_cast<bool>(pending_path_);
  const bool was_valid = has_global_path_valid_ && global_path_valid_;
  if (!had_cached_path && !was_valid && has_invalidation_) {
    global_path_valid_ = false;
    return false;
  }

  path_.reset();
  pending_path_.reset();
  global_path_valid_ = false;
  has_invalidation_ = true;
  ++invalidation_generation_;
  pending_path_generation_ = 0U;
  return true;
}

bool GlobalPathValidityGate::activatePending()
{
  if (!pending_path_ || pending_path_generation_ != invalidation_generation_) {
    return false;
  }
  path_ = pending_path_;
  pending_path_.reset();
  pending_path_generation_ = 0U;
  return true;
}

}  // namespace wpnt_publisher
