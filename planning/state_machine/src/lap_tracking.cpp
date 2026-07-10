#include "state_machine/planning_state_machine_node.hpp"

#include <algorithm>
#include <cmath>

namespace state_machine
{

namespace
{

void resolveWaypointXY(
  const eufs_msgs::msg::Waypoint & waypoint, double & x, double & y)
{
  x = waypoint.x_m;
  y = waypoint.y_m;
  if (x == 0.0 && y == 0.0 &&
    (waypoint.position.x != 0.0 || waypoint.position.y != 0.0))
  {
    x = waypoint.position.x;
    y = waypoint.position.y;
  }
}

}

LapTrackingPolicy::LapTrackingPolicy(
  double closure_tolerance_m, double closing_duplicate_tolerance_m)
: closure_tolerance_m_(closure_tolerance_m),
  closing_duplicate_tolerance_m_(closing_duplicate_tolerance_m)
{
}

bool LapTrackingPolicy::observeGraphSlamStatus(const std::string & status)
{
  const bool transitioned_from_mapping =
    previous_graph_slam_status_ == "mapping" ||
    previous_graph_slam_status_ == "mapping_converged";
  const bool count_discovery_lap =
    !discovery_lap_counted_ && transitioned_from_mapping && status == "localization";

  previous_graph_slam_status_ = status;
  if (count_discovery_lap) {
    discovery_lap_counted_ = true;
  }
  return count_discovery_lap;
}

bool LapTrackingPolicy::acceptPath(const eufs_msgs::msg::WaypointArrayStamped & msg)
{
  const auto reject = [this]() {
      path_valid_ = false;
      path_length_ = 0.0;
      disarm();
      has_previous_sample_ = false;
      return false;
    };

  if (msg.header.frame_id != "map" || msg.waypoints.size() < 3U) {
    return reject();
  }

  double previous_s = 0.0;
  double first_x = 0.0;
  double first_y = 0.0;
  double last_x = 0.0;
  double last_y = 0.0;
  for (std::size_t index = 0; index < msg.waypoints.size(); ++index) {
    const auto & waypoint = msg.waypoints[index];
    double x = 0.0;
    double y = 0.0;
    resolveWaypointXY(waypoint, x, y);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(waypoint.s_m)) {
      return reject();
    }
    if (index > 0U && waypoint.s_m <= previous_s) {
      return reject();
    }
    if (index == 0U) {
      first_x = x;
      first_y = y;
    }
    last_x = x;
    last_y = y;
    previous_s = waypoint.s_m;
  }

  const double closure = std::hypot(last_x - first_x, last_y - first_y);
  if (!std::isfinite(closure) || closure > closure_tolerance_m_) {
    return reject();
  }

  const double last_s = msg.waypoints.back().s_m;
  const double normalized_length = closure <= closing_duplicate_tolerance_m_ ?
    last_s : last_s + closure;
  if (!std::isfinite(normalized_length) || normalized_length < 20.0) {
    return reject();
  }

  path_valid_ = true;
  path_length_ = normalized_length;
  ++accepted_path_generation_;
  disarm();
  has_previous_sample_ = false;
  return true;
}

bool LapTrackingPolicy::observeFrenetSample(
  double s, double receive_time_sec, double freshness_timeout_sec, double cooldown_sec)
{
  if (!path_valid_ || !std::isfinite(s) || !std::isfinite(receive_time_sec) ||
    s < 0.0 || s > path_length_)
  {
    disarm();
    has_previous_sample_ = false;
    return false;
  }

  if (has_previous_sample_) {
    const double sample_age = receive_time_sec - previous_sample_time_sec_;
    if (!std::isfinite(sample_age) || sample_age < 0.0 ||
      sample_age > freshness_timeout_sec)
    {
      disarm();
      has_previous_sample_ = false;
    }
  }

  if (!has_previous_sample_) {
    previous_s_ = s;
    previous_sample_time_sec_ = receive_time_sec;
    has_previous_sample_ = true;
    return false;
  }

  const double zone = std::min(5.0, 0.1 * path_length_);
  const bool previous_in_middle = previous_s_ > zone && previous_s_ < path_length_ - zone;
  const bool current_in_middle = s > zone && s < path_length_ - zone;
  const bool forward_progress = s > previous_s_;
  const bool seam_wrap = previous_s_ >= path_length_ - zone && s <= zone;
  bool counted = false;

  if (s < previous_s_) {
    if (seam_wrap && armed_) {
      const bool cooldown_complete = !has_count_time_ ||
        receive_time_sec - last_count_time_sec_ >= cooldown_sec;
      if (cooldown_complete) {
        counted = true;
        has_count_time_ = true;
        last_count_time_sec_ = receive_time_sec;
      }
    }
    disarm();
  } else if (forward_progress && previous_in_middle && current_in_middle) {
    armed_ = true;
  }

  previous_s_ = s;
  previous_sample_time_sec_ = receive_time_sec;
  return counted;
}

void LapTrackingPolicy::refresh(double current_time_sec, double freshness_timeout_sec)
{
  if (!has_previous_sample_) {
    return;
  }

  const double sample_age = current_time_sec - previous_sample_time_sec_;
  if (!std::isfinite(sample_age) || sample_age < 0.0 || sample_age > freshness_timeout_sec) {
    disarm();
    has_previous_sample_ = false;
  }
}

bool LapTrackingPolicy::pathValid() const
{
  return path_valid_;
}

bool LapTrackingPolicy::armed() const
{
  return armed_;
}

double LapTrackingPolicy::pathLength() const
{
  return path_length_;
}

std::uint64_t LapTrackingPolicy::acceptedPathGeneration() const
{
  return accepted_path_generation_;
}

void LapTrackingPolicy::disarm()
{
  armed_ = false;
}

bool PlanningStateMachineNode::detectStartFinishGate() const
{
  // TODO(haejun): Implement start/finish gate detection using local /cones.
  return false;
}

bool PlanningStateMachineNode::hasCrossedStartFinishGate() const
{
  // TODO(haejun): Implement crossing logic using Frenet s or gate pose.
  return false;
}

void PlanningStateMachineNode::updateLapCount()
{
  if (lap_tracking_policy_) {
    lap_tracking_policy_->refresh(now().seconds(), frenet_odom_timeout_sec_);
  }
}

}
