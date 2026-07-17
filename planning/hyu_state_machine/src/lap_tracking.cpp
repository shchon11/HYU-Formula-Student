#include "hyu_state_machine/planning_hyu_state_machine_node.hpp"

#include <algorithm>
#include <cmath>

namespace hyu_state_machine
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

OrangeGateLapTracker::OrangeGateLapTracker(
  double cluster_tolerance_m, double min_gate_width_m, double max_gate_width_m,
  double arm_distance_m, double cooldown_sec)
: cluster_tolerance_m_(cluster_tolerance_m),
  min_gate_width_m_(min_gate_width_m),
  max_gate_width_m_(max_gate_width_m),
  arm_distance_m_(arm_distance_m),
  cooldown_sec_(cooldown_sec)
{
}

bool OrangeGateLapTracker::updateGate(
  const std::vector<std::array<double, 2>> & big_orange_xy)
{
  // Single-linkage clustering (the sets are tiny: a handful of gate cones).
  std::vector<std::vector<std::array<double, 2>>> clusters;
  for (const auto & cone : big_orange_xy) {
    if (!std::isfinite(cone[0]) || !std::isfinite(cone[1])) {
      continue;
    }
    std::vector<std::size_t> touching;
    for (std::size_t index = 0; index < clusters.size(); ++index) {
      for (const auto & member : clusters[index]) {
        if (std::hypot(cone[0] - member[0], cone[1] - member[1]) <=
          cluster_tolerance_m_)
        {
          touching.push_back(index);
          break;
        }
      }
    }
    if (touching.empty()) {
      clusters.push_back({cone});
      continue;
    }
    clusters[touching.front()].push_back(cone);
    for (std::size_t merge = touching.size(); merge > 1U; --merge) {
      auto & source = clusters[touching[merge - 1U]];
      auto & target = clusters[touching.front()];
      target.insert(target.end(), source.begin(), source.end());
      clusters.erase(clusters.begin() + static_cast<std::ptrdiff_t>(touching[merge - 1U]));
    }
  }

  std::vector<std::array<double, 3>> centroids;  // x, y, member count
  for (const auto & cluster : clusters) {
    double x = 0.0;
    double y = 0.0;
    for (const auto & member : cluster) {
      x += member[0];
      y += member[1];
    }
    centroids.push_back(
      {x / static_cast<double>(cluster.size()),
        y / static_cast<double>(cluster.size()),
        static_cast<double>(cluster.size())});
  }

  // Gate = the pair of side clusters a track width apart. Prefer pairs where
  // both sides have >=2 cones (the FS start line has two per side), then the
  // separation closest to a nominal 3.5 m track width.
  bool found = false;
  double best_score = 0.0;
  std::array<double, 2> best_a{{0.0, 0.0}};
  std::array<double, 2> best_b{{0.0, 0.0}};
  for (std::size_t i = 0; i < centroids.size(); ++i) {
    for (std::size_t j = i + 1U; j < centroids.size(); ++j) {
      const double separation = std::hypot(
        centroids[i][0] - centroids[j][0], centroids[i][1] - centroids[j][1]);
      if (separation < min_gate_width_m_ || separation > max_gate_width_m_) {
        continue;
      }
      const bool both_pairs = centroids[i][2] >= 2.0 && centroids[j][2] >= 2.0;
      const double score = (both_pairs ? 0.0 : 10.0) + std::abs(separation - 3.5);
      if (!found || score < best_score) {
        found = true;
        best_score = score;
        best_a = {centroids[i][0], centroids[i][1]};
        best_b = {centroids[j][0], centroids[j][1]};
      }
    }
  }

  if (found) {
    gate_ax_ = best_a[0];
    gate_ay_ = best_a[1];
    gate_bx_ = best_b[0];
    gate_by_ = best_b[1];
    gate_cx_ = 0.5 * (best_a[0] + best_b[0]);
    gate_cy_ = 0.5 * (best_a[1] + best_b[1]);
    gate_valid_ = true;
  }
  // A previously valid gate is kept when the new set cannot form one.
  return gate_valid_;
}

bool OrangeGateLapTracker::observeEgo(double x, double y, double time_sec)
{
  if (!gate_valid_ || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(time_sec)) {
    return false;
  }

  if (std::hypot(x - gate_cx_, y - gate_cy_) > arm_distance_m_) {
    armed_ = true;
  }

  if (!has_prev_ || time_sec < prev_time_sec_ || time_sec - prev_time_sec_ > 2.0) {
    prev_x_ = x;
    prev_y_ = y;
    prev_time_sec_ = time_sec;
    has_prev_ = true;
    return false;
  }

  // Proper segment intersection between the ego motion (prev -> current)
  // and the gate segment (A -> B).
  const double gate_dx = gate_bx_ - gate_ax_;
  const double gate_dy = gate_by_ - gate_ay_;
  const double move_dx = x - prev_x_;
  const double move_dy = y - prev_y_;
  const double d1 = gate_dx * (prev_y_ - gate_ay_) - gate_dy * (prev_x_ - gate_ax_);
  const double d2 = gate_dx * (y - gate_ay_) - gate_dy * (x - gate_ax_);
  const double d3 = move_dx * (gate_ay_ - prev_y_) - move_dy * (gate_ax_ - prev_x_);
  const double d4 = move_dx * (gate_by_ - prev_y_) - move_dy * (gate_bx_ - prev_x_);
  const bool crossed = d1 * d2 < 0.0 && d3 * d4 < 0.0;

  bool counted = false;
  if (crossed) {
    // Crossing direction relative to the gate normal.
    const double sign = move_dx * -gate_dy + move_dy * gate_dx;
    if (!has_direction_ && sign != 0.0) {
      has_direction_ = true;
      direction_sign_ = sign;
    }
    const bool same_direction = sign * direction_sign_ > 0.0;
    const bool cooldown_complete = !has_count_time_ ||
      time_sec - last_count_time_sec_ >= cooldown_sec_;
    if (armed_ && same_direction && cooldown_complete) {
      if (timer_started_) {
        last_lap_sec_ = time_sec - lap_start_time_sec_;
        has_lap_time_ = true;
        if (best_lap_sec_ <= 0.0 || last_lap_sec_ < best_lap_sec_) {
          best_lap_sec_ = last_lap_sec_;
        }
      }
      timer_started_ = true;
      lap_start_time_sec_ = time_sec;
      has_count_time_ = true;
      last_count_time_sec_ = time_sec;
      ++counted_laps_;
      armed_ = false;
      counted = true;
    } else if (!armed_ && same_direction && counted_laps_ == 0) {
      // Spawn-area start-line crossing: (re)base the lap timer without
      // counting — the first lap starts here.
      timer_started_ = true;
      lap_start_time_sec_ = time_sec;
    }
  }

  prev_x_ = x;
  prev_y_ = y;
  prev_time_sec_ = time_sec;
  return counted;
}

bool OrangeGateLapTracker::gateValid() const
{
  return gate_valid_;
}

bool OrangeGateLapTracker::armed() const
{
  return armed_;
}

int OrangeGateLapTracker::countedLaps() const
{
  return counted_laps_;
}

bool OrangeGateLapTracker::hasLapTime() const
{
  return has_lap_time_;
}

double OrangeGateLapTracker::lastLapSec() const
{
  return last_lap_sec_;
}

double OrangeGateLapTracker::bestLapSec() const
{
  return best_lap_sec_;
}

double OrangeGateLapTracker::currentLapElapsedSec(double now_sec) const
{
  if (!timer_started_ || !std::isfinite(now_sec)) {
    return -1.0;
  }
  return now_sec - lap_start_time_sec_;
}

bool PlanningStateMachineNode::detectStartFinishGate() const
{
  return gate_tracker_ && gate_tracker_->gateValid();
}

bool PlanningStateMachineNode::hasCrossedStartFinishGate() const
{
  return gate_tracker_ && gate_tracker_->hasLapTime();
}

void PlanningStateMachineNode::updateLapCount()
{
  if (lap_tracking_policy_) {
    lap_tracking_policy_->refresh(now().seconds(), frenet_odom_timeout_sec_);
  }
}

void PlanningStateMachineNode::recomputeLapCount()
{
  // The gate is the accurate primary counter; the fallback is a floor. Taking
  // the max means a stalled gate can never pull the lap count back down, and a
  // stalled fallback can never cap it — the count only ever moves forward.
  const int gate_laps = initial_lap_count_ +
    (gate_tracker_ ? gate_tracker_->countedLaps() : 0);
  lap_count_ = std::max(gate_laps, fallback_lap_count_);
}

}
