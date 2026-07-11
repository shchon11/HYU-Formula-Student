// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "eufs_graph_slam/observation_time_alignment.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <vector>

namespace eufs_graph_slam
{
namespace time_alignment
{

namespace
{

Pose2d interpolatePose(
  const TimedPose2d & lower,
  const TimedPose2d & upper,
  std::int64_t stamp_ns)
{
  const double interval_ns = static_cast<double>(upper.stamp_ns - lower.stamp_ns);
  const double alpha = static_cast<double>(stamp_ns - lower.stamp_ns) / interval_ns;
  const double yaw_delta = normalizeAngle(upper.pose.yaw - lower.pose.yaw);
  return Pose2d{
    lower.pose.x + alpha * (upper.pose.x - lower.pose.x),
    lower.pose.y + alpha * (upper.pose.y - lower.pose.y),
    normalizeAngle(lower.pose.yaw + alpha * yaw_delta)};
}

}  // namespace

TimedPoseHistory::TimedPoseHistory(const PoseHistoryOptions & options)
: options_(options)
{
  options_.duration_ns = std::max<std::int64_t>(1, options_.duration_ns);
  options_.max_samples = std::max<std::size_t>(2U, options_.max_samples);
}

PushStatus TimedPoseHistory::push(const TimedPose2d & sample)
{
  if (samples_.empty()) {
    samples_.push_back(sample);
    return PushStatus::Inserted;
  }

  const std::int64_t newest_stamp_ns = samples_.back().stamp_ns;
  const std::int64_t retention_cutoff_ns = newest_stamp_ns >
    std::numeric_limits<std::int64_t>::min() + options_.duration_ns ?
    newest_stamp_ns - options_.duration_ns :
    std::numeric_limits<std::int64_t>::min();
  if (sample.stamp_ns < retention_cutoff_ns) {
    return PushStatus::IgnoredTooOld;
  }

  const auto insertion_point = std::lower_bound(
    samples_.begin(), samples_.end(), sample.stamp_ns,
    [](const TimedPose2d & existing, std::int64_t candidate_stamp_ns) {
      return existing.stamp_ns < candidate_stamp_ns;
    });

  if (insertion_point != samples_.end() && insertion_point->stamp_ns == sample.stamp_ns) {
    return PushStatus::IgnoredDuplicate;
  }

  const bool out_of_order = sample.stamp_ns < newest_stamp_ns;
  samples_.insert(insertion_point, sample);
  prune();
  return out_of_order ? PushStatus::InsertedOutOfOrder : PushStatus::Inserted;
}

PoseLookup TimedPoseHistory::interpolate(std::int64_t stamp_ns) const
{
  const Pose2d empty_pose{0.0, 0.0, 0.0};
  if (samples_.empty()) {
    return PoseLookup{LookupStatus::Empty, empty_pose};
  }

  if (stamp_ns < oldestAcceptedStampNs()) {
    return PoseLookup{LookupStatus::TooOld, empty_pose};
  }
  if (stamp_ns > samples_.back().stamp_ns) {
    return PoseLookup{LookupStatus::AwaitingFuture, empty_pose};
  }

  const auto upper = std::lower_bound(
    samples_.begin(), samples_.end(), stamp_ns,
    [](const TimedPose2d & existing, std::int64_t query_stamp_ns) {
      return existing.stamp_ns < query_stamp_ns;
    });

  if (upper != samples_.end() && upper->stamp_ns == stamp_ns) {
    return PoseLookup{LookupStatus::Ready, upper->pose};
  }
  if (upper == samples_.begin() || upper == samples_.end()) {
    return PoseLookup{LookupStatus::TooOld, empty_pose};
  }

  return PoseLookup{
    LookupStatus::Ready,
    interpolatePose(*std::prev(upper), *upper, stamp_ns)};
}

void TimedPoseHistory::clear()
{
  samples_.clear();
}

std::size_t TimedPoseHistory::size() const
{
  return samples_.size();
}

bool TimedPoseHistory::empty() const
{
  return samples_.empty();
}

std::int64_t TimedPoseHistory::oldestAcceptedStampNs() const
{
  if (samples_.empty()) {
    return 0;
  }

  const std::int64_t newest_stamp_ns = samples_.back().stamp_ns;
  const std::int64_t cutoff = newest_stamp_ns >
    std::numeric_limits<std::int64_t>::min() + options_.duration_ns ?
    newest_stamp_ns - options_.duration_ns :
    std::numeric_limits<std::int64_t>::min();
  return std::max(samples_.front().stamp_ns, cutoff);
}

std::int64_t TimedPoseHistory::latestStampNs() const
{
  return samples_.empty() ? 0 : samples_.back().stamp_ns;
}

void TimedPoseHistory::prune()
{
  if (samples_.size() < 2U) {
    return;
  }

  const std::int64_t cutoff = oldestAcceptedStampNs();
  const auto first_at_or_after = std::lower_bound(
    samples_.begin(), samples_.end(), cutoff,
    [](const TimedPose2d & existing, std::int64_t cutoff_stamp_ns) {
      return existing.stamp_ns < cutoff_stamp_ns;
    });
  if (first_at_or_after != samples_.begin()) {
    const auto predecessor = std::prev(first_at_or_after);
    samples_.erase(samples_.begin(), predecessor);
  }

  while (samples_.size() > options_.max_samples) {
    samples_.pop_front();
  }
}

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Pose2d compose(const Pose2d & lhs, const Pose2d & rhs)
{
  const double cosine = std::cos(lhs.yaw);
  const double sine = std::sin(lhs.yaw);
  return Pose2d{
    lhs.x + cosine * rhs.x - sine * rhs.y,
    lhs.y + sine * rhs.x + cosine * rhs.y,
    normalizeAngle(lhs.yaw + rhs.yaw)};
}

Pose2d inverse(const Pose2d & pose)
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return Pose2d{
    -cosine * pose.x - sine * pose.y,
    sine * pose.x - cosine * pose.y,
    normalizeAngle(-pose.yaw)};
}

Eigen::Vector2d transformPoint(const Pose2d & transform, const Eigen::Vector2d & point)
{
  const double cosine = std::cos(transform.yaw);
  const double sine = std::sin(transform.yaw);
  return Eigen::Vector2d(
    transform.x + cosine * point.x() - sine * point.y(),
    transform.y + sine * point.x() + cosine * point.y());
}

Eigen::Matrix2d rotateCovariance(double yaw, const Eigen::Matrix2d & covariance)
{
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  Eigen::Matrix2d rotation;
  rotation << cosine, -sine, sine, cosine;
  Eigen::Matrix2d rotated = rotation * covariance * rotation.transpose();
  return (0.5 * (rotated + rotated.transpose())).eval();
}

ObservationFrameAlignment makeObservationFrameAlignment(
  const Pose2d & raw_keyframe,
  const Pose2d & optimized_keyframe,
  const Pose2d & raw_observation)
{
  const Pose2d keyframe_to_observation = compose(inverse(raw_keyframe), raw_observation);
  return ObservationFrameAlignment{
    keyframe_to_observation,
    compose(optimized_keyframe, keyframe_to_observation)};
}

AlignedObservation2d alignObservation(
  const ObservationFrameAlignment & alignment,
  const Eigen::Vector2d & point,
  const Eigen::Matrix2d & covariance)
{
  return AlignedObservation2d{
    transformPoint(alignment.keyframe_to_observation, point),
    rotateCovariance(alignment.keyframe_to_observation.yaw, covariance),
    transformPoint(alignment.map_to_observation, point),
    rotateCovariance(alignment.map_to_observation.yaw, covariance)};
}

std::optional<std::size_t> latestIndexNotAfter(
  const std::vector<std::int64_t> & sorted_stamps_ns,
  std::int64_t stamp_ns)
{
  const auto after = std::upper_bound(
    sorted_stamps_ns.begin(), sorted_stamps_ns.end(), stamp_ns);
  if (after == sorted_stamps_ns.begin()) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(std::distance(sorted_stamps_ns.begin(), std::prev(after)));
}

bool isStrictlyNewerStamp(
  std::int64_t stamp_ns,
  const std::optional<std::int64_t> & latest_processed_stamp_ns)
{
  return !latest_processed_stamp_ns.has_value() ||
         stamp_ns > *latest_processed_stamp_ns;
}

bool exceedsFutureLead(
  std::int64_t stamp_ns,
  std::int64_t reference_stamp_ns,
  std::int64_t max_future_lead_ns)
{
  const std::int64_t bounded_lead_ns = std::max<std::int64_t>(0, max_future_lead_ns);
  if (reference_stamp_ns >
    std::numeric_limits<std::int64_t>::max() - bounded_lead_ns)
  {
    return false;
  }
  return stamp_ns > reference_stamp_ns + bounded_lead_ns;
}

bool startsNewClockEpoch(
  std::int64_t delta_ns,
  bool clock_source_changed,
  std::int64_t rollback_threshold_ns)
{
  const std::int64_t bounded_threshold_ns =
    std::max<std::int64_t>(1, rollback_threshold_ns);
  return clock_source_changed || delta_ns <= -bounded_threshold_ns;
}

bool hasClockRolledBack(
  std::int64_t previous_time_ns,
  std::int64_t current_time_ns,
  std::int64_t rollback_threshold_ns)
{
  const std::int64_t bounded_threshold_ns =
    std::max<std::int64_t>(1, rollback_threshold_ns);
  return previous_time_ns >= bounded_threshold_ns &&
         current_time_ns <= previous_time_ns - bounded_threshold_ns;
}

std::int64_t advanceClockHighWatermark(
  std::int64_t current_high_watermark_ns,
  std::int64_t observed_time_ns)
{
  return std::max(current_high_watermark_ns, observed_time_ns);
}

std::optional<std::int64_t> secondsToNanoseconds(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return std::nullopt;
  }

  const long double nanoseconds =
    static_cast<long double>(seconds) * 1000000000.0L;
  if (nanoseconds > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(nanoseconds);
}

bool isCanonicalNonzeroStamp(std::int32_t seconds, std::uint32_t nanoseconds)
{
  return seconds >= 0 && nanoseconds < 1000000000U &&
         (seconds != 0 || nanoseconds != 0U);
}

bool isStampInCurrentEpoch(
  std::int64_t stamp_ns,
  const std::optional<std::int64_t> & epoch_start_ns,
  std::int64_t now_ns,
  const std::optional<std::int64_t> & replay_guard_end_ns,
  std::int64_t max_future_lead_ns)
{
  if (epoch_start_ns.has_value() && stamp_ns < *epoch_start_ns) {
    return false;
  }

  const bool replaying_rolled_back_interval =
    replay_guard_end_ns.has_value() && now_ns < *replay_guard_end_ns;
  if (replaying_rolled_back_interval && stamp_ns >= *replay_guard_end_ns) {
    return false;
  }
  return !exceedsFutureLead(stamp_ns, now_ns, max_future_lead_ns);
}

std::optional<std::int64_t> replayGuardEndStamp(
  std::int64_t epoch_start_ns,
  std::int64_t clock_delta_ns,
  bool clock_source_changed)
{
  if (clock_source_changed || clock_delta_ns >= 0) {
    return std::nullopt;
  }

  if (epoch_start_ns > std::numeric_limits<std::int64_t>::max() + clock_delta_ns) {
    return std::numeric_limits<std::int64_t>::max();
  }
  return epoch_start_ns - clock_delta_ns;
}

bool isValidPlanarPose(
  double x,
  double y,
  double qx,
  double qy,
  double qz,
  double qw)
{
  if (!std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(qx) || !std::isfinite(qy) ||
    !std::isfinite(qz) || !std::isfinite(qw))
  {
    return false;
  }

  const double norm_squared = qx * qx + qy * qy + qz * qz + qw * qw;
  return std::isfinite(norm_squared) && std::abs(norm_squared - 1.0) <= 1e-2;
}

}  // namespace time_alignment
}  // namespace eufs_graph_slam
