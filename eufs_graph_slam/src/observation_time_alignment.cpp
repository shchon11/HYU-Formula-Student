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
  options_.rollback_threshold_ns =
    std::max<std::int64_t>(1, options_.rollback_threshold_ns);
}

PushStatus TimedPoseHistory::push(const TimedPose2d & sample)
{
  if (samples_.empty()) {
    samples_.push_back(sample);
    return PushStatus::Inserted;
  }

  const std::int64_t newest_stamp_ns = samples_.back().stamp_ns;
  if (sample.stamp_ns < newest_stamp_ns &&
    newest_stamp_ns - sample.stamp_ns > options_.rollback_threshold_ns)
  {
    samples_.clear();
    samples_.push_back(sample);
    return PushStatus::ClockRollback;
  }

  const std::int64_t oldest_accepted_stamp_ns = oldestAcceptedStampNs();
  if (sample.stamp_ns < oldest_accepted_stamp_ns) {
    return PushStatus::IgnoredTooOld;
  }

  const auto insertion_point = std::lower_bound(
    samples_.begin(), samples_.end(), sample.stamp_ns,
    [](const TimedPose2d & existing, std::int64_t candidate_stamp_ns) {
      return existing.stamp_ns < candidate_stamp_ns;
    });

  if (insertion_point != samples_.end() && insertion_point->stamp_ns == sample.stamp_ns) {
    insertion_point->pose = sample.pose;
    return PushStatus::ReplacedDuplicate;
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

}  // namespace time_alignment
}  // namespace eufs_graph_slam
