// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef EUFS_GRAPH_SLAM__OBSERVATION_TIME_ALIGNMENT_HPP_
#define EUFS_GRAPH_SLAM__OBSERVATION_TIME_ALIGNMENT_HPP_

#include <optional>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace eufs_graph_slam
{
namespace time_alignment
{

struct Pose2d
{
  double x;
  double y;
  double yaw;
};

struct TimedPose2d
{
  std::int64_t stamp_ns;
  Pose2d pose;
};

enum class PushStatus
{
  Inserted,
  ReplacedDuplicate,
  InsertedOutOfOrder,
  IgnoredTooOld,
  ClockRollback
};

enum class LookupStatus
{
  Ready,
  Empty,
  TooOld,
  AwaitingFuture
};

struct PoseLookup
{
  LookupStatus status;
  Pose2d pose;
};

struct PoseHistoryOptions
{
  std::int64_t duration_ns;
  std::size_t max_samples;
  std::int64_t rollback_threshold_ns;
};

class TimedPoseHistory
{
public:
  explicit TimedPoseHistory(const PoseHistoryOptions & options);

  PushStatus push(const TimedPose2d & sample);
  PoseLookup interpolate(std::int64_t stamp_ns) const;
  void clear();

  std::size_t size() const;
  bool empty() const;
  std::int64_t oldestAcceptedStampNs() const;
  std::int64_t latestStampNs() const;

private:
  void prune();

  PoseHistoryOptions options_;
  std::deque<TimedPose2d> samples_;
};

struct ObservationFrameAlignment
{
  Pose2d keyframe_to_observation;
  Pose2d map_to_observation;
};

struct AlignedObservation2d
{
  Eigen::Vector2d keyframe_point;
  Eigen::Matrix2d keyframe_covariance;
  Eigen::Vector2d map_point;
  Eigen::Matrix2d map_covariance;
};

double normalizeAngle(double angle);
Pose2d compose(const Pose2d & lhs, const Pose2d & rhs);
Pose2d inverse(const Pose2d & pose);
Eigen::Vector2d transformPoint(const Pose2d & transform, const Eigen::Vector2d & point);
Eigen::Matrix2d rotateCovariance(double yaw, const Eigen::Matrix2d & covariance);

ObservationFrameAlignment makeObservationFrameAlignment(
  const Pose2d & raw_keyframe,
  const Pose2d & optimized_keyframe,
  const Pose2d & raw_observation);

AlignedObservation2d alignObservation(
  const ObservationFrameAlignment & alignment,
  const Eigen::Vector2d & point,
  const Eigen::Matrix2d & covariance);

std::optional<std::size_t> latestIndexNotAfter(
  const std::vector<std::int64_t> & sorted_stamps_ns,
  std::int64_t stamp_ns);

}  // namespace time_alignment
}  // namespace eufs_graph_slam

#endif  // EUFS_GRAPH_SLAM__OBSERVATION_TIME_ALIGNMENT_HPP_
