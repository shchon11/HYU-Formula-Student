// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <vector>

#include "eufs_graph_slam/observation_time_alignment.hpp"

namespace
{

using eufs_graph_slam::time_alignment::LookupStatus;
using eufs_graph_slam::time_alignment::Pose2d;
using eufs_graph_slam::time_alignment::PoseHistoryOptions;
using eufs_graph_slam::time_alignment::PushStatus;
using eufs_graph_slam::time_alignment::TimedPose2d;
using eufs_graph_slam::time_alignment::TimedPoseHistory;

constexpr std::int64_t kSecondNs = 1000000000LL;
constexpr double kPi = 3.14159265358979323846;

TimedPoseHistory makeHistory(
  double duration_seconds = 3.0,
  std::size_t max_samples = 1024U,
  double rollback_seconds = 1.0)
{
  return TimedPoseHistory(
    PoseHistoryOptions{
      static_cast<std::int64_t>(duration_seconds * static_cast<double>(kSecondNs)),
      max_samples,
      static_cast<std::int64_t>(rollback_seconds * static_cast<double>(kSecondNs))});
}

TEST(TimedPoseHistoryTest, InterpolatesDelayedTenMetersPerSecondObservation)
{
  auto history = makeHistory();
  ASSERT_EQ(
    history.push(TimedPose2d{100 * kSecondNs, Pose2d{0.0, 0.0, 0.0}}),
    PushStatus::Inserted);
  ASSERT_EQ(
    history.push(TimedPose2d{101 * kSecondNs, Pose2d{10.0, 0.0, 0.0}}),
    PushStatus::Inserted);

  const auto lookup = history.interpolate(100 * kSecondNs + kSecondNs / 2);
  ASSERT_EQ(lookup.status, LookupStatus::Ready);
  EXPECT_DOUBLE_EQ(lookup.pose.x, 5.0);

  const auto alignment = eufs_graph_slam::time_alignment::makeObservationFrameAlignment(
    Pose2d{0.0, 0.0, 0.0},
    Pose2d{100.0, 0.0, 0.0},
    lookup.pose);
  const auto aligned = eufs_graph_slam::time_alignment::alignObservation(
    alignment,
    Eigen::Vector2d(2.0, 0.0),
    Eigen::Matrix2d::Identity());

  EXPECT_NEAR(aligned.keyframe_point.x(), 7.0, 1e-12);
  EXPECT_NEAR(aligned.keyframe_point.y(), 0.0, 1e-12);
  EXPECT_NEAR(aligned.map_point.x(), 107.0, 1e-12);
  EXPECT_NEAR(aligned.map_point.y(), 0.0, 1e-12);
}

TEST(TimedPoseHistoryTest, InterpolatesYawAcrossWrapOnShortestPath)
{
  auto history = makeHistory();
  ASSERT_EQ(
    history.push(TimedPose2d{0, Pose2d{0.0, 0.0, 170.0 * kPi / 180.0}}),
    PushStatus::Inserted);
  ASSERT_EQ(
    history.push(TimedPose2d{kSecondNs, Pose2d{0.0, 0.0, -170.0 * kPi / 180.0}}),
    PushStatus::Inserted);

  const auto lookup = history.interpolate(kSecondNs / 2);
  ASSERT_EQ(lookup.status, LookupStatus::Ready);
  EXPECT_NEAR(
    std::abs(eufs_graph_slam::time_alignment::normalizeAngle(lookup.pose.yaw)),
    kPi,
    1e-12);
}

TEST(ObservationAlignmentTest, RotatesAnisotropicCovarianceIntoBothFrames)
{
  const auto alignment = eufs_graph_slam::time_alignment::makeObservationFrameAlignment(
    Pose2d{0.0, 0.0, 0.0},
    Pose2d{10.0, 20.0, kPi / 2.0},
    Pose2d{0.0, 0.0, kPi / 2.0});
  Eigen::Matrix2d covariance;
  covariance << 4.0, 0.5, 0.5, 1.0;

  const auto aligned = eufs_graph_slam::time_alignment::alignObservation(
    alignment, Eigen::Vector2d(1.0, 0.0), covariance);

  Eigen::Matrix2d expected_keyframe;
  expected_keyframe << 1.0, -0.5, -0.5, 4.0;
  EXPECT_TRUE(aligned.keyframe_covariance.isApprox(expected_keyframe, 1e-12));
  EXPECT_TRUE(aligned.map_covariance.isApprox(covariance, 1e-12));
}

TEST(TimedPoseHistoryTest, AcceptsEndpointsAndDistinguishesOutsideStatuses)
{
  auto history = makeHistory();
  ASSERT_EQ(history.push(TimedPose2d{kSecondNs, Pose2d{1.0, 0.0, 0.0}}), PushStatus::Inserted);
  ASSERT_EQ(
    history.push(TimedPose2d{2 * kSecondNs, Pose2d{2.0, 0.0, 0.0}}),
    PushStatus::Inserted);

  EXPECT_EQ(history.interpolate(kSecondNs).status, LookupStatus::Ready);
  EXPECT_EQ(history.interpolate(2 * kSecondNs).status, LookupStatus::Ready);
  EXPECT_EQ(history.interpolate(kSecondNs - 1).status, LookupStatus::TooOld);
  EXPECT_EQ(history.interpolate(2 * kSecondNs + 1).status, LookupStatus::AwaitingFuture);

  ASSERT_EQ(
    history.push(TimedPose2d{3 * kSecondNs, Pose2d{3.0, 0.0, 0.0}}),
    PushStatus::Inserted);
  EXPECT_EQ(history.interpolate(2 * kSecondNs + 1).status, LookupStatus::Ready);
}

TEST(TimedPoseHistoryTest, DurationPruningRetainsBoundaryPredecessor)
{
  auto history = makeHistory(2.0, 1024U, 4.0);
  ASSERT_EQ(history.push(TimedPose2d{0, Pose2d{0.0, 0.0, 0.0}}), PushStatus::Inserted);
  ASSERT_EQ(
    history.push(TimedPose2d{3 * kSecondNs / 2, Pose2d{1.5, 0.0, 0.0}}),
    PushStatus::Inserted);
  ASSERT_EQ(
    history.push(TimedPose2d{3 * kSecondNs, Pose2d{3.0, 0.0, 0.0}}),
    PushStatus::Inserted);

  const auto boundary = history.interpolate(kSecondNs);
  ASSERT_EQ(boundary.status, LookupStatus::Ready);
  EXPECT_NEAR(boundary.pose.x, 1.0, 1e-12);
  EXPECT_EQ(history.interpolate(kSecondNs - 1).status, LookupStatus::TooOld);
}

TEST(TimedPoseHistoryTest, ReplacesDuplicatesAndBoundsSampleCount)
{
  auto history = makeHistory(3.0, 4U, 4.0);
  ASSERT_EQ(history.push(TimedPose2d{0, Pose2d{0.0, 0.0, 0.0}}), PushStatus::Inserted);
  EXPECT_EQ(
    history.push(TimedPose2d{0, Pose2d{5.0, 0.0, 0.0}}),
    PushStatus::ReplacedDuplicate);
  ASSERT_EQ(history.size(), 1U);
  EXPECT_DOUBLE_EQ(history.interpolate(0).pose.x, 5.0);

  for (std::int64_t index = 1; index <= 8; ++index) {
    history.push(TimedPose2d{index, Pose2d{static_cast<double>(index), 0.0, 0.0}});
  }
  EXPECT_LE(history.size(), 4U);
}

TEST(TimedPoseHistoryTest, ClockRollbackStartsNewEpoch)
{
  auto history = makeHistory(3.0, 1024U, 0.1);
  ASSERT_EQ(
    history.push(TimedPose2d{4 * kSecondNs / 5, Pose2d{0.8, 0.0, 0.0}}),
    PushStatus::Inserted);

  EXPECT_EQ(
    history.push(TimedPose2d{0, Pose2d{0.0, 0.0, 0.0}}),
    PushStatus::ClockRollback);
  ASSERT_EQ(history.size(), 1U);
  EXPECT_EQ(history.latestStampNs(), 0);
  EXPECT_EQ(history.interpolate(4 * kSecondNs / 5).status, LookupStatus::AwaitingFuture);
}

TEST(KeyframeSelectionTest, SelectsLatestKeyframeAtOrBeforeObservation)
{
  const std::vector<std::int64_t> stamps{10, 20, 30};
  EXPECT_FALSE(eufs_graph_slam::time_alignment::latestIndexNotAfter(stamps, 9).has_value());
  EXPECT_EQ(eufs_graph_slam::time_alignment::latestIndexNotAfter(stamps, 10).value(), 0U);
  EXPECT_EQ(eufs_graph_slam::time_alignment::latestIndexNotAfter(stamps, 29).value(), 1U);
  EXPECT_EQ(eufs_graph_slam::time_alignment::latestIndexNotAfter(stamps, 30).value(), 2U);
  EXPECT_EQ(eufs_graph_slam::time_alignment::latestIndexNotAfter(stamps, 31).value(), 2U);
}

}  // namespace
