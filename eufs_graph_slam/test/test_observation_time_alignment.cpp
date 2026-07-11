// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <optional>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "eufs_graph_slam/observation_time_alignment.hpp"
#include "eufs_graph_slam/optimizer_policy.hpp"

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
  std::size_t max_samples = 1024U)
{
  return TimedPoseHistory(
    PoseHistoryOptions{
      static_cast<std::int64_t>(duration_seconds * static_cast<double>(kSecondNs)),
      max_samples});
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
  auto history = makeHistory(2.0, 1024U);
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

TEST(TimedPoseHistoryTest, IgnoresDuplicatesAndBoundsSampleCount)
{
  auto history = makeHistory(3.0, 4U);
  ASSERT_EQ(history.push(TimedPose2d{0, Pose2d{0.0, 0.0, 0.0}}), PushStatus::Inserted);
  EXPECT_EQ(
    history.push(TimedPose2d{0, Pose2d{5.0, 0.0, 0.0}}),
    PushStatus::IgnoredDuplicate);
  ASSERT_EQ(history.size(), 1U);
  EXPECT_DOUBLE_EQ(history.interpolate(0).pose.x, 0.0);

  for (std::int64_t index = 1; index <= 8; ++index) {
    history.push(TimedPose2d{index, Pose2d{static_cast<double>(index), 0.0, 0.0}});
  }
  EXPECT_LE(history.size(), 4U);
}

TEST(TimedPoseHistoryTest, DelayedSampleDoesNotStartNewEpoch)
{
  auto history = makeHistory(3.0, 1024U);
  ASSERT_EQ(
    history.push(TimedPose2d{10 * kSecondNs, Pose2d{10.0, 0.0, 0.0}}),
    PushStatus::Inserted);

  EXPECT_EQ(
    history.push(TimedPose2d{9 * kSecondNs, Pose2d{9.0, 0.0, 0.0}}),
    PushStatus::InsertedOutOfOrder);
  ASSERT_EQ(history.size(), 2U);
  EXPECT_EQ(history.latestStampNs(), 10 * kSecondNs);
  EXPECT_EQ(history.interpolate(9 * kSecondNs).status, LookupStatus::Ready);
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

TEST(ObservationStampGateTest, RejectsReplayAndOutOfOrderStamps)
{
  const std::optional<std::int64_t> latest_stamp{100};
  EXPECT_FALSE(eufs_graph_slam::time_alignment::isStrictlyNewerStamp(100, latest_stamp));
  EXPECT_FALSE(eufs_graph_slam::time_alignment::isStrictlyNewerStamp(99, latest_stamp));
  EXPECT_TRUE(eufs_graph_slam::time_alignment::isStrictlyNewerStamp(101, latest_stamp));
  EXPECT_TRUE(eufs_graph_slam::time_alignment::isStrictlyNewerStamp(1, std::nullopt));
}

TEST(ObservationStampGateTest, BoundsFutureLeadWithoutOverflow)
{
  EXPECT_FALSE(eufs_graph_slam::time_alignment::exceedsFutureLead(150, 100, 50));
  EXPECT_TRUE(eufs_graph_slam::time_alignment::exceedsFutureLead(151, 100, 50));
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::exceedsFutureLead(
      std::numeric_limits<std::int64_t>::max(),
      std::numeric_limits<std::int64_t>::max() - 10,
      20));
}

TEST(ClockEpochGateTest, IgnoresNormalForwardTicksAndSmallBackwardJitter)
{
  constexpr std::int64_t threshold_ns = 100000000;
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::startsNewClockEpoch(10000000, false, threshold_ns));
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::startsNewClockEpoch(-99999999, false, threshold_ns));
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::hasClockRolledBack(1000000000, 900000001, threshold_ns));
}

TEST(ClockEpochGateTest, AccumulatesSmallBackwardStepsAgainstHighWatermark)
{
  constexpr std::int64_t threshold_ns = 100000000;
  std::int64_t high_watermark_ns = 1000000000;
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::hasClockRolledBack(
      high_watermark_ns, 950000000, threshold_ns));
  high_watermark_ns = eufs_graph_slam::time_alignment::advanceClockHighWatermark(
    high_watermark_ns, 950000000);
  EXPECT_EQ(high_watermark_ns, 1000000000);
  EXPECT_TRUE(
    eufs_graph_slam::time_alignment::hasClockRolledBack(
      high_watermark_ns, 900000000, threshold_ns));
}

TEST(ClockEpochGateTest, AcceptsThresholdRollbackAndClockSourceTransitions)
{
  constexpr std::int64_t threshold_ns = 100000000;
  EXPECT_TRUE(
    eufs_graph_slam::time_alignment::startsNewClockEpoch(-threshold_ns, false, threshold_ns));
  EXPECT_TRUE(
    eufs_graph_slam::time_alignment::startsNewClockEpoch(-2 * threshold_ns, false, threshold_ns));
  EXPECT_TRUE(
    eufs_graph_slam::time_alignment::startsNewClockEpoch(0, true, threshold_ns));
  EXPECT_TRUE(
    eufs_graph_slam::time_alignment::startsNewClockEpoch(threshold_ns, true, threshold_ns));
  EXPECT_TRUE(
    eufs_graph_slam::time_alignment::hasClockRolledBack(1000000000, 900000000, threshold_ns));
}

TEST(ClockEpochGateTest, ValidatesCanonicalNonzeroRosStamps)
{
  EXPECT_TRUE(eufs_graph_slam::time_alignment::isCanonicalNonzeroStamp(1, 0U));
  EXPECT_TRUE(eufs_graph_slam::time_alignment::isCanonicalNonzeroStamp(0, 1U));
  EXPECT_FALSE(eufs_graph_slam::time_alignment::isCanonicalNonzeroStamp(0, 0U));
  EXPECT_FALSE(eufs_graph_slam::time_alignment::isCanonicalNonzeroStamp(-1, 0U));
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::isCanonicalNonzeroStamp(1, 1000000000U));
}

TEST(ClockEpochGateTest, AppliesBoundedFutureLeadDuringRollbackReplay)
{
  const std::optional<std::int64_t> epoch_start{980};
  const std::optional<std::int64_t> replay_end{1000};
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::isStampInCurrentEpoch(
      979, epoch_start, 980, replay_end, 5));
  EXPECT_TRUE(
    eufs_graph_slam::time_alignment::isStampInCurrentEpoch(
      980, epoch_start, 980, replay_end, 5));
  EXPECT_TRUE(
    eufs_graph_slam::time_alignment::isStampInCurrentEpoch(
      985, epoch_start, 980, replay_end, 5));
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::isStampInCurrentEpoch(
      986, epoch_start, 980, replay_end, 5));
  EXPECT_TRUE(
    eufs_graph_slam::time_alignment::isStampInCurrentEpoch(
      999, epoch_start, 995, replay_end, 5));
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::isStampInCurrentEpoch(
      1000, epoch_start, 995, replay_end, 5));
  EXPECT_TRUE(
    eufs_graph_slam::time_alignment::isStampInCurrentEpoch(
      1005, epoch_start, 1000, replay_end, 5));
}

TEST(ClockEpochGateTest, ComputesRollbackReplayEndWithoutOverflow)
{
  EXPECT_EQ(
    eufs_graph_slam::time_alignment::replayGuardEndStamp(980, -20, false).value(),
    1000);
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::replayGuardEndStamp(980, -20, true).has_value());
  EXPECT_EQ(
    eufs_graph_slam::time_alignment::replayGuardEndStamp(
      0, std::numeric_limits<std::int64_t>::min(), false).value(),
    std::numeric_limits<std::int64_t>::max());
}

TEST(ClockEpochGateTest, ConvertsDurationsWithoutIntegerOverflow)
{
  EXPECT_EQ(
    eufs_graph_slam::time_alignment::secondsToNanoseconds(0.1).value(),
    100000000);
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::secondsToNanoseconds(-0.1).has_value());
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::secondsToNanoseconds(
      std::numeric_limits<double>::infinity()).has_value());

  const double rounded_int64_max_seconds =
    static_cast<double>(std::numeric_limits<std::int64_t>::max()) * 1e-9;
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::secondsToNanoseconds(
      rounded_int64_max_seconds).has_value());
  EXPECT_TRUE(
    eufs_graph_slam::time_alignment::secondsToNanoseconds(
      std::nextafter(rounded_int64_max_seconds, 0.0)).has_value());
}

TEST(CarStateGateTest, RequiresFinitePositionAndUnitQuaternion)
{
  EXPECT_TRUE(
    eufs_graph_slam::time_alignment::isValidPlanarPose(1.0, 2.0, 0.0, 0.0, 0.0, 1.0));
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::isValidPlanarPose(
      std::numeric_limits<double>::quiet_NaN(), 2.0, 0.0, 0.0, 0.0, 1.0));
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::isValidPlanarPose(1.0, 2.0, 0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(
    eufs_graph_slam::time_alignment::isValidPlanarPose(1.0, 2.0, 0.0, 0.0, 0.0, 2.0));
}

TEST(OptimizerPolicyTest, FixesOriginAndLeavesFullBatchPosesVariable)
{
  EXPECT_TRUE(eufs_graph_slam::optimizer_policy::shouldFixPose(0U, 4U, 0));
  EXPECT_FALSE(eufs_graph_slam::optimizer_policy::shouldFixPose(1U, 4U, 0));
  EXPECT_FALSE(eufs_graph_slam::optimizer_policy::shouldFixPose(3U, 4U, 0));
}

TEST(OptimizerPolicyTest, FixesOnlyPosesOlderThanActiveWindow)
{
  EXPECT_TRUE(eufs_graph_slam::optimizer_policy::shouldFixPose(0U, 6U, 3));
  EXPECT_TRUE(eufs_graph_slam::optimizer_policy::shouldFixPose(1U, 6U, 3));
  EXPECT_TRUE(eufs_graph_slam::optimizer_policy::shouldFixPose(2U, 6U, 3));
  EXPECT_FALSE(eufs_graph_slam::optimizer_policy::shouldFixPose(3U, 6U, 3));
  EXPECT_FALSE(eufs_graph_slam::optimizer_policy::shouldFixPose(5U, 6U, 3));
}

TEST(OptimizerPolicyTest, KeepsAllNonOriginPosesVariableAtOrBelowWindow)
{
  EXPECT_TRUE(eufs_graph_slam::optimizer_policy::shouldFixPose(0U, 3U, 3));
  EXPECT_FALSE(eufs_graph_slam::optimizer_policy::shouldFixPose(1U, 3U, 3));
  EXPECT_FALSE(eufs_graph_slam::optimizer_policy::shouldFixPose(2U, 3U, 3));
}

}  // namespace
