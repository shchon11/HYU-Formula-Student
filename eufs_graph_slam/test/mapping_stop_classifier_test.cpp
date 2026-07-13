// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "eufs_graph_slam/slam_lifecycle_classifiers.hpp"

#include <gtest/gtest.h>

namespace eufs_graph_slam
{
namespace
{

TEST(MappingStopClassifierTest, ClassifiesExpectedLocalizationOnlyAfterConfirmedLoop)
{
  // Given: a mapping stop with all localization completion evidence present.
  MappingStopClassifierInput input;
  input.localization_mode = true;
  input.map_converged = true;
  input.map_saved = true;
  input.lap_return_criteria_satisfied = true;
  input.loop_confirmed = true;
  input.stable_map_geometry = true;

  // When: the stop is classified.
  const auto reason = classifyMappingStop(input);

  // Then: it is an expected localization handoff, not a false early convergence.
  EXPECT_EQ(reason, MappingStopReason::ExpectedLocalization);
  EXPECT_STREQ(toString(reason), "expected_localization");
}

TEST(MappingStopClassifierTest, NamesOptimizerPoseLimitSkipBeforeConvergence)
{
  // Given: optimization is skipped by pose limit while SLAM inputs continue.
  MappingStopClassifierInput input;
  input.optimizer_skipped_pose_limit = true;
  input.odometry_fresh = true;
  input.cones_fresh = true;

  // When: the stop is classified.
  const auto reason = classifyMappingStop(input);

  // Then: the reason points at optimizer window policy, not convergence.
  EXPECT_EQ(reason, MappingStopReason::OptimizerPoseLimitSkip);
  EXPECT_STREQ(toString(reason), "optimizer_pose_limit_skip");
}

TEST(MappingStopClassifierTest, ClassifiesUnconfirmedLocalizationAsFalseEarlyConvergence)
{
  // Given: localization starts without confirmed loop evidence.
  MappingStopClassifierInput input;
  input.localization_mode = true;
  input.map_converged = true;
  input.map_saved = true;
  input.lap_return_criteria_satisfied = true;
  input.loop_confirmed = false;
  input.stable_map_geometry = true;

  // When: the stop is classified.
  const auto reason = classifyMappingStop(input);

  // Then: the missing loop confirmation is surfaced as the exact failure branch.
  EXPECT_EQ(reason, MappingStopReason::FalseEarlyConvergence);
  EXPECT_STREQ(toString(reason), "false_early_convergence");
}

TEST(MappingStopClassifierTest, ClassifiesUnconfirmedLoopCandidateAsPending)
{
  // Given: a lap-return candidate exists, but the loop confirmation threshold is not met.
  MappingStopClassifierInput input;
  input.loop_pending = true;

  // When: the stop is classified.
  const auto reason = classifyMappingStop(input);

  // Then: telemetry says the map is still waiting on loop confirmation.
  EXPECT_EQ(reason, MappingStopReason::LoopPending);
  EXPECT_STREQ(toString(reason), "loop_pending");
}

TEST(MappingStopClassifierTest, ClassifiesAmbiguousLoopAssociation)
{
  // Given: a loop candidate was rejected because two landmarks explained it.
  MappingStopClassifierInput input;
  input.ambiguous_association = true;

  // When: the stop is classified.
  const auto reason = classifyMappingStop(input);

  // Then: QA can distinguish ambiguity from expected localization.
  EXPECT_EQ(reason, MappingStopReason::AmbiguousAssociation);
  EXPECT_STREQ(toString(reason), "ambiguous_association");
}

}  // namespace
}  // namespace eufs_graph_slam
