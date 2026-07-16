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

TEST(LapFinishGateTest, DisabledSeamAndZeroDwellFinishImmediately)
{
  // Default config = previous behavior: return + convergence freeze at once.
  LapFinishGate gate;
  EXPECT_EQ(gate.evaluate(false, false, 0U, 100.0), LapFinishState::WaitingReturn);
  EXPECT_EQ(gate.evaluate(true, false, 0U, 101.0), LapFinishState::GatedByConvergence);
  EXPECT_EQ(gate.evaluate(true, true, 0U, 102.0), LapFinishState::Finished);
}

TEST(LapFinishGateTest, SeamRequirementBlocksMiniLoopConvergence)
{
  // The peanut case: converged on a waist mini-loop, zero seam candidates.
  LapFinishGate gate(LapFinishGateConfig{true, 2U, 0.0});
  EXPECT_EQ(gate.evaluate(true, true, 0U, 200.0), LapFinishState::GatedBySeam);
  EXPECT_EQ(gate.evaluate(true, true, 1U, 201.0), LapFinishState::GatedBySeam);
  EXPECT_FALSE(gate.armed());
  EXPECT_EQ(gate.evaluate(true, true, 2U, 202.0), LapFinishState::Finished);
}

TEST(LapFinishGateTest, DwellDelaysFreezeByTraveledDistance)
{
  LapFinishGate gate(LapFinishGateConfig{true, 2U, 5.0});
  EXPECT_EQ(gate.evaluate(true, true, 2U, 300.0), LapFinishState::Dwelling);
  EXPECT_TRUE(gate.armed());
  EXPECT_EQ(gate.evaluate(true, true, 2U, 304.9), LapFinishState::Dwelling);
  EXPECT_EQ(gate.evaluate(true, true, 2U, 305.0), LapFinishState::Finished);
}

TEST(LapFinishGateTest, ArmingLatchesAgainstLeavingTheOriginRadius)
{
  // Once armed, driving out of the return radius (criteria false again) or a
  // seam counter that stops growing must not un-arm the finish.
  LapFinishGate gate(LapFinishGateConfig{true, 2U, 5.0});
  EXPECT_EQ(gate.evaluate(true, true, 2U, 400.0), LapFinishState::Dwelling);
  EXPECT_EQ(gate.evaluate(false, false, 0U, 403.0), LapFinishState::Dwelling);
  EXPECT_EQ(gate.evaluate(false, false, 0U, 405.0), LapFinishState::Finished);
}

TEST(LapFinishGateTest, StaysFinishedAndResetRearms)
{
  LapFinishGate gate;
  EXPECT_EQ(gate.evaluate(true, true, 0U, 500.0), LapFinishState::Finished);
  EXPECT_EQ(gate.evaluate(false, false, 0U, 501.0), LapFinishState::Finished);
  gate.reset();
  EXPECT_FALSE(gate.armed());
  EXPECT_EQ(gate.evaluate(false, false, 0U, 502.0), LapFinishState::WaitingReturn);
}

TEST(LoopConfirmationTest, LapReturnRelaxationLowersOnlyTheCandidateThreshold)
{
  LoopConfirmationWindow window;
  EXPECT_EQ(window.requiredCandidates(), 3U);
  window.setRequiredCandidates(2U);
  EXPECT_EQ(window.requiredCandidates(), 2U);

  // Two consistent candidates now confirm...
  const auto first = window.observeCandidate({0.0, 0.0, 0.30});
  const auto second = window.observeCandidate({2.1, 1.1, 0.20});
  EXPECT_FALSE(first.confirmed);
  EXPECT_TRUE(second.confirmed);

  // ...but the residual hard bound is untouched: a bad candidate still resets.
  window.reset();
  const auto bad = window.observeCandidate({4.0, 2.0, 5.0});
  EXPECT_FALSE(bad.confirmed);
  EXPECT_EQ(bad.reason, LoopConfirmationReason::ResidualHardBound);
  EXPECT_EQ(bad.candidate_count, 0U);

  // The threshold never drops below one candidate.
  window.setRequiredCandidates(0U);
  EXPECT_EQ(window.requiredCandidates(), 1U);
}

}  // namespace
}  // namespace eufs_graph_slam
