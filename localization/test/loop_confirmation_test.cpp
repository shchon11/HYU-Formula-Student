// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "hyu_localization/slam_lifecycle_classifiers.hpp"

#include <gtest/gtest.h>

namespace hyu_localization
{
namespace
{

TEST(LoopConfirmationTest, ConfirmsOnlyAfterConsistentCandidateThreshold)
{
  // Given: a confirmation window using the locked Todo 4 threshold defaults.
  LoopConfirmationWindow window;

  // When: three low-residual loop candidates arrive across enough travel/time.
  const auto first = window.observeCandidate({0.0, 0.0, 0.30});
  const auto second = window.observeCandidate({1.0, 0.4, 0.40});
  const auto third = window.observeCandidate({2.1, 1.1, 0.20});

  // Then: the old single re-association behavior stays pending until the third candidate.
  EXPECT_FALSE(first.confirmed);
  EXPECT_EQ(first.reason, LoopConfirmationReason::PendingThreshold);
  EXPECT_EQ(first.candidate_count, 1U);
  EXPECT_FALSE(second.confirmed);
  EXPECT_EQ(second.reason, LoopConfirmationReason::PendingThreshold);
  EXPECT_EQ(second.candidate_count, 2U);
  EXPECT_TRUE(third.confirmed);
  EXPECT_EQ(third.reason, LoopConfirmationReason::Confirmed);
  EXPECT_EQ(third.candidate_count, 3U);
}

TEST(LoopConfirmationTest, ResetsAfterAmbiguityAndRejectsHardResidual)
{
  // Given: a stale single re-association that would have counted as loop closure before Todo 4.
  LoopConfirmationWindow window;
  const auto stale_single = window.observeCandidate({20.0, 5.0, 0.20});

  // When: association ambiguity is rejected, then a hard residual outlier appears.
  const auto ambiguity = window.rejectAmbiguousAssociation();
  const auto after_reset = window.observeCandidate({22.5, 6.2, 0.20});
  const auto hard_residual = window.observeCandidate({23.0, 6.4, 2.01});

  // Then: no convergence is reported, the reset is observable, and the hard bound is exact.
  EXPECT_FALSE(stale_single.confirmed);
  EXPECT_EQ(stale_single.reason, LoopConfirmationReason::PendingThreshold);
  EXPECT_FALSE(ambiguity.confirmed);
  EXPECT_EQ(ambiguity.reason, LoopConfirmationReason::AmbiguousAssociation);
  EXPECT_EQ(ambiguity.candidate_count, 0U);
  EXPECT_FALSE(after_reset.confirmed);
  EXPECT_EQ(after_reset.candidate_count, 1U);
  EXPECT_FALSE(hard_residual.confirmed);
  EXPECT_EQ(hard_residual.reason, LoopConfirmationReason::ResidualHardBound);
  EXPECT_EQ(hard_residual.candidate_count, 0U);
}

}  // namespace
}  // namespace hyu_localization
