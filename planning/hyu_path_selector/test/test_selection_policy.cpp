#include <gtest/gtest.h>

#include <optional>
#include <string>

#include "hyu_path_selector/selection_policy.hpp"

namespace hyu_path_selector
{
namespace
{

SelectionInputs freshInputs(const std::string & source)
{
  return SelectionInputs{source, 9.8, 10.0, true, true, true, false};
}

TEST(SelectionPolicyTest, LocalSelectsOnlyLocalCandidate)
{
  const auto result = SelectionPolicy().decide(freshInputs("LOCAL"));

  EXPECT_TRUE(result.valid());
  EXPECT_EQ(result.requested_source, RequestedSource::Local);
  EXPECT_EQ(result.selected_candidate, SelectedCandidate::Local);
  EXPECT_EQ(result.failure, SelectionFailure::None);
}

TEST(SelectionPolicyTest, GlobalFullSelectsOnlyGlobalCandidate)
{
  const auto result = SelectionPolicy().decide(freshInputs("GLOBAL_FULL"));

  EXPECT_TRUE(result.valid());
  EXPECT_EQ(result.requested_source, RequestedSource::GlobalFull);
  EXPECT_EQ(result.selected_candidate, SelectedCandidate::Global);
  EXPECT_EQ(result.failure, SelectionFailure::None);
}

TEST(SelectionPolicyTest, GlobalEntryBridgesHandoffWaitOnTheLocalCandidate)
{
  auto inputs = freshInputs("GLOBAL_FULL");
  inputs.global_handoff_ready = false;

  const auto result = SelectionPolicy().decide(inputs);

  // The car was driving the LOCAL path the instant before the flip; keep
  // driving it (flagged degraded) instead of brake-blinking until handoff.
  EXPECT_TRUE(result.valid());
  EXPECT_EQ(result.selected_candidate, SelectedCandidate::Local);
  EXPECT_TRUE(result.degraded_local_fallback);

  // Without a local candidate the wait stays fail-closed.
  inputs.local_candidate_ready = false;
  const auto braked = SelectionPolicy().decide(inputs);
  EXPECT_FALSE(braked.valid());
  EXPECT_EQ(braked.failure, SelectionFailure::HandoffNotReady);
}

TEST(SelectionPolicyTest, ActiveGlobalDoesNotRegateOnTransientHandoffLoss)
{
  auto inputs = freshInputs("GLOBAL_FULL");
  inputs.global_handoff_ready = false;
  inputs.global_entry_handoff_consumed = true;

  const auto result = SelectionPolicy().decide(inputs);

  EXPECT_TRUE(result.valid());
  EXPECT_EQ(result.selected_candidate, SelectedCandidate::Global);
  EXPECT_EQ(result.failure, SelectionFailure::None);
}

TEST(SelectionPolicyTest, GlobalFinalStopSelectsSameGlobalCandidate)
{
  const auto result = SelectionPolicy().decide(freshInputs("GLOBAL_FINAL_STOP"));

  EXPECT_TRUE(result.valid());
  EXPECT_EQ(result.requested_source, RequestedSource::GlobalFinalStop);
  EXPECT_EQ(result.selected_candidate, SelectedCandidate::Global);
  EXPECT_EQ(result.failure, SelectionFailure::None);
}

TEST(SelectionPolicyTest, StopAndUnknownSourcePublishInvalid)
{
  const auto stop = SelectionPolicy().decide(freshInputs("STOP"));
  const auto unknown = SelectionPolicy().decide(freshInputs("GLOBALISH"));

  EXPECT_FALSE(stop.valid());
  EXPECT_EQ(stop.requested_source, RequestedSource::Stop);
  EXPECT_EQ(stop.failure, SelectionFailure::StopRequested);
  EXPECT_FALSE(unknown.valid());
  EXPECT_EQ(unknown.requested_source, RequestedSource::Unknown);
  EXPECT_EQ(unknown.failure, SelectionFailure::UnknownSource);
}

TEST(SelectionPolicyTest, StaleOrMissingPathSourceInvalidates)
{
  auto missing_inputs = freshInputs("LOCAL");
  missing_inputs.requested_source = std::nullopt;
  auto stale_inputs = freshInputs("LOCAL");
  stale_inputs.source_receive_time_sec = 9.49;

  const auto missing = SelectionPolicy().decide(missing_inputs);
  const auto stale = SelectionPolicy().decide(stale_inputs);

  EXPECT_EQ(missing.failure, SelectionFailure::MissingSource);
  EXPECT_EQ(stale.failure, SelectionFailure::StaleSource);
  EXPECT_FALSE(missing.valid());
  EXPECT_FALSE(stale.valid());
}

TEST(SelectionPolicyTest, GlobalFullDegradesToLocalWhenTheWindowIsUnavailable)
{
  // A stale global window used to select nothing: Pure Pursuit hard-braked
  // and, once stopped mid-lap, the window never revalidated (permanent park
  // after a TMPC fault). The LOCAL candidate regenerates from the SLAM map
  // anywhere on track, so GLOBAL_FULL degrades onto it, flagged.
  auto global_inputs = freshInputs("GLOBAL_FULL");
  global_inputs.global_candidate_ready = false;

  const auto degraded = SelectionPolicy().decide(global_inputs);
  EXPECT_TRUE(degraded.valid());
  EXPECT_EQ(degraded.selected_candidate, SelectedCandidate::Local);
  EXPECT_TRUE(degraded.degraded_local_fallback);

  // With both candidates gone the brake is the only safe output.
  global_inputs.local_candidate_ready = false;
  const auto braked = SelectionPolicy().decide(global_inputs);
  EXPECT_EQ(braked.selected_candidate, SelectedCandidate::None);
  EXPECT_EQ(braked.failure, SelectionFailure::GlobalUnavailable);
}

TEST(SelectionPolicyTest, LocalAndFinalStopStayFailClosed)
{
  auto local_inputs = freshInputs("LOCAL");
  local_inputs.local_candidate_ready = false;
  // GLOBAL_FINAL_STOP never degrades onto the local candidate: stopping is
  // the mission there, so a missing window brakes.
  auto final_stop_inputs = freshInputs("GLOBAL_FINAL_STOP");
  final_stop_inputs.global_candidate_ready = false;
  auto discontinuous_inputs = freshInputs("GLOBAL_FINAL_STOP");
  discontinuous_inputs.global_handoff_ready = false;
  discontinuous_inputs.global_entry_handoff_consumed = true;

  const auto local = SelectionPolicy().decide(local_inputs);
  const auto final_stop = SelectionPolicy().decide(final_stop_inputs);
  const auto discontinuous = SelectionPolicy().decide(discontinuous_inputs);

  EXPECT_EQ(local.selected_candidate, SelectedCandidate::None);
  EXPECT_EQ(local.failure, SelectionFailure::LocalUnavailable);
  EXPECT_EQ(final_stop.selected_candidate, SelectedCandidate::None);
  EXPECT_EQ(final_stop.failure, SelectionFailure::GlobalUnavailable);
  EXPECT_TRUE(discontinuous.valid());
  EXPECT_EQ(discontinuous.selected_candidate, SelectedCandidate::Global);
  EXPECT_EQ(discontinuous.failure, SelectionFailure::None);
}

}
}
