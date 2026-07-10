#include <gtest/gtest.h>

#include <optional>
#include <string>

#include "path_selector/selection_policy.hpp"

namespace path_selector
{
namespace
{

SelectionInputs freshInputs(const std::string & source)
{
  return SelectionInputs{source, 9.8, 10.0, true, true};
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

TEST(SelectionPolicyTest, GlobalFullSelectsWhenEntryHandoffIsNotReady)
{
  auto inputs = freshInputs("GLOBAL_FULL");

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

TEST(SelectionPolicyTest, NeverFallsBackBehindStateMachine)
{
  auto local_inputs = freshInputs("LOCAL");
  local_inputs.local_candidate_ready = false;
  auto global_inputs = freshInputs("GLOBAL_FULL");
  global_inputs.global_candidate_ready = false;
  auto discontinuous_inputs = freshInputs("GLOBAL_FINAL_STOP");

  const auto local = SelectionPolicy().decide(local_inputs);
  const auto global = SelectionPolicy().decide(global_inputs);
  const auto discontinuous = SelectionPolicy().decide(discontinuous_inputs);

  EXPECT_EQ(local.selected_candidate, SelectedCandidate::None);
  EXPECT_EQ(local.failure, SelectionFailure::LocalUnavailable);
  EXPECT_EQ(global.selected_candidate, SelectedCandidate::None);
  EXPECT_EQ(global.failure, SelectionFailure::GlobalUnavailable);
  EXPECT_TRUE(discontinuous.valid());
  EXPECT_EQ(discontinuous.selected_candidate, SelectedCandidate::Global);
  EXPECT_EQ(discontinuous.failure, SelectionFailure::None);
}

}
}
