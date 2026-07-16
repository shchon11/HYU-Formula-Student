#include <limits>

#include "gtest/gtest.h"
#include "tmpc_cmd_selector/selector_policy.hpp"

namespace
{

using tmpc_cmd_selector::CommandSource;
using tmpc_cmd_selector::SelectorDecision;
using tmpc_cmd_selector::SelectorInputs;
using tmpc_cmd_selector::SelectorPolicy;
using tmpc_cmd_selector::SelectorStatus;

SelectorInputs FreshInputs(const std::string & state, double now_sec = 1.0)
{
  SelectorInputs inputs;
  inputs.planning_state = state;
  inputs.state_age_sec = 0.0;
  inputs.has_stop_request = true;
  inputs.stop_age_sec = 0.0;
  inputs.has_local_command = true;
  inputs.local_command_valid = true;
  inputs.local_command_age_sec = 0.0;
  inputs.has_tmpc_command = true;
  inputs.tmpc_command_valid = true;
  inputs.tmpc_command_age_sec = 0.0;
  inputs.has_tmpc_valid = true;
  inputs.tmpc_valid = true;
  inputs.tmpc_valid_age_sec = 0.0;
  inputs.now_sec = now_sec;
  return inputs;
}

void ExpectDecision(
  const SelectorDecision & decision, CommandSource source, SelectorStatus status)
{
  EXPECT_EQ(decision.source, source);
  EXPECT_EQ(decision.status, status);
}

TEST(SelectorPolicy, UsesPurePursuitInLocal)
{
  SelectorPolicy policy;
  const auto decision = policy.update(FreshInputs("LOCAL"));

  ExpectDecision(
    decision, CommandSource::kPurePursuit, SelectorStatus::kLocalPurePursuit);
}

TEST(SelectorPolicy, WaitsForContinuousTmpcReadinessBeforeTakeover)
{
  SelectorPolicy policy;
  auto inputs = FreshInputs("GLOBAL", 10.0);
  inputs.tmpc_valid = false;
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kGlobalWaitingTmpc);

  inputs.tmpc_valid = true;
  inputs.now_sec = 10.01;
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kGlobalWaitingTmpc);

  inputs.now_sec = 10.09;
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kGlobalWaitingTmpc);

  inputs.now_sec = 10.12;
  ExpectDecision(
    policy.update(inputs), CommandSource::kTmpc, SelectorStatus::kGlobalTmpc);
  EXPECT_TRUE(policy.tmpcActive());
}

TEST(SelectorPolicy, ReadinessDwellRestartsAfterInvalidSample)
{
  SelectorPolicy policy;
  auto inputs = FreshInputs("GLOBAL", 20.0);
  policy.update(inputs);

  inputs.now_sec = 20.06;
  inputs.tmpc_valid = false;
  policy.update(inputs);

  inputs.now_sec = 20.07;
  inputs.tmpc_valid = true;
  policy.update(inputs);

  inputs.now_sec = 20.12;
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kGlobalWaitingTmpc);

  inputs.now_sec = 20.18;
  ExpectDecision(
    policy.update(inputs), CommandSource::kTmpc, SelectorStatus::kGlobalTmpc);
}

TEST(SelectorPolicy, LatchesBrakeAfterTmpcTakeoverFaultUntilLocal)
{
  SelectorPolicy policy;
  auto inputs = FreshInputs("GLOBAL", 30.0);
  policy.update(inputs);
  inputs.now_sec = 30.11;
  policy.update(inputs);
  ASSERT_TRUE(policy.tmpcActive());

  inputs.now_sec = 30.12;
  inputs.tmpc_valid = false;
  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kFaultBrake);
  ASSERT_TRUE(policy.faultLatched());

  inputs.now_sec = 30.13;
  inputs.tmpc_valid = true;
  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kFaultBrake);

  inputs = FreshInputs("STOP", 30.14);
  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kStopBrake);
  EXPECT_TRUE(policy.faultLatched());

  inputs = FreshInputs("LOCAL", 30.15);
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kLocalPurePursuit);
  EXPECT_FALSE(policy.faultLatched());
}

TEST(SelectorPolicy, BrakesForStopUnknownAndStaleSafetyInputs)
{
  SelectorPolicy policy;

  auto inputs = FreshInputs("LOCAL");
  inputs.stop_requested = true;
  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kStopBrake);

  inputs = FreshInputs("UNKNOWN");
  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kInputBrake);

  inputs = FreshInputs("LOCAL");
  inputs.state_age_sec = 0.251;
  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kInputBrake);

  inputs = FreshInputs("LOCAL");
  inputs.stop_age_sec = 0.251;
  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kInputBrake);

  inputs = FreshInputs("LOCAL");
  inputs.local_command_age_sec = 0.251;
  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kInputBrake);
}

TEST(SelectorPolicy, BrakesWhenTmpcTimesOutAfterTakeover)
{
  SelectorPolicy policy;
  auto inputs = FreshInputs("GLOBAL", 40.0);
  policy.update(inputs);
  inputs.now_sec = 40.11;
  policy.update(inputs);

  inputs.now_sec = 40.12;
  inputs.tmpc_command_age_sec = 0.101;
  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kFaultBrake);
}

TEST(SelectorPolicy, RejectsNonFiniteTime)
{
  SelectorPolicy policy;
  auto inputs = FreshInputs("LOCAL");
  inputs.now_sec = std::numeric_limits<double>::quiet_NaN();

  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kInputBrake);
}

TEST(SelectorPolicy, RejectsInvalidCandidateCommands)
{
  SelectorPolicy policy;
  auto inputs = FreshInputs("LOCAL");
  inputs.local_command_valid = false;
  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kInputBrake);

  inputs = FreshInputs("GLOBAL", 50.0);
  inputs.tmpc_command_valid = false;
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kGlobalWaitingTmpc);
}

}  // namespace
