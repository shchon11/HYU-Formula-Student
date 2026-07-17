#include <limits>

#include "gtest/gtest.h"
#include "hyu_cmd_selector/selector_policy.hpp"

namespace
{

using hyu_cmd_selector::CommandSource;
using hyu_cmd_selector::SelectorDecision;
using hyu_cmd_selector::SelectorInputs;
using hyu_cmd_selector::SelectorPolicy;
using hyu_cmd_selector::SelectorStatus;

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

TEST(SelectorPolicy, FallsBackToPurePursuitAfterTakeoverFaultAndRequiresRedwell)
{
  SelectorPolicy policy;
  auto inputs = FreshInputs("GLOBAL", 30.0);
  policy.update(inputs);
  inputs.now_sec = 30.11;
  policy.update(inputs);
  ASSERT_TRUE(policy.tmpcActive());

  // A TMPC dropout mid-drive degrades to Pure Pursuit, not a parked car.
  inputs.now_sec = 30.12;
  inputs.tmpc_valid = false;
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kGlobalPurePursuitFallback);
  EXPECT_FALSE(policy.tmpcActive());
  EXPECT_TRUE(policy.hadFault());

  // Recovered validity does not retake instantly: the dwell restarts.
  inputs.now_sec = 30.13;
  inputs.tmpc_valid = true;
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kGlobalPurePursuitFallback);

  inputs.now_sec = 30.24;
  ExpectDecision(
    policy.update(inputs), CommandSource::kTmpc, SelectorStatus::kGlobalTmpc);
  EXPECT_TRUE(policy.tmpcActive());

  inputs = FreshInputs("LOCAL", 30.30);
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kLocalPurePursuit);
  EXPECT_FALSE(policy.hadFault());
}

TEST(SelectorPolicy, BrakesAfterTakeoverFaultWhenPurePursuitUnavailable)
{
  SelectorPolicy policy;
  auto inputs = FreshInputs("GLOBAL", 31.0);
  policy.update(inputs);
  inputs.now_sec = 31.11;
  policy.update(inputs);
  ASSERT_TRUE(policy.tmpcActive());

  inputs.now_sec = 31.12;
  inputs.tmpc_valid = false;
  inputs.local_command_age_sec = 0.251;
  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kFaultBrake);
}

TEST(SelectorPolicy, StaleSafetyInputSuspendsTakeoverAndRequiresRedwell)
{
  SelectorPolicy policy;
  auto inputs = FreshInputs("GLOBAL", 32.0);
  policy.update(inputs);
  inputs.now_sec = 32.11;
  policy.update(inputs);
  ASSERT_TRUE(policy.tmpcActive());

  inputs.now_sec = 32.12;
  inputs.state_age_sec = 0.251;
  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kInputBrake);
  EXPECT_FALSE(policy.tmpcActive());

  // On recovery TMPC must re-earn the dwell before driving again.
  inputs.state_age_sec = 0.0;
  inputs.now_sec = 32.13;
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kGlobalWaitingTmpc);

  inputs.now_sec = 32.24;
  ExpectDecision(
    policy.update(inputs), CommandSource::kTmpc, SelectorStatus::kGlobalTmpc);
}

TEST(SelectorPolicy, ForwardsPurePursuitOnStopAndBrakesWhenItIsUnavailable)
{
  SelectorPolicy policy;

  // Pure Pursuit brakes along the path on stop requests; forward it.
  auto inputs = FreshInputs("LOCAL");
  inputs.stop_requested = true;
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kStopPurePursuit);

  inputs = FreshInputs("STOP");
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kStopPurePursuit);

  // Straight-line brake only when the Pure Pursuit command is unusable.
  inputs = FreshInputs("STOP");
  inputs.local_command_age_sec = 0.251;
  ExpectDecision(
    policy.update(inputs), CommandSource::kSafeBrake, SelectorStatus::kStopBrake);
}

TEST(SelectorPolicy, DisagreementGateBlocksEntryButNotActiveDriving)
{
  SelectorPolicy policy;

  // Entry blocked while TMPC steering contradicts fresh Pure Pursuit.
  auto inputs = FreshInputs("GLOBAL", 10.0);
  inputs.has_steering_disagreement = true;
  inputs.steering_disagreement_rad = 0.6;
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kGlobalWaitingTmpc);
  inputs.now_sec = 11.5;
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kGlobalWaitingTmpc);
  EXPECT_FALSE(policy.tmpcActive());

  // Agreement restores the dwell; takeover proceeds.
  inputs.steering_disagreement_rad = 0.1;
  inputs.now_sec = 12.0;
  policy.update(inputs);
  inputs.now_sec = 12.2;
  ExpectDecision(
    policy.update(inputs), CommandSource::kTmpc, SelectorStatus::kGlobalTmpc);

  // A mid-drive disagreement spike must NOT eject the active controller.
  inputs.steering_disagreement_rad = 0.9;
  inputs.now_sec = 12.3;
  ExpectDecision(
    policy.update(inputs), CommandSource::kTmpc, SelectorStatus::kGlobalTmpc);
  EXPECT_TRUE(policy.tmpcActive());
}

TEST(SelectorPolicy, BrakesForUnknownAndStaleSafetyInputs)
{
  SelectorPolicy policy;

  auto inputs = FreshInputs("UNKNOWN");
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

TEST(SelectorPolicy, FallsBackWhenTmpcCommandTimesOutAfterTakeover)
{
  SelectorPolicy policy;
  auto inputs = FreshInputs("GLOBAL", 40.0);
  policy.update(inputs);
  inputs.now_sec = 40.11;
  policy.update(inputs);

  inputs.now_sec = 40.12;
  inputs.tmpc_command_age_sec = 0.101;
  ExpectDecision(
    policy.update(inputs), CommandSource::kPurePursuit,
    SelectorStatus::kGlobalPurePursuitFallback);
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
