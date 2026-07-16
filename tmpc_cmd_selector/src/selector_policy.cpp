#include "tmpc_cmd_selector/selector_policy.hpp"

#include <cmath>
#include <stdexcept>

namespace tmpc_cmd_selector
{

namespace
{

bool IsPositiveFinite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

SelectorPolicy::SelectorPolicy(const SelectorConfig & config)
: config_(config)
{
  if (!IsPositiveFinite(config_.state_timeout_sec) ||
    !IsPositiveFinite(config_.stop_timeout_sec) ||
    !IsPositiveFinite(config_.local_command_timeout_sec) ||
    !IsPositiveFinite(config_.tmpc_command_timeout_sec) ||
    !IsPositiveFinite(config_.tmpc_valid_timeout_sec) ||
    !std::isfinite(config_.tmpc_ready_dwell_sec) || config_.tmpc_ready_dwell_sec < 0.0)
  {
    throw std::invalid_argument("selector timeouts must be positive and dwell non-negative");
  }
}

SelectorDecision SelectorPolicy::update(const SelectorInputs & inputs)
{
  const bool state_fresh = fresh(
    inputs.planning_state.has_value(), inputs.state_age_sec, config_.state_timeout_sec);
  const PlanningState state = state_fresh ?
    parsePlanningState(*inputs.planning_state) : PlanningState::kUnknown;

  // A confirmed return to LOCAL starts a new controller-selection episode even
  // if another safety input is temporarily missing.
  if (state_fresh && state == PlanningState::kLocal) {
    resetGlobalEpisode();
  }

  const bool local_ok = inputs.local_command_valid && fresh(
    inputs.has_local_command, inputs.local_command_age_sec,
    config_.local_command_timeout_sec);

  if (!std::isfinite(inputs.now_sec) || !state_fresh ||
    !fresh(inputs.has_stop_request, inputs.stop_age_sec, config_.stop_timeout_sec))
  {
    // Losing a safety input interrupts any takeover: TMPC must re-earn the
    // dwell once the inputs recover instead of resuming instantly.
    suspendTmpc();
    return {CommandSource::kSafeBrake, SelectorStatus::kInputBrake};
  }

  if (inputs.stop_requested || state == PlanningState::kStop) {
    suspendTmpc();
    // Pure Pursuit already brakes along the path on stop requests; prefer that
    // over a straight-line stop and brake blind only when its command is
    // stale or non-finite.
    if (local_ok) {
      return {CommandSource::kPurePursuit, SelectorStatus::kStopPurePursuit};
    }
    return {CommandSource::kSafeBrake, SelectorStatus::kStopBrake};
  }

  if (state == PlanningState::kUnknown) {
    return {CommandSource::kSafeBrake, SelectorStatus::kInputBrake};
  }

  if (state == PlanningState::kLocal) {
    if (local_ok) {
      return {CommandSource::kPurePursuit, SelectorStatus::kLocalPurePursuit};
    }
    return {CommandSource::kSafeBrake, SelectorStatus::kInputBrake};
  }

  const bool ready = tmpcReady(inputs);
  if (tmpc_active_) {
    if (ready) {
      return {CommandSource::kTmpc, SelectorStatus::kGlobalTmpc};
    }
    // TMPC dropped out mid-drive. Fall back to Pure Pursuit instead of
    // parking the car; TMPC has to stay valid for a full dwell to retake.
    suspendTmpc();
    had_fault_ = true;
  }

  if (ready) {
    if (!tracking_tmpc_ready_) {
      tracking_tmpc_ready_ = true;
      tmpc_ready_since_sec_ = inputs.now_sec;
    }

    const double ready_duration = inputs.now_sec - tmpc_ready_since_sec_;
    if (!std::isfinite(ready_duration) || ready_duration < 0.0) {
      tracking_tmpc_ready_ = false;
    } else if (ready_duration >= config_.tmpc_ready_dwell_sec) {
      tmpc_active_ = true;
      return {CommandSource::kTmpc, SelectorStatus::kGlobalTmpc};
    }
  } else {
    tracking_tmpc_ready_ = false;
  }

  if (local_ok) {
    return {CommandSource::kPurePursuit,
      had_fault_ ? SelectorStatus::kGlobalPurePursuitFallback :
      SelectorStatus::kGlobalWaitingTmpc};
  }
  return {CommandSource::kSafeBrake,
    had_fault_ ? SelectorStatus::kFaultBrake : SelectorStatus::kInputBrake};
}

void SelectorPolicy::resetGlobalEpisode()
{
  tmpc_active_ = false;
  had_fault_ = false;
  tracking_tmpc_ready_ = false;
  tmpc_ready_since_sec_ = 0.0;
}

void SelectorPolicy::suspendTmpc()
{
  tmpc_active_ = false;
  tracking_tmpc_ready_ = false;
}

bool SelectorPolicy::tmpcActive() const
{
  return tmpc_active_;
}

bool SelectorPolicy::hadFault() const
{
  return had_fault_;
}

PlanningState SelectorPolicy::parsePlanningState(const std::string & value)
{
  if (value == "LOCAL") {
    return PlanningState::kLocal;
  }
  if (value == "GLOBAL") {
    return PlanningState::kGlobal;
  }
  if (value == "STOP") {
    return PlanningState::kStop;
  }
  return PlanningState::kUnknown;
}

bool SelectorPolicy::fresh(bool present, double age_sec, double timeout_sec) const
{
  return present && std::isfinite(age_sec) && age_sec >= 0.0 && age_sec <= timeout_sec;
}

bool SelectorPolicy::tmpcReady(const SelectorInputs & inputs) const
{
  return inputs.tmpc_valid && inputs.tmpc_command_valid &&
         fresh(
    inputs.has_tmpc_valid, inputs.tmpc_valid_age_sec, config_.tmpc_valid_timeout_sec) &&
         fresh(
    inputs.has_tmpc_command, inputs.tmpc_command_age_sec,
    config_.tmpc_command_timeout_sec);
}

const char * ToString(CommandSource source)
{
  switch (source) {
    case CommandSource::kPurePursuit:
      return "PURE_PURSUIT";
    case CommandSource::kTmpc:
      return "TMPC";
    case CommandSource::kSafeBrake:
      return "SAFE_BRAKE";
  }
  return "SAFE_BRAKE";
}

const char * ToString(SelectorStatus status)
{
  switch (status) {
    case SelectorStatus::kLocalPurePursuit:
      return "LOCAL_PP";
    case SelectorStatus::kGlobalWaitingTmpc:
      return "GLOBAL_WAITING_TMPC";
    case SelectorStatus::kGlobalTmpc:
      return "GLOBAL_TMPC";
    case SelectorStatus::kGlobalPurePursuitFallback:
      return "GLOBAL_PP_FALLBACK";
    case SelectorStatus::kStopPurePursuit:
      return "STOP_PP";
    case SelectorStatus::kFaultBrake:
      return "FAULT_BRAKE";
    case SelectorStatus::kStopBrake:
      return "STOP_BRAKE";
    case SelectorStatus::kInputBrake:
      return "INPUT_BRAKE";
  }
  return "INPUT_BRAKE";
}

}  // namespace tmpc_cmd_selector
