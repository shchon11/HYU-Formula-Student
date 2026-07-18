#ifndef HYU_CMD_SELECTOR__SELECTOR_POLICY_HPP_
#define HYU_CMD_SELECTOR__SELECTOR_POLICY_HPP_

#include <optional>
#include <string>

namespace hyu_cmd_selector
{

enum class PlanningState
{
  kLocal,
  kGlobal,
  kStop,
  kUnknown,
};

enum class CommandSource
{
  kPurePursuit,
  kTmpc,
  kSafeBrake,
};

enum class SelectorStatus
{
  kLocalPurePursuit,
  kGlobalWaitingTmpc,
  kGlobalTmpc,
  kGlobalPurePursuitFallback,
  kStopPurePursuit,
  kFaultBrake,
  kStopBrake,
  kInputBrake,
};

struct SelectorConfig
{
  double state_timeout_sec{0.25};
  double stop_timeout_sec{0.25};
  double local_command_timeout_sec{0.25};
  // LOCAL forwarding hold: in LOCAL the selector must be a transparent pass of
  // Pure Pursuit, exactly like the plain trackdrive stack where PP owns /cmd
  // directly. The 0.25 s freshness gate above, applied to the LOCAL branch,
  // injected a zero-steer safe brake whenever a PP command arrived late under
  // the extra TMPC-chain scheduling load -- a momentary mid-corner steer-to-
  // zero that, on the marginal lap-1 corners, tipped the car into a spin (the
  // intermittent lap-1 failure that plain mode never shows). Hold the last PP
  // command through such blips; a genuine PP loss (this much silence) still
  // brakes, and PP owns its own along-path braking anyway.
  double local_forward_hold_sec{1.0};
  double tmpc_command_timeout_sec{0.1};
  double tmpc_valid_timeout_sec{0.1};
  double tmpc_ready_dwell_sec{0.1};
  // Sanity envelope against a wrong-branch TMPC reference: with a fresh Pure
  // Pursuit command available, TMPC is only trusted while its steering agrees
  // with Pure Pursuit's to within this bound. A healthy tube MPC tracks the
  // same raceline and differs by <= ~0.28 rad; the observed failure (path
  // matching snapping to a parallel/folded track section) commands 0.6+ rad
  // the WRONG way and throws the car off the corridor. <= 0 disables.
  double max_steering_disagreement_rad{0.4};
};

struct SelectorInputs
{
  std::optional<std::string> planning_state;
  double state_age_sec{0.0};
  bool has_stop_request{false};
  bool stop_requested{false};
  double stop_age_sec{0.0};
  bool has_local_command{false};
  bool local_command_valid{false};
  double local_command_age_sec{0.0};
  bool has_tmpc_command{false};
  bool tmpc_command_valid{false};
  double tmpc_command_age_sec{0.0};
  bool has_tmpc_valid{false};
  bool tmpc_valid{false};
  double tmpc_valid_age_sec{0.0};
  // |tmpc steering - pure pursuit steering| when both commands are present and
  // finite; guarded by has_steering_disagreement.
  bool has_steering_disagreement{false};
  double steering_disagreement_rad{0.0};
  double now_sec{0.0};
};

struct SelectorDecision
{
  CommandSource source{CommandSource::kSafeBrake};
  SelectorStatus status{SelectorStatus::kInputBrake};
};

class SelectorPolicy
{
public:
  explicit SelectorPolicy(const SelectorConfig & config = SelectorConfig{});

  SelectorDecision update(const SelectorInputs & inputs);
  void resetGlobalEpisode();

  bool tmpcActive() const;
  bool hadFault() const;

  static PlanningState parsePlanningState(const std::string & value);

private:
  bool fresh(bool present, double age_sec, double timeout_sec) const;
  bool tmpcReady(const SelectorInputs & inputs, bool entry) const;
  void suspendTmpc();

  SelectorConfig config_;
  bool tmpc_active_{false};
  bool had_fault_{false};
  bool tracking_tmpc_ready_{false};
  double tmpc_ready_since_sec_{0.0};
};

const char * ToString(CommandSource source);
const char * ToString(SelectorStatus status);

}  // namespace hyu_cmd_selector

#endif  // HYU_CMD_SELECTOR__SELECTOR_POLICY_HPP_
