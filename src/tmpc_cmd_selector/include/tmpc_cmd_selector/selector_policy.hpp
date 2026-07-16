#ifndef TMPC_CMD_SELECTOR__SELECTOR_POLICY_HPP_
#define TMPC_CMD_SELECTOR__SELECTOR_POLICY_HPP_

#include <optional>
#include <string>

namespace tmpc_cmd_selector
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
  kFaultBrake,
  kStopBrake,
  kInputBrake,
};

struct SelectorConfig
{
  double state_timeout_sec{0.25};
  double stop_timeout_sec{0.25};
  double local_command_timeout_sec{0.25};
  double tmpc_command_timeout_sec{0.1};
  double tmpc_valid_timeout_sec{0.1};
  double tmpc_ready_dwell_sec{0.1};
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
  bool faultLatched() const;

  static PlanningState parsePlanningState(const std::string & value);

private:
  bool fresh(bool present, double age_sec, double timeout_sec) const;
  bool tmpcReady(const SelectorInputs & inputs) const;

  SelectorConfig config_;
  bool tmpc_active_{false};
  bool fault_latched_{false};
  bool tracking_tmpc_ready_{false};
  double tmpc_ready_since_sec_{0.0};
};

const char * ToString(CommandSource source);
const char * ToString(SelectorStatus status);

}  // namespace tmpc_cmd_selector

#endif  // TMPC_CMD_SELECTOR__SELECTOR_POLICY_HPP_
