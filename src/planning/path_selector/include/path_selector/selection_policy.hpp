#pragma once

#include <optional>
#include <string>

namespace path_selector
{

enum class RequestedSource
{
  Local,
  GlobalFull,
  GlobalFinalStop,
  Stop,
  Unknown,
};

enum class SelectedCandidate
{
  None,
  Local,
  Global,
};

enum class SelectionFailure
{
  None,
  NotImplemented,
  MissingSource,
  StaleSource,
  StopRequested,
  UnknownSource,
  LocalUnavailable,
  GlobalUnavailable,
  HandoffNotReady,
};

struct SelectionInputs
{
  std::optional<std::string> requested_source;
  double source_receive_time_sec{0.0};
  double now_sec{0.0};
  bool local_candidate_ready{false};
  bool global_candidate_ready{false};
  bool global_handoff_ready{false};
  bool global_entry_handoff_consumed{false};
};

struct SelectionDecision
{
  RequestedSource requested_source{RequestedSource::Unknown};
  SelectedCandidate selected_candidate{SelectedCandidate::None};
  SelectionFailure failure{SelectionFailure::NotImplemented};

  bool valid() const;
};

class SelectionPolicy
{
public:
  explicit SelectionPolicy(double path_source_timeout_sec = 0.5);

  SelectionDecision decide(const SelectionInputs & inputs) const;
  static RequestedSource parseSource(const std::string & value);

  double pathSourceTimeoutSec() const;

private:
  double path_source_timeout_sec_;
};

const char * toString(RequestedSource source);
const char * toString(SelectedCandidate candidate);
const char * toString(SelectionFailure failure);

}
