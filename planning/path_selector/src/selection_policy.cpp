#include "path_selector/selection_policy.hpp"

#include <cmath>
#include <stdexcept>

namespace path_selector
{

SelectionPolicy::SelectionPolicy(double path_source_timeout_sec)
: path_source_timeout_sec_(path_source_timeout_sec)
{
  if (!std::isfinite(path_source_timeout_sec_) || path_source_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("path_source_timeout_sec must be positive");
  }
}

SelectionDecision SelectionPolicy::decide(const SelectionInputs & inputs) const
{
  if (!inputs.requested_source.has_value()) {
    return {RequestedSource::Unknown, SelectedCandidate::None, SelectionFailure::MissingSource};
  }

  const auto requested_source = parseSource(*inputs.requested_source);
  const double source_age = inputs.now_sec - inputs.source_receive_time_sec;
  if (!std::isfinite(inputs.now_sec) || !std::isfinite(inputs.source_receive_time_sec) ||
    source_age < 0.0 || source_age > path_source_timeout_sec_)
  {
    return {requested_source, SelectedCandidate::None, SelectionFailure::StaleSource};
  }

  switch (requested_source) {
    case RequestedSource::Local:
      if (!inputs.local_candidate_ready) {
        return {
          requested_source, SelectedCandidate::None, SelectionFailure::LocalUnavailable};
      }
      return {requested_source, SelectedCandidate::Local, SelectionFailure::None};
    case RequestedSource::GlobalFull:
    case RequestedSource::GlobalFinalStop:
      if (!inputs.global_candidate_ready) {
        return {
          requested_source, SelectedCandidate::None, SelectionFailure::GlobalUnavailable};
      }
      if (!inputs.global_entry_handoff_consumed && !inputs.global_handoff_ready) {
        return {
          requested_source, SelectedCandidate::None, SelectionFailure::HandoffNotReady};
      }
      return {requested_source, SelectedCandidate::Global, SelectionFailure::None};
    case RequestedSource::Stop:
      return {requested_source, SelectedCandidate::None, SelectionFailure::StopRequested};
    case RequestedSource::Unknown:
      return {requested_source, SelectedCandidate::None, SelectionFailure::UnknownSource};
  }

  return {RequestedSource::Unknown, SelectedCandidate::None, SelectionFailure::UnknownSource};
}

RequestedSource SelectionPolicy::parseSource(const std::string & value)
{
  if (value == "LOCAL") {
    return RequestedSource::Local;
  }
  if (value == "GLOBAL_FULL") {
    return RequestedSource::GlobalFull;
  }
  if (value == "GLOBAL_FINAL_STOP") {
    return RequestedSource::GlobalFinalStop;
  }
  if (value == "STOP") {
    return RequestedSource::Stop;
  }
  return RequestedSource::Unknown;
}

double SelectionPolicy::pathSourceTimeoutSec() const
{
  return path_source_timeout_sec_;
}

bool SelectionDecision::valid() const
{
  return failure == SelectionFailure::None && selected_candidate != SelectedCandidate::None;
}

const char * toString(RequestedSource source)
{
  switch (source) {
    case RequestedSource::Local:
      return "LOCAL";
    case RequestedSource::GlobalFull:
      return "GLOBAL_FULL";
    case RequestedSource::GlobalFinalStop:
      return "GLOBAL_FINAL_STOP";
    case RequestedSource::Stop:
      return "STOP";
    case RequestedSource::Unknown:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

const char * toString(SelectedCandidate candidate)
{
  switch (candidate) {
    case SelectedCandidate::None:
      return "NONE";
    case SelectedCandidate::Local:
      return "LOCAL";
    case SelectedCandidate::Global:
      return "GLOBAL";
  }
  return "NONE";
}

const char * toString(SelectionFailure failure)
{
  switch (failure) {
    case SelectionFailure::None:
      return "none";
    case SelectionFailure::NotImplemented:
      return "not_implemented";
    case SelectionFailure::MissingSource:
      return "missing_source";
    case SelectionFailure::StaleSource:
      return "stale_source";
    case SelectionFailure::StopRequested:
      return "stop_requested";
    case SelectionFailure::UnknownSource:
      return "unknown_source";
    case SelectionFailure::LocalUnavailable:
      return "local_unavailable";
    case SelectionFailure::GlobalUnavailable:
      return "global_unavailable";
    case SelectionFailure::HandoffNotReady:
      return "handoff_not_ready";
  }
  return "unknown_source";
}

}
