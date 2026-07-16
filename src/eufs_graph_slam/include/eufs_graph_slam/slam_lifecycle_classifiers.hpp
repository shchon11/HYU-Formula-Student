// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef EUFS_GRAPH_SLAM__SLAM_LIFECYCLE_CLASSIFIERS_HPP_
#define EUFS_GRAPH_SLAM__SLAM_LIFECYCLE_CLASSIFIERS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace eufs_graph_slam
{

enum class LoopConfirmationReason
{
  Confirmed,
  PendingThreshold,
  AmbiguousAssociation,
  OdomJump,
  ResidualHardBound,
  ResidualWindowTooLarge,
  NonFiniteInput
};

struct LoopCandidate
{
  double traveled_m;
  double stamp_sec;
  double residual_m;
};

struct LoopConfirmationConfig
{
  std::size_t required_candidates{3U};
  double min_travel_m{2.0};
  double min_elapsed_sec{1.0};
  double median_residual_max_m{1.0};
  double max_residual_m{2.0};
};

struct LoopConfirmationDecision
{
  bool confirmed{false};
  LoopConfirmationReason reason{LoopConfirmationReason::PendingThreshold};
  std::size_t candidate_count{0U};
};

class LoopConfirmationWindow
{
public:
  explicit LoopConfirmationWindow(LoopConfirmationConfig config = {})
  : config_(config)
  {
  }

  LoopConfirmationDecision observeCandidate(const LoopCandidate & candidate)
  {
    if (!std::isfinite(candidate.traveled_m) || !std::isfinite(candidate.stamp_sec) ||
      !std::isfinite(candidate.residual_m))
    {
      reset();
      return decision(LoopConfirmationReason::NonFiniteInput);
    }

    if (candidate.residual_m > config_.max_residual_m || candidate.residual_m < 0.0) {
      reset();
      return decision(LoopConfirmationReason::ResidualHardBound);
    }

    candidates_.push_back(candidate);
    if (!hasEnoughCandidates() || !spansEnoughMotion()) {
      return decision(LoopConfirmationReason::PendingThreshold);
    }

    if (medianResidual() > config_.median_residual_max_m) {
      reset();
      return decision(LoopConfirmationReason::ResidualWindowTooLarge);
    }

    return {true, LoopConfirmationReason::Confirmed, candidates_.size()};
  }

  LoopConfirmationDecision rejectAmbiguousAssociation()
  {
    reset();
    return decision(LoopConfirmationReason::AmbiguousAssociation);
  }

  LoopConfirmationDecision rejectOdomJump()
  {
    reset();
    return decision(LoopConfirmationReason::OdomJump);
  }

  void reset()
  {
    candidates_.clear();
  }

private:
  LoopConfirmationDecision decision(LoopConfirmationReason reason) const
  {
    return {false, reason, candidates_.size()};
  }

  bool hasEnoughCandidates() const
  {
    return candidates_.size() >= config_.required_candidates;
  }

  bool spansEnoughMotion() const
  {
    const LoopCandidate & first = candidates_.front();
    const LoopCandidate & last = candidates_.back();
    return last.traveled_m - first.traveled_m >= config_.min_travel_m ||
           last.stamp_sec - first.stamp_sec >= config_.min_elapsed_sec;
  }

  double medianResidual() const
  {
    std::vector<double> residuals;
    residuals.reserve(candidates_.size());
    for (const LoopCandidate & candidate : candidates_) {
      residuals.push_back(candidate.residual_m);
    }
    const auto middle = residuals.begin() + static_cast<std::ptrdiff_t>(residuals.size() / 2U);
    std::nth_element(residuals.begin(), middle, residuals.end());
    return *middle;
  }

  LoopConfirmationConfig config_;
  std::vector<LoopCandidate> candidates_;
};

enum class MappingStopReason
{
  MappingContinues,
  ExpectedLocalization,
  LoopPending,
  AmbiguousAssociation,
  FalseEarlyConvergence,
  OptimizerPoseLimitSkip,
  OdometryDropout,
  MutexOrCoasting
};

struct MappingStopClassifierInput
{
  bool localization_mode{false};
  bool map_converged{false};
  bool map_saved{false};
  bool lap_return_criteria_satisfied{false};
  bool loop_confirmed{false};
  bool loop_pending{false};
  bool ambiguous_association{false};
  bool stable_map_geometry{false};
  bool optimizer_skipped_pose_limit{false};
  bool odometry_fresh{true};
  bool cones_fresh{true};
  bool live_odometry_published{false};
  bool map_updates_stalled{false};
};

inline MappingStopReason classifyMappingStop(const MappingStopClassifierInput & input)
{
  if (!input.odometry_fresh) {
    return MappingStopReason::OdometryDropout;
  }

  if (input.optimizer_skipped_pose_limit && input.odometry_fresh && input.cones_fresh) {
    return MappingStopReason::OptimizerPoseLimitSkip;
  }

  if (input.live_odometry_published && input.map_updates_stalled) {
    return MappingStopReason::MutexOrCoasting;
  }

  if (input.ambiguous_association) {
    return MappingStopReason::AmbiguousAssociation;
  }

  if (input.loop_pending) {
    return MappingStopReason::LoopPending;
  }

  if (!input.localization_mode) {
    return MappingStopReason::MappingContinues;
  }

  if (input.map_converged && input.map_saved && input.lap_return_criteria_satisfied &&
    input.loop_confirmed && input.stable_map_geometry)
  {
    return MappingStopReason::ExpectedLocalization;
  }

  return MappingStopReason::FalseEarlyConvergence;
}

inline const char * toString(MappingStopReason reason)
{
  switch (reason) {
    case MappingStopReason::MappingContinues:
      return "mapping_continues";
    case MappingStopReason::ExpectedLocalization:
      return "expected_localization";
    case MappingStopReason::LoopPending:
      return "loop_pending";
    case MappingStopReason::AmbiguousAssociation:
      return "ambiguous_association";
    case MappingStopReason::FalseEarlyConvergence:
      return "false_early_convergence";
    case MappingStopReason::OptimizerPoseLimitSkip:
      return "optimizer_pose_limit_skip";
    case MappingStopReason::OdometryDropout:
      return "odometry_dropout";
    case MappingStopReason::MutexOrCoasting:
      return "mutex_or_coasting";
  }
  return "unknown";
}

inline const char * toString(LoopConfirmationReason reason)
{
  switch (reason) {
    case LoopConfirmationReason::Confirmed:
      return "confirmed";
    case LoopConfirmationReason::PendingThreshold:
      return "pending_threshold";
    case LoopConfirmationReason::AmbiguousAssociation:
      return "ambiguous_association";
    case LoopConfirmationReason::OdomJump:
      return "odom_jump";
    case LoopConfirmationReason::ResidualHardBound:
      return "residual_hard_bound";
    case LoopConfirmationReason::ResidualWindowTooLarge:
      return "residual_window_too_large";
    case LoopConfirmationReason::NonFiniteInput:
      return "non_finite_input";
  }
  return "unknown";
}

}  // namespace eufs_graph_slam

#endif  // EUFS_GRAPH_SLAM__SLAM_LIFECYCLE_CLASSIFIERS_HPP_
