#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "hyu_local_planner/input_policy.hpp"
#include "hyu_local_planner/local_path_builder.hpp"
#include "hyu_local_planner/ros_inputs.hpp"
#include "hyu_local_planner/ros_outputs.hpp"

namespace hyu_local_planner
{

class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void processLivePair(const LiveInputPair & input);
  void processSlamMap(const SlamMapInput & input);

  // Debug-only. When log_planner_diagnostics is set, emit a throttled line
  // reporting the chosen path mode, ROI cone counts, and path curvature stats
  // so a left/right-circle asymmetry can be diagnosed live. No effect on the
  // planned path; the counts recompute the same crop/classify pipeline.
  void logPlannerDiagnostics(const ConeSet & cones, const BuildResult & result);

  SourceMode source_mode_;
  PlannerConfig planner_config_;
  double max_stamp_skew_sec_{0.1};
  double max_input_age_sec_{0.5};
  double heartbeat_hz_{10.0};
  bool log_diagnostics_{false};
  // Live-cone extension (slam mode): stitch fresh perception cones beyond the
  // SLAM map frontier onto the ConeSet so the path does not end at the last
  // confirmed landmark. Map cones always win (dedupe radius); stale frames
  // fall back to map-only.
  bool use_live_cone_extension_{false};
  double live_cone_max_age_sec_{0.4};
  LiveExtensionConfig live_extension_config_{};
  std::unique_ptr<LocalPlannerOutput> output_;
  std::unique_ptr<LocalPlannerInputs> inputs_;
};

}
