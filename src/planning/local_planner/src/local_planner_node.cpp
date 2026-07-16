#include "local_planner/local_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

#include "local_path_builder_internal.hpp"

namespace local_planner
{

LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions & options)
: Node("local_planner_node", options),
  source_mode_(parseSourceMode(declare_parameter<std::string>("source_mode", "slam_map")))
{
  const auto cones_topic = declare_parameter<std::string>("cones_topic", "/cones");
  const auto slam_map_topic = declare_parameter<std::string>(
    "slam_map_topic", "/localization/cone_map");
  const auto odom_topic = declare_parameter<std::string>(
    "odom_topic", "/localization/ego_odom");
  const auto slam_status_topic = declare_parameter<std::string>(
    "slam_status_topic", "/graph_slam/status");
  const auto waypoints_topic = declare_parameter<std::string>(
    "waypoints_topic", "/planning/local_waypoints");
  const auto path_topic = declare_parameter<std::string>(
    "path_topic", "/planning/local_waypoints/path");
  const auto validity_topic = declare_parameter<std::string>(
    "validity_topic", "/planning/local_path_valid");
  const auto reason_topic = declare_parameter<std::string>(
    "reason_topic", "/planning/local_path_reason");
  max_stamp_skew_sec_ = declare_parameter<double>("max_stamp_skew_sec", 0.1);
  max_input_age_sec_ = declare_parameter<double>("max_input_age_sec", 0.5);
  heartbeat_hz_ = declare_parameter<double>("heartbeat_hz", 10.0);

  planner_config_.roi_min_x = declare_parameter<double>("roi_min_x", -1.0);
  planner_config_.roi_max_x = declare_parameter<double>("roi_max_x", 20.0);
  planner_config_.roi_abs_y = declare_parameter<double>("roi_abs_y", 8.0);
  planner_config_.endpoint_match_tolerance_m = declare_parameter<double>(
    "endpoint_match_tolerance_m", 0.05);
  planner_config_.min_track_width_m = declare_parameter<double>("min_track_width_m", 2.0);
  planner_config_.max_track_width_m = declare_parameter<double>("max_track_width_m", 6.0);
  planner_config_.duplicate_tolerance_m = declare_parameter<double>(
    "duplicate_tolerance_m", 0.05);
  planner_config_.min_forward_projection_m = declare_parameter<double>(
    "min_forward_projection_m", 0.10);
  planner_config_.max_traversal_gap_m = declare_parameter<double>("max_traversal_gap_m", 6.0);
  planner_config_.centerline_max_link_gap_m =
    declare_parameter<double>("centerline_max_link_gap_m", 4.0);
  planner_config_.max_heading_change_rad = declare_parameter<double>(
    "max_heading_change_rad", 1.047);
  planner_config_.max_u_turn_heading_change_rad = declare_parameter<double>(
    "max_u_turn_heading_change_rad", 2.618);
  planner_config_.waypoint_spacing_m = declare_parameter<double>("waypoint_spacing_m", 0.5);
  planner_config_.max_start_distance_m = declare_parameter<double>("max_start_distance_m", 4.0);
  planner_config_.two_sided_horizon_m = declare_parameter<double>("two_sided_horizon_m", 20.0);
  planner_config_.fallback_horizon_m = declare_parameter<double>("fallback_horizon_m", 8.0);
  planner_config_.fallback_offset_m = declare_parameter<double>("fallback_offset_m", 1.5);
  planner_config_.two_sided_speed_mps = declare_parameter<double>("two_sided_speed_mps", 3.0);
  planner_config_.fallback_speed_mps = declare_parameter<double>("fallback_speed_mps", 1.5);
  planner_config_.max_lateral_accel_mps2 =
    declare_parameter<double>("max_lateral_accel_mps2", 5.0);
  planner_config_.min_speed_mps = declare_parameter<double>("min_speed_mps", 1.0);
  planner_config_.allow_partial_boundary = declare_parameter<bool>(
    "allow_partial_boundary", source_mode_ == SourceMode::kSlamMap);
  planner_config_.use_unknown_cones = declare_parameter<bool>("use_unknown_cones", true);
  planner_config_.unknown_absorb_lateral_m =
    declare_parameter<double>("unknown_absorb_lateral_m", 0.75);
  planner_config_.unknown_geom_deadband_m =
    declare_parameter<double>("unknown_geom_deadband_m", 0.75);
  planner_config_.extend_straight_to_horizon =
    declare_parameter<bool>("extend_straight_to_horizon", false);
  planner_config_.straight_extension_cap_m =
    declare_parameter<double>("straight_extension_cap_m", 5.0);
  log_diagnostics_ = declare_parameter<bool>("log_planner_diagnostics", false);

  if (!std::isfinite(max_stamp_skew_sec_) || max_stamp_skew_sec_ < 0.0 ||
    !std::isfinite(max_input_age_sec_) || max_input_age_sec_ <= 0.0 ||
    !std::isfinite(heartbeat_hz_) || heartbeat_hz_ <= 0.0)
  {
    throw std::invalid_argument("local planner timing parameters are invalid");
  }

  output_ = std::make_unique<LocalPlannerOutput>(
    *this, LocalPlannerOutputTopics{waypoints_topic, path_topic, validity_topic, reason_topic},
    max_input_age_sec_, heartbeat_hz_);
  inputs_ = std::make_unique<LocalPlannerInputs>(
    *this, source_mode_,
    LocalPlannerInputTopics{cones_topic, slam_map_topic, odom_topic, slam_status_topic},
    max_stamp_skew_sec_,
    [this](const std::string & reason) {output_->invalidateImmediately(reason);},
    [this](const LiveInputPair & input) {processLivePair(input);},
    [this](const SlamMapInput & input) {processSlamMap(input);});
}

void LocalPlannerNode::processLivePair(const LiveInputPair & input)
{
  const auto odom_metadata = odomMetadata(*input.odom, input.odom_receive_time);
  const auto validation = validateLiveInput(
    headerMetadata(input.cones->header, input.cones_receive_time), odom_metadata,
    max_stamp_skew_sec_, max_input_age_sec_);
  if (!validation.valid) {
    output_->invalidateImmediately(validation.reason);
    return;
  }

  const auto cone_set = liveConeSet(*input.cones);
  const auto result = buildLocalPath(cone_set, planner_config_);
  logPlannerDiagnostics(cone_set, result);
  if (!result.valid) {
    output_->retainUntilStale(result.reason);
    return;
  }
  output_->publishPath(
    result, *input.odom, input.odom->header.stamp,
    std::max(input.cones_receive_time, input.odom_receive_time));
}

void LocalPlannerNode::processSlamMap(const SlamMapInput & input)
{
  const auto odom_metadata = odomMetadata(*input.odom, input.odom_receive_time);
  const auto validation = validateSlamMapInput(
    headerMetadata(input.map->header, input.map_receive_time), odom_metadata, max_input_age_sec_);
  if (!validation.valid) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "SLAM local input invalid: %s", validation.reason.c_str());
    output_->invalidateImmediately(validation.reason);
    return;
  }

  const auto cone_set = slamConeSet(*input.map, odom_metadata);
  const auto result = buildLocalPath(cone_set, planner_config_);
  logPlannerDiagnostics(cone_set, result);
  if (result.valid) {
    output_->publishPath(result, *input.odom, input.odom->header.stamp, input.odom_receive_time);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "SLAM local path invalid: %s", result.reason.c_str());
    output_->invalidateImmediately(result.reason);
  }
}

void LocalPlannerNode::logPlannerDiagnostics(const ConeSet & cones, const BuildResult & result)
{
  if (!log_diagnostics_) {
    return;
  }

  // Recompute the ROI crop + unknown classification exactly as buildLocalPath
  // does, so the reported counts are the ones the planner actually used.
  auto blue = internal::cropToRoi(cones.blue, planner_config_);
  auto yellow = internal::cropToRoi(cones.yellow, planner_config_);
  // Pre-classify (already-coloured) and raw unknown counts, so we can tell a
  // colour dropout (unk high, colour low) from a map that never saw the cone.
  const std::size_t raw_blue = blue.size();
  const std::size_t raw_yellow = yellow.size();
  std::size_t roi_unknown = 0U;
  if (planner_config_.use_unknown_cones && !cones.unknown.empty()) {
    const auto unknown = internal::cropToRoi(cones.unknown, planner_config_);
    roi_unknown = unknown.size();
    internal::classifyUnknownCones(blue, yellow, unknown, planner_config_);
  }

  // Signed curvature: mean sign is the turn direction (left circle > 0, right
  // circle < 0), |mean| is 1/radius, and the spread is the frame-to-frame
  // jitter. An outward-bulging left circle reads as a smaller |mean| than the
  // nominal 1/9.1 with a larger spread than the (stable) right circle.
  double kappa_sum = 0.0;
  double kappa_sq_sum = 0.0;
  const std::size_t n = result.waypoints.size();
  for (const auto & waypoint : result.waypoints) {
    kappa_sum += waypoint.kappa;
    kappa_sq_sum += waypoint.kappa * waypoint.kappa;
  }
  const double kappa_mean = n > 0U ? kappa_sum / static_cast<double>(n) : 0.0;
  const double kappa_var =
    n > 0U ? std::max(0.0, kappa_sq_sum / static_cast<double>(n) - kappa_mean * kappa_mean) : 0.0;
  const double kappa_std = std::sqrt(kappa_var);
  const double radius = std::abs(kappa_mean) > 1.0e-6 ? 1.0 / std::abs(kappa_mean) : 0.0;

  const char * kind = "none";
  switch (result.kind) {
    case PathKind::kTwoSided: kind = "two_sided"; break;
    case PathKind::kBlueOnly: kind = "blue_only"; break;
    case PathKind::kYellowOnly: kind = "yellow_only"; break;
    case PathKind::kNone: kind = "none"; break;
  }
  const char * turn = kappa_mean > 0.02 ? "LEFT" : (kappa_mean < -0.02 ? "RIGHT" : "straight");

  // roi_blue/roi_yellow are POST-classify (raw colour + absorbed unknowns).
  // raw_* is colour-only; unk is uncoloured cones available to absorb. If unk is
  // high while raw_blue stays low, the inner unknowns are being dropped, not
  // absorbed (classifyUnknownCones can't fit a line from a sparse boundary).
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 500,
    "[localdiag] turn=%s kind=%s valid=%d roi_blue=%zu(raw%zu) roi_yellow=%zu(raw%zu) "
    "unk=%zu wp=%zu kappa_mean=%.3f radius=%.2f kappa_std=%.3f reason=%s",
    turn, kind, result.valid ? 1 : 0, blue.size(), raw_blue, yellow.size(), raw_yellow,
    roi_unknown, n, kappa_mean, radius, kappa_std, result.reason.c_str());
}

}
