#include "local_planner/local_planner_node.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace local_planner
{

LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions & options)
: Node("local_planner_node", options),
  source_mode_(parseSourceMode(declare_parameter<std::string>("source_mode", "live_cones")))
{
  const auto cones_topic = declare_parameter<std::string>("cones_topic", "/cones");
  const auto slam_map_topic = declare_parameter<std::string>(
    "slam_map_topic", "/localization/cone_map");
  const auto odom_topic = declare_parameter<std::string>(
    "odom_topic", "/localization/ego_odom");
  const auto waypoints_topic = declare_parameter<std::string>(
    "waypoints_topic", "/planning/local_waypoints");
  const auto path_topic = declare_parameter<std::string>(
    "path_topic", "/planning/local_waypoints/path");
  const auto validity_topic = declare_parameter<std::string>(
    "validity_topic", "/planning/local_path_valid");
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
  planner_config_.max_heading_change_rad = declare_parameter<double>(
    "max_heading_change_rad", 1.047);
  planner_config_.waypoint_spacing_m = declare_parameter<double>("waypoint_spacing_m", 0.5);
  planner_config_.two_sided_horizon_m = declare_parameter<double>("two_sided_horizon_m", 20.0);
  planner_config_.fallback_horizon_m = declare_parameter<double>("fallback_horizon_m", 8.0);
  planner_config_.fallback_offset_m = declare_parameter<double>("fallback_offset_m", 1.5);
  planner_config_.two_sided_speed_mps = declare_parameter<double>("two_sided_speed_mps", 3.0);
  planner_config_.fallback_speed_mps = declare_parameter<double>("fallback_speed_mps", 1.5);

  if (!std::isfinite(max_stamp_skew_sec_) || max_stamp_skew_sec_ < 0.0 ||
    !std::isfinite(max_input_age_sec_) || max_input_age_sec_ <= 0.0 ||
    !std::isfinite(heartbeat_hz_) || heartbeat_hz_ <= 0.0)
  {
    throw std::invalid_argument("local planner timing parameters are invalid");
  }

  output_ = std::make_unique<LocalPlannerOutput>(
    *this, LocalPlannerOutputTopics{waypoints_topic, path_topic, validity_topic},
    max_input_age_sec_, heartbeat_hz_);
  inputs_ = std::make_unique<LocalPlannerInputs>(
    *this, source_mode_, LocalPlannerInputTopics{cones_topic, slam_map_topic, odom_topic},
    max_stamp_skew_sec_, [this] {output_->invalidateImmediately();},
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
    output_->invalidateImmediately();
    return;
  }

  const auto result = buildLocalPath(liveConeSet(*input.cones), planner_config_);
  if (!result.valid) {
    output_->retainUntilStale();
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
    output_->invalidateImmediately();
    return;
  }

  const auto result = buildLocalPath(slamConeSet(*input.map, odom_metadata), planner_config_);
  if (result.valid) {
    output_->publishPath(result, *input.odom, input.odom->header.stamp, input.odom_receive_time);
  } else {
    output_->invalidateImmediately();
  }
}

}
