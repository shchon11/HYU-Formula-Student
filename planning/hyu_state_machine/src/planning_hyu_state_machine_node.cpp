#include "hyu_state_machine/planning_hyu_state_machine_node.hpp"

#include "hyu_state_machine/planning_state_debug.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

namespace hyu_state_machine
{

PlanningStateMachineNode::PlanningStateMachineNode()
: Node("planning_hyu_state_machine_node")
{
  using std::placeholders::_1;

  frenet_odom_topic_ = declare_parameter<std::string>(
    "frenet_odom_topic", "/planning/frenet_odom");
  global_waypoints_topic_ = declare_parameter<std::string>(
    "global_waypoints_topic", "/planning/global_waypoints");
  graph_slam_status_topic_ = declare_parameter<std::string>(
    "graph_slam_status_topic", "/localization/status");
  global_path_valid_topic_ = declare_parameter<std::string>(
    "global_path_valid_topic", "/planning/global_path_valid");
  local_path_valid_topic_ = declare_parameter<std::string>(
    "local_path_valid_topic", "/planning/local_path_valid");
  global_handoff_ready_topic_ = declare_parameter<std::string>(
    "global_handoff_ready_topic", "/planning/global_handoff_ready");
  cone_map_topic_ = declare_parameter<std::string>("cone_map_topic", "/perception/cones");
  slam_cone_map_topic_ = declare_parameter<std::string>(
    "slam_cone_map_topic", "/localization/cone_map");
  ego_odom_topic_ = declare_parameter<std::string>(
    "ego_odom_topic", "/localization/ego_odom");
  stop_zone_s_start_topic_ = declare_parameter<std::string>(
    "stop_zone_s_start_topic", "/planning/stop_zone/s_start");
  stop_zone_s_end_topic_ = declare_parameter<std::string>(
    "stop_zone_s_end_topic", "/planning/stop_zone/s_end");
  stop_zone_valid_topic_ = declare_parameter<std::string>(
    "stop_zone_valid_topic", "/planning/stop_zone/valid");

  target_lap_count_ = declare_parameter<int>("target_lap_count", 4);
  initial_lap_count_ = declare_parameter<int>("initial_lap_count", 0);
  // Autocross-only (see header): trackdrive stops via GLOBAL_FINAL_STOP, and
  // skidpad crosses the gate line every circle, so both keep this off.
  stop_on_target_laps_ = declare_parameter<bool>("stop_on_target_laps", false);
  // Finished reporting (KASE USS rule) — see header.
  finish_on_rest_ = declare_parameter<bool>("finish_on_rest", false);
  finish_rest_speed_mps_ = declare_parameter<double>("finish_rest_speed_mps", 0.15);
  finish_rest_duration_sec_ =
    declare_parameter<double>("finish_rest_duration_sec", 3.0);
  // -1 (default) derives target_lap_count - 1, so changing the lap target
  // cannot silently desync the final-lap trigger; an explicit value wins.
  final_lap_start_count_ = declare_parameter<int>("final_lap_start_count", -1);
  if (final_lap_start_count_ < 0) {
    final_lap_start_count_ = target_lap_count_ - 1;
  } else if (final_lap_start_count_ != target_lap_count_ - 1) {
    RCLCPP_WARN(
      get_logger(),
      "final_lap_start_count=%d != target_lap_count-1=%d — final-lap "
      "behaviour (GLOBAL_FINAL_STOP) will not align with the last lap",
      final_lap_start_count_, target_lap_count_ - 1);
  }
  // 1.0 s (was 0.5): under sim load spikes the whole 100 Hz graph stalls
  // 0.3-0.4 s at a time (measured 2026-07-19); 0.5 left the LOCAL->GLOBAL
  // re-entry gate flapping on "frenet odom stale" after every demotion.
  frenet_odom_timeout_sec_ = declare_parameter<double>("frenet_odom_timeout_sec", 1.0);
  global_path_valid_timeout_sec_ = declare_parameter<double>("global_path_valid_timeout_sec", 0.5);
  global_handoff_timeout_sec_ = declare_parameter<double>("global_handoff_timeout_sec", 0.5);
  global_entry_dwell_sec_ = declare_parameter<double>("global_entry_dwell_sec", 0.5);
  cone_map_timeout_sec_ = declare_parameter<double>("cone_map_timeout_sec", 1.0);
  stop_zone_timeout_sec_ = declare_parameter<double>("stop_zone_timeout_sec", 1.0);
  lap_path_closure_tolerance_m_ =
    declare_parameter<double>("lap_path_closure_tolerance_m", 1.0);
  lap_closing_duplicate_tolerance_m_ =
    declare_parameter<double>("lap_closing_duplicate_tolerance_m", 0.05);
  lap_gate_cluster_tolerance_m_ =
    declare_parameter<double>("lap_gate_cluster_tolerance_m", 2.0);
  lap_gate_min_width_m_ = declare_parameter<double>("lap_gate_min_width_m", 2.0);
  lap_gate_max_width_m_ = declare_parameter<double>("lap_gate_max_width_m", 7.0);
  lap_gate_arm_distance_m_ =
    declare_parameter<double>("lap_gate_arm_distance_m", 12.0);
  lap_gate_cooldown_sec_ = declare_parameter<double>("lap_gate_cooldown_sec", 10.0);
  min_lap_count_speed_mps_ =
    declare_parameter<double>("min_lap_count_speed_mps", 0.3);
  final_path_end_threshold_ = declare_parameter<double>("final_path_end_threshold", 2.0);
  stop_zone_s_margin_ = declare_parameter<double>("stop_zone_s_margin", 0.0);
  max_abs_d_for_global_ = declare_parameter<double>("max_abs_d_for_global", 2.0);
  state_timer_period_ms_ = declare_parameter<int>("state_timer_period_ms", 50);
  enable_manual_lap_override_ = declare_parameter<bool>("enable_manual_lap_override", false);
  global_requires_graph_slam_localization_ =
    declare_parameter<bool>("global_requires_graph_slam_localization", true);

  lap_count_ = initial_lap_count_;
  fallback_lap_count_ = 0;
  discovery_lap_seen_ = false;
  gate_laps_at_discovery_ = 0;
  lap_tracking_policy_ = std::make_unique<LapTrackingPolicy>(
    lap_path_closure_tolerance_m_, lap_closing_duplicate_tolerance_m_);
  gate_tracker_ = std::make_unique<OrangeGateLapTracker>(
    lap_gate_cluster_tolerance_m_, lap_gate_min_width_m_, lap_gate_max_width_m_,
    lap_gate_arm_distance_m_, lap_gate_cooldown_sec_);

  frenet_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    frenet_odom_topic_, 10, std::bind(&PlanningStateMachineNode::onFrenetOdom, this, _1));

  auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto validity_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  global_waypoints_sub_ = create_subscription<hyu_msgs::msg::WaypointArrayStamped>(
    global_waypoints_topic_, latched_qos,
    std::bind(&PlanningStateMachineNode::onGlobalWaypoints, this, _1));

  graph_slam_status_sub_ = create_subscription<std_msgs::msg::String>(
    graph_slam_status_topic_, latched_qos,
    std::bind(&PlanningStateMachineNode::onGraphSlamStatus, this, _1));

  global_path_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
    global_path_valid_topic_, validity_qos,
    std::bind(&PlanningStateMachineNode::onGlobalPathValid, this, _1));

  local_path_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
    local_path_valid_topic_, validity_qos,
    std::bind(&PlanningStateMachineNode::onLocalPathValid, this, _1));

  global_handoff_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
    global_handoff_ready_topic_, validity_qos,
    std::bind(&PlanningStateMachineNode::onGlobalHandoffReady, this, _1));

  cone_map_sub_ = create_subscription<hyu_msgs::msg::ConeArrayWithCovariance>(
    cone_map_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PlanningStateMachineNode::onCones, this, _1));

  // Start/finish gate: orange landmarks (big and small are deliberately not
  // distinguished — classification between the two is unreliable) from the
  // latched SLAM map plus the map-frame ego stream drive the gate lap counter.
  slam_cone_map_sub_ = create_subscription<hyu_msgs::msg::ConeArrayWithCovariance>(
    slam_cone_map_topic_, latched_qos,
    std::bind(&PlanningStateMachineNode::onSlamConeMap, this, _1));
  ego_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    ego_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
    std::bind(&PlanningStateMachineNode::onEgoOdom, this, _1));

  stop_zone_s_start_sub_ = create_subscription<std_msgs::msg::Float64>(
    stop_zone_s_start_topic_, 10,
    std::bind(&PlanningStateMachineNode::onStopZoneSStart, this, _1));
  stop_zone_s_end_sub_ = create_subscription<std_msgs::msg::Float64>(
    stop_zone_s_end_topic_, 10,
    std::bind(&PlanningStateMachineNode::onStopZoneSEnd, this, _1));
  stop_zone_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
    stop_zone_valid_topic_, 10,
    std::bind(&PlanningStateMachineNode::onStopZoneValid, this, _1));

  // Lap-counting inhibit until AS_DRIVING (see vehicle_driving_seen_).
  // Best-effort: matches the sim plugin, which also only publishes the topic
  // while it has subscribers — this subscription is part of what makes it flow.
  as_state_sub_ = create_subscription<hyu_msgs::msg::CanState>(
    declare_parameter<std::string>("as_state_topic", "/vehicle/as_state"),
    rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
    std::bind(&PlanningStateMachineNode::onAsState, this, _1));

  state_pub_ = create_publisher<std_msgs::msg::String>("/planning/state", 10);
  path_source_pub_ = create_publisher<std_msgs::msg::String>("/planning/path_source", 10);
  lap_count_pub_ = create_publisher<std_msgs::msg::Int32>("/planning/lap_count", 10);
  lap_time_last_pub_ = create_publisher<std_msgs::msg::Float64>("/planning/lap_time_last", 10);
  lap_time_best_pub_ = create_publisher<std_msgs::msg::Float64>("/planning/lap_time_best", 10);
  stop_request_pub_ = create_publisher<std_msgs::msg::Bool>("/planning/stop_request", 10);
  mission_completed_pub_ = create_publisher<std_msgs::msg::Bool>(
    declare_parameter<std::string>(
      "mission_completed_topic", "/vehicle/mission_completed"), 10);
  debug_pub_ = create_publisher<std_msgs::msg::String>("/planning/debug", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(state_timer_period_ms_),
    std::bind(&PlanningStateMachineNode::onTimer, this));
}

void PlanningStateMachineNode::onAsState(const hyu_msgs::msg::CanState::SharedPtr msg)
{
  if (!vehicle_driving_seen_ && msg->as_state == hyu_msgs::msg::CanState::AS_DRIVING) {
    vehicle_driving_seen_ = true;
    RCLCPP_INFO(get_logger(), "AS_DRIVING observed — lap counting enabled.");
  }
}

void PlanningStateMachineNode::onFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time receive_time = now();
  current_s_ = msg->pose.pose.position.x;
  current_d_ = msg->pose.pose.position.y;
  closest_segment_id_ = msg->child_frame_id;
  last_frenet_odom_time_ = receive_time;
  has_frenet_odom_ = true;

  // Frenet seam-wrap is an INDEPENDENT fallback estimator that counts even
  // while the orange gate is valid — so a gate that mislocates or misses a
  // crossing can never wedge the lap count (the published count is the max of
  // the two estimators; see recomputeLapCount).
  if (vehicle_driving_seen_ && lap_tracking_policy_ && lap_tracking_policy_->observeFrenetSample(
      current_s_, receive_time.seconds(), frenet_odom_timeout_sec_, 2.0))
  {
    ++fallback_lap_count_;
    recomputeLapCount();
    RCLCPP_INFO(
      get_logger(), "Frenet seam-wrap: fallback lap %d (s=%.2f) — lap %d.",
      fallback_lap_count_, current_s_, lap_count_);
  }
}

void PlanningStateMachineNode::onSlamConeMap(
  const hyu_msgs::msg::ConeArrayWithCovariance::SharedPtr msg)
{
  if (!gate_tracker_) {
    return;
  }
  std::vector<std::array<double, 2>> gate_markers;
  gate_markers.reserve(msg->big_orange_cones.size() + msg->orange_cones.size());
  for (const auto * cones : {&msg->big_orange_cones, &msg->orange_cones}) {
    for (const auto & cone : *cones) {
      gate_markers.push_back({cone.point.x, cone.point.y});
    }
  }
  gate_tracker_->updateGate(gate_markers);
}

void PlanningStateMachineNode::onEgoOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Pre-driving the car sits ON the gate and ego transients can fake a
  // crossing (see vehicle_driving_seen_) — do not even feed the tracker.
  if (!vehicle_driving_seen_) {
    return;
  }
  const double ego_speed = std::hypot(
    msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  // Rest tracking for the Finished report (see header): "moved" needs real
  // driving speed once; "at rest" means no sample above the rest threshold
  // for finish_rest_duration_sec.
  if (std::isfinite(ego_speed)) {
    const rclcpp::Time sample_time = now();
    if (ego_speed > 0.5) {
      has_vehicle_moved_ = true;
    }
    if (ego_speed > finish_rest_speed_mps_ || !has_last_motion_time_) {
      last_motion_time_ = sample_time;
      has_last_motion_time_ = true;
    }
  }
  // Standstill guard: SLAM (re)initialisation can SNAP the pose estimate by
  // meters while the car is not moving — a segment that "crosses" the gate
  // and can even fake the 3 m arming distance. A real crossing happens at
  // speed; the measured body velocity stays ~0 through a standstill pose
  // snap, so it cleanly separates the two.
  if (!std::isfinite(ego_speed) || ego_speed < min_lap_count_speed_mps_) {
    return;
  }
  if (gate_tracker_ && gate_tracker_->observeEgo(
      msg->pose.pose.position.x, msg->pose.pose.position.y, now().seconds()))
  {
    recomputeLapCount();
    RCLCPP_INFO(
      get_logger(),
      "Start/finish gate crossed: lap %d, lap time %.2f s "
      "(ego %.2f,%.2f v=%.2f gate A %.2f,%.2f B %.2f,%.2f)",
      lap_count_, gate_tracker_->lastLapSec(),
      msg->pose.pose.position.x, msg->pose.pose.position.y,
      std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y),
      gate_tracker_->gateAx(), gate_tracker_->gateAy(),
      gate_tracker_->gateBx(), gate_tracker_->gateBy());
  }
}

void PlanningStateMachineNode::onGlobalWaypoints(
  const hyu_msgs::msg::WaypointArrayStamped::SharedPtr msg)
{
  const rclcpp::Time receive_time = now();
  if (lap_tracking_policy_) {
    lap_tracking_policy_->acceptPath(*msg);
  }
  if (!global_path_readiness_.onWaypoints(*msg, receive_time)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received empty global waypoint snapshot; rejecting it.");
  }
}

void PlanningStateMachineNode::onGraphSlamStatus(const std_msgs::msg::String::SharedPtr msg)
{
  // Discovery-lap floor: completing the mapping lap (mapping -> localization)
  // guarantees at least one lap. This is part of the fallback estimator (see
  // onFrenetOdom) and floors the count unconditionally so lap_count is never
  // stuck below 1 even if the gate never registers a crossing.
  if (lap_tracking_policy_ && lap_tracking_policy_->observeGraphSlamStatus(msg->data) &&
    vehicle_driving_seen_)
  {
    // The mapping lap is complete: it is lap 1, and it becomes the shared
    // base for BOTH estimators. Anchor the gate count here so every later
    // gate crossing is a DELTA from this moment — spawn-independent, so it
    // does not matter whether the gate already consumed the start-box
    // departure or the mapping-return as its race-start.
    discovery_lap_seen_ = true;
    gate_laps_at_discovery_ =
      gate_tracker_ ? gate_tracker_->countedLaps() : 0;
    recomputeLapCount();
    RCLCPP_INFO(
      get_logger(), "Discovery lap: mapping complete — lap %d (base).",
      lap_count_);
  }
  global_path_readiness_.onGraphSlamStatus(msg->data);
}

void PlanningStateMachineNode::onGlobalPathValid(const std_msgs::msg::Bool::SharedPtr msg)
{
  global_path_readiness_.onValidity(msg->data, now(), global_path_valid_timeout_sec_);
}

void PlanningStateMachineNode::onLocalPathValid(const std_msgs::msg::Bool::SharedPtr msg)
{
  local_path_valid_ = msg->data;
  last_local_path_valid_time_ = now();
  has_local_path_valid_ = true;
}

void PlanningStateMachineNode::onGlobalHandoffReady(const std_msgs::msg::Bool::SharedPtr msg)
{
  global_path_readiness_.onHandoffReady(
    msg->data, now(), global_handoff_timeout_sec_);
}

void PlanningStateMachineNode::onCones(
  const hyu_msgs::msg::ConeArrayWithCovariance::SharedPtr msg)
{
  blue_cone_count_ = msg->blue_cones.size();
  yellow_cone_count_ = msg->yellow_cones.size();
  orange_cone_count_ = msg->orange_cones.size();
  big_orange_cone_count_ = msg->big_orange_cones.size();
  unknown_cone_count_ = msg->unknown_color_cones.size();
  cone_frame_id_ = msg->header.frame_id;
  last_cone_map_time_ = now();
  has_cone_map_ = true;
}

void PlanningStateMachineNode::onStopZoneSStart(const std_msgs::msg::Float64::SharedPtr msg)
{
  stop_zone_s_start_ = msg->data;
  last_stop_zone_s_start_time_ = now();
  has_stop_zone_s_start_ = true;
}

void PlanningStateMachineNode::onStopZoneSEnd(const std_msgs::msg::Float64::SharedPtr msg)
{
  stop_zone_s_end_ = msg->data;
  last_stop_zone_s_end_time_ = now();
  has_stop_zone_s_end_ = true;
}

void PlanningStateMachineNode::onStopZoneValid(const std_msgs::msg::Bool::SharedPtr msg)
{
  stop_zone_valid_ = msg->data;
  last_stop_zone_valid_time_ = now();
  has_stop_zone_valid_ = true;
}

void PlanningStateMachineNode::onTimer()
{
  updateState();
  updateMissionFinished();
  publishOutputs();
}

void PlanningStateMachineNode::updateMissionFinished()
{
  if (mission_finished_) {
    return;  // Latched: Finished never reverts (제6조⑥ — the run is over).
  }
  if (!vehicle_driving_seen_ || !has_vehicle_moved_ || !has_last_motion_time_) {
    return;
  }
  if (state_ != PlanningState::STOP && !finish_on_rest_) {
    return;
  }
  if ((now() - last_motion_time_).seconds() < finish_rest_duration_sec_) {
    return;
  }
  mission_finished_ = true;
  RCLCPP_INFO(
    get_logger(),
    "Mission FINISHED (at rest %.1f s, state %s) — reporting mission_completed.",
    finish_rest_duration_sec_, planningStateToString(state_).c_str());
}

void PlanningStateMachineNode::updateState()
{
  const rclcpp::Time current_time = now();
  global_path_readiness_.refreshHandoff(current_time, global_handoff_timeout_sec_);
  updateLapCount();
  global_path_readiness_.refreshValidity(current_time, global_path_valid_timeout_sec_);

  if (state_ == PlanningState::STOP) {
    // TODO(haejun): Add finished/mission-complete handling later.
    return;
  }

  if (stop_on_target_laps_ && vehicle_driving_seen_ && lap_count_ >= target_lap_count_) {
    state_ = PlanningState::STOP;
    last_stop_request_reason_ = "target_laps_reached";
    RCLCPP_INFO(
      get_logger(), "Lap target reached (%d/%d) — stopping (stop_on_target_laps).",
      lap_count_, target_lap_count_);
    return;
  }

  const std::string stop_reason = stopRequestReason();
  const bool stop_requested = stop_reason == "final_path_end_reached" ||
    stop_reason == "stopline_detected";
  last_stop_request_reason_ = stop_reason;

  state_ = transitionForGlobalReadiness(
    state_,
    state_ == PlanningState::GLOBAL && stop_requested,
    GlobalEntryConditions{
      global_path_readiness_.ready(
        current_time, global_path_valid_timeout_sec_, global_requires_graph_slam_localization_),
      hasFreshFrenetOdom(),
      current_d_,
      max_abs_d_for_global_,
      global_path_readiness_.handoffDwellReady(
        current_time, global_handoff_timeout_sec_, global_entry_dwell_sec_)});
}

std::string PlanningStateMachineNode::globalEntryReason(const rclcpp::Time & current_time) const
{
  if (!global_path_readiness_.ready(
      current_time, global_path_valid_timeout_sec_, global_requires_graph_slam_localization_))
  {
    return global_path_readiness_.readinessReason(
      current_time, global_path_valid_timeout_sec_, global_requires_graph_slam_localization_);
  }
  if (!hasFreshFrenetOdom()) {
    return "stale_frenet_odom";
  }
  if (!std::isfinite(current_d_) || !std::isfinite(max_abs_d_for_global_) ||
    std::abs(current_d_) > max_abs_d_for_global_)
  {
    return "frenet_lateral_offset_exceeded";
  }
  if (!global_path_readiness_.hasHandoff()) {
    return "missing_handoff";
  }
  if (!global_path_readiness_.hasFreshHandoff(current_time, global_handoff_timeout_sec_)) {
    return "stale_handoff";
  }
  if (!global_path_readiness_.handoffReady()) {
    return "handoff_not_ready";
  }
  if (!global_path_readiness_.handoffDwellReady(
      current_time, global_handoff_timeout_sec_, global_entry_dwell_sec_))
  {
    return "handoff_dwell_pending";
  }
  return "ready";
}

std::string PlanningStateMachineNode::stopRequestReason() const
{
  if (!hasFreshFrenetOdom()) {
    return "stale_frenet_odom";
  }
  // Allow the finish once we are ON the final lap, not after it. The final
  // raceline (GLOBAL_FINAL_STOP) engages at final_lap_start_count_ (= target-1,
  // e.g. lap_count 9 for a 10-lap mission = driving the 10th lap), and the car
  // stops when it reaches that line's END = the finish of the target-th lap.
  // Gating on lap_count_ >= target_lap_count_ instead demanded the target lap
  // be COMPLETED first and only THEN drove to a path end — a whole extra lap
  // (trackdrive 10 ran 11: mapping lap 1 + 10 racing). Autocross is unaffected
  // (it stops via stop_on_target_laps_).
  if (lap_count_ < final_lap_start_count_) {
    return "target_lap_pending";
  }
  if (global_path_readiness_.finalPathEndReached(
      now(), global_path_valid_timeout_sec_, global_requires_graph_slam_localization_,
      has_frenet_odom_, current_s_, final_path_end_threshold_))
  {
    return "final_path_end_reached";
  }
  if (isStoplineDetected()) {
    return "stopline_detected";
  }
  return "not_requested";
}

bool PlanningStateMachineNode::isStoplineDetected() const
{
  return hasFreshStopZone() && stop_zone_valid_ && isSInStopZone(current_s_);
}

bool PlanningStateMachineNode::isFresh(const rclcpp::Time & stamp, double timeout_sec) const
{
  return (now() - stamp).seconds() <= timeout_sec;
}

bool PlanningStateMachineNode::hasFreshFrenetOdom() const
{
  return has_frenet_odom_ &&
         isFresh(last_frenet_odom_time_, frenet_odom_timeout_sec_);
}

bool PlanningStateMachineNode::hasFreshLocalPathValid() const
{
  return has_local_path_valid_ &&
         isFresh(last_local_path_valid_time_, global_path_valid_timeout_sec_);
}

bool PlanningStateMachineNode::hasFreshConeMap() const
{
  return has_cone_map_ && isFresh(last_cone_map_time_, cone_map_timeout_sec_);
}

bool PlanningStateMachineNode::hasFreshStopZone() const
{
  return has_stop_zone_s_start_ &&
         has_stop_zone_s_end_ &&
         has_stop_zone_valid_ &&
         isFresh(last_stop_zone_s_start_time_, stop_zone_timeout_sec_) &&
         isFresh(last_stop_zone_s_end_time_, stop_zone_timeout_sec_) &&
         isFresh(last_stop_zone_valid_time_, stop_zone_timeout_sec_);
}

bool PlanningStateMachineNode::isSInStopZone(double s) const
{
  const double start = stop_zone_s_start_ - stop_zone_s_margin_;
  const double end = stop_zone_s_end_ + stop_zone_s_margin_;

  if (end >= start) {
    return s >= start && s <= end;
  }

  if (global_path_readiness_.pathLength() <= 0.0) {
    return false;
  }

  return s >= start || s <= end;
}

}  // namespace hyu_state_machine
