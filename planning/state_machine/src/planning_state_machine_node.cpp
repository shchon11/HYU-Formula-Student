#include "state_machine/planning_state_machine_node.hpp"

#include <cmath>
#include <chrono>
#include <functional>
#include <string>

namespace state_machine
{

PlanningStateMachineNode::PlanningStateMachineNode()
: Node("planning_state_machine_node")
{
  using std::placeholders::_1;

  frenet_odom_topic_ = declare_parameter<std::string>(
    "frenet_odom_topic", "/car_state/frenet/odom");
  global_waypoints_topic_ = declare_parameter<std::string>(
    "global_waypoints_topic", "/global_waypoints");
  graph_slam_status_topic_ = declare_parameter<std::string>(
    "graph_slam_status_topic", "/graph_slam/status");
  global_path_valid_topic_ = declare_parameter<std::string>(
    "global_path_valid_topic", "/planning/global_path_valid");
  cone_map_topic_ = declare_parameter<std::string>("cone_map_topic", "/cones");
  stop_zone_s_start_topic_ = declare_parameter<std::string>(
    "stop_zone_s_start_topic", "/stop_zone_s_start");
  stop_zone_s_end_topic_ = declare_parameter<std::string>(
    "stop_zone_s_end_topic", "/stop_zone_s_end");
  stop_zone_valid_topic_ = declare_parameter<std::string>(
    "stop_zone_valid_topic", "/stop_zone_valid");

  target_lap_count_ = declare_parameter<int>("target_lap_count", 4);
  initial_lap_count_ = declare_parameter<int>("initial_lap_count", 0);
  final_lap_start_count_ = declare_parameter<int>("final_lap_start_count", 3);
  frenet_odom_timeout_sec_ = declare_parameter<double>("frenet_odom_timeout_sec", 0.5);
  global_path_valid_timeout_sec_ = declare_parameter<double>("global_path_valid_timeout_sec", 0.5);
  cone_map_timeout_sec_ = declare_parameter<double>("cone_map_timeout_sec", 1.0);
  stop_zone_timeout_sec_ = declare_parameter<double>("stop_zone_timeout_sec", 1.0);
  final_path_end_threshold_ = declare_parameter<double>("final_path_end_threshold", 2.0);
  stop_zone_s_margin_ = declare_parameter<double>("stop_zone_s_margin", 0.0);
  max_abs_d_for_global_ = declare_parameter<double>("max_abs_d_for_global", 2.0);
  state_timer_period_ms_ = declare_parameter<int>("state_timer_period_ms", 50);
  enable_manual_lap_override_ = declare_parameter<bool>("enable_manual_lap_override", false);

  lap_count_ = initial_lap_count_;

  frenet_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    frenet_odom_topic_, 10, std::bind(&PlanningStateMachineNode::onFrenetOdom, this, _1));

  auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto validity_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  global_waypoints_sub_ = create_subscription<eufs_msgs::msg::WaypointArrayStamped>(
    global_waypoints_topic_, latched_qos,
    std::bind(&PlanningStateMachineNode::onGlobalWaypoints, this, _1));

  graph_slam_status_sub_ = create_subscription<std_msgs::msg::String>(
    graph_slam_status_topic_, latched_qos,
    std::bind(&PlanningStateMachineNode::onGraphSlamStatus, this, _1));

  global_path_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
    global_path_valid_topic_, validity_qos,
    std::bind(&PlanningStateMachineNode::onGlobalPathValid, this, _1));

  cone_map_sub_ = create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
    cone_map_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PlanningStateMachineNode::onCones, this, _1));

  stop_zone_s_start_sub_ = create_subscription<std_msgs::msg::Float64>(
    stop_zone_s_start_topic_, 10,
    std::bind(&PlanningStateMachineNode::onStopZoneSStart, this, _1));
  stop_zone_s_end_sub_ = create_subscription<std_msgs::msg::Float64>(
    stop_zone_s_end_topic_, 10,
    std::bind(&PlanningStateMachineNode::onStopZoneSEnd, this, _1));
  stop_zone_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
    stop_zone_valid_topic_, 10,
    std::bind(&PlanningStateMachineNode::onStopZoneValid, this, _1));

  state_pub_ = create_publisher<std_msgs::msg::String>("/planning/state", 10);
  path_source_pub_ = create_publisher<std_msgs::msg::String>("/planning/path_source", 10);
  lap_count_pub_ = create_publisher<std_msgs::msg::Int32>("/planning/lap_count", 10);
  stop_request_pub_ = create_publisher<std_msgs::msg::Bool>("/planning/stop_request", 10);
  debug_pub_ = create_publisher<std_msgs::msg::String>("/planning/debug", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(state_timer_period_ms_),
    std::bind(&PlanningStateMachineNode::onTimer, this));
}

void PlanningStateMachineNode::onFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_s_ = msg->pose.pose.position.x;
  current_d_ = msg->pose.pose.position.y;
  closest_segment_id_ = msg->child_frame_id;
  last_frenet_odom_time_ = now();
  has_frenet_odom_ = true;
}

void PlanningStateMachineNode::onGlobalWaypoints(
  const eufs_msgs::msg::WaypointArrayStamped::SharedPtr msg)
{
  if (!global_path_readiness_.onWaypoints(*msg, now())) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received empty global waypoint snapshot; rejecting it.");
  }
}

void PlanningStateMachineNode::onGraphSlamStatus(const std_msgs::msg::String::SharedPtr msg)
{
  global_path_readiness_.onGraphSlamStatus(msg->data);
}

void PlanningStateMachineNode::onGlobalPathValid(const std_msgs::msg::Bool::SharedPtr msg)
{
  global_path_readiness_.onValidity(msg->data, now(), global_path_valid_timeout_sec_);
}

void PlanningStateMachineNode::onCones(
  const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg)
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
  publishOutputs();
}

void PlanningStateMachineNode::updateState()
{
  updateLapCount();
  global_path_readiness_.refreshValidity(now(), global_path_valid_timeout_sec_);

  if (state_ == PlanningState::STOP) {
    // TODO(haejun): Add finished/mission-complete handling later.
    return;
  }

  if (state_ == PlanningState::GLOBAL) {
    if (shouldEnterStop()) {
      state_ = PlanningState::STOP;
      return;
    }

    if (!isGlobalPathReady()) {
      state_ = PlanningState::LOCAL;
    }
    return;
  }

  if (state_ == PlanningState::LOCAL) {
    if (shouldEnterGlobal()) {
      state_ = PlanningState::GLOBAL;
    }
    return;
  }
}

bool PlanningStateMachineNode::shouldEnterGlobal() const
{
  return lap_count_ >= 1 &&
         isGlobalPathReady() &&
         hasFreshFrenetOdom() &&
         std::abs(current_d_) <= max_abs_d_for_global_;
}

bool PlanningStateMachineNode::shouldEnterStop() const
{
  if (!hasFreshFrenetOdom()) {
    return false;
  }

  if (lap_count_ < target_lap_count_) {
    return false;
  }

  return isFinalPathEndReached() || isStoplineDetected();
}

bool PlanningStateMachineNode::isFinalPathEndReached() const
{
  if (!has_frenet_odom_ || !isGlobalPathReady()) {
    return false;
  }

  const double remaining_s = global_path_readiness_.pathLength() - current_s_;

  // TODO(haejun): Revisit this for closed-loop wrap-around and final-stop-path behavior.
  return remaining_s < final_path_end_threshold_;
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

bool PlanningStateMachineNode::isGlobalPathReady() const
{
  return global_path_readiness_.ready(now(), global_path_valid_timeout_sec_);
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

}  // namespace state_machine
