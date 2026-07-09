#include "state_machine/planning_state_machine_node.hpp"

#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

namespace state_machine
{

PlanningStateMachineNode::PlanningStateMachineNode()
: Node("planning_state_machine_node")
{
  frenet_odom_topic_ = declare_parameter<std::string>(
    "frenet_odom_topic", "/car_state/frenet/odom");
  global_waypoints_topic_ = declare_parameter<std::string>(
    "global_waypoints_topic", "/global_waypoints");
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
  global_waypoints_timeout_sec_ = declare_parameter<double>("global_waypoints_timeout_sec", 2.0);
  cone_map_timeout_sec_ = declare_parameter<double>("cone_map_timeout_sec", 1.0);
  stop_zone_timeout_sec_ = declare_parameter<double>("stop_zone_timeout_sec", 1.0);
  final_path_end_threshold_ = declare_parameter<double>("final_path_end_threshold", 2.0);
  stop_zone_s_margin_ = declare_parameter<double>("stop_zone_s_margin", 0.0);
  max_abs_d_for_global_ = declare_parameter<double>("max_abs_d_for_global", 2.0);
  state_timer_period_ms_ = declare_parameter<int>("state_timer_period_ms", 50);
  enable_manual_lap_override_ = declare_parameter<bool>("enable_manual_lap_override", false);

  lap_count_ = initial_lap_count_;

  frenet_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    frenet_odom_topic_,
    10,
    std::bind(&PlanningStateMachineNode::onFrenetOdom, this, std::placeholders::_1));

  // TODO(haejun): Create /global_waypoints subscriber after the waypoint message type is finalized.
  // Current requested type f110_msgs/msg/WpntArray is not available in this HYU workspace.

  cone_map_sub_ = create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
    cone_map_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&PlanningStateMachineNode::onCones, this, std::placeholders::_1));

  stop_zone_s_start_sub_ = create_subscription<std_msgs::msg::Float64>(
    stop_zone_s_start_topic_,
    10,
    std::bind(&PlanningStateMachineNode::onStopZoneSStart, this, std::placeholders::_1));
  stop_zone_s_end_sub_ = create_subscription<std_msgs::msg::Float64>(
    stop_zone_s_end_topic_,
    10,
    std::bind(&PlanningStateMachineNode::onStopZoneSEnd, this, std::placeholders::_1));
  stop_zone_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
    stop_zone_valid_topic_,
    10,
    std::bind(&PlanningStateMachineNode::onStopZoneValid, this, std::placeholders::_1));

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

  if (state_ == PlanningState::STOP) {
    // TODO(haejun): Add finished/mission-complete handling later.
    return;
  }

  if (state_ == PlanningState::LOCAL) {
    if (shouldEnterGlobal()) {
      state_ = PlanningState::GLOBAL;
    }
    return;
  }

  if (state_ == PlanningState::GLOBAL && shouldEnterStop()) {
    state_ = PlanningState::STOP;
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

  const double remaining_s = global_path_length_ - current_s_;

  // TODO(haejun): Revisit this for closed-loop wrap-around and final-stop-path behavior.
  return remaining_s < final_path_end_threshold_;
}

bool PlanningStateMachineNode::isStoplineDetected() const
{
  return hasFreshStopZone() && stop_zone_valid_ && isSInStopZone(current_s_);
}

bool PlanningStateMachineNode::detectStartFinishGate() const
{
  // TODO(haejun): Implement start/finish gate detection using local /cones.
  return false;
}

bool PlanningStateMachineNode::hasCrossedStartFinishGate() const
{
  // TODO(haejun): Implement crossing logic using Frenet s or gate pose.
  return false;
}

void PlanningStateMachineNode::updateLapCount()
{
  (void)detectStartFinishGate();
  (void)hasCrossedStartFinishGate();

  // TODO(haejun): Implement lap count update condition later.
  // For now, keep lap_count_ unchanged except initial_lap_count parameter.
  // enable_manual_lap_override_ is declared for a future manual override interface.
}

bool PlanningStateMachineNode::isFresh(const rclcpp::Time & stamp, double timeout_sec) const
{
  return (now() - stamp).seconds() <= timeout_sec;
}

bool PlanningStateMachineNode::hasFreshFrenetOdom() const
{
  return has_frenet_odom_ && isFresh(last_frenet_odom_time_, frenet_odom_timeout_sec_);
}

bool PlanningStateMachineNode::hasFreshGlobalWaypoints() const
{
  return has_global_waypoints_ &&
         isFresh(last_global_waypoints_time_, global_waypoints_timeout_sec_);
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
  return has_global_waypoints_ && global_path_ready_ && hasFreshGlobalWaypoints();
}

bool PlanningStateMachineNode::isSInStopZone(double s) const
{
  const double start = stop_zone_s_start_ - stop_zone_s_margin_;
  const double end = stop_zone_s_end_ + stop_zone_s_margin_;

  if (end >= start) {
    return s >= start && s <= end;
  }

  if (global_path_length_ <= 0.0) {
    return false;
  }

  return s >= start || s <= end;
}

PathSource PlanningStateMachineNode::currentPathSource() const
{
  if (state_ == PlanningState::STOP) {
    return PathSource::STOP;
  }

  if (state_ == PlanningState::LOCAL) {
    return PathSource::LOCAL;
  }

  if (lap_count_ >= final_lap_start_count_) {
    return PathSource::GLOBAL_FINAL_STOP;
  }

  return PathSource::GLOBAL_FULL;
}

std::string PlanningStateMachineNode::stateToString(PlanningState state) const
{
  switch (state) {
    case PlanningState::LOCAL:
      return "LOCAL";
    case PlanningState::GLOBAL:
      return "GLOBAL";
    case PlanningState::STOP:
      return "STOP";
  }

  return "LOCAL";
}

std::string PlanningStateMachineNode::pathSourceToString(PathSource source) const
{
  switch (source) {
    case PathSource::LOCAL:
      return "LOCAL";
    case PathSource::GLOBAL_FULL:
      return "GLOBAL_FULL";
    case PathSource::GLOBAL_FINAL_STOP:
      return "GLOBAL_FINAL_STOP";
    case PathSource::STOP:
      return "STOP";
  }

  return "LOCAL";
}

std::string PlanningStateMachineNode::makeDebugString(
  PathSource source,
  bool stop_request) const
{
  std::ostringstream ss;
  ss << "state=" << stateToString(state_)
     << " path_source=" << pathSourceToString(source)
     << " lap_count=" << lap_count_
     << " current_s=" << current_s_
     << " current_d=" << current_d_
     << " global_path_length=" << global_path_length_
     << " frenet_fresh=" << (hasFreshFrenetOdom() ? "true" : "false")
     << " global_waypoints_fresh=" << (hasFreshGlobalWaypoints() ? "true" : "false")
     << " cone_map_fresh=" << (hasFreshConeMap() ? "true" : "false")
     << " stop_zone_fresh=" << (hasFreshStopZone() ? "true" : "false")
     << " stop_zone_valid=" << (stop_zone_valid_ ? "true" : "false")
     << " stop_zone_s_start=" << stop_zone_s_start_
     << " stop_zone_s_end=" << stop_zone_s_end_
     << " global_path_ready=" << (isGlobalPathReady() ? "true" : "false")
     << " stop_request=" << (stop_request ? "true" : "false")
     << " closest_segment_id=" << closest_segment_id_
     << " cone_frame_id=" << cone_frame_id_
     << " cones_blue=" << blue_cone_count_
     << " cones_yellow=" << yellow_cone_count_
     << " cones_orange=" << orange_cone_count_
     << " cones_big_orange=" << big_orange_cone_count_
     << " cones_unknown=" << unknown_cone_count_;

  return ss.str();
}

void PlanningStateMachineNode::publishOutputs()
{
  const PathSource source = currentPathSource();
  const bool stop_request = state_ == PlanningState::STOP;

  std_msgs::msg::String state_msg;
  state_msg.data = stateToString(state_);
  state_pub_->publish(state_msg);

  std_msgs::msg::String path_source_msg;
  path_source_msg.data = pathSourceToString(source);
  path_source_pub_->publish(path_source_msg);

  std_msgs::msg::Int32 lap_count_msg;
  lap_count_msg.data = lap_count_;
  lap_count_pub_->publish(lap_count_msg);

  std_msgs::msg::Bool stop_request_msg;
  stop_request_msg.data = stop_request;
  stop_request_pub_->publish(stop_request_msg);

  std_msgs::msg::String debug_msg;
  debug_msg.data = makeDebugString(source, stop_request);
  debug_pub_->publish(debug_msg);
}

}  // namespace state_machine

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<state_machine::PlanningStateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
