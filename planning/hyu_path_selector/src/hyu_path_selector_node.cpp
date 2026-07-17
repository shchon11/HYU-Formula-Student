#include "hyu_path_selector/hyu_path_selector_node.hpp"

#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>

namespace hyu_path_selector
{
namespace
{

ContinuityThresholds declareContinuityThresholds(rclcpp::Node & node)
{
  return ContinuityThresholds{
    node.declare_parameter<double>("input_timeout_sec", 0.5),
    node.declare_parameter<double>("minimum_common_length_m", 5.0),
    node.declare_parameter<double>("maximum_start_separation_m", 1.5),
    node.declare_parameter<double>("maximum_heading_difference_rad", 0.52)};
}

}

PathSelectorNode::PathSelectorNode()
: Node("hyu_path_selector_node"),
  path_source_topic_(declare_parameter<std::string>(
      "path_source_topic", "/planning/path_source")),
  local_path_topic_(declare_parameter<std::string>(
      "local_path_topic", "/planning/local_waypoints")),
  local_validity_topic_(declare_parameter<std::string>(
      "local_validity_topic", "/planning/local_path_valid")),
  global_path_topic_(declare_parameter<std::string>(
      "global_path_topic", "/planning/global_path_waypoints")),
  global_validity_topic_(declare_parameter<std::string>(
      "global_validity_topic", "/planning/global_path_valid")),
  odometry_topic_(declare_parameter<std::string>(
      "odometry_topic", "/localization/ego_odom")),
  selected_path_topic_(declare_parameter<std::string>(
      "selected_path_topic", "/path_waypoints")),
  selected_path_viz_topic_(declare_parameter<std::string>(
      "selected_path_viz_topic", "/path_waypoints/path")),
  selected_validity_topic_(declare_parameter<std::string>(
      "selected_validity_topic", "/planning/selected_path_valid")),
  handoff_ready_topic_(declare_parameter<std::string>(
      "handoff_ready_topic", "/planning/global_handoff_ready")),
  debug_topic_(declare_parameter<std::string>(
      "debug_topic", "/planning/hyu_path_selector/debug")),
  selection_policy_(declare_parameter<double>("path_source_timeout_sec", 0.5)),
  continuity_check_(declareContinuityThresholds(*this))
{
  const double heartbeat_hz = declare_parameter<double>("heartbeat_hz", 10.0);
  if (!std::isfinite(heartbeat_hz) || heartbeat_hz <= 0.0) {
    throw std::invalid_argument("heartbeat_hz must be positive");
  }

  const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto odometry_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();

  path_source_sub_ = create_subscription<std_msgs::msg::String>(
    path_source_topic_, reliable_qos,
    std::bind(&PathSelectorNode::onPathSource, this, std::placeholders::_1));
  local_path_sub_ = create_subscription<eufs_msgs::msg::WaypointArrayStamped>(
    local_path_topic_, reliable_qos,
    std::bind(&PathSelectorNode::onLocalPath, this, std::placeholders::_1));
  global_path_sub_ = create_subscription<eufs_msgs::msg::WaypointArrayStamped>(
    global_path_topic_, reliable_qos,
    std::bind(&PathSelectorNode::onGlobalPath, this, std::placeholders::_1));
  local_validity_sub_ = create_subscription<std_msgs::msg::Bool>(
    local_validity_topic_, reliable_qos,
    std::bind(&PathSelectorNode::onLocalValidity, this, std::placeholders::_1));
  global_validity_sub_ = create_subscription<std_msgs::msg::Bool>(
    global_validity_topic_, reliable_qos,
    std::bind(&PathSelectorNode::onGlobalValidity, this, std::placeholders::_1));
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_, odometry_qos,
    std::bind(&PathSelectorNode::onOdometry, this, std::placeholders::_1));

  selected_path_pub_ = create_publisher<eufs_msgs::msg::WaypointArrayStamped>(
    selected_path_topic_, reliable_qos);
  selected_path_viz_pub_ = create_publisher<nav_msgs::msg::Path>(
    selected_path_viz_topic_, reliable_qos);
  selected_validity_pub_ = create_publisher<std_msgs::msg::Bool>(
    selected_validity_topic_, reliable_qos);
  handoff_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
    handoff_ready_topic_, reliable_qos);
  debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, reliable_qos);

  heartbeat_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / heartbeat_hz)),
    std::bind(&PathSelectorNode::publishHeartbeat, this));

  RCLCPP_INFO(
    get_logger(),
    "Strict selector ready: source=%s local=%s global=%s output=%s",
    path_source_topic_.c_str(), local_path_topic_.c_str(), global_path_topic_.c_str(),
    selected_path_topic_.c_str());
}

void PathSelectorNode::onPathSource(const std_msgs::msg::String::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.requested_source = message->data;
  state_.source_receive_time_sec = steadyNowSec();
  if (message->data == "LOCAL") {
    state_.global_entry_handoff_consumed = false;
  }
}

void PathSelectorNode::onLocalPath(
  const eufs_msgs::msg::WaypointArrayStamped::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.local.path = *message;
  state_.local.path_receive_time_sec = steadyNowSec();
}

void PathSelectorNode::onGlobalPath(
  const eufs_msgs::msg::WaypointArrayStamped::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.global.path = *message;
  state_.global.path_receive_time_sec = steadyNowSec();
}

void PathSelectorNode::onLocalValidity(const std_msgs::msg::Bool::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.local.validity_received = true;
  state_.local.validity = message->data;
  state_.local.validity_receive_time_sec = steadyNowSec();
}

void PathSelectorNode::onGlobalValidity(const std_msgs::msg::Bool::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.global.validity_received = true;
  state_.global.validity = message->data;
  state_.global.validity_receive_time_sec = steadyNowSec();
}

void PathSelectorNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.odometry.odometry = *message;
  state_.odometry.receive_time_sec = steadyNowSec();
}

void PathSelectorNode::publishHeartbeat()
{
  NodeState state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = state_;
  }

  const double receive_now_sec = steadyNowSec();
  const double stamp_now_sec = now().seconds();
  const auto local_validation = continuity_check_.validateCandidate(
    state.local, receive_now_sec, stamp_now_sec);
  const auto global_validation = continuity_check_.validateCandidate(
    state.global, receive_now_sec, stamp_now_sec);
  const auto odometry_validation = continuity_check_.validateOdometry(
    state.odometry, receive_now_sec, stamp_now_sec);
  const auto continuity = continuity_check_.evaluate(
    ContinuityInputs{
      state.local, state.global, state.odometry, receive_now_sec, stamp_now_sec});

  TrimResult local_trimmed;
  TrimResult global_trimmed;
  if (local_validation.ready && odometry_validation.ready) {
    local_trimmed = continuity_check_.trimAtEgoNearestPoint(
      *state.local.path, *state.odometry.odometry);
  }
  if (global_validation.ready && odometry_validation.ready) {
    global_trimmed = continuity_check_.trimAtEgoNearestPoint(
      *state.global.path, *state.odometry.odometry);
  }

  const auto decision = selection_policy_.decide(
    SelectionInputs{
      state.requested_source,
      state.source_receive_time_sec,
      receive_now_sec,
      local_validation.ready && odometry_validation.ready && local_trimmed.success(),
      global_validation.ready && odometry_validation.ready && global_trimmed.success(),
      continuity.ready,
      state.global_entry_handoff_consumed});

  const eufs_msgs::msg::WaypointArrayStamped * selected_path = nullptr;
  if (decision.valid() && decision.selected_candidate == SelectedCandidate::Local) {
    selected_path = &*local_trimmed.path;
  }
  if (decision.valid() && decision.selected_candidate == SelectedCandidate::Global) {
    selected_path = &*global_trimmed.path;
  }
  const bool selected_valid = selected_path != nullptr;

  if (selected_valid && decision.selected_candidate == SelectedCandidate::Global &&
    !state.global_entry_handoff_consumed)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_.requested_source == state.requested_source) {
      state_.global_entry_handoff_consumed = true;
    }
  }

  std_msgs::msg::Bool handoff_message;
  handoff_message.data = continuity.ready;
  handoff_ready_pub_->publish(handoff_message);

  if (selected_valid) {
    selected_path_pub_->publish(*selected_path);
    selected_path_viz_pub_->publish(continuity_check_.toPath(*selected_path));
  }

  std_msgs::msg::Bool validity_message;
  validity_message.data = selected_valid;
  selected_validity_pub_->publish(validity_message);

  std_msgs::msg::String debug_message;
  debug_message.data = makeDiagnostic(state, decision, continuity, selected_valid);
  debug_pub_->publish(debug_message);
}

double PathSelectorNode::steadyNowSec()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string PathSelectorNode::makeDiagnostic(
  const NodeState & state,
  const SelectionDecision & decision,
  const ContinuityResult & continuity,
  bool selected_valid) const
{
  std::ostringstream output;
  output << "requested_source=" << state.requested_source.value_or("MISSING")
         << " parsed_source=" << toString(decision.requested_source)
         << " selected_candidate=" << toString(decision.selected_candidate)
         << " selected_valid=" << (selected_valid ? "true" : "false")
         << " selection_failure=" << toString(decision.failure)
         << " global_handoff_ready=" << (continuity.ready ? "true" : "false")
         << " continuity_failure=" << toString(continuity.failure)
         << " local_candidate_failure=" << toString(continuity.local_candidate_failure)
         << " global_candidate_failure=" << toString(continuity.global_candidate_failure)
         << " odometry_failure=" << toString(continuity.odometry_failure)
         << " local_trim_failure=" << toString(continuity.local_trim_failure)
         << " global_trim_failure=" << toString(continuity.global_trim_failure)
         << " common_length_m=" << continuity.common_forward_length_m
         << " start_separation_m=" << continuity.start_separation_m
         << " heading_difference_rad=" << continuity.heading_difference_rad;
  return output.str();
}

}
