#include "hyu_cmd_selector/hyu_cmd_selector_node.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

namespace hyu_cmd_selector
{

namespace
{

SelectorConfig DeclareSelectorConfig(rclcpp::Node & node)
{
  SelectorConfig config;
  config.state_timeout_sec = node.declare_parameter<double>(
    "state_timeout_sec", config.state_timeout_sec);
  config.stop_timeout_sec = node.declare_parameter<double>(
    "stop_timeout_sec", config.stop_timeout_sec);
  config.local_command_timeout_sec = node.declare_parameter<double>(
    "local_command_timeout_sec", config.local_command_timeout_sec);
  config.tmpc_command_timeout_sec = node.declare_parameter<double>(
    "tmpc_command_timeout_sec", config.tmpc_command_timeout_sec);
  config.tmpc_valid_timeout_sec = node.declare_parameter<double>(
    "tmpc_valid_timeout_sec", config.tmpc_valid_timeout_sec);
  config.tmpc_ready_dwell_sec = node.declare_parameter<double>(
    "tmpc_ready_dwell_sec", config.tmpc_ready_dwell_sec);
  config.max_steering_disagreement_rad = node.declare_parameter<double>(
    "max_steering_disagreement_rad", config.max_steering_disagreement_rad);
  return config;
}

}  // namespace

TmpcCmdSelectorNode::TmpcCmdSelectorNode(const rclcpp::NodeOptions & options)
: Node("hyu_cmd_selector", options),
  planning_state_topic_(declare_parameter<std::string>(
      "planning_state_topic", "/planning/state")),
  stop_request_topic_(declare_parameter<std::string>(
      "stop_request_topic", "/planning/stop_request")),
  local_command_topic_(declare_parameter<std::string>(
      "local_command_topic", "/control/pp/cmd")),
  tmpc_command_topic_(declare_parameter<std::string>(
      "tmpc_command_topic", "/control/tmpc/cmd")),
  tmpc_valid_topic_(declare_parameter<std::string>(
      "tmpc_valid_topic", "/control/tmpc/valid")),
  output_topic_(declare_parameter<std::string>("output_topic", "/vehicle/cmd")),
  status_topic_(declare_parameter<std::string>(
      "status_topic", "/control/selector/status")),
  publish_rate_hz_(declare_parameter<double>("publish_rate_hz", 100.0)),
  safe_brake_mps2_(declare_parameter<double>("safe_brake_mps2", -5.0)),
  selector_config_(DeclareSelectorConfig(*this)),
  policy_(selector_config_)
{
  if (planning_state_topic_.empty() || stop_request_topic_.empty() ||
    local_command_topic_.empty() || tmpc_command_topic_.empty() ||
    tmpc_valid_topic_.empty() || output_topic_.empty() || status_topic_.empty())
  {
    throw std::invalid_argument("selector topic names must not be empty");
  }
  if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 ||
    !std::isfinite(safe_brake_mps2_) || safe_brake_mps2_ > 0.0)
  {
    throw std::invalid_argument("publish rate must be positive and safe brake non-positive");
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  planning_state_sub_ = create_subscription<std_msgs::msg::String>(
    planning_state_topic_, qos,
    std::bind(&TmpcCmdSelectorNode::OnPlanningState, this, std::placeholders::_1));
  stop_request_sub_ = create_subscription<std_msgs::msg::Bool>(
    stop_request_topic_, qos,
    std::bind(&TmpcCmdSelectorNode::OnStopRequest, this, std::placeholders::_1));
  local_command_sub_ = create_subscription<AckermannCommand>(
    local_command_topic_, qos,
    std::bind(&TmpcCmdSelectorNode::OnLocalCommand, this, std::placeholders::_1));
  tmpc_command_sub_ = create_subscription<AckermannCommand>(
    tmpc_command_topic_, qos,
    std::bind(&TmpcCmdSelectorNode::OnTmpcCommand, this, std::placeholders::_1));
  tmpc_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
    tmpc_valid_topic_, qos,
    std::bind(&TmpcCmdSelectorNode::OnTmpcValid, this, std::placeholders::_1));

  output_pub_ = create_publisher<AckermannCommand>(output_topic_, qos);
  status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, qos);

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_rate_hz_));
  // Node-clock timer: freshness ages and the publish cadence must follow sim
  // time under use_sim_time, or RTF < 1 false-triggers the safety timeouts.
  timer_ = rclcpp::create_timer(
    this, get_clock(), period, std::bind(&TmpcCmdSelectorNode::OnTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "TMPC command selector started: LOCAL=%s GLOBAL=%s -> %s, status=%s",
    local_command_topic_.c_str(), tmpc_command_topic_.c_str(), output_topic_.c_str(),
    status_topic_.c_str());
}

void TmpcCmdSelectorNode::OnPlanningState(const std_msgs::msg::String::SharedPtr message)
{
  const auto stamp = get_clock()->now();
  std::lock_guard<std::mutex> lock(data_mutex_);
  planning_state_ = message->data;
  planning_state_time_ = stamp;
  has_planning_state_ = true;
}

void TmpcCmdSelectorNode::OnStopRequest(const std_msgs::msg::Bool::SharedPtr message)
{
  const auto stamp = get_clock()->now();
  std::lock_guard<std::mutex> lock(data_mutex_);
  stop_requested_ = message->data;
  stop_request_time_ = stamp;
  has_stop_request_ = true;
}

void TmpcCmdSelectorNode::OnLocalCommand(const AckermannCommand::SharedPtr message)
{
  const auto stamp = get_clock()->now();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    local_command_ = *message;
    local_command_time_ = stamp;
    has_local_command_ = true;
    ++local_command_seq_;
  }
  // Forward the fresh command now instead of at the next timer tick.
  EvaluateAndPublish();
}

void TmpcCmdSelectorNode::OnTmpcCommand(const AckermannCommand::SharedPtr message)
{
  const auto stamp = get_clock()->now();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    tmpc_command_ = *message;
    tmpc_command_time_ = stamp;
    has_tmpc_command_ = true;
    ++tmpc_command_seq_;
  }
  EvaluateAndPublish();
}

void TmpcCmdSelectorNode::OnTmpcValid(const std_msgs::msg::Bool::SharedPtr message)
{
  const auto stamp = get_clock()->now();
  std::lock_guard<std::mutex> lock(data_mutex_);
  tmpc_valid_ = message->data;
  tmpc_valid_time_ = stamp;
  has_tmpc_valid_ = true;
}

void TmpcCmdSelectorNode::OnTimer()
{
  EvaluateAndPublish();
}

void TmpcCmdSelectorNode::EvaluateAndPublish()
{
  const auto current_time = get_clock()->now();
  SelectorInputs inputs;
  AckermannCommand local_command;
  AckermannCommand tmpc_command;
  uint64_t local_seq = 0U;
  uint64_t tmpc_seq = 0U;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (has_planning_state_) {
      inputs.planning_state = planning_state_;
    }
    inputs.state_age_sec = AgeSeconds(
      has_planning_state_, planning_state_time_, current_time);
    inputs.has_stop_request = has_stop_request_;
    inputs.stop_requested = stop_requested_;
    inputs.stop_age_sec = AgeSeconds(has_stop_request_, stop_request_time_, current_time);
    inputs.has_local_command = has_local_command_;
    inputs.local_command_age_sec = AgeSeconds(
      has_local_command_, local_command_time_, current_time);
    inputs.has_tmpc_command = has_tmpc_command_;
    inputs.tmpc_command_age_sec = AgeSeconds(
      has_tmpc_command_, tmpc_command_time_, current_time);
    inputs.has_tmpc_valid = has_tmpc_valid_;
    inputs.tmpc_valid = tmpc_valid_;
    inputs.tmpc_valid_age_sec = AgeSeconds(
      has_tmpc_valid_, tmpc_valid_time_, current_time);
    local_command = local_command_;
    tmpc_command = tmpc_command_;
    local_seq = local_command_seq_;
    tmpc_seq = tmpc_command_seq_;
  }
  inputs.now_sec = current_time.seconds();
  inputs.local_command_valid = IsCommandFinite(local_command);
  inputs.tmpc_command_valid = IsCommandFinite(tmpc_command);
  const bool local_fresh_for_reference = inputs.has_local_command &&
    inputs.local_command_valid &&
    inputs.local_command_age_sec <= selector_config_.local_command_timeout_sec;
  if (local_fresh_for_reference && inputs.has_tmpc_command && inputs.tmpc_command_valid) {
    inputs.has_steering_disagreement = true;
    inputs.steering_disagreement_rad = std::abs(
      tmpc_command.drive.steering_angle - local_command.drive.steering_angle);
  }

  const SelectorDecision decision = policy_.update(inputs);

  // Relay, don't resample: forward each source message exactly once (plus once
  // on a source switch so takeover/fallback is not held to the next message).
  // Timer ticks between messages therefore publish nothing while forwarding,
  // keeping the LOCAL /vehicle/cmd stream identical to Pure Pursuit driving /vehicle/cmd
  // directly. The synthesized safe brake has no upstream stream, so it heartbeats
  // at the evaluate cadence.
  const bool source_switched = !has_published_ || last_published_source_ != decision.source;
  bool publish_command = false;
  AckermannCommand output;
  if (decision.source == CommandSource::kPurePursuit) {
    if (source_switched || local_seq != last_forwarded_local_seq_) {
      output = std::move(local_command);
      last_forwarded_local_seq_ = local_seq;
      publish_command = true;
    }
  } else if (decision.source == CommandSource::kTmpc) {
    if (source_switched || tmpc_seq != last_forwarded_tmpc_seq_) {
      output = std::move(tmpc_command);
      last_forwarded_tmpc_seq_ = tmpc_seq;
      publish_command = true;
    }
  } else {
    output = SafeBrakeCommand();
    publish_command = true;
  }
  if (publish_command) {
    output.header.stamp = current_time;
    output_pub_->publish(output);
    last_published_source_ = decision.source;
    has_published_ = true;
  }

  std_msgs::msg::String status;
  status.data = ToString(decision.status);
  status_pub_->publish(status);

  if (!has_last_status_ || last_status_ != decision.status) {
    if (decision.source == CommandSource::kSafeBrake) {
      RCLCPP_WARN(
        get_logger(), "selector status: %s (output=%s)", ToString(decision.status),
        ToString(decision.source));
    } else {
      RCLCPP_INFO(
        get_logger(), "selector status: %s (output=%s)", ToString(decision.status),
        ToString(decision.source));
    }
    last_status_ = decision.status;
    has_last_status_ = true;
  }
}

double TmpcCmdSelectorNode::AgeSeconds(
  bool present, const rclcpp::Time & received, const rclcpp::Time & now)
{
  if (!present) {
    return std::numeric_limits<double>::infinity();
  }
  return (now - received).seconds();
}

bool TmpcCmdSelectorNode::IsCommandFinite(const AckermannCommand & command)
{
  return std::isfinite(command.drive.steering_angle) &&
         std::isfinite(command.drive.steering_angle_velocity) &&
         std::isfinite(command.drive.speed) &&
         std::isfinite(command.drive.acceleration) &&
         std::isfinite(command.drive.jerk);
}

TmpcCmdSelectorNode::AckermannCommand TmpcCmdSelectorNode::SafeBrakeCommand() const
{
  AckermannCommand command;
  command.drive.speed = 0.0;
  command.drive.acceleration = safe_brake_mps2_;
  command.drive.steering_angle = 0.0;
  return command;
}

}  // namespace hyu_cmd_selector
