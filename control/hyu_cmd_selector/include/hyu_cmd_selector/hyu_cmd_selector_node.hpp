#ifndef TMPC_CMD_SELECTOR__TMPC_CMD_SELECTOR_NODE_HPP_
#define TMPC_CMD_SELECTOR__TMPC_CMD_SELECTOR_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "hyu_cmd_selector/selector_policy.hpp"

namespace hyu_cmd_selector
{

class TmpcCmdSelectorNode final : public rclcpp::Node
{
public:
  explicit TmpcCmdSelectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using AckermannCommand = ackermann_msgs::msg::AckermannDriveStamped;

  void OnPlanningState(const std_msgs::msg::String::SharedPtr message);
  void OnStopRequest(const std_msgs::msg::Bool::SharedPtr message);
  void OnLocalCommand(const AckermannCommand::SharedPtr message);
  void OnTmpcCommand(const AckermannCommand::SharedPtr message);
  void OnTmpcValid(const std_msgs::msg::Bool::SharedPtr message);
  void OnTimer();
  // Run the selection policy and publish the chosen command + status. Invoked
  // both by the heartbeat timer and immediately whenever a fresh candidate
  // command arrives, so a new Pure Pursuit / TMPC command reaches /cmd without
  // waiting up to a full timer period. Under a coarse sim /clock (10 Hz) that
  // resampling wait was ~100 ms of pure latency on top of the plant's control
  // delay -- enough to blow the first corner and corrupt the map. The default
  // single-threaded executor serializes this with the timer, so policy_ needs
  // no extra locking.
  //
  // Publish discipline: the selector is a 1:1 RELAY, not a resampler. A
  // forwarded source command is published exactly once per received message
  // (tracked by sequence number), plus once immediately on a source switch, so
  // in LOCAL the /cmd stream is message-for-message identical to Pure Pursuit
  // publishing /cmd directly (the proven plain-trackdrive path). Only the
  // safe-brake synthesizes at the timer rate -- it has no upstream message
  // stream and the plant's 1 s command-staleness rule needs a heartbeat.
  void EvaluateAndPublish();

  static double AgeSeconds(
    bool present, const rclcpp::Time & received, const rclcpp::Time & now);
  static bool IsCommandFinite(const AckermannCommand & command);
  AckermannCommand SafeBrakeCommand() const;

  std::string planning_state_topic_;
  std::string stop_request_topic_;
  std::string local_command_topic_;
  std::string tmpc_command_topic_;
  std::string tmpc_valid_topic_;
  std::string output_topic_;
  std::string status_topic_;
  double publish_rate_hz_{100.0};
  double safe_brake_mps2_{-5.0};

  SelectorConfig selector_config_;
  SelectorPolicy policy_;

  mutable std::mutex data_mutex_;
  std::string planning_state_;
  bool stop_requested_{false};
  AckermannCommand local_command_;
  AckermannCommand tmpc_command_;
  // Receive counters and relay bookkeeping: a stored command is forwarded at
  // most once per received message (see EvaluateAndPublish).
  uint64_t local_command_seq_{0U};
  uint64_t tmpc_command_seq_{0U};
  uint64_t last_forwarded_local_seq_{0U};
  uint64_t last_forwarded_tmpc_seq_{0U};
  CommandSource last_published_source_{CommandSource::kSafeBrake};
  bool has_published_{false};
  bool tmpc_valid_{false};
  rclcpp::Time planning_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stop_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time local_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time tmpc_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time tmpc_valid_time_{0, 0, RCL_ROS_TIME};
  bool has_planning_state_{false};
  bool has_stop_request_{false};
  bool has_local_command_{false};
  bool has_tmpc_command_{false};
  bool has_tmpc_valid_{false};

  SelectorStatus last_status_{SelectorStatus::kInputBrake};
  bool has_last_status_{false};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planning_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_request_sub_;
  rclcpp::Subscription<AckermannCommand>::SharedPtr local_command_sub_;
  rclcpp::Subscription<AckermannCommand>::SharedPtr tmpc_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tmpc_valid_sub_;
  rclcpp::Publisher<AckermannCommand>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hyu_cmd_selector

#endif  // TMPC_CMD_SELECTOR__TMPC_CMD_SELECTOR_NODE_HPP_
