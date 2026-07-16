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
#include "tmpc_cmd_selector/selector_policy.hpp"

namespace tmpc_cmd_selector
{

class TmpcCmdSelectorNode final : public rclcpp::Node
{
public:
  explicit TmpcCmdSelectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using AckermannCommand = ackermann_msgs::msg::AckermannDriveStamped;
  using SteadyClock = std::chrono::steady_clock;

  void OnPlanningState(const std_msgs::msg::String::SharedPtr message);
  void OnStopRequest(const std_msgs::msg::Bool::SharedPtr message);
  void OnLocalCommand(const AckermannCommand::SharedPtr message);
  void OnTmpcCommand(const AckermannCommand::SharedPtr message);
  void OnTmpcValid(const std_msgs::msg::Bool::SharedPtr message);
  void OnTimer();

  static double SteadySeconds(const SteadyClock::time_point & time);
  static double AgeSeconds(
    bool present, const SteadyClock::time_point & received, const SteadyClock::time_point & now);
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

  SelectorPolicy policy_;

  mutable std::mutex data_mutex_;
  std::string planning_state_;
  bool stop_requested_{false};
  AckermannCommand local_command_;
  AckermannCommand tmpc_command_;
  bool tmpc_valid_{false};
  SteadyClock::time_point planning_state_time_{};
  SteadyClock::time_point stop_request_time_{};
  SteadyClock::time_point local_command_time_{};
  SteadyClock::time_point tmpc_command_time_{};
  SteadyClock::time_point tmpc_valid_time_{};
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

}  // namespace tmpc_cmd_selector

#endif  // TMPC_CMD_SELECTOR__TMPC_CMD_SELECTOR_NODE_HPP_
