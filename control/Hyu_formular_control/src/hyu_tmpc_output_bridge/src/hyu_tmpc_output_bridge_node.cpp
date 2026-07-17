#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "hyu_tmpc_msgs/msg/tum_mpc_output.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "hyu_tmpc_output_bridge/command_conversion.hpp"

namespace hyu_tmpc_output_bridge
{

class TumMpcOutputBridgeNode final : public rclcpp::Node
{
public:
  TumMpcOutputBridgeNode()
  : Node("hyu_tmpc_output_bridge")
  {
    input_topic_ = declare_parameter<std::string>(
      "hyu_tmpc_output_bridge/input_topic", "/output");
    output_topic_ = declare_parameter<std::string>(
      "hyu_tmpc_output_bridge/output_topic", "/tmpc/cmd_shadow");
    valid_topic_ = declare_parameter<std::string>(
      "hyu_tmpc_output_bridge/valid_topic", "/tmpc/cmd_valid");
    config_.conversion_mass_kg = declare_parameter<double>(
      "hyu_tmpc_output_bridge/conversion_mass_kg", config_.conversion_mass_kg);
    config_.steering_min_rad = declare_parameter<double>(
      "hyu_tmpc_output_bridge/steering_min_rad", config_.steering_min_rad);
    config_.steering_max_rad = declare_parameter<double>(
      "hyu_tmpc_output_bridge/steering_max_rad", config_.steering_max_rad);
    config_.acceleration_min_mps2 = declare_parameter<double>(
      "hyu_tmpc_output_bridge/acceleration_min_mps2", config_.acceleration_min_mps2);
    config_.acceleration_max_mps2 = declare_parameter<double>(
      "hyu_tmpc_output_bridge/acceleration_max_mps2", config_.acceleration_max_mps2);
    config_.safe_brake_mps2 = declare_parameter<double>(
      "hyu_tmpc_output_bridge/safe_brake_mps2", config_.safe_brake_mps2);
    config_.output_timeout_sec = declare_parameter<double>(
      "hyu_tmpc_output_bridge/output_timeout_sec", config_.output_timeout_sec);
    config_.steering_reject_factor = declare_parameter<double>(
      "hyu_tmpc_output_bridge/steering_reject_factor", config_.steering_reject_factor);
    publish_rate_hz_ = declare_parameter<double>(
      "hyu_tmpc_output_bridge/publish_rate_hz", 100.0);

    if (input_topic_.empty() || output_topic_.empty() || valid_topic_.empty()) {
      throw std::invalid_argument("input_topic, output_topic, and valid_topic must not be empty");
    }
    if (!IsConversionConfigValid(config_)) {
      throw std::invalid_argument("invalid TMPC output bridge conversion parameters");
    }
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be finite and positive");
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    command_publisher_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(output_topic_, qos);
    valid_publisher_ = create_publisher<std_msgs::msg::Bool>(valid_topic_, qos);
    output_subscription_ = create_subscription<hyu_tmpc_msgs::msg::TumMpcOutput>(
      input_topic_, qos,
      std::bind(&TumMpcOutputBridgeNode::OnMpcOutput, this, std::placeholders::_1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    // Node-clock timer so the 100 Hz loop and freshness ages follow sim time
    // under use_sim_time (a wall timer would false-trigger the staleness gate
    // whenever RTF < 1 and lock the bridge into safe-brake).
    timer_ = rclcpp::create_timer(
      this, get_clock(), period, std::bind(&TumMpcOutputBridgeNode::OnTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "TMPC output bridge started: %s -> %s, valid=%s, conversion_mass_kg=%.3f",
      input_topic_.c_str(), output_topic_.c_str(), valid_topic_.c_str(),
      config_.conversion_mass_kg);
  }

private:
  void OnMpcOutput(const hyu_tmpc_msgs::msg::TumMpcOutput::SharedPtr message)
  {
    MpcCommand input;
    input.steering_angle_rad = message->request_steering_angle_rad;
    input.long_force_n = message->request_long_force_n;
    input.tube_mpc_status = message->tube_mpc_status;

    const auto stamp = get_clock()->now();
    std::lock_guard<std::mutex> lock(input_mutex_);
    latest_input_ = input;
    latest_input_time_ = stamp;
    has_input_ = true;
  }

  void OnTimer()
  {
    MpcCommand input;
    rclcpp::Time input_time;
    bool has_input = false;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      input = latest_input_;
      input_time = latest_input_time_;
      has_input = has_input_;
    }

    const auto now = get_clock()->now();
    const double age_sec = has_input ? (now - input_time).seconds() : 0.0;
    const auto result = BuildCommand(input, has_input, age_sec, config_);

    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = get_clock()->now();
    command.drive.speed = result.command.speed_mps;
    command.drive.acceleration = result.command.acceleration_mps2;
    command.drive.steering_angle = result.command.steering_angle_rad;
    command_publisher_->publish(command);

    std_msgs::msg::Bool valid;
    valid.data = result.valid();
    valid_publisher_->publish(valid);

    if (!has_last_state_ || result.state != last_state_) {
      if (result.valid()) {
        RCLCPP_INFO(get_logger(), "TMPC command state: valid");
      } else {
        RCLCPP_WARN(
          get_logger(), "TMPC command state: %s; publishing safe brake",
          CommandStateName(result.state));
      }
      last_state_ = result.state;
      has_last_state_ = true;
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string valid_topic_;
  ConversionConfig config_;
  double publish_rate_hz_{100.0};

  std::mutex input_mutex_;
  MpcCommand latest_input_{};
  rclcpp::Time latest_input_time_{0, 0, RCL_ROS_TIME};
  bool has_input_{false};

  CommandState last_state_{CommandState::kNoInput};
  bool has_last_state_{false};

  rclcpp::Subscription<hyu_tmpc_msgs::msg::TumMpcOutput>::SharedPtr
    output_subscription_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hyu_tmpc_output_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hyu_tmpc_output_bridge::TumMpcOutputBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
