#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tmpc_cmd_selector/tmpc_cmd_selector_node.hpp"

namespace
{

using namespace std::chrono_literals;
using AckermannCommand = ackermann_msgs::msg::AckermannDriveStamped;

class TmpcCmdSelectorNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    const std::vector<rclcpp::Parameter> overrides{
      rclcpp::Parameter("planning_state_topic", "/test_selector/planning_state"),
      rclcpp::Parameter("stop_request_topic", "/test_selector/stop_request"),
      rclcpp::Parameter("local_command_topic", "/test_selector/local_command"),
      rclcpp::Parameter("tmpc_command_topic", "/test_selector/tmpc_command"),
      rclcpp::Parameter("tmpc_valid_topic", "/test_selector/tmpc_valid"),
      rclcpp::Parameter("output_topic", "/test_selector/cmd"),
      rclcpp::Parameter("status_topic", "/test_selector/status"),
      rclcpp::Parameter("publish_rate_hz", 100.0),
      rclcpp::Parameter("state_timeout_sec", 0.25),
      rclcpp::Parameter("stop_timeout_sec", 0.25),
      rclcpp::Parameter("local_command_timeout_sec", 0.25),
      rclcpp::Parameter("tmpc_command_timeout_sec", 0.1),
      rclcpp::Parameter("tmpc_valid_timeout_sec", 0.1),
      rclcpp::Parameter("tmpc_ready_dwell_sec", 0.1)};
    selector_ = std::make_shared<tmpc_cmd_selector::TmpcCmdSelectorNode>(
      rclcpp::NodeOptions().parameter_overrides(overrides));
    driver_ = std::make_shared<rclcpp::Node>("tmpc_cmd_selector_test_driver");

    const auto qos = rclcpp::QoS(10).reliable().durability_volatile();
    state_pub_ = driver_->create_publisher<std_msgs::msg::String>(
      "/test_selector/planning_state", qos);
    stop_pub_ = driver_->create_publisher<std_msgs::msg::Bool>(
      "/test_selector/stop_request", qos);
    local_pub_ = driver_->create_publisher<AckermannCommand>(
      "/test_selector/local_command", qos);
    tmpc_pub_ = driver_->create_publisher<AckermannCommand>(
      "/test_selector/tmpc_command", qos);
    valid_pub_ = driver_->create_publisher<std_msgs::msg::Bool>(
      "/test_selector/tmpc_valid", qos);
    output_sub_ = driver_->create_subscription<AckermannCommand>(
      "/test_selector/cmd", qos,
      [this](AckermannCommand::ConstSharedPtr message) {
        ++output_count_;
        last_output_ = *message;
      });
    status_sub_ = driver_->create_subscription<std_msgs::msg::String>(
      "/test_selector/status", qos,
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        last_status_ = message->data;
      });

    executor_.add_node(selector_);
    executor_.add_node(driver_);
    spinFor(100ms, false);
  }

  void TearDown() override
  {
    executor_.remove_node(driver_);
    executor_.remove_node(selector_);
    driver_.reset();
    selector_.reset();
  }

  AckermannCommand LocalCommand() const
  {
    AckermannCommand command;
    command.drive.speed = 3.0;
    command.drive.acceleration = 1.0;
    command.drive.steering_angle = 0.1;
    return command;
  }

  AckermannCommand TmpcCommand() const
  {
    AckermannCommand command;
    command.drive.speed = 0.0;
    command.drive.acceleration = 0.8;
    command.drive.steering_angle = -0.15;
    return command;
  }

  void publishInputs(const std::string & state, bool tmpc_valid)
  {
    current_state_ = state;
    current_tmpc_valid_ = tmpc_valid;

    std_msgs::msg::String state_message;
    state_message.data = state;
    state_pub_->publish(state_message);
    std_msgs::msg::Bool stop_message;
    stop_message.data = false;
    stop_pub_->publish(stop_message);
    local_pub_->publish(LocalCommand());
    tmpc_pub_->publish(TmpcCommand());
    std_msgs::msg::Bool valid_message;
    valid_message.data = tmpc_valid;
    valid_pub_->publish(valid_message);
  }

  void spinFor(std::chrono::milliseconds duration, bool republish = true)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      if (republish) {
        publishInputs(current_state_, current_tmpc_valid_);
      }
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some();
  }

  bool spinUntilStatus(const std::string & expected, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (last_status_ != expected && std::chrono::steady_clock::now() < deadline) {
      publishInputs(current_state_, current_tmpc_valid_);
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    return last_status_ == expected;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<tmpc_cmd_selector::TmpcCmdSelectorNode> selector_;
  rclcpp::Node::SharedPtr driver_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<AckermannCommand>::SharedPtr local_pub_;
  rclcpp::Publisher<AckermannCommand>::SharedPtr tmpc_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Subscription<AckermannCommand>::SharedPtr output_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  std::string current_state_{"LOCAL"};
  bool current_tmpc_valid_{false};
  std::string last_status_;
  AckermannCommand last_output_;
  std::size_t output_count_{0U};
};

TEST_F(TmpcCmdSelectorNodeTest, SelectsCommandsAndFallsBackOnGlobalFault)
{
  EXPECT_EQ(driver_->count_publishers("/test_selector/cmd"), 1U);

  publishInputs("LOCAL", false);
  ASSERT_TRUE(spinUntilStatus("LOCAL_PP", 300ms));
  EXPECT_DOUBLE_EQ(last_output_.drive.speed, 3.0);
  EXPECT_FLOAT_EQ(last_output_.drive.steering_angle, 0.1F);

  publishInputs("GLOBAL", false);
  ASSERT_TRUE(spinUntilStatus("GLOBAL_WAITING_TMPC", 300ms));
  EXPECT_DOUBLE_EQ(last_output_.drive.speed, 3.0);

  const std::size_t count_before_dwell = output_count_;
  publishInputs("GLOBAL", true);
  spinFor(50ms);
  EXPECT_EQ(last_status_, "GLOBAL_WAITING_TMPC");
  ASSERT_TRUE(spinUntilStatus("GLOBAL_TMPC", 300ms));
  EXPECT_FLOAT_EQ(last_output_.drive.acceleration, 0.8F);
  EXPECT_FLOAT_EQ(last_output_.drive.steering_angle, -0.15F);
  EXPECT_GT(output_count_ - count_before_dwell, 8U);

  // A TMPC dropout mid-drive degrades to the fresh Pure Pursuit command.
  publishInputs("GLOBAL", false);
  ASSERT_TRUE(spinUntilStatus("GLOBAL_PP_FALLBACK", 300ms));
  EXPECT_DOUBLE_EQ(last_output_.drive.speed, 3.0);
  EXPECT_FLOAT_EQ(last_output_.drive.steering_angle, 0.1F);

  // Recovered validity retakes only after a full dwell.
  publishInputs("GLOBAL", true);
  ASSERT_TRUE(spinUntilStatus("GLOBAL_TMPC", 300ms));
  EXPECT_FLOAT_EQ(last_output_.drive.acceleration, 0.8F);

  publishInputs("LOCAL", false);
  ASSERT_TRUE(spinUntilStatus("LOCAL_PP", 300ms));
  // The relay publishes the switch command just before the status message;
  // drain one more round so the command subscription has delivered it too.
  spinFor(30ms);
  EXPECT_DOUBLE_EQ(last_output_.drive.speed, 3.0);
}

}  // namespace
