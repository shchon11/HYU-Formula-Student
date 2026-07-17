#include <memory>

#include <gtest/gtest.h>

#include <std_msgs/msg/bool.hpp>

#include "hyu_local_planner/hyu_local_planner_node.hpp"
#include "hyu_local_planner_validity_test_support.hpp"

namespace hyu_local_planner
{
namespace
{

using test_support::ConeArray;
using test_support::Observations;
using test_support::Odometry;
using test_support::ShutdownGuard;
using test_support::cones;
using test_support::currentStamp;
using test_support::odom;
using test_support::spinUntil;

TEST(LocalPlannerValidity, ValidPathSurvivesUnmatchedRawConeAndOdomCallbacks)
{
  rclcpp::init(0, nullptr);
  ShutdownGuard shutdown_guard;

  rclcpp::NodeOptions planner_options;
  planner_options.append_parameter_override("source_mode", "live_cones");
  planner_options.append_parameter_override("cones_topic", "/t9_hyu_local_planner/cones");
  planner_options.append_parameter_override("odom_topic", "/t9_hyu_local_planner/odom");
  planner_options.append_parameter_override(
    "waypoints_topic", "/t9_hyu_local_planner/waypoints");
  planner_options.append_parameter_override("path_topic", "/t9_hyu_local_planner/path");
  planner_options.append_parameter_override(
    "validity_topic", "/t9_hyu_local_planner/validity");
  planner_options.append_parameter_override("heartbeat_hz", 50.0);

  auto planner = std::make_shared<LocalPlannerNode>(planner_options);
  auto driver = std::make_shared<rclcpp::Node>("hyu_local_planner_validity_driver");
  auto cones_publisher = driver->create_publisher<ConeArray>(
    "/t9_hyu_local_planner/cones", rclcpp::SensorDataQoS());
  auto odom_publisher = driver->create_publisher<Odometry>(
    "/t9_hyu_local_planner/odom", rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

  Observations observations;
  const auto waypoint_subscription = driver->create_subscription<
    hyu_msgs::msg::WaypointArrayStamped>(
    "/t9_hyu_local_planner/waypoints", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [&observations](const hyu_msgs::msg::WaypointArrayStamped::SharedPtr message) {
      if (!message->waypoints.empty()) {
        ++observations.path_count;
      }
    });
  const auto validity_subscription = driver->create_subscription<std_msgs::msg::Bool>(
    "/t9_hyu_local_planner/validity", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [&observations](const std_msgs::msg::Bool::SharedPtr message) {
      observations.validity_values.push_back(message->data);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner);
  executor.add_node(driver);
  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      return cones_publisher->get_subscription_count() > 0U &&
             odom_publisher->get_subscription_count() > 0U;
    }, std::chrono::milliseconds(1000)));
  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      return waypoint_subscription->get_publisher_count() > 0U &&
             validity_subscription->get_publisher_count() > 0U;
    }, std::chrono::milliseconds(1000))) << "local planner outputs did not connect";

  const auto valid_stamp = currentStamp(*driver);
  auto valid_cones = cones(0);
  valid_cones.header.stamp = valid_stamp;
  auto valid_odom = odom(0);
  valid_odom.header.stamp = valid_stamp;
  const auto publish_valid_pair = [&]() {
    cones_publisher->publish(valid_cones);
    odom_publisher->publish(valid_odom);
  };
  publish_valid_pair();
  const auto valid_path_seen = spinUntil(
    executor,
    [&]() {
      if (observations.path_count > 0U &&
        !observations.validity_values.empty() && observations.validity_values.back())
      {
        return true;
      }
      publish_valid_pair();
      return false;
    }, std::chrono::milliseconds(2000));
  ASSERT_TRUE(valid_path_seen)
    << "valid local path never reached true heartbeat; paths=" << observations.path_count
    << " heartbeats=" << observations.validity_values.size();

  const auto validity_count_before_raw = observations.validity_values.size();
  auto unmatched_cones = cones(0);
  unmatched_cones.header.stamp.sec = valid_stamp.sec + 1;
  auto unmatched_odom = odom(0);
  unmatched_odom.header.stamp.sec = valid_stamp.sec + 2;
  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      if (observations.validity_values.size() > validity_count_before_raw) {
        return true;
      }
      cones_publisher->publish(unmatched_cones);
      odom_publisher->publish(unmatched_odom);
      return false;
    },
    std::chrono::milliseconds(250)));

  EXPECT_TRUE(observations.validity_values.back())
    << "unmatched raw cone/odom callbacks cleared a fresh valid local path";
}

TEST(LocalPlannerValidity, SingleInvalidBuildRetainsFreshPathUntilInputAgeExpires)
{
  rclcpp::init(0, nullptr);
  ShutdownGuard shutdown_guard;

  rclcpp::NodeOptions planner_options;
  planner_options.append_parameter_override("source_mode", "live_cones");
  planner_options.append_parameter_override("cones_topic", "/t9_retention/cones");
  planner_options.append_parameter_override("odom_topic", "/t9_retention/odom");
  planner_options.append_parameter_override("waypoints_topic", "/t9_retention/waypoints");
  planner_options.append_parameter_override("path_topic", "/t9_retention/path");
  planner_options.append_parameter_override("validity_topic", "/t9_retention/validity");
  planner_options.append_parameter_override("max_input_age_sec", 0.5);
  planner_options.append_parameter_override("heartbeat_hz", 50.0);

  auto planner = std::make_shared<LocalPlannerNode>(planner_options);
  auto driver = std::make_shared<rclcpp::Node>("hyu_local_planner_retention_driver");
  auto cones_publisher = driver->create_publisher<ConeArray>(
    "/t9_retention/cones", rclcpp::SensorDataQoS());
  auto odom_publisher = driver->create_publisher<Odometry>(
    "/t9_retention/odom", rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

  Observations observations;
  const auto waypoint_subscription = driver->create_subscription<
    hyu_msgs::msg::WaypointArrayStamped>(
    "/t9_retention/waypoints", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [&observations](const hyu_msgs::msg::WaypointArrayStamped::SharedPtr message) {
      if (!message->waypoints.empty()) {
        ++observations.path_count;
      }
    });
  const auto validity_subscription = driver->create_subscription<std_msgs::msg::Bool>(
    "/t9_retention/validity", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [&observations](const std_msgs::msg::Bool::SharedPtr message) {
      observations.validity_values.push_back(message->data);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner);
  executor.add_node(driver);
  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      return cones_publisher->get_subscription_count() > 0U &&
             odom_publisher->get_subscription_count() > 0U &&
             waypoint_subscription->get_publisher_count() > 0U &&
             validity_subscription->get_publisher_count() > 0U;
    }, std::chrono::milliseconds(1000)));

  const auto valid_stamp = currentStamp(*driver);
  auto valid_cones = cones(0);
  valid_cones.header.stamp = valid_stamp;
  auto valid_odom = odom(0);
  valid_odom.header.stamp = valid_stamp;
  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      if (observations.path_count > 0U && !observations.validity_values.empty() &&
        observations.validity_values.back())
      {
        return true;
      }
      cones_publisher->publish(valid_cones);
      odom_publisher->publish(valid_odom);
      return false;
    }, std::chrono::milliseconds(2000)));

  const auto heartbeat_count_before_invalid = observations.validity_values.size();
  const auto invalid_stamp = currentStamp(*driver);
  auto invalid_cones = cones(0);
  invalid_cones.header.stamp = invalid_stamp;
  invalid_cones.blue_cones.clear();
  invalid_cones.yellow_cones.clear();
  auto invalid_odom = odom(0);
  invalid_odom.header.stamp = invalid_stamp;
  cones_publisher->publish(invalid_cones);
  odom_publisher->publish(invalid_odom);

  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      return observations.validity_values.size() > heartbeat_count_before_invalid;
    }, std::chrono::milliseconds(200)));
  EXPECT_TRUE(observations.validity_values.back())
    << "a single invalid local-path build cleared a still-fresh path";

  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      return !observations.validity_values.empty() && !observations.validity_values.back();
    }, std::chrono::milliseconds(500)))
    << "local-path validity remained true after the configured input-age timeout";
}

}
}
