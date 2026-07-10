#include <chrono>
#include <memory>

#include <gtest/gtest.h>

#include <std_msgs/msg/bool.hpp>

#include "local_planner/local_planner_node.hpp"
#include "local_planner_validity_test_support.hpp"

namespace local_planner
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

TEST(LocalPlannerValidity, StaleMatchedPairInvalidatesThenFreshPairRecovers)
{
  rclcpp::init(0, nullptr);
  ShutdownGuard shutdown_guard;

  rclcpp::NodeOptions planner_options;
  planner_options.append_parameter_override("cones_topic", "/t9_stale_recovery/cones");
  planner_options.append_parameter_override("odom_topic", "/t9_stale_recovery/odom");
  planner_options.append_parameter_override(
    "waypoints_topic", "/t9_stale_recovery/waypoints");
  planner_options.append_parameter_override("path_topic", "/t9_stale_recovery/path");
  planner_options.append_parameter_override(
    "validity_topic", "/t9_stale_recovery/validity");
  planner_options.append_parameter_override("max_input_age_sec", 0.1);
  planner_options.append_parameter_override("heartbeat_hz", 50.0);

  auto planner = std::make_shared<LocalPlannerNode>(planner_options);
  auto driver = std::make_shared<rclcpp::Node>("local_planner_stale_recovery_driver");
  auto cones_publisher = driver->create_publisher<ConeArray>(
    "/t9_stale_recovery/cones", rclcpp::SensorDataQoS());
  auto odom_publisher = driver->create_publisher<Odometry>(
    "/t9_stale_recovery/odom", rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

  Observations observations;
  const auto waypoint_subscription = driver->create_subscription<
    eufs_msgs::msg::WaypointArrayStamped>(
    "/t9_stale_recovery/waypoints", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [&observations](const eufs_msgs::msg::WaypointArrayStamped::SharedPtr message) {
      if (!message->waypoints.empty()) {
        ++observations.path_count;
      }
    });
  const auto validity_subscription = driver->create_subscription<std_msgs::msg::Bool>(
    "/t9_stale_recovery/validity", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
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

  const auto publish_fresh_pair = [&](const builtin_interfaces::msg::Time & stamp) {
    auto fresh_cones = cones(0);
    fresh_cones.header.stamp = stamp;
    auto fresh_odom = odom(0);
    fresh_odom.header.stamp = stamp;
    cones_publisher->publish(fresh_cones);
    odom_publisher->publish(fresh_odom);
  };

  const auto first_stamp = currentStamp(*driver);
  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      if (observations.path_count > 0U && !observations.validity_values.empty() &&
        observations.validity_values.back())
      {
        return true;
      }
      publish_fresh_pair(first_stamp);
      return false;
    }, std::chrono::milliseconds(2000)));

  auto stale_cones = cones(0);
  stale_cones.header.stamp = currentStamp(*driver);
  cones_publisher->publish(stale_cones);
  const auto stale_wait_start = std::chrono::steady_clock::now();
  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      return std::chrono::steady_clock::now() - stale_wait_start >=
             std::chrono::milliseconds(125);
    }, std::chrono::milliseconds(500)));
  auto stale_odom = odom(0);
  stale_odom.header.stamp = stale_cones.header.stamp;
  odom_publisher->publish(stale_odom);

  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      return !observations.validity_values.empty() && !observations.validity_values.back();
    }, std::chrono::milliseconds(500)))
    << "a stale matched pair did not invalidate the previous local path";

  const auto path_count_before_recovery = observations.path_count;
  const auto recovery_stamp = currentStamp(*driver);
  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      if (observations.path_count > path_count_before_recovery &&
        !observations.validity_values.empty() && observations.validity_values.back())
      {
        return true;
      }
      publish_fresh_pair(recovery_stamp);
      return false;
    }, std::chrono::milliseconds(2000)))
    << "a fresh matched pair did not recover local-path validity";
}

}
}
