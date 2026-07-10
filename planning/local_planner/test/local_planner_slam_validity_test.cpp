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

TEST(LocalPlannerValidity, MalformedSlamMapClearsFreshValidPath)
{
  rclcpp::init(0, nullptr);
  ShutdownGuard shutdown_guard;

  rclcpp::NodeOptions planner_options;
  planner_options.append_parameter_override("source_mode", "slam_map");
  planner_options.append_parameter_override("slam_map_topic", "/t9_slam_invalid/map");
  planner_options.append_parameter_override("odom_topic", "/t9_slam_invalid/odom");
  planner_options.append_parameter_override("waypoints_topic", "/t9_slam_invalid/waypoints");
  planner_options.append_parameter_override("path_topic", "/t9_slam_invalid/path");
  planner_options.append_parameter_override("validity_topic", "/t9_slam_invalid/validity");
  planner_options.append_parameter_override("heartbeat_hz", 50.0);

  auto planner = std::make_shared<LocalPlannerNode>(planner_options);
  auto driver = std::make_shared<rclcpp::Node>("local_planner_slam_invalid_driver");
  const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto map_publisher = driver->create_publisher<ConeArray>("/t9_slam_invalid/map", map_qos);
  auto odom_publisher = driver->create_publisher<Odometry>(
    "/t9_slam_invalid/odom", rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

  Observations observations;
  const auto waypoint_subscription = driver->create_subscription<
    eufs_msgs::msg::WaypointArrayStamped>(
    "/t9_slam_invalid/waypoints", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [&observations](const eufs_msgs::msg::WaypointArrayStamped::SharedPtr message) {
      if (!message->waypoints.empty()) {
        ++observations.path_count;
      }
    });
  const auto validity_subscription = driver->create_subscription<std_msgs::msg::Bool>(
    "/t9_slam_invalid/validity", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [&observations](const std_msgs::msg::Bool::SharedPtr message) {
      observations.validity_values.push_back(message->data);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner);
  executor.add_node(driver);
  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      return map_publisher->get_subscription_count() > 0U &&
             odom_publisher->get_subscription_count() > 0U;
    }, std::chrono::milliseconds(1000)));
  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      return waypoint_subscription->get_publisher_count() > 0U &&
             validity_subscription->get_publisher_count() > 0U;
    }, std::chrono::milliseconds(1000))) << "local planner outputs did not connect";

  const auto valid_stamp = currentStamp(*driver);
  auto valid_map = cones(0);
  valid_map.header.frame_id = "map";
  valid_map.header.stamp = valid_stamp;
  auto valid_odom = odom(0);
  valid_odom.header.stamp = valid_stamp;
  map_publisher->publish(valid_map);
  const auto publish_valid_odom = [&]() {odom_publisher->publish(valid_odom);};
  publish_valid_odom();
  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      if (observations.path_count > 0U && !observations.validity_values.empty() &&
        observations.validity_values.back())
      {
        return true;
      }
      publish_valid_odom();
      return false;
    }, std::chrono::milliseconds(2000))) << "valid SLAM-map path never reached true heartbeat";

  auto malformed_map = valid_map;
  malformed_map.header.frame_id = "odom";
  ++malformed_map.header.stamp.sec;
  auto matched_odom = valid_odom;
  matched_odom.header.stamp = malformed_map.header.stamp;
  const auto validity_count_before_malformed = observations.validity_values.size();
  ASSERT_TRUE(spinUntil(
    executor,
    [&]() {
      if (observations.validity_values.size() > validity_count_before_malformed) {
        return true;
      }
      map_publisher->publish(malformed_map);
      odom_publisher->publish(matched_odom);
      return false;
    }, std::chrono::milliseconds(250)));

  EXPECT_FALSE(observations.validity_values.back())
    << "a malformed SLAM map left the prior local path valid";
}

}
}
