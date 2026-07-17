#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>

#include "hyu_local_planner/hyu_local_planner_node.hpp"
#include "hyu_local_planner/ros_inputs.hpp"
#include "hyu_local_planner_validity_test_support.hpp"

// Live-cone extension: perception cones stitched beyond the SLAM map frontier.
// Contract under test: map cones always win (dedupe), live cones are
// re-expressed from the pose they were measured at, conservative unknown
// routing, capacity pressure sheds live cones without invalidating the frame,
// and end-to-end the path actually grows past the frontier.

namespace hyu_local_planner
{
namespace
{

using test_support::ShutdownGuard;
using test_support::currentStamp;
using test_support::spinUntil;

eufs_msgs::msg::ConeWithCovariance coneAt(double x, double y)
{
  eufs_msgs::msg::ConeWithCovariance cone;
  cone.point.x = x;
  cone.point.y = y;
  return cone;
}

OdomMetadata poseAt(double x, double y, double yaw)
{
  OdomMetadata odom;
  odom.position = {x, y};
  odom.yaw = yaw;
  return odom;
}

TEST(LiveExtension, ReexpressesConesFromTheMeasurementPose)
{
  ConeSet cone_set;
  ConeArray live;
  live.blue_cones.push_back(coneAt(5.0, 1.0));
  // Measured at x=10; the car has since moved to x=12: the cone must land two
  // metres closer in the current ego frame.
  extendConeSetWithLiveCones(
    cone_set, live, poseAt(10.0, 0.0, 0.0), poseAt(12.0, 0.0, 0.0),
    LiveExtensionConfig{true, 1.0});
  ASSERT_EQ(cone_set.unknown.size(), 1U);
  EXPECT_NEAR(cone_set.unknown.front().x, 3.0, 1.0e-12);
  EXPECT_NEAR(cone_set.unknown.front().y, 1.0, 1.0e-12);
}

TEST(LiveExtension, MapWinsInsideTheMergeRadiusAcrossBuckets)
{
  ConeSet cone_set;
  cone_set.blue.push_back({3.0, 1.0});
  cone_set.yellow.push_back({3.0, -1.0});
  ConeArray live;
  live.blue_cones.push_back(coneAt(3.4, 1.0));    // 0.4 m from a map blue -> drop
  live.blue_cones.push_back(coneAt(3.2, -1.0));   // 0.9 m from a map YELLOW -> drop
  live.blue_cones.push_back(coneAt(4.5, 1.0));    // clear of everything -> keep
  extendConeSetWithLiveCones(
    cone_set, live, poseAt(0.0, 0.0, 0.0), poseAt(0.0, 0.0, 0.0),
    LiveExtensionConfig{true, 1.0});
  ASSERT_EQ(cone_set.unknown.size(), 1U);
  EXPECT_NEAR(cone_set.unknown.front().x, 4.5, 1.0e-12);
  EXPECT_EQ(cone_set.blue.size(), 1U);  // map untouched
}

TEST(LiveExtension, ColorRoutingFollowsAsUnknown)
{
  ConeSet as_unknown_set;
  ConeSet colored_set;
  ConeArray live;
  live.blue_cones.push_back(coneAt(5.0, 1.5));
  live.yellow_cones.push_back(coneAt(5.0, -1.5));
  extendConeSetWithLiveCones(
    as_unknown_set, live, poseAt(0, 0, 0), poseAt(0, 0, 0), LiveExtensionConfig{true, 1.0});
  extendConeSetWithLiveCones(
    colored_set, live, poseAt(0, 0, 0), poseAt(0, 0, 0), LiveExtensionConfig{false, 1.0});
  EXPECT_EQ(as_unknown_set.unknown.size(), 2U);
  EXPECT_TRUE(as_unknown_set.blue.empty() && as_unknown_set.yellow.empty());
  EXPECT_EQ(colored_set.blue.size(), 1U);
  EXPECT_EQ(colored_set.yellow.size(), 1U);
  EXPECT_TRUE(colored_set.unknown.empty());
}

TEST(LiveExtension, SkipsNonFiniteAndShedsAtCapacityWithoutOverflow)
{
  ConeSet cone_set;
  for (std::size_t i = 0; i < kMaxBoundaryCones; ++i) {
    cone_set.unknown.push_back({static_cast<double>(i) * 3.0, 50.0});
  }
  ConeArray live;
  live.blue_cones.push_back(coneAt(std::numeric_limits<double>::quiet_NaN(), 0.0));
  live.blue_cones.push_back(coneAt(5.0, -50.0));
  extendConeSetWithLiveCones(
    cone_set, live, poseAt(0, 0, 0), poseAt(0, 0, 0), LiveExtensionConfig{true, 1.0});
  EXPECT_EQ(cone_set.unknown.size(), kMaxBoundaryCones);  // shed, not grown
  EXPECT_FALSE(cone_set.input_overflow);                  // and never invalidated
}

// The end-to-end value proposition: a corridor whose map ends at x=8 plus
// live cones ahead must yield a LONGER valid path than the map alone — in
// BOTH color modes (as_unknown=false is the shipped config; true is the
// conservative fallback).
TEST(LiveExtension, PathExtendsPastTheMapFrontier)
{
  PlannerConfig config;
  config.allow_partial_boundary = true;  // slam_map source mode

  ConeSet map_only;
  for (const double x : {0.0, 3.0, 6.0, 8.0}) {
    map_only.blue.push_back({x, 1.75});
    map_only.yellow.push_back({x, -1.75});
  }
  const BuildResult base = buildLocalPath(map_only, config);
  ASSERT_TRUE(base.valid) << base.reason;

  ConeArray live;
  // Overlap at the frontier (dedupes) plus genuinely new cones ahead.
  for (const double x : {8.0, 10.5, 13.0, 15.5}) {
    live.blue_cones.push_back(coneAt(x, 1.75));
    live.yellow_cones.push_back(coneAt(x, -1.75));
  }

  for (const bool as_unknown : {false, true}) {
    ConeSet extended = map_only;
    extendConeSetWithLiveCones(
      extended, live, poseAt(0, 0, 0), poseAt(0, 0, 0),
      LiveExtensionConfig{as_unknown, 1.0});
    const BuildResult grown = buildLocalPath(extended, config);
    ASSERT_TRUE(grown.valid) << grown.reason << " as_unknown=" << as_unknown;
    EXPECT_GT(grown.waypoints.back().s, base.waypoints.back().s + 5.0)
      << "extension should add several metres of path, as_unknown=" << as_unknown;
    EXPECT_GT(grown.waypoints.back().x, 13.0) << "as_unknown=" << as_unknown;
  }
}

// Node-level wiring: with use_live_cone_extension the planner's published
// waypoints reach past the map frontier.
TEST(LiveExtension, NodeStitchesLiveConesOntoSlamPath)
{
  rclcpp::init(0, nullptr);
  ShutdownGuard shutdown_guard;

  rclcpp::NodeOptions options;
  options.append_parameter_override("source_mode", "slam_map");
  options.append_parameter_override("cones_topic", "/t_live_ext/cones");
  options.append_parameter_override("slam_map_topic", "/t_live_ext/map");
  options.append_parameter_override("slam_status_topic", "");
  options.append_parameter_override("odom_topic", "/t_live_ext/odom");
  options.append_parameter_override("waypoints_topic", "/t_live_ext/waypoints");
  options.append_parameter_override("path_topic", "/t_live_ext/path");
  options.append_parameter_override("validity_topic", "/t_live_ext/validity");
  options.append_parameter_override("reason_topic", "/t_live_ext/reason");
  options.append_parameter_override("use_live_cone_extension", true);
  options.append_parameter_override("live_extension_as_unknown", false);  // shipped config
  options.append_parameter_override("live_cone_max_age_sec", 5.0);
  options.append_parameter_override("heartbeat_hz", 50.0);

  auto planner = std::make_shared<LocalPlannerNode>(options);
  auto driver = std::make_shared<rclcpp::Node>("live_extension_driver");
  const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto map_publisher = driver->create_publisher<ConeArray>("/t_live_ext/map", map_qos);
  auto live_publisher = driver->create_publisher<ConeArray>(
    "/t_live_ext/cones", rclcpp::SensorDataQoS());
  auto odom_publisher = driver->create_publisher<Odometry>(
    "/t_live_ext/odom", rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

  double max_waypoint_x = 0.0;
  const auto waypoint_subscription = driver->create_subscription<
    eufs_msgs::msg::WaypointArrayStamped>(
    "/t_live_ext/waypoints", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [&max_waypoint_x](const eufs_msgs::msg::WaypointArrayStamped::SharedPtr message) {
      for (const auto & waypoint : message->waypoints) {
        max_waypoint_x = std::max(max_waypoint_x, waypoint.x_m);
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner);
  executor.add_node(driver);

  ConeArray map_message;
  map_message.header.frame_id = "map";
  for (const double x : {0.0, 3.0, 6.0, 8.0}) {
    map_message.blue_cones.push_back(coneAt(x, 1.75));
    map_message.yellow_cones.push_back(coneAt(x, -1.75));
  }
  ConeArray live_message;
  live_message.header.frame_id = "base_footprint";
  for (const double x : {10.5, 13.0, 15.5}) {
    live_message.blue_cones.push_back(coneAt(x, 1.75));
    live_message.yellow_cones.push_back(coneAt(x, -1.75));
  }

  const bool extended = spinUntil(
    executor,
    [&]() {
      const auto stamp = currentStamp(*driver);
      map_message.header.stamp = stamp;
      live_message.header.stamp = stamp;
      test_support::Odometry odom_message = test_support::odom(stamp.sec);
      odom_message.header.stamp = stamp;
      map_publisher->publish(map_message);
      live_publisher->publish(live_message);
      odom_publisher->publish(odom_message);
      return max_waypoint_x > 13.0;
    },
    std::chrono::seconds(10));
  EXPECT_TRUE(extended)
    << "published path never reached past the map frontier: max_x=" << max_waypoint_x;
}

}  // namespace
}  // namespace hyu_local_planner
