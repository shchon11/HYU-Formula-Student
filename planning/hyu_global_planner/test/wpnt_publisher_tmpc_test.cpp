#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hyu_msgs/msg/waypoint_array_stamped.hpp"
#include "hyu_tmpc_msgs/msg/tum_trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "wpnt_publisher_node.hpp"

namespace
{
using namespace std::chrono_literals;
using TumTrajectory = hyu_tmpc_msgs::msg::TumTrajectory;

hyu_msgs::msg::WaypointArrayStamped makeCircularPath()
{
  constexpr std::size_t kCount = 100U;
  constexpr double kRadius = 20.0;
  constexpr double kPi = 3.14159265358979323846;
  const double segment_length = 2.0 * kRadius * std::sin(kPi / static_cast<double>(kCount));

  hyu_msgs::msg::WaypointArrayStamped path;
  path.header.frame_id = "map";
  path.waypoints.reserve(kCount);
  for (std::size_t i = 0; i < kCount; ++i) {
    const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kCount);
    hyu_msgs::msg::Waypoint waypoint;
    waypoint.x_m = kRadius * std::cos(theta);
    waypoint.y_m = kRadius * std::sin(theta);
    waypoint.position.x = waypoint.x_m;
    waypoint.position.y = waypoint.y_m;
    waypoint.s_m = segment_length * static_cast<double>(i);
    waypoint.psi_rad = std::remainder(theta + 0.5 * kPi, 2.0 * kPi);
    waypoint.kappa_radpm = 1.0 / kRadius;
    waypoint.speed = 2.0;
    waypoint.vx_mps = 2.0;
    waypoint.ax_mps2 = 0.0;
    path.waypoints.push_back(waypoint);
  }
  return path;
}

class WpntPublisherTmpcTest : public ::testing::Test
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
      rclcpp::Parameter("global_waypoints_topic", "/test_tmpc/global_waypoints"),
      rclcpp::Parameter("global_path_valid_topic", "/test_tmpc/global_path_valid"),
      rclcpp::Parameter("frenet_odom_topic", "/test_tmpc/frenet_odom"),
      rclcpp::Parameter("ego_odom_topic", "/test_tmpc/ego_odom"),
      rclcpp::Parameter("path_source_topic", "/test_tmpc/path_source"),
      rclcpp::Parameter("lap_count_topic", "/test_tmpc/lap_count"),
      rclcpp::Parameter("path_waypoints_topic", "/test_tmpc/rolling_waypoints"),
      rclcpp::Parameter("path_topic", "/test_tmpc/rolling_path"),
      rclcpp::Parameter("tmpc_performance_trajectory_topic", "/test_tmpc/performance"),
      rclcpp::Parameter("tmpc_emergency_trajectory_topic", "/test_tmpc/emergency"),
      rclcpp::Parameter("global_path_valid_timeout_sec", 0.15),
      rclcpp::Parameter("tmpc_input_timeout_sec", 0.15),
      rclcpp::Parameter("tmpc_publish_rate_hz", 20.0),
      rclcpp::Parameter("tmpc_input_heading_convention", "ros_yaw"),
      rclcpp::Parameter("tmpc_ggv_csv_path", ""),
      rclcpp::Parameter("tmpc_ggv_speed_mps", std::vector<double>{0.0, 10.0}),
      rclcpp::Parameter("tmpc_ggv_ax_max_mps2", std::vector<double>{5.0, 5.0}),
      rclcpp::Parameter("tmpc_ggv_ay_max_mps2", std::vector<double>{8.0, 8.0}),
      rclcpp::Parameter("tmpc_ggv_ax_decel_max_mps2", std::vector<double>{6.0, 6.0})};
    node_ = std::make_shared<wpnt_publisher::WpntPublisher>(
      rclcpp::NodeOptions().parameter_overrides(overrides));
    driver_ = std::make_shared<rclcpp::Node>("wpnt_publisher_tmpc_test_driver");

    global_pub_ = driver_->create_publisher<hyu_msgs::msg::WaypointArrayStamped>(
      "/test_tmpc/global_waypoints",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    valid_pub_ = driver_->create_publisher<std_msgs::msg::Bool>(
      "/test_tmpc/global_path_valid", rclcpp::QoS(10).reliable());
    frenet_pub_ = driver_->create_publisher<nav_msgs::msg::Odometry>(
      "/test_tmpc/frenet_odom", rclcpp::QoS(20).reliable());
    ego_pub_ = driver_->create_publisher<nav_msgs::msg::Odometry>(
      "/test_tmpc/ego_odom", rclcpp::QoS(20).reliable());
    source_pub_ = driver_->create_publisher<std_msgs::msg::String>(
      "/test_tmpc/path_source", rclcpp::QoS(10).reliable());
    lap_pub_ = driver_->create_publisher<std_msgs::msg::Int32>(
      "/test_tmpc/lap_count", rclcpp::QoS(10).reliable());

    performance_sub_ = driver_->create_subscription<TumTrajectory>(
      "/test_tmpc/performance", rclcpp::QoS(10).reliable().durability_volatile(),
      [this](TumTrajectory::ConstSharedPtr msg) {
        ++performance_count_;
        last_performance_ = *msg;
      });
    emergency_sub_ = driver_->create_subscription<TumTrajectory>(
      "/test_tmpc/emergency", rclcpp::QoS(10).reliable().durability_volatile(),
      [this](TumTrajectory::ConstSharedPtr msg) {
        ++emergency_count_;
        last_emergency_ = *msg;
      });
    rolling_sub_ = driver_->create_subscription<hyu_msgs::msg::WaypointArrayStamped>(
      "/test_tmpc/rolling_waypoints", rclcpp::QoS(10).reliable(),
      [this](hyu_msgs::msg::WaypointArrayStamped::ConstSharedPtr msg) {
        ++rolling_count_;
        last_rolling_ = *msg;
      });
    rolling_path_sub_ = driver_->create_subscription<nav_msgs::msg::Path>(
      "/test_tmpc/rolling_path", rclcpp::QoS(10).reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        ++rolling_path_count_;
        last_rolling_path_ = *msg;
      });

    executor_.add_node(node_);
    executor_.add_node(driver_);
    spinFor(100ms);
  }

  void TearDown() override
  {
    executor_.remove_node(driver_);
    executor_.remove_node(node_);
    driver_.reset();
    node_.reset();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    executor_.spin_some();
  }

  bool spinUntil(const std::function<bool()> & predicate, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      publishDynamicInputs(current_source_, true);
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some();
    return predicate();
  }

  void publishDynamicInputs(const std::string & source, bool validity)
  {
    current_source_ = source;
    std_msgs::msg::Bool valid;
    valid.data = validity;
    valid_pub_->publish(valid);

    nav_msgs::msg::Odometry frenet;
    frenet.pose.pose.position.x = 1.0;
    frenet_pub_->publish(frenet);

    nav_msgs::msg::Odometry ego;
    ego.twist.twist.linear.x = 2.0;
    ego_pub_->publish(ego);

    std_msgs::msg::String source_msg;
    source_msg.data = source;
    source_pub_->publish(source_msg);

    std_msgs::msg::Int32 lap;
    lap.data = 3;
    lap_pub_->publish(lap);
  }

  void publishFreshInputsWithoutValidity(const std::string & source)
  {
    current_source_ = source;

    nav_msgs::msg::Odometry frenet;
    frenet.pose.pose.position.x = 1.0;
    frenet_pub_->publish(frenet);

    nav_msgs::msg::Odometry ego;
    ego.twist.twist.linear.x = 2.0;
    ego_pub_->publish(ego);

    std_msgs::msg::String source_msg;
    source_msg.data = source;
    source_pub_->publish(source_msg);

    std_msgs::msg::Int32 lap;
    lap.data = 3;
    lap_pub_->publish(lap);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<wpnt_publisher::WpntPublisher> node_;
  rclcpp::Node::SharedPtr driver_;
  rclcpp::Publisher<hyu_msgs::msg::WaypointArrayStamped>::SharedPtr global_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr frenet_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ego_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lap_pub_;
  rclcpp::Subscription<TumTrajectory>::SharedPtr performance_sub_;
  rclcpp::Subscription<TumTrajectory>::SharedPtr emergency_sub_;
  rclcpp::Subscription<hyu_msgs::msg::WaypointArrayStamped>::SharedPtr rolling_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rolling_path_sub_;
  std::size_t performance_count_{0U};
  std::size_t emergency_count_{0U};
  TumTrajectory last_performance_;
  TumTrajectory last_emergency_;
  std::size_t rolling_count_{0U};
  hyu_msgs::msg::WaypointArrayStamped last_rolling_;
  std::size_t rolling_path_count_{0U};
  nav_msgs::msg::Path last_rolling_path_;
  std::string current_source_{"LOCAL"};
};

TEST_F(WpntPublisherTmpcTest, PublishesOnlyFreshGlobalTrajectoryPairs)
{
  global_pub_->publish(makeCircularPath());

  for (int i = 0; i < 25; ++i) {
    publishDynamicInputs("LOCAL", true);
    spinFor(10ms);
  }
  EXPECT_EQ(performance_count_, 0U);
  EXPECT_EQ(emergency_count_, 0U);
  ASSERT_GT(rolling_count_, 0U);
  ASSERT_EQ(last_rolling_.waypoints.size(), 50U);
  const auto global_path = makeCircularPath();
  EXPECT_DOUBLE_EQ(last_rolling_.waypoints[0].x_m, global_path.waypoints[1].x_m);
  EXPECT_DOUBLE_EQ(last_rolling_.waypoints[0].y_m, global_path.waypoints[1].y_m);
  EXPECT_DOUBLE_EQ(last_rolling_.waypoints[1].x_m, global_path.waypoints[2].x_m);
  EXPECT_DOUBLE_EQ(last_rolling_.waypoints[1].y_m, global_path.waypoints[2].y_m);
  ASSERT_GT(rolling_path_count_, 0U);
  ASSERT_EQ(last_rolling_path_.poses.size(), 50U);
  EXPECT_DOUBLE_EQ(last_rolling_path_.poses[0].pose.position.x, global_path.waypoints[1].x_m);
  EXPECT_DOUBLE_EQ(last_rolling_path_.poses[0].pose.position.y, global_path.waypoints[1].y_m);
  EXPECT_DOUBLE_EQ(last_rolling_path_.poses[1].pose.position.x, global_path.waypoints[2].x_m);
  EXPECT_DOUBLE_EQ(last_rolling_path_.poses[1].pose.position.y, global_path.waypoints[2].y_m);

  for (const std::string source : {"STOP", "UNKNOWN_SOURCE"}) {
    for (int i = 0; i < 10; ++i) {
      publishDynamicInputs(source, true);
      spinFor(10ms);
    }
  }
  EXPECT_EQ(performance_count_, 0U);
  EXPECT_EQ(emergency_count_, 0U);

  current_source_ = "GLOBAL_FULL";
  ASSERT_TRUE(spinUntil(
      [this]() {return performance_count_ > 0U && emergency_count_ > 0U;}, 500ms));
  EXPECT_EQ(last_performance_.traj_cnt, last_emergency_.traj_cnt);
  EXPECT_GT(last_performance_.traj_cnt, 0U);
  EXPECT_EQ(last_performance_.lap_cnt, 3U);
  EXPECT_EQ(last_emergency_.lap_cnt, 3U);

  const std::size_t performance_before_false = performance_count_;
  const std::size_t emergency_before_false = emergency_count_;
  publishDynamicInputs("GLOBAL_FULL", false);
  spinFor(200ms);
  EXPECT_EQ(performance_count_, performance_before_false);
  EXPECT_EQ(emergency_count_, emergency_before_false);

  // Recovery after an invalidation requires a newer global snapshot.
  global_pub_->publish(makeCircularPath());
  current_source_ = "GLOBAL_FINAL_STOP";
  ASSERT_TRUE(spinUntil(
      [this, performance_before_false]() {
        return performance_count_ > performance_before_false;
      }, 500ms));
  EXPECT_EQ(last_performance_.traj_cnt, last_emergency_.traj_cnt);

  // Keep Frenet/ego/source/lap fresh while stopping only the global-validity
  // heartbeat. Publication must cease once that heartbeat exceeds its timeout.
  for (int i = 0; i < 30; ++i) {
    publishFreshInputsWithoutValidity("GLOBAL_FINAL_STOP");
    spinFor(10ms);
  }
  const std::size_t performance_after_validity_stale = performance_count_;
  const std::size_t emergency_after_validity_stale = emergency_count_;
  for (int i = 0; i < 10; ++i) {
    publishFreshInputsWithoutValidity("GLOBAL_FINAL_STOP");
    spinFor(10ms);
  }
  EXPECT_EQ(performance_count_, performance_after_validity_stale);
  EXPECT_EQ(emergency_count_, emergency_after_validity_stale);

  // A stale-heartbeat invalidation also requires a new snapshot before a
  // fresh true heartbeat can reactivate publication.
  global_pub_->publish(makeCircularPath());
  ASSERT_TRUE(spinUntil(
      [this, performance_after_validity_stale]() {
        return performance_count_ > performance_after_validity_stale;
      }, 500ms));

  // Keep only the validity heartbeat alive. Once the four dynamic inputs age
  // past tmpc_input_timeout_sec, no further trajectory is allowed out.
  for (int i = 0; i < 30; ++i) {
    std_msgs::msg::Bool valid;
    valid.data = true;
    valid_pub_->publish(valid);
    spinFor(10ms);
  }
  const std::size_t performance_after_stale = performance_count_;
  const std::size_t emergency_after_stale = emergency_count_;
  spinFor(100ms);
  EXPECT_EQ(performance_count_, performance_after_stale);
  EXPECT_EQ(emergency_count_, emergency_after_stale);
}

}  // namespace
