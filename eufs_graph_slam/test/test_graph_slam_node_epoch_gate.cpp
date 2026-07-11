// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <optional>
#include <cstdint>
#include <memory>
#include <vector>

#include "eufs_graph_slam/graph_slam_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace eufs_graph_slam
{

class GraphSlamNodeTestPeer
{
public:
  static void configureEpochGate(
    GraphSlamNode & node,
    std::int64_t epoch_start_ns,
    std::int64_t replay_guard_end_ns,
    std::int64_t max_future_lead_ns)
  {
    node.epoch_start_stamp_ns_ = epoch_start_ns;
    node.replay_guard_end_stamp_ns_ = replay_guard_end_ns;
    node.replay_guard_end_stamps_ns_ = {replay_guard_end_ns};
    node.max_future_stamp_lead_ns_ = max_future_lead_ns;
  }

  static bool accepts(
    const GraphSlamNode & node,
    std::int64_t stamp_ns,
    std::int64_t now_ns)
  {
    return node.isInputStampInCurrentEpoch(stamp_ns, now_ns);
  }

  static void observeClock(
    GraphSlamNode & node,
    std::int64_t now_ns,
    std::int64_t rollback_threshold_ns)
  {
    node.clock_rollback_threshold_ns_ = rollback_threshold_ns;
    node.observeNodeClock(rclcpp::Time(now_ns, node.get_clock()->get_clock_type()));
  }

  static void configureClockHighWatermark(
    GraphSlamNode & node,
    std::int64_t high_watermark_ns)
  {
    node.node_time_high_watermark_ns_ = high_watermark_ns;
  }

  static std::vector<std::int64_t> replayGuards(const GraphSlamNode & node)
  {
    return node.replay_guard_end_stamps_ns_;
  }
};

class GraphSlamNodeEpochGateTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(GraphSlamNodeEpochGateTest, UsesConfiguredFutureLeadDuringRollbackReplay)
{
  auto node = std::make_shared<GraphSlamNode>();
  GraphSlamNodeTestPeer::configureEpochGate(*node, 980, 1000, 5);

  EXPECT_FALSE(GraphSlamNodeTestPeer::accepts(*node, 979, 980));
  EXPECT_TRUE(GraphSlamNodeTestPeer::accepts(*node, 985, 980));
  EXPECT_FALSE(GraphSlamNodeTestPeer::accepts(*node, 986, 980));
  EXPECT_TRUE(GraphSlamNodeTestPeer::accepts(*node, 999, 995));
  EXPECT_FALSE(GraphSlamNodeTestPeer::accepts(*node, 1000, 995));
  EXPECT_TRUE(GraphSlamNodeTestPeer::accepts(*node, 1005, 1000));
}

TEST_F(GraphSlamNodeEpochGateTest, SerializesClockUpdatesWithGraphCallbacks)
{
  auto node = std::make_shared<GraphSlamNode>();

  EXPECT_FALSE(node->get_node_options().use_clock_thread());
}

TEST_F(GraphSlamNodeEpochGateTest, DoesNotReactivateReplayFenceAfterClockCatchesUp)
{
  auto node = std::make_shared<GraphSlamNode>();
  GraphSlamNodeTestPeer::configureEpochGate(*node, 900, 1000, 50);

  GraphSlamNodeTestPeer::observeClock(*node, 1000, 100);
  GraphSlamNodeTestPeer::observeClock(*node, 950, 100);

  EXPECT_TRUE(GraphSlamNodeTestPeer::accepts(*node, 1000, 950));
}

TEST_F(GraphSlamNodeEpochGateTest, KeepsOuterFenceAfterNestedRollbackCatchesUp)
{
  auto node = std::make_shared<GraphSlamNode>();
  GraphSlamNodeTestPeer::configureEpochGate(*node, 0, 1000, 90);
  GraphSlamNodeTestPeer::configureClockHighWatermark(*node, 200);

  GraphSlamNodeTestPeer::observeClock(*node, 100, 100);
  EXPECT_EQ(
    GraphSlamNodeTestPeer::replayGuards(*node),
    (std::vector<std::int64_t>{200, 1000}));
  EXPECT_FALSE(GraphSlamNodeTestPeer::accepts(*node, 200, 110));

  GraphSlamNodeTestPeer::observeClock(*node, 200, 100);
  EXPECT_EQ(
    GraphSlamNodeTestPeer::replayGuards(*node),
    (std::vector<std::int64_t>{1000}));
  EXPECT_FALSE(GraphSlamNodeTestPeer::accepts(*node, 1000, 910));

  GraphSlamNodeTestPeer::observeClock(*node, 1000, 100);
  EXPECT_TRUE(GraphSlamNodeTestPeer::replayGuards(*node).empty());
}

}  // namespace eufs_graph_slam
