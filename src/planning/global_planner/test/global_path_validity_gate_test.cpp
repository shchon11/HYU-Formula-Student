#include "global_planner/global_path_validity_gate.hpp"

#include <memory>

#include <gtest/gtest.h>

namespace wpnt_publisher
{
namespace
{

TEST(GlobalPathValidityGateTest, InvalidatesOnRosClockBackwardJump)
{
  GlobalPathValidityGate gate(0.5);
  auto snapshot = std::make_shared<PathSnapshot>();
  snapshot->track_length = 10.0;

  const rclcpp::Time accepted_time(10, 0, RCL_ROS_TIME);
  gate.onSnapshot(snapshot, accepted_time);
  EXPECT_TRUE(gate.onValidity(true, accepted_time).activated);
  ASSERT_EQ(gate.readForOdom(accepted_time).snapshot, snapshot);

  const rclcpp::Time reset_time(9, 0, RCL_ROS_TIME);
  const auto result = gate.readForOdom(reset_time);
  EXPECT_FALSE(result.validity_true);
  EXPECT_EQ(result.snapshot, nullptr);
}

}  // namespace
}  // namespace wpnt_publisher
