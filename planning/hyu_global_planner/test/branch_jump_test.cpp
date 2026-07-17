#include <gtest/gtest.h>

#include "hyu_global_planner/slam_centerline_builder.hpp"
#include "slam_test_fixtures.hpp"

namespace hyu_global_planner::test
{
namespace
{

void addCone(std::vector<hyu_msgs::msg::ConeWithCovariance> & cones, double x, double y)
{
  hyu_msgs::msg::ConeWithCovariance cone;
  cone.point.x = x;
  cone.point.y = y;
  cones.push_back(cone);
}

}  // namespace

TEST(BranchJump, FailsClosedWithExactReasonAndNoStaleWaypoints)
{
  hyu_msgs::msg::ConeArrayWithCovariance map;
  addCone(map.blue_cones, 0.0, 0.0);
  addCone(map.blue_cones, 4.0, 0.0);
  addCone(map.blue_cones, 40.0, 0.0);
  addCone(map.yellow_cones, 0.0, 3.0);
  addCone(map.yellow_cones, 4.0, 3.0);
  addCone(map.yellow_cones, 40.0, 3.0);
  std::vector<PlannerWaypoint> waypoints{{9.0, 9.0, 9.0, 9.0, 9.0}};
  std::string reason;

  EXPECT_FALSE(buildCenterlineFromSlamMap(map, egoAtOrigin(), fixtureConfig(), waypoints, reason));

  EXPECT_TRUE(waypoints.empty());
  EXPECT_EQ(reason, "branch_jump");
}

}  // namespace hyu_global_planner::test
