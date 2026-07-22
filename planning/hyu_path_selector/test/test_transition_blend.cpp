#include <cmath>
#include <optional>

#include <gtest/gtest.h>

#include "hyu_path_selector/transition_blend.hpp"

namespace hyu_path_selector
{
namespace
{

hyu_msgs::msg::WaypointArrayStamped makeStraightPath(
  double start_x, double start_y, double heading_rad, double length_m,
  double spacing_m, double speed_mps)
{
  hyu_msgs::msg::WaypointArrayStamped path;
  path.header.frame_id = "map";
  const double cos_h = std::cos(heading_rad);
  const double sin_h = std::sin(heading_rad);
  for (double s = 0.0; s <= length_m + 1.0e-9; s += spacing_m) {
    hyu_msgs::msg::Waypoint waypoint;
    waypoint.x_m = start_x + s * cos_h;
    waypoint.y_m = start_y + s * sin_h;
    waypoint.position.x = waypoint.x_m;
    waypoint.position.y = waypoint.y_m;
    waypoint.s_m = s;
    waypoint.psi_rad = heading_rad;
    waypoint.kappa_radpm = 0.0;
    waypoint.vx_mps = speed_mps;
    waypoint.speed = speed_mps;
    path.waypoints.push_back(waypoint);
  }
  return path;
}

TransitionBlendConfig defaultConfig()
{
  return TransitionBlendConfig{10.0, 0.5, 3.0, 1.0};
}

TEST(TransitionBlendTest, StartsOnLocalAndEndsOnGlobal)
{
  // Local runs 1.2 m left of (parallel to) the global path.
  const auto local = makeStraightPath(0.0, 1.2, 0.0, 15.0, 0.5, 4.0);
  const auto global = makeStraightPath(0.0, 0.0, 0.0, 40.0, 0.5, 8.0);

  const auto blended = buildTransitionPath(local, global, 0.0, defaultConfig());
  ASSERT_TRUE(blended.has_value());

  // Starts on the local path (weight 0 at ego).
  EXPECT_NEAR(blended->waypoints.front().y_m, 1.2, 0.05);
  // Ends of the array are the untouched global tail.
  EXPECT_NEAR(blended->waypoints.back().y_m, 0.0, 1.0e-9);
  EXPECT_NEAR(blended->waypoints.back().vx_mps, 8.0, 1.0e-9);

  // Lateral offset decays monotonically (within sampling noise).
  double previous_offset = blended->waypoints.front().y_m;
  for (const auto & waypoint : blended->waypoints) {
    EXPECT_LE(waypoint.y_m, previous_offset + 1.0e-6);
    previous_offset = waypoint.y_m;
  }
}

TEST(TransitionBlendTest, ProgressAdvancesStartWeightTowardGlobal)
{
  const auto local = makeStraightPath(0.0, 1.2, 0.0, 15.0, 0.5, 4.0);
  const auto global = makeStraightPath(0.0, 0.0, 0.0, 40.0, 0.5, 8.0);

  const auto early = buildTransitionPath(local, global, 0.0, defaultConfig());
  const auto late = buildTransitionPath(local, global, 7.0, defaultConfig());
  ASSERT_TRUE(early.has_value());
  ASSERT_TRUE(late.has_value());
  // With 7 m already travelled the start of the path sits much closer to the
  // global line than at the flip.
  EXPECT_LT(late->waypoints.front().y_m, early->waypoints.front().y_m * 0.5);
}

TEST(TransitionBlendTest, ExhaustedWindowReturnsNullopt)
{
  const auto local = makeStraightPath(0.0, 1.2, 0.0, 15.0, 0.5, 4.0);
  const auto global = makeStraightPath(0.0, 0.0, 0.0, 40.0, 0.5, 8.0);

  EXPECT_FALSE(buildTransitionPath(local, global, 10.0, defaultConfig()).has_value());
  EXPECT_FALSE(buildTransitionPath(local, global, 25.0, defaultConfig()).has_value());
}

TEST(TransitionBlendTest, MonotonicArcAndFiniteFields)
{
  // 30-degree heading mismatch: the worst case the old gate rejected.
  const auto local = makeStraightPath(0.0, 1.0, 0.52, 12.0, 0.5, 4.0);
  const auto global = makeStraightPath(0.0, 0.0, 0.0, 40.0, 0.5, 8.0);

  const auto blended = buildTransitionPath(local, global, 2.0, defaultConfig());
  ASSERT_TRUE(blended.has_value());
  ASSERT_GE(blended->waypoints.size(), 5U);
  double previous_s = -1.0;
  for (const auto & waypoint : blended->waypoints) {
    EXPECT_TRUE(std::isfinite(waypoint.x_m));
    EXPECT_TRUE(std::isfinite(waypoint.y_m));
    EXPECT_TRUE(std::isfinite(waypoint.psi_rad));
    EXPECT_TRUE(std::isfinite(waypoint.kappa_radpm));
    EXPECT_TRUE(std::isfinite(waypoint.vx_mps));
    EXPECT_GT(waypoint.s_m, previous_s);
    previous_s = waypoint.s_m;
  }
}

TEST(TransitionBlendTest, SpeedRespectsCurvatureCapAndFloor)
{
  // Sharp sideways jump forces curvature into the blend segment.
  const auto local = makeStraightPath(0.0, 2.0, 0.0, 12.0, 0.5, 9.0);
  const auto global = makeStraightPath(0.0, 0.0, 0.0, 40.0, 0.5, 9.0);
  TransitionBlendConfig config{6.0, 0.5, 3.0, 1.0};

  const auto blended = buildTransitionPath(local, global, 0.0, config);
  ASSERT_TRUE(blended.has_value());
  for (const auto & waypoint : blended->waypoints) {
    EXPECT_GE(waypoint.vx_mps, config.min_speed_mps - 1.0e-9);
    const double kappa = std::abs(waypoint.kappa_radpm);
    if (kappa > 1.0e-6 && waypoint.s_m < 5.5) {
      EXPECT_LE(
        waypoint.vx_mps,
        std::sqrt(config.max_lateral_accel_mps2 / kappa) + 1.0e-6);
    }
  }
}

TEST(TransitionBlendTest, ShortLocalPathShrinksWindowButStillBlends)
{
  const auto local = makeStraightPath(0.0, 1.0, 0.0, 3.0, 0.5, 4.0);
  const auto global = makeStraightPath(0.0, 0.0, 0.0, 40.0, 0.5, 8.0);

  const auto blended = buildTransitionPath(local, global, 0.0, defaultConfig());
  ASSERT_TRUE(blended.has_value());
  // Weight still reaches 1 by the end of the shortened window: past the
  // local horizon the output must lie on the global line.
  for (const auto & waypoint : blended->waypoints) {
    if (waypoint.s_m > 3.5) {
      EXPECT_NEAR(waypoint.y_m, 0.0, 0.05);
    }
  }
}

TEST(TransitionBlendTest, DegenerateInputsReturnNullopt)
{
  const auto global = makeStraightPath(0.0, 0.0, 0.0, 40.0, 0.5, 8.0);
  hyu_msgs::msg::WaypointArrayStamped empty;
  EXPECT_FALSE(buildTransitionPath(empty, global, 0.0, defaultConfig()).has_value());
  EXPECT_FALSE(buildTransitionPath(global, empty, 0.0, defaultConfig()).has_value());
  EXPECT_FALSE(
    buildTransitionPath(
      global, global, std::nan(""), defaultConfig()).has_value());
}

}
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
