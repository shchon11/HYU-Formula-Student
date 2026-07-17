#include "hyu_global_planner/tmpc_trajectory_builder.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace hyu_global_planner::tmpc
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

wpnt_publisher::PathSnapshot makeCircle(
  std::size_t count, double radius_m, double speed_mps)
{
  wpnt_publisher::PathSnapshot path;
  path.track_length = 2.0 * kPi * radius_m;
  path.frame_id = "map";
  path.waypoints.reserve(count);
  path.xs.reserve(count);
  path.ys.reserve(count);
  path.s.reserve(count);

  for (std::size_t i = 0U; i < count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(count);
    const double theta = ratio * 2.0 * kPi;
    eufs_msgs::msg::Waypoint waypoint;
    waypoint.s_m = ratio * path.track_length;
    waypoint.x_m = radius_m * std::cos(theta);
    waypoint.y_m = radius_m * std::sin(theta);
    waypoint.position.x = waypoint.x_m;
    waypoint.position.y = waypoint.y_m;
    waypoint.psi_rad = normalizeAngle(theta + 0.5 * kPi);
    waypoint.kappa_radpm = 1.0 / radius_m;
    waypoint.vx_mps = speed_mps;
    waypoint.ax_mps2 = 0.0;
    waypoint.speed = speed_mps;
    path.s.push_back(waypoint.s_m);
    path.xs.push_back(waypoint.x_m);
    path.ys.push_back(waypoint.y_m);
    path.waypoints.push_back(waypoint);
  }
  return path;
}

GgvTable makeGgv(double ax_mps2 = 9.81, double ay_mps2 = 20.0)
{
  std::string error;
  auto table = GgvTable::fromVectors(
    {0.0, 30.0}, {ax_mps2, ax_mps2}, {ay_mps2, ay_mps2}, &error);
  EXPECT_TRUE(table.has_value()) << error;
  return table.value();
}

BuildResult buildCircle(
  std::size_t input_count,
  double current_s_m = 5.0,
  double path_speed_mps = 5.0,
  double vehicle_speed_mps = 5.0,
  BuilderConfig config = BuilderConfig{},
  GgvTable ggv = makeGgv())
{
  static wpnt_publisher::PathSnapshot path;
  path = makeCircle(input_count, 20.0, path_speed_mps);
  TmpcTrajectoryBuilder builder(config, std::move(ggv));
  BuildInput input;
  input.path = &path;
  input.current_s_m = current_s_m;
  input.current_speed_mps = vehicle_speed_mps;
  input.lap_cnt = 3U;
  input.traj_cnt = 7U;
  return builder.build(input);
}

TEST(GgvTableTest, SupportsThreeAndFourColumnSemantics)
{
  std::string error;
  const auto three_column = GgvTable::fromVectors(
    {0.0, 10.0}, {4.0, 6.0}, {8.0, 12.0}, &error);
  ASSERT_TRUE(three_column.has_value()) << error;
  const auto interpolated = three_column->limitsAt(5.0);
  EXPECT_DOUBLE_EQ(interpolated.ax_mps2, 5.0);
  EXPECT_DOUBLE_EQ(interpolated.ay_mps2, 10.0);

  const auto four_column = GgvTable::fromVectors(
    {0.0, 10.0}, {7.0, 8.0}, {12.0, 13.0}, {5.0, 9.0}, &error);
  ASSERT_TRUE(four_column.has_value()) << error;
  EXPECT_DOUBLE_EQ(four_column->limitsAt(0.0).ax_mps2, 5.0);
  EXPECT_DOUBLE_EQ(four_column->limitsAt(10.0).ax_mps2, 8.0);
}

TEST(GgvTableTest, LoadsCsvAndRejectsNonIncreasingSpeedAxis)
{
  const std::string path = "/tmp/tmpc_trajectory_builder_ggv_test.csv";
  {
    std::ofstream stream(path);
    ASSERT_TRUE(stream.is_open());
    stream << "# v,ax,ay,decel\n";
    stream << "0.0,11.0,20.0,9.0\n";
    stream << "8.5,10.0,19.0,8.0\n";
  }

  std::string error;
  const auto table = GgvTable::loadCsv(path, &error);
  ASSERT_TRUE(table.has_value()) << error;
  EXPECT_DOUBLE_EQ(table->limitsAt(0.0).ax_mps2, 9.0);
  EXPECT_DOUBLE_EQ(table->limitsAt(8.5).ax_mps2, 8.0);
  std::remove(path.c_str());

  EXPECT_FALSE(GgvTable::fromVectors(
    {0.0, 0.0}, {5.0, 5.0}, {8.0, 8.0}, &error).has_value());
  EXPECT_NE(error.find("strictly increasing"), std::string::npos);
}

TEST(GgvTableTest, RejectsInvalidAccelerationOrDecelerationBeforeTakingMinimum)
{
  std::string error;
  EXPECT_FALSE(GgvTable::fromVectors(
    {0.0, 10.0},
    {7.0, 8.0},
    {12.0, 13.0},
    {5.0, std::numeric_limits<double>::quiet_NaN()},
    &error).has_value());
  EXPECT_NE(error.find("finite and greater than zero"), std::string::npos);

  EXPECT_FALSE(GgvTable::fromVectors(
    {0.0, 10.0}, {7.0, 8.0}, {12.0, 13.0}, {5.0, 0.0}, &error).has_value());
  EXPECT_NE(error.find("finite and greater than zero"), std::string::npos);
}

TEST(TmpcTrajectoryBuilderTest, ResamplesTwentyAndOneHundredInputsToExactlyFiftyPoints)
{
  for (const std::size_t input_count : {20U, 100U}) {
    const BuildResult result = buildCircle(input_count);
    ASSERT_TRUE(result.success) << result.error;
    EXPECT_EQ(result.performance.s_loc_m.size(), kTrajectoryPointCount);
    EXPECT_EQ(result.emergency.s_loc_m.size(), kTrajectoryPointCount);
    EXPECT_EQ(result.performance.lap_cnt, 3U);
    EXPECT_EQ(result.performance.traj_cnt, 7U);
    EXPECT_EQ(result.emergency.lap_cnt, result.performance.lap_cnt);
    EXPECT_EQ(result.emergency.traj_cnt, result.performance.traj_cnt);
  }
}

TEST(TmpcTrajectoryBuilderTest, LocalSStartsAtZeroAndIsStrictlyIncreasing)
{
  const BuildResult result = buildCircle(100U);
  ASSERT_TRUE(result.success) << result.error;
  EXPECT_DOUBLE_EQ(result.performance.s_loc_m.front(), 0.0);
  for (std::size_t i = 1U; i < kTrajectoryPointCount; ++i) {
    EXPECT_GT(result.performance.s_loc_m[i], result.performance.s_loc_m[i - 1U]);
  }
}

TEST(TmpcTrajectoryBuilderTest, ClosedLoopSeamWrapsOnlyGlobalS)
{
  const auto path = makeCircle(100U, 20.0, 5.0);
  TmpcTrajectoryBuilder builder(BuilderConfig{}, makeGgv());
  BuildInput input;
  input.path = &path;
  input.current_s_m = path.track_length - 1.0;
  input.current_speed_mps = 5.0;
  input.lap_cnt = 1U;
  input.traj_cnt = 1U;
  const BuildResult result = builder.build(input);
  ASSERT_TRUE(result.success) << result.error;

  bool saw_global_wrap = false;
  for (std::size_t i = 1U; i < kTrajectoryPointCount; ++i) {
    saw_global_wrap = saw_global_wrap ||
      result.performance.s_glob_m[i] < result.performance.s_glob_m[i - 1U];
    EXPECT_GT(result.performance.s_loc_m[i], result.performance.s_loc_m[i - 1U]);
    EXPECT_LT(
      std::hypot(
        result.performance.x_m[i] - result.performance.x_m[i - 1U],
        result.performance.y_m[i] - result.performance.y_m[i - 1U]),
      1.0);
  }
  EXPECT_TRUE(saw_global_wrap);
}

TEST(TmpcTrajectoryBuilderTest, HeadingInterpolationUnwrapsPiBoundary)
{
  const auto path = makeCircle(120U, 20.0, 5.0);
  TmpcTrajectoryBuilder builder(BuilderConfig{}, makeGgv());
  BuildInput input;
  input.path = &path;
  // ROS tangent heading crosses +pi/-pi at theta=pi/2.
  input.current_s_m = 0.5 * kPi * 20.0;
  input.current_speed_mps = 5.0;
  input.traj_cnt = 1U;
  const BuildResult result = builder.build(input);
  ASSERT_TRUE(result.success) << result.error;
  for (std::size_t i = 1U; i < kTrajectoryPointCount; ++i) {
    EXPECT_LT(
      std::abs(normalizeAngle(
        result.performance.psi_rad[i] - result.performance.psi_rad[i - 1U])),
      0.1);
  }
}

TEST(TmpcTrajectoryBuilderTest, ConvertsRosYawToFormulaNorthZeroConvention)
{
  EXPECT_NEAR(convertHeading(0.0, HeadingConvention::kRosEastZero), -0.5 * kPi, 1.0e-12);
  EXPECT_NEAR(
    convertHeading(0.5 * kPi, HeadingConvention::kRosEastZero), 0.0, 1.0e-12);
  EXPECT_NEAR(
    convertHeading(-0.75, HeadingConvention::kFormulaNorthZero), -0.75, 1.0e-12);
}

TEST(TmpcTrajectoryBuilderTest, EmergencyCopiesGeometryAndEndsBelowHalfMeterPerSecond)
{
  const BuildResult result = buildCircle(100U);
  ASSERT_TRUE(result.success) << result.error;
  EXPECT_LE(result.emergency.v_mps.back(), 0.5);
  for (std::size_t i = 0U; i < kTrajectoryPointCount; ++i) {
    EXPECT_DOUBLE_EQ(result.emergency.s_loc_m[i], result.performance.s_loc_m[i]);
    EXPECT_DOUBLE_EQ(result.emergency.s_glob_m[i], result.performance.s_glob_m[i]);
    EXPECT_DOUBLE_EQ(result.emergency.x_m[i], result.performance.x_m[i]);
    EXPECT_DOUBLE_EQ(result.emergency.y_m[i], result.performance.y_m[i]);
    EXPECT_DOUBLE_EQ(result.emergency.psi_rad[i], result.performance.psi_rad[i]);
    EXPECT_DOUBLE_EQ(result.emergency.kappa_radpm[i], result.performance.kappa_radpm[i]);
    EXPECT_DOUBLE_EQ(result.emergency.banking_rad[i], result.performance.banking_rad[i]);
    EXPECT_DOUBLE_EQ(result.emergency.tube_l_m[i], result.performance.tube_l_m[i]);
    EXPECT_DOUBLE_EQ(result.emergency.tube_r_m[i], result.performance.tube_r_m[i]);
    EXPECT_GE(result.emergency.v_mps[i], 0.0);
    if (i > 0U) {
      EXPECT_LE(result.emergency.v_mps[i], result.emergency.v_mps[i - 1U]);
    }
  }
}

TEST(TmpcTrajectoryBuilderTest, RecomputesAccelerationFromFinalVelocityAndLocalS)
{
  const BuildResult result = buildCircle(100U);
  ASSERT_TRUE(result.success) << result.error;
  for (const auto * trajectory : {&result.performance, &result.emergency}) {
    for (std::size_t i = 0U; i + 1U < kTrajectoryPointCount; ++i) {
      const double ds = trajectory->s_loc_m[i + 1U] - trajectory->s_loc_m[i];
      const double expected =
        (trajectory->v_mps[i + 1U] * trajectory->v_mps[i + 1U] -
        trajectory->v_mps[i] * trajectory->v_mps[i]) / (2.0 * ds);
      EXPECT_NEAR(trajectory->ax_mps2[i], expected, 1.0e-12);
    }
    EXPECT_DOUBLE_EQ(trajectory->ax_mps2.back(), trajectory->ax_mps2[48]);
  }
}

TEST(TmpcTrajectoryBuilderTest, ReportsGgvEndpointClampingForPublisherWarning)
{
  const auto path = makeCircle(100U, 20.0, 7.0);
  std::string error;
  auto short_table = GgvTable::fromVectors(
    {0.0, 5.0}, {9.81, 9.81}, {20.0, 20.0}, &error);
  ASSERT_TRUE(short_table.has_value()) << error;
  TmpcTrajectoryBuilder builder(BuilderConfig{}, *short_table);
  BuildInput input;
  input.path = &path;
  input.current_s_m = 5.0;
  input.current_speed_mps = 7.0;
  input.traj_cnt = 1U;
  const BuildResult result = builder.build(input);
  ASSERT_TRUE(result.success) << result.error;
  EXPECT_TRUE(result.ggv_endpoint_clamped);
}

TEST(TmpcTrajectoryBuilderTest, ExpandsBothTrajectoriesToMaximumHorizonForEmergencyStop)
{
  BuilderConfig config;
  config.foresight_time_sec = 0.5;
  config.min_horizon_m = 10.0;
  config.max_horizon_m = 30.0;
  const auto path = makeCircle(200U, 50.0, 20.0);
  TmpcTrajectoryBuilder builder(config, makeGgv(8.0, 50.0));
  BuildInput input;
  input.path = &path;
  input.current_s_m = 5.0;
  input.current_speed_mps = 20.0;
  input.traj_cnt = 1U;
  const BuildResult result = builder.build(input);
  ASSERT_TRUE(result.success) << result.error;
  EXPECT_TRUE(result.used_max_horizon);
  EXPECT_DOUBLE_EQ(result.horizon_m, 30.0);
  EXPECT_LE(result.emergency.v_mps.back(), 0.5);
  EXPECT_GT(result.performance.s_loc_m.back(), 29.0);
}

TEST(TmpcTrajectoryBuilderTest, FailsClosedWhenMaximumHorizonCannotStopVehicle)
{
  BuilderConfig config;
  config.foresight_time_sec = 0.5;
  config.min_horizon_m = 10.0;
  config.max_horizon_m = 30.0;
  const auto path = makeCircle(200U, 50.0, 30.0);
  TmpcTrajectoryBuilder builder(config, makeGgv(3.0, 50.0));
  BuildInput input;
  input.path = &path;
  input.current_s_m = 5.0;
  input.current_speed_mps = 30.0;
  input.traj_cnt = 1U;
  const BuildResult result = builder.build(input);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.used_max_horizon || result.horizon_m == config.max_horizon_m);
  EXPECT_NE(result.error.find("terminal speed"), std::string::npos);
}

TEST(TmpcTrajectoryBuilderTest, RejectsNonFiniteAndDuplicateGeometry)
{
  auto path = makeCircle(100U, 20.0, 5.0);
  TmpcTrajectoryBuilder builder(BuilderConfig{}, makeGgv());
  BuildInput input;
  input.path = &path;
  input.current_speed_mps = 5.0;
  input.traj_cnt = 1U;

  path.waypoints[4].kappa_radpm = std::numeric_limits<double>::quiet_NaN();
  BuildResult result = builder.build(input);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error.find("non-finite"), std::string::npos);

  path = makeCircle(100U, 20.0, 5.0);
  for (std::size_t i = 0U; i < path.xs.size(); ++i) {
    path.xs[i] = 1.0;
    path.ys[i] = 1.0;
  }
  result = builder.build(input);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(
    result.error.find("duplicate") != std::string::npos ||
    result.error.find("closely spaced") != std::string::npos);
}

TEST(TmpcTrajectoryBuilderTest, RejectsSpanOfOneLapOrMore)
{
  BuilderConfig config;
  config.min_horizon_m = 30.0;
  config.max_horizon_m = 30.0;
  const auto path = makeCircle(30U, 4.0, 5.0);
  TmpcTrajectoryBuilder builder(config, makeGgv());
  BuildInput input;
  input.path = &path;
  input.current_speed_mps = 5.0;
  input.traj_cnt = 1U;
  const BuildResult result = builder.build(input);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error.find("full lap"), std::string::npos);
}

TEST(TmpcTrajectoryBuilderTest, RejectsEmergencyFinalSpeedAboveFormulaLimit)
{
  BuilderConfig config;
  config.emergency_final_speed_mps = 0.6;
  TmpcTrajectoryBuilder builder(config, makeGgv());
  std::string error;
  EXPECT_FALSE(builder.valid(&error));
  EXPECT_NE(error.find("<= 0.5"), std::string::npos);
}

TEST(TmpcTrajectoryBuilderTest, RejectsAccelerationUtilizationAboveFormulaTolerance)
{
  BuilderConfig config;
  config.acceleration_utilization_limit = 1.026;
  TmpcTrajectoryBuilder builder(config, makeGgv());
  std::string error;
  EXPECT_FALSE(builder.valid(&error));
  EXPECT_NE(error.find("<= 1.025"), std::string::npos);
}

TEST(TmpcTrajectoryBuilderTest, ReportsDetailedAccelerationUtilizationViolation)
{
  BuildResult result = buildCircle(100U);
  ASSERT_TRUE(result.success) << result.error;

  constexpr std::size_t violation_point = 7U;
  result.performance.kappa_radpm[violation_point] = 1.0;

  TmpcTrajectoryBuilder builder(BuilderConfig{}, makeGgv());
  std::string error;
  EXPECT_FALSE(builder.validateTrajectory(result.performance, false, &error));
  EXPECT_NE(error.find("Formula L1 acceleration-utilization limit"), std::string::npos);
  EXPECT_NE(error.find("trajectory=performance"), std::string::npos);
  EXPECT_NE(error.find("point=7"), std::string::npos);
  EXPECT_NE(error.find("utilization="), std::string::npos);
  EXPECT_NE(error.find("longitudinal_utilization="), std::string::npos);
  EXPECT_NE(error.find("lateral_utilization="), std::string::npos);
  EXPECT_NE(error.find("v_mps="), std::string::npos);
  EXPECT_NE(error.find("ax_mps2="), std::string::npos);
  EXPECT_NE(error.find("kappa_radpm="), std::string::npos);
  EXPECT_NE(error.find("ax_lim_mps2="), std::string::npos);
  EXPECT_NE(error.find("ay_lim_mps2="), std::string::npos);
  EXPECT_NE(error.find("s_loc_m="), std::string::npos);
  EXPECT_NE(error.find("s_glob_m="), std::string::npos);
}

}  // namespace
}  // namespace hyu_global_planner::tmpc
