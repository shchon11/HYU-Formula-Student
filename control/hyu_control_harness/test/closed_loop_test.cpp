#include <gtest/gtest.h>

#include <cmath>

#include "hyu_control_harness/closed_loop.hpp"
#include "hyu_control_harness/track.hpp"

namespace ch = hyu_control_harness;

namespace
{

// Circular test track: centerline radius 15 m, width 3.5 m. Cones every ~2.6 m
// so the local planner's Delaunay pairing has real work to do.
ch::Track circularTrack()
{
  ch::Track track;
  constexpr double kRadius = 15.0;
  constexpr double kHalfWidth = 1.75;
  constexpr int kCones = 36;
  for (int i = 0; i < kCones; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / kCones;
    track.blue.push_back(
      {(kRadius + kHalfWidth) * std::cos(angle), (kRadius + kHalfWidth) * std::sin(angle)});
    track.yellow.push_back(
      {(kRadius - kHalfWidth) * std::cos(angle), (kRadius - kHalfWidth) * std::sin(angle)});
  }
  track.start = {kRadius, 0.0, M_PI / 2.0};
  return track;
}

ch::HarnessConfig baseConfig()
{
  ch::HarnessConfig config;
  config.plant_yaml = PLANT_CONFIG_PATH;
  config.sim_time_s = 60.0;
  config.controller = ch::trackdriveControllerConfig();
  config.lut_model = ch::trackdriveLutModel();
  config.lut_grid = ch::trackdriveLutGrid();
  config.planner = ch::trackdrivePlannerConfig();
  return config;
}

}  // namespace

TEST(ClosedLoop, MapModeLapsACircleInsideTheCones)
{
  const auto result = ch::runMapHarness(baseConfig(), circularTrack());
  EXPECT_FALSE(result.dnf) << result.dnf_reason << " / " << result.last_planner_reason;
  EXPECT_GE(result.laps, 1);
  EXPECT_LT(result.cte_rmse_m, 0.6);
  EXPECT_GT(result.avg_speed_mps, 1.0);
}

TEST(ClosedLoop, GeometricModeAlsoCompletes)
{
  auto config = baseConfig();
  config.controller.steering_mode = hyu_pure_pursuit::SteeringMode::GEOMETRIC;
  config.controller.lookahead_m = 3.5;
  const auto result = ch::runMapHarness(config, circularTrack());
  EXPECT_FALSE(result.dnf) << result.dnf_reason << " / " << result.last_planner_reason;
  EXPECT_GE(result.laps, 1);
}

TEST(ClosedLoop, ResultSerializesToJson)
{
  ch::HarnessResult result;
  result.laps = 2;
  result.lap_times_s = {30.5, 29.75};
  const std::string json = result.toJson();
  EXPECT_NE(json.find("\"laps\":2"), std::string::npos);
  EXPECT_NE(json.find("\"lap_times_s\":[30.5,29.75]"), std::string::npos);
}
