#pragma once

// Bridges the vendored ksae_local synthetic-scenario fixtures (test_utils/,
// vendor/ksae_local/) to this planner. The fixtures produce sensor-frame
// observation frames along a moving vehicle; our planner consumes a latched
// SLAM cone map rendered into the ego frame (slam_map source mode). The
// MockSlamMap stands in for graph_slam: it accumulates every observation into
// an odom-frame map with radius deduplication, so scenario faults (dropout,
// color swaps, duplicates, noise) reach buildLocalPath the same way a real
// degraded map would.
//
// Vendored fixture code is kept verbatim (namespace ksae_local::test); only
// this adapter knows both worlds.

#include <cstddef>
#include <limits>
#include <vector>

#include "ksae_local/geometry.hpp"
#include "test_utils/synthetic_scenario.hpp"

#include "hyu_local_planner/local_path_builder.hpp"

namespace hyu_local_planner
{
namespace test
{

enum class MockColor
{
  kBlue,
  kYellow,
  kOrange,
  kBigOrange,
  kUnknown,
};

struct MockMapCone
{
  ksae_local::Point2D position_odom;
  MockColor color{MockColor::kUnknown};
};

struct MockSlamMapConfig
{
  // Re-observations of one physical cone merge into the first-seen position,
  // mirroring graph_slam's data association (its creation gate merges
  // observations well inside one cone spacing).
  double merge_radius_m{0.5};
};

// Deterministic argmax over the color probabilities; ties resolve in bucket
// declaration order so repeated runs bucket identically.
inline MockColor dominantColor(const ksae_local::ConeColorProbability & p)
{
  MockColor best_color = MockColor::kBlue;
  double best = p.blue;
  const auto consider = [&](double value, MockColor color) {
      if (value > best) {
        best = value;
        best_color = color;
      }
    };
  consider(p.yellow, MockColor::kYellow);
  consider(p.orange, MockColor::kOrange);
  consider(p.big_orange, MockColor::kBigOrange);
  consider(p.unknown, MockColor::kUnknown);
  return best_color;
}

class MockSlamMap
{
public:
  MockSlamMap() = default;
  explicit MockSlamMap(MockSlamMapConfig config)
  : config_(config) {}

  void ingestFrame(const ksae_local::test::ScenarioFrame & frame)
  {
    for (const auto & observation : frame.observations_sensor) {
      const ksae_local::Point2D odom =
        ksae_local::transformPoint(observation.position_sensor, frame.sensor_pose_odom);
      upsert(odom, dominantColor(observation.color_probability));
    }
  }

  void ingestScenario(const ksae_local::test::SyntheticScenario & scenario)
  {
    for (const auto & frame : scenario.frames) {
      ingestFrame(frame);
    }
  }

  const std::vector<MockMapCone> & cones() const noexcept {return cones_;}

  // Render the accumulated map into the ego frame of `ego_pose_odom`, exactly
  // as ros_inputs does with the latched map + odom pose. ROI cropping is the
  // planner's job, not the map's.
  ConeSet renderEgoFrame(const ksae_local::Pose2D & ego_pose_odom) const
  {
    ConeSet cones;
    for (const auto & cone : cones_) {
      const ksae_local::Point2D ego =
        ksae_local::inverseTransformPoint(cone.position_odom, ego_pose_odom);
      const Point2 point{ego.x, ego.y};
      switch (cone.color) {
        case MockColor::kBlue:
          cones.blue.push_back(point);
          break;
        case MockColor::kYellow:
          cones.yellow.push_back(point);
          break;
        case MockColor::kOrange:
          cones.orange.push_back(point);
          break;
        case MockColor::kBigOrange:
          cones.big_orange.push_back(point);
          break;
        case MockColor::kUnknown:
          cones.unknown.push_back(point);
          break;
      }
    }
    return cones;
  }

private:
  void upsert(const ksae_local::Point2D & position_odom, MockColor color)
  {
    MockMapCone * nearest = nullptr;
    double best = config_.merge_radius_m;
    for (auto & cone : cones_) {
      const double gap = ksae_local::distance(cone.position_odom, position_odom);
      if (gap <= best) {
        best = gap;
        nearest = &cone;
      }
    }
    if (nearest == nullptr) {
      cones_.push_back({position_odom, color});
      return;
    }
    // Keep the first-seen position (a latched map does not jitter). A known
    // color upgrades an unknown landmark; conflicting known colors keep the
    // incumbent, mirroring a stable association.
    if (nearest->color == MockColor::kUnknown && color != MockColor::kUnknown) {
      nearest->color = color;
    }
  }

  MockSlamMapConfig config_;
  std::vector<MockMapCone> cones_;
};

inline PlannerConfig slamModeConfig()
{
  PlannerConfig config;
  config.allow_partial_boundary = true;  // slam_map source mode
  return config;
}

// Accumulate the whole scenario into a mock map and plan from the sensor pose
// of `frame_index` (default: the last frame). This is the canonical way a
// scenario reaches buildLocalPath.
inline BuildResult planScenario(
  const ksae_local::test::SyntheticScenario & scenario,
  std::size_t frame_index = std::numeric_limits<std::size_t>::max(),
  const PlannerConfig & config = slamModeConfig(),
  MockSlamMapConfig map_config = MockSlamMapConfig{})
{
  MockSlamMap map(map_config);
  map.ingestScenario(scenario);
  if (scenario.frames.empty()) {
    return buildLocalPath(ConeSet{}, config);
  }
  if (frame_index >= scenario.frames.size()) {
    frame_index = scenario.frames.size() - 1U;
  }
  const auto & ego_pose = scenario.frames[frame_index].sensor_pose_odom;
  return buildLocalPath(map.renderEgoFrame(ego_pose), config);
}

}  // namespace test
}  // namespace hyu_local_planner
