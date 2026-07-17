#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "hyu_local_planner/local_path_builder.hpp"
#include "../src/local_path_builder_internal.hpp"

namespace hyu_local_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

PlannerConfig slamModeConfig()
{
  PlannerConfig config;
  config.allow_partial_boundary = true;  // slam_map source mode
  return config;
}

double headingStep(const PathWaypoint & a, const PathWaypoint & b)
{
  double step = std::abs(b.psi - a.psi);
  if (step > kPi) {
    step = 2.0 * kPi - step;
  }
  return step;
}

double maxHeadingStep(const std::vector<PathWaypoint> & waypoints)
{
  double worst = 0.0;
  for (std::size_t i = 1; i + 1U < waypoints.size(); ++i) {
    worst = std::max(worst, headingStep(waypoints[i - 1U], waypoints[i]));
  }
  return worst;
}

// Regression for the chicane fold: a Z that bridges onto the adjacent
// corridor leg needs a sharp turn immediately followed by a sharp turn of the
// OPPOSITE sign. The traversal must stop at the fold instead of chaining it.
TEST(ChicaneFold, OppositeSignSharpTurnStopsTheChain)
{
  const std::vector<Point2> points{
    {0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}, {6.0, 0.0},
    {6.2, 2.5},    // sharp LEFT off the leg (u-turn class)
    // Sharp RIGHT from the previous link's tangent, and out of traversal-gap
    // reach from the straight leg so it cannot be taken as a normal link.
    {12.1, 3.1},
  };
  const auto result = internal::forwardTraversalWithReason(points, slamModeConfig());
  EXPECT_EQ(result.failure, internal::TraversalFailure::kFoldBack);
  EXPECT_EQ(result.points.size(), 5U);  // chain ends before the folding link
}

// A hairpin rounds with consecutive sharp turns of the SAME sign and must
// keep chaining exactly as before.
TEST(ChicaneFold, HairpinSameSignSharpTurnsStillChain)
{
  const std::vector<Point2> points{
    {0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}, {6.0, 0.0},
    {6.3, 2.5},   // sharp LEFT into the hairpin
    {4.3, 2.8},   // return lane, still turning left
    {2.3, 3.0},
  };
  const auto result = internal::forwardTraversalWithReason(points, slamModeConfig());
  EXPECT_EQ(result.failure, internal::TraversalFailure::kNone);
  EXPECT_EQ(result.points.size(), points.size());
}

// Full-track sweep on boa_constrictor (the chicane track where the fold was
// observed live): plan from dense ego poses with lateral/heading offsets and
// degraded (sub-sampled) cone maps; no valid output may fold back on itself.
class BoaSweep : public ::testing::Test
{
protected:
  struct Pose
  {
    double x;
    double y;
    double yaw;
  };

  static std::vector<std::array<double, 2>> blue_;
  static std::vector<std::array<double, 2>> yellow_;
  static std::vector<Pose> poses_;

  static void SetUpTestSuite()
  {
    std::ifstream file(BOA_TRACK_CSV);
    ASSERT_TRUE(file.is_open()) << BOA_TRACK_CSV;
    std::string line;
    std::getline(file, line);
    double start_x = 0.0;
    double start_y = 0.0;
    while (std::getline(file, line)) {
      std::stringstream stream(line);
      std::string tag, x_text, y_text;
      std::getline(stream, tag, ',');
      std::getline(stream, x_text, ',');
      std::getline(stream, y_text, ',');
      if (tag != "blue" && tag != "yellow" && tag != "car_start") {
        continue;
      }
      const double x = std::stod(x_text);
      const double y = std::stod(y_text);
      if (tag == "blue") {
        blue_.push_back({x, y});
      } else if (tag == "yellow") {
        yellow_.push_back({x, y});
      } else {
        start_x = x;
        start_y = y;
      }
    }

    // Midline: nearest-neighbour walk over the blue ring from the start,
    // each blue paired with its nearest yellow.
    std::vector<bool> used(blue_.size(), false);
    std::size_t current = 0U;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < blue_.size(); ++i) {
      const double d = std::hypot(blue_[i][0] - start_x, blue_[i][1] - start_y);
      if (d < best) {
        best = d;
        current = i;
      }
    }
    std::vector<std::array<double, 2>> midline;
    for (std::size_t count = 0; count < blue_.size(); ++count) {
      used[current] = true;
      double nearest = std::numeric_limits<double>::infinity();
      std::array<double, 2> mate{0.0, 0.0};
      for (const auto & cone : yellow_) {
        const double d = std::hypot(cone[0] - blue_[current][0], cone[1] - blue_[current][1]);
        if (d < nearest) {
          nearest = d;
          mate = cone;
        }
      }
      midline.push_back(
        {0.5 * (blue_[current][0] + mate[0]), 0.5 * (blue_[current][1] + mate[1])});
      std::size_t next = blue_.size();
      double next_gap = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < blue_.size(); ++i) {
        if (used[i]) {
          continue;
        }
        const double d =
          std::hypot(blue_[i][0] - blue_[current][0], blue_[i][1] - blue_[current][1]);
        if (d < next_gap) {
          next_gap = d;
          next = i;
        }
      }
      if (next == blue_.size()) {
        break;
      }
      current = next;
    }

    for (std::size_t i = 0; i + 1U < midline.size(); ++i) {
      const double yaw = std::atan2(
        midline[i + 1U][1] - midline[i][1], midline[i + 1U][0] - midline[i][0]);
      poses_.push_back({midline[i][0], midline[i][1], yaw});
    }
    ASSERT_GT(poses_.size(), 20U);
  }

  static ConeSet vehicleFrame(
    const Pose & pose, std::size_t drop_modulo, std::size_t drop_phase)
  {
    ConeSet cones;
    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    const auto transform =
      [&](const std::vector<std::array<double, 2>> & source, std::vector<Point2> & sink) {
        for (std::size_t i = 0; i < source.size(); ++i) {
          if (drop_modulo > 0U && i % drop_modulo == drop_phase) {
            continue;  // emulate a partially-built SLAM map
          }
          const double dx = source[i][0] - pose.x;
          const double dy = source[i][1] - pose.y;
          sink.push_back({cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy});
        }
      };
    transform(blue_, cones.blue);
    transform(yellow_, cones.yellow);
    return cones;
  }
};

std::vector<std::array<double, 2>> BoaSweep::blue_;
std::vector<std::array<double, 2>> BoaSweep::yellow_;
std::vector<BoaSweep::Pose> BoaSweep::poses_;

TEST_F(BoaSweep, NoValidPathFoldsBack)
{
  const auto config = slamModeConfig();
  std::size_t valid_count = 0U;
  for (const auto & base : poses_) {
    for (const double lateral : {0.0, 0.4, -0.4}) {
      for (const double yaw_offset : {0.0, 0.17, -0.17}) {
        for (const std::size_t drop : {std::size_t{0}, std::size_t{3}}) {
          const Pose pose{
            base.x - std::sin(base.yaw) * lateral,
            base.y + std::cos(base.yaw) * lateral,
            base.yaw + yaw_offset};
          const auto result =
            buildLocalPath(vehicleFrame(pose, drop, drop == 0U ? 1U : 0U), config);
          if (!result.valid) {
            continue;
          }
          ++valid_count;
          EXPECT_LT(maxHeadingStep(result.waypoints), 0.5 * kPi)
            << "fold at ego=(" << pose.x << "," << pose.y << ") yaw=" << pose.yaw
            << " lateral=" << lateral << " drop=" << drop;
        }
      }
    }
  }
  // The sweep must actually exercise the planner, not vacuously pass.
  EXPECT_GT(valid_count, poses_.size());
}

}  // namespace
}  // namespace hyu_local_planner
