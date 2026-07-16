#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "local_planner/local_path_builder.hpp"

// Stage-0 golden gate: buildLocalPath output at 20 ego poses spread around the
// boa_constrictor track, captured verbatim (17 significant digits — exact
// double round-trip). Every later stage that touches the builder must leave
// this file byte-identical, or regenerate it as an explicitly reviewed,
// deliberate behavior change:
//
//   LOCAL_PLANNER_WRITE_GOLDEN=1 ./local_planner_boa_golden_test
//
// The pose set reuses the chicane_fold_test midline walk (nearest-neighbour
// over the blue ring, each blue paired with its nearest yellow) subsampled to
// 20 poses, with the full undegraded map in view. Orange/big-orange cones ride
// along in the ConeSet so the same harness covers upcoming orange-cone work;
// under today's default config the builder ignores them.

namespace local_planner
{
namespace
{

struct Pose
{
  double x;
  double y;
  double yaw;
};

struct BoaTrack
{
  std::vector<std::array<double, 2>> blue;
  std::vector<std::array<double, 2>> yellow;
  std::vector<std::array<double, 2>> orange;
  std::vector<std::array<double, 2>> big_orange;
  double start_x{0.0};
  double start_y{0.0};
};

BoaTrack loadTrack()
{
  BoaTrack track;
  std::ifstream file(BOA_TRACK_CSV);
  EXPECT_TRUE(file.is_open()) << BOA_TRACK_CSV;
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::stringstream stream(line);
    std::string tag, x_text, y_text;
    std::getline(stream, tag, ',');
    std::getline(stream, x_text, ',');
    std::getline(stream, y_text, ',');
    if (tag != "blue" && tag != "yellow" && tag != "orange" &&
      tag != "big_orange" && tag != "car_start")
    {
      continue;
    }
    const double x = std::stod(x_text);
    const double y = std::stod(y_text);
    if (tag == "blue") {
      track.blue.push_back({x, y});
    } else if (tag == "yellow") {
      track.yellow.push_back({x, y});
    } else if (tag == "orange") {
      track.orange.push_back({x, y});
    } else if (tag == "big_orange") {
      track.big_orange.push_back({x, y});
    } else {
      track.start_x = x;
      track.start_y = y;
    }
  }
  return track;
}

// Midline walk identical to chicane_fold_test's BoaSweep::SetUpTestSuite.
std::vector<Pose> midlinePoses(const BoaTrack & track)
{
  std::vector<bool> used(track.blue.size(), false);
  std::size_t current = 0U;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < track.blue.size(); ++i) {
    const double d =
      std::hypot(track.blue[i][0] - track.start_x, track.blue[i][1] - track.start_y);
    if (d < best) {
      best = d;
      current = i;
    }
  }
  std::vector<std::array<double, 2>> midline;
  for (std::size_t count = 0; count < track.blue.size(); ++count) {
    used[current] = true;
    double nearest = std::numeric_limits<double>::infinity();
    std::array<double, 2> mate{0.0, 0.0};
    for (const auto & cone : track.yellow) {
      const double d =
        std::hypot(cone[0] - track.blue[current][0], cone[1] - track.blue[current][1]);
      if (d < nearest) {
        nearest = d;
        mate = cone;
      }
    }
    midline.push_back(
      {0.5 * (track.blue[current][0] + mate[0]), 0.5 * (track.blue[current][1] + mate[1])});
    std::size_t next = track.blue.size();
    double next_gap = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < track.blue.size(); ++i) {
      if (used[i]) {
        continue;
      }
      const double d = std::hypot(
        track.blue[i][0] - track.blue[current][0],
        track.blue[i][1] - track.blue[current][1]);
      if (d < next_gap) {
        next_gap = d;
        next = i;
      }
    }
    if (next == track.blue.size()) {
      break;
    }
    current = next;
  }

  std::vector<Pose> poses;
  for (std::size_t i = 0; i + 1U < midline.size(); ++i) {
    const double yaw = std::atan2(
      midline[i + 1U][1] - midline[i][1], midline[i + 1U][0] - midline[i][0]);
    poses.push_back({midline[i][0], midline[i][1], yaw});
  }
  return poses;
}

std::vector<Pose> goldenPoses(const BoaTrack & track, std::size_t target_count)
{
  const std::vector<Pose> all = midlinePoses(track);
  EXPECT_GE(all.size(), target_count);
  std::vector<Pose> picked;
  const std::size_t stride = std::max<std::size_t>(1U, all.size() / target_count);
  for (std::size_t i = 0; i < all.size() && picked.size() < target_count; i += stride) {
    picked.push_back(all[i]);
  }
  return picked;
}

ConeSet vehicleFrame(const BoaTrack & track, const Pose & pose)
{
  ConeSet cones;
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  const auto transform =
    [&](const std::vector<std::array<double, 2>> & source, std::vector<Point2> & sink) {
      for (const auto & cone : source) {
        const double dx = cone[0] - pose.x;
        const double dy = cone[1] - pose.y;
        sink.push_back({cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy});
      }
    };
  transform(track.blue, cones.blue);
  transform(track.yellow, cones.yellow);
  transform(track.orange, cones.orange);
  transform(track.big_orange, cones.big_orange);
  return cones;
}

PlannerConfig slamModeConfig()
{
  PlannerConfig config;
  config.allow_partial_boundary = true;  // slam_map source mode
  return config;
}

std::string renderGolden()
{
  const BoaTrack track = loadTrack();
  const std::vector<Pose> poses = goldenPoses(track, 20U);
  const PlannerConfig config = slamModeConfig();

  std::ostringstream out;
  out << std::setprecision(17);
  out << "# boa_constrictor golden waypoints; regenerate with"
      << " LOCAL_PLANNER_WRITE_GOLDEN=1\n";
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const Pose & pose = poses[i];
    const BuildResult result = buildLocalPath(vehicleFrame(track, pose), config);
    out << "pose " << i << ' ' << pose.x << ' ' << pose.y << ' ' << pose.yaw
        << " valid=" << (result.valid ? 1 : 0)
        << " kind=" << static_cast<int>(result.kind)
        << " n=" << result.waypoints.size();
    if (!result.valid) {
      out << " reason=" << result.reason;
    }
    out << '\n';
    for (std::size_t j = 0; j < result.waypoints.size(); ++j) {
      const PathWaypoint & waypoint = result.waypoints[j];
      out << "wp " << i << ' ' << j << ' ' << waypoint.x << ' ' << waypoint.y
          << ' ' << waypoint.s << ' ' << waypoint.psi << ' ' << waypoint.kappa
          << ' ' << waypoint.speed << '\n';
    }
  }
  return out.str();
}

TEST(BoaGolden, WaypointsMatchGoldenBaseline)
{
  const std::string current = renderGolden();
  ASSERT_FALSE(current.empty());

  if (std::getenv("LOCAL_PLANNER_WRITE_GOLDEN") != nullptr) {
    std::ofstream out(BOA_GOLDEN_PATH, std::ios::trunc);
    ASSERT_TRUE(out.is_open()) << BOA_GOLDEN_PATH;
    out << current;
    std::cout << "[golden] wrote " << BOA_GOLDEN_PATH << "\n";
    return;
  }

  std::ifstream in(BOA_GOLDEN_PATH);
  ASSERT_TRUE(in.is_open())
    << BOA_GOLDEN_PATH << " missing; run once with LOCAL_PLANNER_WRITE_GOLDEN=1";
  std::stringstream stored;
  stored << in.rdbuf();

  if (current == stored.str()) {
    SUCCEED();
    return;
  }
  // Point at the first differing line so an unintended builder change is
  // diagnosable without diffing by hand.
  std::istringstream lhs(current);
  std::istringstream rhs(stored.str());
  std::string lhs_line, rhs_line;
  std::size_t line_number = 0U;
  while (true) {
    const bool lhs_ok = static_cast<bool>(std::getline(lhs, lhs_line));
    const bool rhs_ok = static_cast<bool>(std::getline(rhs, rhs_line));
    ++line_number;
    if (!lhs_ok && !rhs_ok) {
      break;
    }
    if (!lhs_ok || !rhs_ok || lhs_line != rhs_line) {
      FAIL() << "golden mismatch at line " << line_number << "\n  current: "
             << (lhs_ok ? lhs_line : "<eof>") << "\n  golden:  "
             << (rhs_ok ? rhs_line : "<eof>")
             << "\nIf this change is intentional, regenerate with"
             << " LOCAL_PLANNER_WRITE_GOLDEN=1 and review the diff.";
    }
  }
  FAIL() << "golden content mismatch";  // unreachable guard
}

}  // namespace
}  // namespace local_planner
