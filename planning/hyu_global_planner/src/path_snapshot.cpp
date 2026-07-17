#include "hyu_global_planner/path_snapshot.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace wpnt_publisher
{
namespace
{

void resolveXY(const eufs_msgs::msg::Waypoint & waypoint, double & x, double & y)
{
  x = waypoint.x_m;
  y = waypoint.y_m;
  if (x == 0.0 && y == 0.0 &&
    (waypoint.position.x != 0.0 || waypoint.position.y != 0.0))
  {
    x = waypoint.position.x;
    y = waypoint.position.y;
  }
}

std::string monotonicError(std::size_t index, double current, double previous)
{
  std::array<char, 160> buffer{};
  std::snprintf(
    buffer.data(), buffer.size(),
    "Global waypoints s_m not strictly increasing at index %zu (%.6f <= %.6f); rejecting snapshot.",
    index, current, previous);
  return buffer.data();
}

}  // namespace

PathSnapshotBuildResult makePathSnapshot(
  const eufs_msgs::msg::WaypointArrayStamped & msg,
  const PathSnapshotOptions & options)
{
  PathSnapshotBuildResult result;
  const auto & waypoints = msg.waypoints;
  result.input_count = waypoints.size();
  if (waypoints.size() < 2U) {
    result.error_message =
      "Received " + std::to_string(waypoints.size()) + " global waypoint(s); need at least 2.";
    return result;
  }

  double front_x = 0.0;
  double front_y = 0.0;
  double back_x = 0.0;
  double back_y = 0.0;
  resolveXY(waypoints.front(), front_x, front_y);
  resolveXY(waypoints.back(), back_x, back_y);
  const double front_back = std::hypot(back_x - front_x, back_y - front_y);
  result.closing_duplicate_removed = front_back <= options.closing_duplicate_tolerance;
  result.unique_count = result.closing_duplicate_removed ? waypoints.size() - 1U : waypoints.size();
  if (result.unique_count < 2U) {
    result.error_message = "Fewer than 2 unique waypoints after closing-duplicate removal.";
    return result;
  }

  auto snap = std::make_shared<PathSnapshot>();
  snap->frame_id = msg.header.frame_id.empty() ? "map" : msg.header.frame_id;
  snap->waypoints.reserve(result.unique_count);
  snap->xs.reserve(result.unique_count);
  snap->ys.reserve(result.unique_count);
  snap->s.reserve(result.unique_count);
  for (std::size_t i = 0; i < result.unique_count; ++i) {
    double x = 0.0;
    double y = 0.0;
    resolveXY(waypoints[i], x, y);
    snap->waypoints.push_back(waypoints[i]);
    snap->xs.push_back(x);
    snap->ys.push_back(y);
    snap->s.push_back(waypoints[i].s_m);
  }

  for (std::size_t i = 1; i < result.unique_count; ++i) {
    if (!(snap->s[i] > snap->s[i - 1])) {
      result.error_message = monotonicError(i, snap->s[i], snap->s[i - 1]);
      return result;
    }
  }

  if (result.closing_duplicate_removed) {
    snap->track_length = waypoints.back().s_m;
  } else if (options.closed_loop) {
    snap->track_length = waypoints.back().s_m + front_back;
  } else {
    snap->track_length = waypoints.back().s_m;
  }
  result.snapshot = std::move(snap);
  return result;
}

nav_msgs::msg::Path buildPathMessage(
  const PathSnapshot & snap,
  std::size_t start,
  std::size_t count,
  const std_msgs::msg::Header & header)
{
  const std::size_t n = snap.s.size();
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(count);

  double prev_yaw = 0.0;
  for (std::size_t k = 0; k < count; ++k) {
    const std::size_t gi = (start + k) % n;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = snap.xs[gi];
    pose.pose.position.y = snap.ys[gi];
    pose.pose.position.z = 0.0;

    double yaw = prev_yaw;
    if (k + 1 < count) {
      const std::size_t gj = (start + k + 1) % n;
      const double dx = snap.xs[gj] - snap.xs[gi];
      const double dy = snap.ys[gj] - snap.ys[gi];
      if (std::hypot(dx, dy) >= 1.0e-9) {
        yaw = std::atan2(dy, dx);
      }
    }
    prev_yaw = yaw;
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    path.poses.push_back(std::move(pose));
  }
  return path;
}

std::optional<int> parseIndex(const std::string & s)
{
  std::size_t i = 0;
  std::size_t j = s.size();
  while (i < j && std::isspace(static_cast<unsigned char>(s[i]))) {
    ++i;
  }
  while (j > i && std::isspace(static_cast<unsigned char>(s[j - 1]))) {
    --j;
  }
  if (i >= j) {
    return std::nullopt;
  }

  std::string trimmed = s.substr(i, j - i);
  std::size_t k = 0;
  if (trimmed[0] == '+' || trimmed[0] == '-') {
    k = 1;
  }
  if (k == trimmed.size()) {
    return std::nullopt;
  }
  for (; k < trimmed.size(); ++k) {
    if (!std::isdigit(static_cast<unsigned char>(trimmed[k]))) {
      return std::nullopt;
    }
  }
  try {
    return std::stoi(trimmed);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

}  // namespace wpnt_publisher
