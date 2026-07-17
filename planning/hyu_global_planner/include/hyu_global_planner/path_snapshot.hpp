#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "eufs_msgs/msg/waypoint.hpp"
#include "eufs_msgs/msg/waypoint_array_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"

namespace wpnt_publisher
{

struct PathSnapshot
{
  std::vector<eufs_msgs::msg::Waypoint> waypoints;
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> s;
  double track_length{0.0};
  std::string frame_id;
};

struct PathSnapshotOptions
{
  bool closed_loop{true};
  double closing_duplicate_tolerance{1.0e-3};
};

struct PathSnapshotBuildResult
{
  std::shared_ptr<PathSnapshot> snapshot;
  std::size_t input_count{0U};
  std::size_t unique_count{0U};
  bool closing_duplicate_removed{false};
  std::string error_message;
};

PathSnapshotBuildResult makePathSnapshot(
  const eufs_msgs::msg::WaypointArrayStamped & msg,
  const PathSnapshotOptions & options);

nav_msgs::msg::Path buildPathMessage(
  const PathSnapshot & snap,
  std::size_t start,
  std::size_t count,
  const std_msgs::msg::Header & header);

std::optional<int> parseIndex(const std::string & s);

}  // namespace wpnt_publisher
