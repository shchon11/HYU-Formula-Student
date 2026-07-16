#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "global_planner/planner_geometry.hpp"
#include "global_planner/slam_boundary_ordering.hpp"
#include "global_planner/slam_centerline_builder.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace global_planner::test
{

inline std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

inline std::string fixturePath(const std::string & relative)
{
  return std::string(GLOBAL_PLANNER_SRC_ROOT) + "/" + relative;
}

inline eufs_msgs::msg::ConeArrayWithCovariance loadConeMapCsv(const std::string & relative)
{
  std::ifstream file(fixturePath(relative));
  if (!file.is_open()) {
    throw std::runtime_error("could not open fixture: " + relative);
  }

  eufs_msgs::msg::ConeArrayWithCovariance map;
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = splitCsvLine(line);
    if (fields.size() < 3U) {
      throw std::runtime_error("bad CSV fixture row: " + line);
    }

    eufs_msgs::msg::ConeWithCovariance cone;
    cone.point.x = std::stod(fields[1]);
    cone.point.y = std::stod(fields[2]);
    cone.point.z = 0.0;

    if (fields[0] == "blue") {
      map.blue_cones.push_back(cone);
    } else if (fields[0] == "yellow") {
      map.yellow_cones.push_back(cone);
    }
  }
  return map;
}

inline nav_msgs::msg::Odometry egoAtOrigin()
{
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  return odom;
}

inline SlamCenterlineConfig fixtureConfig()
{
  SlamCenterlineConfig config;
  config.min_cones_per_side = 3;
  config.max_boundary_gap_m = 12.0;
  config.min_track_width_m = 2.0;
  config.max_track_width_m = 6.0;
  config.close_loop_distance_m = 5.0;
  config.waypoint_spacing_m = 0.5;
  config.duplicate_point_tolerance = 0.001;
  return config;
}

inline std::vector<PlannerPoint> bluePoints(const eufs_msgs::msg::ConeArrayWithCovariance & map)
{
  std::vector<PlannerPoint> points;
  points.reserve(map.blue_cones.size());
  for (const auto & cone : map.blue_cones) {
    points.push_back({cone.point.x, cone.point.y});
  }
  return points;
}

inline std::vector<PlannerPoint> yellowPoints(const eufs_msgs::msg::ConeArrayWithCovariance & map)
{
  std::vector<PlannerPoint> points;
  points.reserve(map.yellow_cones.size());
  for (const auto & cone : map.yellow_cones) {
    points.push_back({cone.point.x, cone.point.y});
  }
  return points;
}

inline double nearestDistance(const PlannerPoint & point, const std::vector<PlannerPoint> & candidates)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & candidate : candidates) {
    best = std::min(best, distance(point, candidate));
  }
  return best;
}

inline bool segmentsCross(
  const PlannerPoint & a, const PlannerPoint & b,
  const PlannerPoint & c, const PlannerPoint & d)
{
  const auto orientation = [](const PlannerPoint & p, const PlannerPoint & q, const PlannerPoint & r) {
      return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    };
  const double ab_c = orientation(a, b, c);
  const double ab_d = orientation(a, b, d);
  const double cd_a = orientation(c, d, a);
  const double cd_b = orientation(c, d, b);
  return ab_c * ab_d < -1.0e-9 && cd_a * cd_b < -1.0e-9;
}

inline bool hasSelfIntersection(const std::vector<PlannerWaypoint> & waypoints)
{
  if (waypoints.size() < 4U) {
    return false;
  }
  const bool closed = distance(
    {waypoints.front().x, waypoints.front().y},
    {waypoints.back().x, waypoints.back().y}) <= 0.001;
  for (std::size_t i = 0; i + 1U < waypoints.size(); ++i) {
    for (std::size_t j = i + 2U; j + 1U < waypoints.size(); ++j) {
      if (j == i + 1U) {
        continue;
      }
      if (closed && i == 0U && j + 1U == waypoints.size() - 1U) {
        continue;
      }
      const PlannerPoint a{waypoints[i].x, waypoints[i].y};
      const PlannerPoint b{waypoints[i + 1U].x, waypoints[i + 1U].y};
      const PlannerPoint c{waypoints[j].x, waypoints[j].y};
      const PlannerPoint d{waypoints[j + 1U].x, waypoints[j + 1U].y};
      if (segmentsCross(a, b, c, d)) {
        return true;
      }
    }
  }
  return false;
}

inline PlannerPoint closestPointOnFixturePolyline(
  const PlannerPoint & query, const std::vector<PlannerPoint> & polyline)
{
  PlannerPoint best = polyline.front();
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1U < polyline.size(); ++i) {
    const PlannerPoint & a = polyline[i];
    const PlannerPoint & b = polyline[i + 1U];
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len2 = vx * vx + vy * vy;
    double t = 0.0;
    if (len2 > 0.0) {
      t = std::clamp(((query.x - a.x) * vx + (query.y - a.y) * vy) / len2, 0.0, 1.0);
    }
    const PlannerPoint projected{a.x + t * vx, a.y + t * vy};
    const double dx = query.x - projected.x;
    const double dy = query.y - projected.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = projected;
    }
  }
  return best;
}

inline std::vector<double> fixturePairingWidths(
  const eufs_msgs::msg::ConeArrayWithCovariance & map,
  const SlamCenterlineConfig & config)
{
  std::vector<PlannerPoint> ordered_blue;
  std::vector<PlannerPoint> ordered_yellow;
  std::string reason;
  if (!orderSlamBoundaries(
      bluePoints(map), yellowPoints(map), {0.0, 0.0}, config.max_boundary_gap_m,
      ordered_blue, ordered_yellow, reason))
  {
    throw std::runtime_error(reason);
  }

  const auto blue_arc = cumulativeArcLengths(ordered_blue);
  const std::size_t pair_count = std::max<std::size_t>(
    2U, static_cast<std::size_t>(std::ceil(blue_arc.back() / config.waypoint_spacing_m)) + 1U);
  std::vector<double> widths;
  widths.reserve(pair_count);
  for (std::size_t i = 0; i < pair_count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(pair_count - 1U);
    const auto blue = interpolateAtArcLength(ordered_blue, blue_arc, ratio * blue_arc.back());
    widths.push_back(distance(blue, closestPointOnFixturePolyline(blue, ordered_yellow)));
  }
  return widths;
}

}  // namespace global_planner::test
