#pragma once

#include <string>
#include <vector>

#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "global_planner/planner_geometry.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace global_planner
{

struct SlamCenterlineConfig
{
  int min_cones_per_side{3};
  double max_boundary_gap_m{12.0};
  double min_track_width_m{2.0};
  double max_track_width_m{6.0};
  double close_loop_distance_m{5.0};
  double waypoint_spacing_m{0.5};
  double duplicate_point_tolerance{0.001};
  // Raceline smoothing: pull the centerline toward a minimum-curvature line
  // inside the cone corridor. 0 iterations = publish the pure centerline
  // (previous behavior). Each point's total displacement is capped so it
  // keeps raceline_margin_m clearance to BOTH boundary polylines, so the
  // "racing line" is only as aggressive as the track width allows.
  int raceline_smoothing_iterations{0};
  double raceline_margin_m{1.2};
  // Laplacian step size per iteration; stable for values well below 0.5.
  double raceline_alpha{0.3};
};

bool buildCenterlineFromSlamMap(
  const eufs_msgs::msg::ConeArrayWithCovariance & cone_map,
  const nav_msgs::msg::Odometry & ego_odom,
  const SlamCenterlineConfig & config,
  std::vector<PlannerWaypoint> & waypoints,
  std::string & reason);

// Curvature-energy descent with a per-point corridor clamp (exposed for
// tests). `centerline` is edited in place; a ring whose last point duplicates
// the first (closed loop) is smoothed with wrap-around continuity, an open
// path keeps its endpoints fixed.
void applyRacelineSmoothing(
  std::vector<PlannerPoint> & centerline,
  const std::vector<PlannerPoint> & left_boundary,
  const std::vector<PlannerPoint> & right_boundary,
  const SlamCenterlineConfig & config);

}  // namespace global_planner
