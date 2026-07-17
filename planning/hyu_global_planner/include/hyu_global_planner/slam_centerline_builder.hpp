#pragma once

#include <string>
#include <vector>

#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "hyu_global_planner/planner_geometry.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace hyu_global_planner
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
  // Minimum-curvature raceline: solve for the lowest-total-curvature line the
  // cone corridor allows. 0 iterations = publish the pure centerline (previous
  // behavior). Every point keeps raceline_margin_m clearance to BOTH boundary
  // polylines, so the racing line is only as aggressive as the track width
  // allows. Iterations are solver sweeps over a convex objective, so more only
  // tightens convergence; a few hundred is plenty.
  int raceline_smoothing_iterations{0};
  double raceline_margin_m{1.2};
  // Unused. Kept so existing parameter files keep loading unchanged: the old
  // Laplacian step size has no meaning for the min-curvature solver.
  double raceline_alpha{0.3};
};

bool buildCenterlineFromSlamMap(
  const eufs_msgs::msg::ConeArrayWithCovariance & cone_map,
  const nav_msgs::msg::Odometry & ego_odom,
  const SlamCenterlineConfig & config,
  std::vector<PlannerWaypoint> & waypoints,
  std::string & reason);

// Minimum-curvature raceline (exposed for tests). Each point may slide along
// its own normal, and the offsets are solved to minimise total squared
// curvature inside the corridor -- so the line runs wide into a corner, clips
// the apex and tracks out again, which a curve-shortening/Laplacian pass cannot
// do (that only contracts the whole corner inward, cutting entry and exit
// alike). `centerline` is edited in place; a ring whose last point duplicates
// the first (closed loop) is optimised with wrap-around continuity, an open
// path keeps its endpoints fixed.
void applyMinimumCurvatureRaceline(
  std::vector<PlannerPoint> & centerline,
  const std::vector<PlannerPoint> & left_boundary,
  const std::vector<PlannerPoint> & right_boundary,
  const SlamCenterlineConfig & config);

}  // namespace hyu_global_planner
