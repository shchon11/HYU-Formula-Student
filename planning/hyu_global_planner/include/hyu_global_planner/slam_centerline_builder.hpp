#pragma once

#include <string>
#include <vector>

#include "hyu_msgs/msg/cone_array_with_covariance.hpp"
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
  // Speed-weighted raceline: weight each point's squared-curvature penalty by
  // (v_max / v_i)^exponent, where v_i comes from a drivable speed profile
  // (friction-circle corner speeds + accel/decel passes) over the CURRENT
  // line. Curvature then costs most where the car actually spends time -- slow
  // hairpins and the still-slow acceleration zone after them -- and almost
  // nothing through fast kinks the car never slows for. Because accel < decel,
  // corner EXITS stay slow longer than entries brake, so the optimum shifts
  // toward a late apex. 0 disables (pure minimum curvature, previous behavior).
  double raceline_speed_weight_exponent{0.0};
  VelocityProfileConfig raceline_speed_model{};
};

bool buildCenterlineFromSlamMap(
  const hyu_msgs::msg::ConeArrayWithCovariance & cone_map,
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

// Time-density weights for the speed-weighted raceline (exposed for tests).
// Runs the friction-circle + accel/decel speed profile over the given
// curvature/spacing sequence (wrap-around on a closed ring) and returns one
// weight per point, (v_max / v_i)^exponent normalised to mean 1. Uniform
// speed therefore returns all-ones, and exponent <= 0 returns all-ones.
// `segment_length[i]` is the distance from point i to point i+1 (wrapping to
// point 0 when closed), so it has size n when closed and n-1 when open.
std::vector<double> computeRacelineSpeedWeights(
  const std::vector<double> & signed_curvature,
  const std::vector<double> & segment_length,
  bool closed,
  double exponent,
  const VelocityProfileConfig & speed_model);

}  // namespace hyu_global_planner
