#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace local_planner
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct ConeSet
{
  std::vector<Point2> blue;
  std::vector<Point2> yellow;
  std::vector<Point2> orange;
  std::vector<Point2> big_orange;
  std::vector<Point2> unknown;
  bool input_overflow{false};
};

constexpr std::size_t kMaxBoundaryCones = 512U;

struct PlannerConfig
{
  double roi_min_x{-1.0};
  double roi_max_x{20.0};
  double roi_abs_y{8.0};
  double endpoint_match_tolerance_m{0.05};
  double min_track_width_m{2.0};
  double max_track_width_m{6.0};
  double duplicate_tolerance_m{0.05};
  double min_forward_projection_m{0.10};
  double max_traversal_gap_m{6.0};
  // Max spacing when chaining corridor centerline midpoints forward from the
  // ego. Must exceed the cone spacing on wide straights (midpoints sit one per
  // cone pair, ~6 m apart on some tracks) or the centerline stalls after one
  // point. Cross-corridor bridging (a folded/return passage) is a ~90 deg
  // lateral jump blocked by the heading limit + normal-forward priority and by
  // the pairing clearance filter, not by this gap.
  double centerline_max_link_gap_m{6.0};
  double max_heading_change_rad{1.047};
  double max_u_turn_heading_change_rad{2.618};
  double waypoint_spacing_m{0.5};
  double max_start_distance_m{4.0};
  double two_sided_horizon_m{20.0};
  double fallback_horizon_m{8.0};
  double fallback_offset_m{1.5};
  double two_sided_speed_mps{3.0};
  double fallback_speed_mps{1.5};
  // Curvature speed profile: corner speed is capped at
  // sqrt(max_lateral_accel / |kappa|), floored at min_speed_mps. The per-path
  // speed above acts as the straight-line cap.
  double max_lateral_accel_mps2{5.0};
  double min_speed_mps{1.0};
  bool allow_partial_boundary{false};
};

enum class PathKind
{
  kNone,
  kTwoSided,
  kBlueOnly,
  kYellowOnly,
};

struct PathWaypoint
{
  double x{0.0};
  double y{0.0};
  double s{0.0};
  double psi{0.0};
  double kappa{0.0};
  double speed{0.0};
};

struct BuildResult
{
  bool evaluated{false};
  bool valid{false};
  PathKind kind{PathKind::kNone};
  std::vector<PathWaypoint> waypoints;
  std::string reason{"not implemented"};
};

BuildResult buildLocalPath(const ConeSet & cones, const PlannerConfig & config = PlannerConfig{});
bool pathSelfIntersects(const std::vector<Point2> & points);

}
