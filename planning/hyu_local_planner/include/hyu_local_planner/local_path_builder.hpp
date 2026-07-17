#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace hyu_local_planner
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
  // Unknown-colour cone handling. Perception/SLAM emit colour drop-outs as
  // unknown_color_cones; when enabled each such cone is classified onto the
  // nearest boundary so it still informs the path. Classification is
  // deliberately conservative: a cone is absorbed only when its side is
  // unambiguous, and centred/off-boundary cones are dropped.
  bool use_unknown_cones{true};
  // Max perpendicular distance from the local boundary line (fitted from the
  // two nearest same-colour cones) for an unknown cone to be absorbed onto it.
  double unknown_absorb_lateral_m{0.75};
  // Fallback when no labelled boundary lies within max_traversal_gap_m of an
  // unknown cone: split by ego-frame side. |y| below this dead-band is too
  // central to call, so the cone is dropped.
  double unknown_geom_deadband_m{0.75};
  // Let orange/big-orange cones inform the boundaries outside the
  // straight-corridor mission (which has its own fold-in). They take the same
  // conservative route as unknown cones: absorbed onto a labelled boundary
  // line when unambiguous, else split by ego side outside the dead-band. This
  // is what keeps a path alive at the start/finish straight after a lap, where
  // the ROI can hold nothing but the orange gate. Off by default: enable per
  // mission via config.
  bool use_orange_cones{false};
  // Straight-corridor mission mode (acceleration). Replaces the normal
  // two-sided/one-sided logic with straightCorridorPath: a line fitted through
  // the cones behind AND ahead of the ego, driven straight and bounded a little
  // past the last cone. Keeps a valid forward path when the car outruns the
  // mapped frontier (no mid-run brake pulses) yet still ends -- and stops -- a
  // bounded distance past the corridor. Only enable where the track really is
  // straight; on any curved track this drives the path into a wall. Pair with a
  // wide (negative) roi_min_x so cones already passed stay available to the fit.
  bool extend_straight_to_horizon{false};
  // How far past the furthest cone straightCorridorPath carries the line. Larger
  // = more tolerance to a lagging frontier, but the car brakes later / stops
  // further down the braking zone. Only used when extend_straight_to_horizon.
  double straight_extension_cap_m{5.0};
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
