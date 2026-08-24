#pragma once

#include <cstddef>
#include <optional>
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
  // The ego-side split is only meaningful where "left/right of the car" is
  // "left/right of the track", i.e. near the car. Unknown cones further away
  // than this (range from the ego) that no labelled boundary line explains
  // are dropped instead of split: on a curve, a far uncoloured cone on the
  // outside of the bend sits on the ego's LEFT while the track's right wall
  // owns it, and splitting it fabricated a boundary that bent the path into
  // right-angle hooks and links back along the driven corridor at the map
  // frontier (0801 replay). Orange/big-orange gate cones are trusted markers
  // and keep the unlimited split (lap-close frames see only the gate).
  double unknown_geom_max_range_m{6.0};
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
  // Open-ended track missions (DLC / demo corridors, no closing loop): taper
  // every published path's speed toward zero at the path end,
  // v(d) = sqrt(2 * end_stop_decel * max(0, d - end_stop_margin)) with d the
  // remaining arc to the last waypoint, so the controller rolls the car to a
  // planned stop where the cones run out instead of carrying the cruise speed
  // past the last pair and tripping its fail-safe brake (steering zeroed, hard
  // decel). While cruising the path end is the map/perception frontier well
  // beyond the taper zone, so speeds are untouched; only an end that stops
  // advancing -- the open track end -- walks the speed down. The taper is
  // deliberately allowed below min_speed_mps: that floor keeps the car moving
  // mid-track, but here zero is the goal.
  bool stop_at_path_end{false};
  double end_stop_decel_mps2{2.0};
  double end_stop_margin_m{1.0};
  // Live-cone extension reconciliation (slam_map mode, see
  // planWithLiveExtension). The SLAM map is the trusted source; live
  // perception cones may only APPEND path beyond the map frontier. A path
  // built from map+live cones is accepted only where it agrees with the
  // map-only path, and its tail past the map frontier is cut at the first
  // kink -- a sharp heading change over a short arc -- which live outliers
  // produce (right-angle hooks, links back along the driven corridor) and
  // real track geometry does not.
  // Max distance from any map-only waypoint to the extended path. Beyond it
  // the extension has moved the mapped part of the path: keep map-only.
  double live_extension_max_deviation_m{0.5};
  // Max heading change over live_extension_turn_window_m of arc in the
  // appended tail (and across the junction). Real cone-spaced corners stay
  // under ~1 rad per metre of chain; a spurious hook is a 90-180 deg turn
  // within one waypoint step.
  double live_extension_max_turn_rad{1.4};
  double live_extension_turn_window_m{1.0};
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
  // Set only when the straight-corridor line fit produced this result: the
  // fitted line endpoints (ego frame; start is the ego anchor at x=0, end is
  // the line carried straight_extension_cap_m past the furthest cone) and the
  // x of the furthest blue/yellow corridor cone (braking-zone orange excluded;
  // -inf when the fit ran on orange only). The node latches these in the odom
  // frame so the line survives observation dropouts.
  bool straight_fit{false};
  Point2 straight_line_start{};
  Point2 straight_line_end{};
  double straight_corridor_max_x{0.0};
};

// Odom-frame latch of the straight-corridor line (acceleration mission). All
// scalars are arc lengths along `dir` measured from `origin`. The latch is a
// snapshot: path_end_s marks last-cone + cap at latch time, so a held path
// still ends -- and the car still brakes -- at the same place a live fit
// would have ended.
struct StraightLineLatch
{
  Point2 origin;               // point on the line (odom frame)
  Point2 dir;                  // unit direction, forward along the run
  double path_end_s{0.0};      // line end: furthest cone + extension cap
  double corridor_end_s{0.0};  // furthest blue/yellow cone; past it -> braking speed
};

// Convert a straight-fit BuildResult into an odom-frame latch using the ego
// pose the fit was computed at. Returns nullopt when the result carries no
// usable fit (not a straight fit, or a degenerate line).
std::optional<StraightLineLatch> latchStraightLine(
  const BuildResult & result, double ego_x, double ego_y, double ego_yaw);

// Rebuild the latched line as a local path from the current ego pose, with no
// cone observations at all: the path runs from the ego's projection onto the
// line forward to the latched end. Full speed while the latched corridor is
// still ahead, braking (fallback) speed past it, and invalid -- brake -- once
// the latched end is closer than the minimum path length. The normal
// start-distance and no-backward gates apply, so a pose jump off the line
// invalidates the hold instead of steering the car across the corridor.
BuildResult buildHeldStraightPath(
  const StraightLineLatch & latch, double ego_x, double ego_y, double ego_yaw,
  const PlannerConfig & config);

BuildResult buildLocalPath(const ConeSet & cones, const PlannerConfig & config = PlannerConfig{});
bool pathSelfIntersects(const std::vector<Point2> & points);

// Reconcile a path built from the SLAM map alone with one built from the
// map plus live perception cones. The map-only path is the reference and
// is returned unchanged whenever the extended path is invalid, adds no
// length, or deviates from it by more than live_extension_max_deviation_m
// anywhere along the map-only path. Otherwise the extended path is taken
// and its tail past the map-only length is truncated at the first kink
// (heading change > live_extension_max_turn_rad within
// live_extension_turn_window_m of arc). With no valid map-only path the
// extended path stands on its own, kink-truncated from the start; if fewer
// than five waypoints survive the result is invalid
// ("live_extension_rejected: ..."). `note` (optional) receives a short
// human-readable account of the decision for diagnostics.
BuildResult reconcileLiveExtension(
  const BuildResult & map_only, const BuildResult & extended,
  const PlannerConfig & config, std::string * note = nullptr);

// slam_map-mode planning step shared by the node and offline tools: plan
// from the map-derived cones, and when `extended` (map + live cones, same
// ego frame) is given, plan from it too and reconcile the two. Without
// `extended` this is exactly buildLocalPath(map_only_cones).
BuildResult planWithLiveExtension(
  const ConeSet & map_only_cones, const ConeSet * extended,
  const PlannerConfig & config, std::string * note = nullptr);

}
