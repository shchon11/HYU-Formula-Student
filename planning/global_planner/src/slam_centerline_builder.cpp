#include "global_planner/slam_centerline_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "global_planner/slam_boundary_ordering.hpp"

namespace global_planner
{
namespace
{

struct MonotonicProjection
{
  PlannerPoint point;
  double arc_s{0.0};
};

MonotonicProjection closestPointOnPolylineAfter(
  const PlannerPoint & query,
  const std::vector<PlannerPoint> & poly,
  const std::vector<double> & arc_lengths,
  double min_arc_s,
  double duplicate_tolerance,
  double forward_window)
{
  PlannerPoint best = poly.front();
  double best_arc_s = arc_lengths.empty() ? 0.0 : arc_lengths.front();
  double best_d2 = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i + 1U < poly.size(); ++i) {
    const PlannerPoint & a = poly[i];
    const PlannerPoint & b = poly[i + 1U];
    const double segment_start_s = arc_lengths[i];
    const double segment_end_s = arc_lengths[i + 1U];
    if (segment_end_s + duplicate_tolerance < min_arc_s) {
      continue;  // wholly behind the monotonic frontier
    }
    // Bound the search to a forward window. Without this, on a CLOSED loop the
    // seam makes ordered_yellow.back() physically adjacent to ordered_yellow
    // .front() near the ego, so the first blue sample projects onto the far
    // end of the arc and pins every later pair to yellow.back() -> the width
    // blows past max_track_width and a geometrically valid loop is rejected.
    if (segment_start_s > min_arc_s + forward_window) {
      continue;
    }
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len2 = vx * vx + vy * vy;
    double t = 0.0;
    if (len2 > 0.0) {
      t = std::clamp(((query.x - a.x) * vx + (query.y - a.y) * vy) / len2, 0.0, 1.0);
      if (segment_end_s > segment_start_s && min_arc_s > segment_start_s) {
        // min_t may land just past a segment end (within duplicate_tolerance);
        // std::min keeps std::clamp's lo <= hi (lo > hi is undefined behaviour).
        const double min_t = (min_arc_s - segment_start_s) / (segment_end_s - segment_start_s);
        t = std::clamp(t, std::min(min_t, 1.0), 1.0);
      }
    }
    const PlannerPoint proj{a.x + t * vx, a.y + t * vy};
    const double dx = query.x - proj.x;
    const double dy = query.y - proj.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = proj;
      best_arc_s = segment_start_s + t * (segment_end_s - segment_start_s);
    }
  }
  return MonotonicProjection{best, best_arc_s};
}

std::vector<PlannerPoint> finiteConePoints(
  const std::vector<eufs_msgs::msg::ConeWithCovariance> & cones)
{
  std::vector<PlannerPoint> points;
  points.reserve(cones.size());
  for (const auto & cone : cones) {
    const PlannerPoint point{cone.point.x, cone.point.y};
    if (isFinitePoint(point)) {
      points.push_back(point);
    }
  }
  return points;
}

double nearestDistance(const PlannerPoint & point, const std::vector<PlannerPoint> & candidates)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & candidate : candidates) {
    best = std::min(best, distance(point, candidate));
  }
  return best;
}

// FS marks the start/finish line with (big) orange cones, placed exactly where
// the blue and yellow boundaries break for the timing gate. The pairing sweep
// only knows blue<->yellow, so those markers are invisible to it and each
// boundary carries one long step across the gate (5.9 m blue / 5.3 m yellow on
// small_track). The car starts ON that line, so the boundary walk seeds beside
// the gate, runs away around the lap and ends on its far side: the ring seam
// then lands on the gate gap, close_loop_distance_m rejects it, and the global
// path is published with a visible break across the start/finish.
//
// Fold each marker into the boundary it flanks (nearest coloured cone wins) so
// the gate carries real boundary points. Markers within merge_tolerance of one
// another (the paired timing cones) collapse to one, and a marker landing on an
// existing cone is dropped, so the same-colour-duplicate gate cannot trip on a
// point this function introduced.
void foldStartFinishMarkers(
  const std::vector<PlannerPoint> & markers, double merge_tolerance,
  std::vector<PlannerPoint> & blue_points, std::vector<PlannerPoint> & yellow_points)
{
  std::vector<PlannerPoint> collapsed;
  for (const auto & marker : markers) {
    if (nearestDistance(marker, collapsed) > merge_tolerance) {
      collapsed.push_back(marker);
    }
  }
  // Decide every side against the ORIGINAL boundaries. Assigning against the sets
  // as they grow lets a marker that was just folded in act as a "nearest cone"
  // for the next one, and the gate's own two cones are closer to each other than
  // the boundaries are to either: on autocross_kase2026 the gate is 4.2 m across
  // while the real boundaries sit 4.75 m away, so the first marker folded into
  // blue then out-competed the true yellow (4.77 m) for the second, and BOTH gate
  // cones landed on the same side. The ordering walk jumps the track there and
  // the pairing collapses -- over a quarter of the pairs fall outside the width
  // band and the whole map is rejected as invalid_width. small_track's tighter
  // gate (boundaries 2.7 m away) happened to survive it.
  const std::vector<PlannerPoint> original_blue = blue_points;
  const std::vector<PlannerPoint> original_yellow = yellow_points;
  for (const auto & marker : collapsed) {
    const bool to_blue = nearestDistance(marker, original_blue) <=
      nearestDistance(marker, original_yellow);
    const std::vector<PlannerPoint> & original_side = to_blue ? original_blue : original_yellow;
    if (nearestDistance(marker, original_side) > merge_tolerance) {
      (to_blue ? blue_points : yellow_points).push_back(marker);
    }
  }
}

constexpr double kPi = 3.14159265358979323846;
// No drivable track turns 60 deg between successive samples (at the 0.5 m
// default spacing that is a sub-0.5 m turn radius). A step this sharp is a
// folded seam or mispaired midpoints, never real geometry.
constexpr double kMaxHeadingStepRad = 60.0 * kPi / 180.0;

bool validateWaypoints(
  const std::vector<PlannerWaypoint> & waypoints, double duplicate_tolerance, std::string & reason)
{
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const auto & waypoint = waypoints[i];
    if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y) ||
      !std::isfinite(waypoint.s) || !std::isfinite(waypoint.psi) ||
      !std::isfinite(waypoint.kappa))
    {
      reason = "generated waypoint contains non-finite fields";
      return false;
    }
    if (i > 0U) {
      const PlannerPoint previous{waypoints[i - 1].x, waypoints[i - 1].y};
      const PlannerPoint current{waypoint.x, waypoint.y};
      if (!(waypoint.s > waypoints[i - 1].s)) {
        reason = "generated s_m is not strictly increasing";
        return false;
      }
      if (distance(previous, current) <= duplicate_tolerance) {
        reason = "generated adjacent duplicate waypoint";
        return false;
      }
      double heading_step = std::abs(waypoint.psi - waypoints[i - 1].psi);
      if (heading_step > kPi) {
        heading_step = 2.0 * kPi - heading_step;
      }
      if (heading_step > kMaxHeadingStepRad) {
        reason = "generated heading reverses between adjacent waypoints";
        return false;
      }
    }
  }
  return true;
}

bool hasSameColorDuplicate(
  const std::vector<PlannerPoint> & points, const char * side, double threshold, std::string & reason)
{
  (void)side;
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1U; j < points.size(); ++j) {
      const double separation = distance(points[i], points[j]);
      if (separation < threshold) {
        reason = "duplicate_ghost";
        return true;
      }
    }
  }
  return false;
}

bool hasInvalidNearestTrackWidth(
  const std::vector<PlannerPoint> & blue_points,
  const std::vector<PlannerPoint> & yellow_points,
  const SlamCenterlineConfig & config,
  std::string & reason)
{
  // A single off-width cone must not reject the whole map: a missing cone
  // (its neighbour's nearest opposite jumps to the next one) or a lone
  // colour misclassification (a blue and yellow reading 1.1 m apart) are
  // routine local defects the centerline builder already skips by pairing.
  // Only a WIDESPREAD failure (crossed/swapped boundaries) is a genuinely
  // wrong map — count the violators and reject on the same 1/4 fraction the
  // downstream width check uses, so the two gates agree.
  std::size_t total = 0U;
  std::size_t violations = 0U;
  for (const auto * points : {&blue_points, &yellow_points}) {
    const auto & opposite = points == &blue_points ? yellow_points : blue_points;
    for (const auto & point : *points) {
      double best_width = std::numeric_limits<double>::max();
      for (const auto & candidate : opposite) {
        best_width = std::min(best_width, distance(point, candidate));
      }
      ++total;
      if (best_width < config.min_track_width_m || best_width > config.max_track_width_m) {
        ++violations;
      }
    }
  }
  if (total > 0U && violations > total / 4U) {
    reason = "invalid_width";
    return true;
  }
  return false;
}

// Nearest point on a polyline. The raceline needs the point, not just the
// distance: which SIDE of a point's normal a boundary sits on decides whether
// that boundary caps the positive or the negative offset.
PlannerPoint closestPointOnPolyline(
  const PlannerPoint & query, const std::vector<PlannerPoint> & poly)
{
  if (poly.size() == 1U) {
    return poly.front();
  }
  PlannerPoint best = poly.front();
  double best_d2 = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i + 1U < poly.size(); ++i) {
    const PlannerPoint & a = poly[i];
    const PlannerPoint & b = poly[i + 1U];
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

}  // namespace

void applyMinimumCurvatureRaceline(
  std::vector<PlannerPoint> & centerline,
  const std::vector<PlannerPoint> & left_boundary,
  const std::vector<PlannerPoint> & right_boundary,
  const SlamCenterlineConfig & config)
{
  if (config.raceline_smoothing_iterations <= 0 || centerline.size() < 3U) {
    return;
  }

  // A closed ring arrives with the first point duplicated at the tail; optimise
  // the ring without the duplicate so the seam is treated like any other point,
  // and restore the duplicate afterwards.
  const bool closed = centerline.size() >= 4U &&
    distance(centerline.front(), centerline.back()) <= config.duplicate_point_tolerance;
  std::vector<PlannerPoint> ring(
    centerline.begin(), closed ? centerline.end() - 1 : centerline.end());
  const std::size_t n = ring.size();
  if (n < 3U) {
    return;
  }

  // Curvature needs both neighbours. An open path additionally pins its
  // endpoints so the join to upstream geometry is not disturbed.
  const std::size_t first = closed ? 0U : 1U;
  const std::size_t last = closed ? n - 1U : n - 2U;
  const auto back_of = [&](std::size_t i) {return closed ? (i + n - 1U) % n : i - 1U;};
  const auto front_of = [&](std::size_t i) {return closed ? (i + 1U) % n : i + 1U;};

  // p_i = c_i + alpha_i * n_i is only linear while n_i is held fixed, but the
  // optimum moves points by the better part of a metre, by which point those
  // frozen normals no longer describe the line. Minimising the stale model then
  // makes the REAL curvature worse the harder it converges. So re-linearise:
  // solve, move the line, rebuild normals/bounds/model from where it now is, and
  // solve again. Each pass is a fresh convex QP over the true corridor, so the
  // clearance guarantee holds at every step rather than accumulating error.
  constexpr int kRelinearisations = 6;
  for (int pass = 0; pass < kRelinearisations; ++pass) {

  // Unit left-normal from a central-difference tangent. Every offset below is
  // measured along this direction, which is what keeps the model linear.
  std::vector<PlannerPoint> normal(n, PlannerPoint{0.0, 0.0});
  for (std::size_t i = 0; i < n; ++i) {
    const PlannerPoint & behind = ring[closed ? (i + n - 1U) % n : (i == 0U ? i : i - 1U)];
    const PlannerPoint & ahead = ring[closed ? (i + 1U) % n : (i + 1U == n ? i : i + 1U)];
    const double tx = ahead.x - behind.x;
    const double ty = ahead.y - behind.y;
    const double length = std::hypot(tx, ty);
    if (!(length > 0.0)) {
      return;  // degenerate sampling: leave the centerline untouched
    }
    normal[i] = PlannerPoint{-ty / length, tx / length};
  }

  // Corridor box per point. Each boundary caps only the side of the normal it
  // actually lies on, so this holds whichever way round the line was ordered.
  // alpha = 0 stays feasible, so the optimum can never be worse than the
  // centerline it started from -- including where a point is already inside the
  // margin, which simply pins it.
  const double cap = std::max(0.0, config.max_track_width_m);
  std::vector<double> lower(n, -cap);
  std::vector<double> upper(n, cap);
  for (std::size_t i = 0; i < n; ++i) {
    for (const auto * boundary : {&left_boundary, &right_boundary}) {
      if (boundary->empty()) {
        continue;
      }
      const PlannerPoint closest = closestPointOnPolyline(ring[i], *boundary);
      const double room = distance(ring[i], closest) - config.raceline_margin_m;
      const double side =
        (closest.x - ring[i].x) * normal[i].x + (closest.y - ring[i].y) * normal[i].y;
      if (side >= 0.0) {
        upper[i] = std::min(upper[i], room);
      } else {
        lower[i] = std::max(lower[i], -room);
      }
    }
    upper[i] = std::max(upper[i], 0.0);
    lower[i] = std::min(lower[i], 0.0);
  }

  // kappa_i = base_i + ca_i*alpha_{i-1} + cc_i*alpha_i + cb_i*alpha_{i+1}, using
  // the three-point second-derivative weights for UNEVEN spacing: the centerline
  // is sampled at uniform blue arc length, which compresses the midpoints
  // through corners, so assuming a constant ds would mis-weight exactly the
  // places that matter most.
  std::vector<double> base(n, 0.0);
  std::vector<double> ca(n, 0.0);
  std::vector<double> cb(n, 0.0);
  std::vector<double> cc(n, 0.0);
  for (std::size_t i = first; i <= last; ++i) {
    const std::size_t p = back_of(i);
    const std::size_t q = front_of(i);
    const double h1 = distance(ring[p], ring[i]);
    const double h2 = distance(ring[i], ring[q]);
    if (!(h1 > 0.0) || !(h2 > 0.0)) {
      return;
    }
    const double w1 = 2.0 / (h1 * (h1 + h2));
    const double w2 = -2.0 / (h1 * h2);
    const double w3 = 2.0 / (h2 * (h1 + h2));
    base[i] = (w1 * ring[p].x + w2 * ring[i].x + w3 * ring[q].x) * normal[i].x +
      (w1 * ring[p].y + w2 * ring[i].y + w3 * ring[q].y) * normal[i].y;
    ca[i] = w1 * (normal[p].x * normal[i].x + normal[p].y * normal[i].y);
    // The spacing is frozen at the reference line's h1/h2, but the offset curve
    // does not keep it: its arc spacing scales as h*(1 - alpha*kappa). Dropping
    // that factor costs exactly the first-order offset term and INVERTS it --
    // the frozen model reports d(kappa)/d(alpha) = -kappa_0^2 where the truth
    // (differentiating 1/(R - alpha)) is +kappa_0^2. Uncorrected, the model
    // believes that sliding a point toward its centre of curvature FLATTENS the
    // line, so on any constant-radius section the solver drives it to the inside
    // bound: the sharpest line the corridor allows, and worse than the
    // centerline it replaced. Adding 2*kappa_0^2 flips the slope back (-k^2 +
    // 2k^2 = +k^2). base[i] is the signed curvature, so squaring it is
    // orientation-safe, and 2*kappa^2 (~1e-2) is negligible beside w2 (~-8), so
    // the dominant alpha'' term is untouched.
    cc[i] = w2 + 2.0 * base[i] * base[i];
    cb[i] = w3 * (normal[q].x * normal[i].x + normal[q].y * normal[i].y);
  }

  // A: alpha -> curvature deviation, and its transpose. Pinned entries of alpha
  // are always 0, so reading them costs nothing and writes to them are ignored.
  const auto applyA = [&](const std::vector<double> & v, std::vector<double> & out) {
      out.assign(n, 0.0);
      for (std::size_t i = first; i <= last; ++i) {
        out[i] = ca[i] * v[back_of(i)] + cc[i] * v[i] + cb[i] * v[front_of(i)];
      }
    };
  const auto applyAT = [&](const std::vector<double> & r, std::vector<double> & out) {
      out.assign(n, 0.0);
      for (std::size_t i = first; i <= last; ++i) {
        out[back_of(i)] += ca[i] * r[i];
        out[i] += cc[i] * r[i];
        out[front_of(i)] += cb[i] * r[i];
      }
    };

  // Step size needs an upper bound on the Lipschitz constant of grad f, i.e. on
  // 2*lambda_max(A^T A). ||A||_1 * ||A||_inf bounds it, is O(n), and is exact
  // enough here (the top mode of a second-difference operator saturates it) --
  // and unlike a power iteration it is trivially deterministic.
  double norm_inf = 0.0;
  std::vector<double> column_sum(n, 0.0);
  for (std::size_t i = first; i <= last; ++i) {
    norm_inf = std::max(norm_inf, std::abs(ca[i]) + std::abs(cc[i]) + std::abs(cb[i]));
    column_sum[back_of(i)] += std::abs(ca[i]);
    column_sum[i] += std::abs(cc[i]);
    column_sum[front_of(i)] += std::abs(cb[i]);
  }
  const double norm_one = *std::max_element(column_sum.begin(), column_sum.end());
  const double lipschitz = 2.0 * norm_one * norm_inf;
  if (!(lipschitz > 0.0) || !std::isfinite(lipschitz)) {
    return;
  }

  // FISTA (accelerated projected gradient). Minimising sum kappa^2 makes the
  // normal equations biharmonic, whose condition number grows like n^4: a
  // coordinate/Gauss-Seidel sweep needs O(kappa) passes and crawls, while an
  // accelerated method needs O(sqrt(kappa)). The projection is exact, so every
  // returned alpha is inside the corridor no matter where the loop is stopped.
  std::vector<double> alpha(n, 0.0);
  std::vector<double> momentum(n, 0.0);
  std::vector<double> previous(n, 0.0);
  std::vector<double> residual;
  std::vector<double> gradient;
  double theta = 1.0;
  for (int iteration = 0; iteration < config.raceline_smoothing_iterations; ++iteration) {
    applyA(momentum, residual);
    for (std::size_t i = first; i <= last; ++i) {
      residual[i] += base[i];
    }
    applyAT(residual, gradient);
    for (std::size_t i = first; i <= last; ++i) {
      alpha[i] = std::clamp(
        momentum[i] - 2.0 * gradient[i] / lipschitz, lower[i], upper[i]);
    }
    const double theta_next = 0.5 * (1.0 + std::sqrt(1.0 + 4.0 * theta * theta));
    const double beta = (theta - 1.0) / theta_next;
    for (std::size_t i = first; i <= last; ++i) {
      momentum[i] = alpha[i] + beta * (alpha[i] - previous[i]);
      previous[i] = alpha[i];
    }
    theta = theta_next;
  }

  for (std::size_t i = 0; i < n; ++i) {
    ring[i] = PlannerPoint{
      ring[i].x + alpha[i] * normal[i].x, ring[i].y + alpha[i] * normal[i].y};
  }

  }  // re-linearisation pass

  centerline.assign(ring.begin(), ring.end());
  if (closed) {
    centerline.push_back(centerline.front());
  }
}

// Everything downstream of the map-level gates is seed-sensitive: the
// boundary ordering walks from the cone nearest `seed`, and the pairing sweep
// inherits that phase. From an awkward seed the same good map can fold at the
// seam ("heading reverses"), phase-lock the monotonic pairing (invalid_width),
// or strand cones (branch_jump). buildCenterlineFromSlamMap therefore treats
// this whole pipeline as one attempt and retries it from alternate seeds.
static bool buildCenterlineFromSeed(
  const std::vector<PlannerPoint> & blue_points,
  const std::vector<PlannerPoint> & yellow_points,
  const PlannerPoint & seed,
  const SlamCenterlineConfig & config,
  std::vector<PlannerWaypoint> & waypoints,
  std::string & reason)
{
  waypoints.clear();

  std::vector<PlannerPoint> ordered_blue;
  std::vector<PlannerPoint> ordered_yellow;
  if (!orderSlamBoundaries(
      blue_points, yellow_points, seed, config.max_boundary_gap_m,
      ordered_blue, ordered_yellow, reason))
  {
    return false;
  }

  const auto blue_arc = cumulativeArcLengths(ordered_blue);
  const auto yellow_arc = cumulativeArcLengths(ordered_yellow);
  if (blue_arc.back() <= config.duplicate_point_tolerance ||
    yellow_arc.back() <= config.duplicate_point_tolerance)
  {
    reason = "boundary arc length too short";
    return false;
  }

  // Pairing is locked to the opposite boundary's forward arc coordinate. This
  // prevents close branches from snapping backward across the track while still
  // allowing unequal boundary lengths.
  const std::size_t pair_count = std::max<std::size_t>(
    2U, static_cast<std::size_t>(std::ceil(blue_arc.back() / config.waypoint_spacing_m)) + 1U);
  // Forward search window for the monotonic pairing: large enough to absorb
  // the per-sample yellow advance and any blue/yellow phase offset, small
  // enough that a blue sample can never reach across the closed-loop seam.
  const double forward_window = std::max(5.0, 2.0 * config.max_track_width_m);

  // The monotonic projection cannot advance past ordered_yellow.back(). Each
  // ring is seeded at its own nearest-to-ego cone, so on a closed loop the
  // yellow arc can end before the blue sweep does; the final pairs then pin
  // to yellow.back(), dragging the centerline tail sideways past the start
  // (the fold-back "Z" at the seam). Extend a closed yellow ring virtually
  // through its seam so the projection can wrap into the next lap's start.
  std::vector<PlannerPoint> projection_yellow = ordered_yellow;
  if (distance(ordered_yellow.front(), ordered_yellow.back()) <= config.close_loop_distance_m) {
    double extended = 0.0;
    PlannerPoint previous_point = ordered_yellow.back();
    for (std::size_t i = 0; i < ordered_yellow.size() && extended < forward_window; ++i) {
      const double step = distance(previous_point, ordered_yellow[i]);
      if (step > config.duplicate_point_tolerance) {
        projection_yellow.push_back(ordered_yellow[i]);
        extended += step;
        previous_point = ordered_yellow[i];
      }
    }
  }
  const auto projection_yellow_arc = cumulativeArcLengths(projection_yellow);

  double previous_yellow_s = 0.0;
  std::vector<PlannerPoint> centerline;
  centerline.reserve(pair_count + 1U);
  std::size_t out_of_range_pairs = 0U;
  for (std::size_t i = 0; i < pair_count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(pair_count - 1U);
    const auto blue = interpolateAtArcLength(ordered_blue, blue_arc, ratio * blue_arc.back());
    const auto yellow_projection = closestPointOnPolylineAfter(
      blue, projection_yellow, projection_yellow_arc, previous_yellow_s,
      config.duplicate_point_tolerance, forward_window);
    previous_yellow_s = yellow_projection.arc_s;
    const auto yellow = yellow_projection.point;
    const double width = distance(blue, yellow);
    if (width < config.min_track_width_m || width > config.max_track_width_m) {
      // A single mispaired/noisy sample (SLAM cone jitter, a local pairing skew)
      // must not discard the entire raceline. Skip this midpoint and let the
      // neighbours bridge it; only a WIDESPREAD width failure (a genuinely
      // wrong map, e.g. crossed boundaries) rejects the path below.
      ++out_of_range_pairs;
      continue;
    }
    const PlannerPoint midpoint = scale(add(blue, yellow), 0.5);
    if (centerline.empty() ||
      distance(centerline.back(), midpoint) > config.duplicate_point_tolerance)
    {
      centerline.push_back(midpoint);
    }
  }

  // Tolerate isolated bad pairs, but a map where a large fraction of samples
  // fail the width gate is genuinely wrong (crossed/duplicated boundaries) and
  // must still fail closed.
  if (out_of_range_pairs > pair_count / 4U) {
    reason = "invalid_width";
    return false;
  }
  if (centerline.size() < 2U) {
    reason = "centerline has fewer than two unique points";
    return false;
  }
  // Close the loop without folding back: the sweep can still carry the tail
  // slightly past the start, and a blind chord to the front would double back
  // against travel. Drop tail points whose closing chord reverses direction,
  // then append the front point.
  if (distance(centerline.front(), centerline.back()) <= config.close_loop_distance_m) {
    while (centerline.size() > 2U &&
      distance(centerline.front(), centerline.back()) > config.duplicate_point_tolerance)
    {
      const PlannerPoint & tail = centerline.back();
      const PlannerPoint & before_tail = centerline[centerline.size() - 2U];
      const double travel_x = tail.x - before_tail.x;
      const double travel_y = tail.y - before_tail.y;
      const double chord_x = centerline.front().x - tail.x;
      const double chord_y = centerline.front().y - tail.y;
      if (travel_x * chord_x + travel_y * chord_y >= 0.0) {
        break;
      }
      centerline.pop_back();
    }
    if (distance(centerline.front(), centerline.back()) <= config.close_loop_distance_m &&
      distance(centerline.front(), centerline.back()) > config.duplicate_point_tolerance)
    {
      centerline.push_back(centerline.front());
    }
  }

  // A ring that did not close is a bad SEED, not a bad map. orderSlamBoundaries
  // already refuses a boundary whose own ends exceed max_boundary_gap_m, so by
  // the time we are here the cone rings ARE closed and the centerline is a loop;
  // the seam is just wherever this particular walk happened to start and end,
  // and the sweep can leave its two ends metres apart there. Accepting that
  // silently is what published a global path broken across the start/finish --
  // intermittently, because the seam moves with the ego seed, so the SAME map
  // closed or broke depending only on where the car was when it was rebuilt
  // (measured: ~4% of ego positions on small_track, ~20% on the seam_fold map,
  // seams up to 19 m). Reject the seed instead and let the caller's retry find
  // one whose walk closes; a genuinely open map fails from every seed, as before.
  if (distance(centerline.front(), centerline.back()) > config.duplicate_point_tolerance) {
    reason = "loop_not_closed";
    return false;
  }

  // Resample BEFORE optimising. The pairing sweep samples at uniform BLUE arc
  // length, so the midpoints it emits are unevenly spaced -- occasionally almost
  // coincident. The curvature weights go like 1/(h1*h2), so those samples
  // dominate the objective and wreck its conditioning; on the shipped track that
  // alone made the "minimum curvature" line twice as sharp as the centerline.
  // A uniformly spaced line is what the finite-difference model assumes.
  auto resampled = resampleBySpacing(
    centerline, config.waypoint_spacing_m, config.duplicate_point_tolerance);
  if (resampled.size() < 3U) {
    reason = "resampled centerline has fewer than three points";
    return false;
  }

  applyMinimumCurvatureRaceline(resampled, ordered_blue, ordered_yellow, config);

  // The lateral offsets stretch and compress the spacing a little; resample once
  // more so the published waypoints keep their uniform-spacing contract.
  resampled = resampleBySpacing(
    resampled, config.waypoint_spacing_m, config.duplicate_point_tolerance);
  if (resampled.size() < 3U) {
    reason = "resampled centerline has fewer than three points";
    return false;
  }

  waypoints.clear();
  waypoints.reserve(resampled.size());
  for (const auto & point : resampled) {
    waypoints.push_back(PlannerWaypoint{point.x, point.y, 0.0, 0.0, 0.0});
  }
  computeWaypointGeometry(waypoints);
  return validateWaypoints(waypoints, config.duplicate_point_tolerance, reason);
}

bool buildCenterlineFromSlamMap(
  const eufs_msgs::msg::ConeArrayWithCovariance & cone_map,
  const nav_msgs::msg::Odometry & ego_odom,
  const SlamCenterlineConfig & config,
  std::vector<PlannerWaypoint> & waypoints,
  std::string & reason)
{
  waypoints.clear();

  if (!std::isfinite(config.waypoint_spacing_m) ||
    config.waypoint_spacing_m < kMinimumWaypointSpacingM)
  {
    reason = "waypoint spacing is below the supported minimum";
    return false;
  }
  if (!std::isfinite(config.duplicate_point_tolerance) ||
    config.duplicate_point_tolerance < 0.0)
  {
    reason = "duplicate point tolerance is invalid";
    return false;
  }

  auto blue_points = finiteConePoints(cone_map.blue_cones);
  auto yellow_points = finiteConePoints(cone_map.yellow_cones);

  // The start/finish markers join the boundaries before every gate below, so the
  // duplicate and width checks validate exactly the points the sweep will pair.
  std::vector<PlannerPoint> markers = finiteConePoints(cone_map.big_orange_cones);
  const auto small_orange = finiteConePoints(cone_map.orange_cones);
  markers.insert(markers.end(), small_orange.begin(), small_orange.end());
  if (!markers.empty()) {
    foldStartFinishMarkers(
      markers, std::max(config.duplicate_point_tolerance, config.min_track_width_m * 0.25),
      blue_points, yellow_points);
  }

  if (blue_points.size() < static_cast<std::size_t>(config.min_cones_per_side) ||
    yellow_points.size() < static_cast<std::size_t>(config.min_cones_per_side))
  {
    reason = "insufficient blue/yellow cones";
    return false;
  }

  const PlannerPoint ego{ego_odom.pose.pose.position.x, ego_odom.pose.pose.position.y};
  if (!isFinitePoint(ego)) {
    reason = "ego odom position is non-finite";
    return false;
  }

  // Map-level gates are seed-independent: a genuinely bad map (ghost
  // duplicates, impossible widths) fails closed once, before any retry.
  const double duplicate_ghost_threshold = std::max(
    config.duplicate_point_tolerance, config.min_track_width_m * 0.25);
  if (hasSameColorDuplicate(blue_points, "blue", duplicate_ghost_threshold, reason) ||
    hasSameColorDuplicate(yellow_points, "yellow", duplicate_ghost_threshold, reason))
  {
    return false;
  }
  if (hasInvalidNearestTrackWidth(blue_points, yellow_points, config, reason)) {
    return false;
  }

  // Seed from a FIXED reference, never the ego. The seed picks which cone the
  // boundary walk starts at, and that sets the ordering and pairing phase for the
  // whole lap -- so seeding at the car made the published path a function of
  // WHERE THE CAR WAS, not of the map. Measured on small_track: the same frozen
  // map rebuilt from two ego positions moved the line by up to 0.20 m, while 2 cm
  // of cone jitter moves it only 0.03-0.05 m. Since every SLAM cone refinement
  // bumps the map signature (it is hashed to the millimetre) and triggers a
  // rebuild, and the car has moved by then, the path stepped ~0.2 m sideways
  // under the controller on an essentially unchanged map -- which the car
  // answered with a visible shake.
  //
  // The map origin is where SLAM started, i.e. the start/finish line: fixed for
  // the whole run, on the track, and a sensible phase to begin a lap on. Its
  // exact value does not matter to consumers anyway -- the path is a closed loop
  // in the map frame and the wpnt_publisher wraps around it -- only that it is
  // the SAME every rebuild. With this the path is finally what this file already
  // claimed it was: a pure function of the cone map.
  constexpr std::size_t kMaxCenterlineSeedAttempts = 12U;
  std::string first_reason;
  if (buildCenterlineFromSeed(
      blue_points, yellow_points, PlannerPoint{0.0, 0.0}, config, waypoints, first_reason))
  {
    return true;
  }
  const std::size_t attempts = std::min(kMaxCenterlineSeedAttempts, blue_points.size());
  for (std::size_t i = 0; i < attempts; ++i) {
    const PlannerPoint & seed = blue_points[(i * blue_points.size()) / attempts];
    std::string retry_reason;
    if (buildCenterlineFromSeed(
        blue_points, yellow_points, seed, config, waypoints, retry_reason))
    {
      return true;
    }
  }

  waypoints.clear();
  reason = first_reason;
  return false;
}

}  // namespace global_planner
