#include "hyu_global_planner/slam_boundary_ordering.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>

namespace hyu_global_planner
{
namespace
{

constexpr double kGeometryEpsilon = 1.0e-9;

bool segmentsCross(
  const PlannerPoint & a, const PlannerPoint & b,
  const PlannerPoint & c, const PlannerPoint & d)
{
  const double ab_c = cross2d(subtract(b, a), subtract(c, a));
  const double ab_d = cross2d(subtract(b, a), subtract(d, a));
  const double cd_a = cross2d(subtract(d, c), subtract(a, c));
  const double cd_b = cross2d(subtract(d, c), subtract(b, c));
  return ab_c * ab_d < -kGeometryEpsilon && cd_a * cd_b < -kGeometryEpsilon;
}

std::size_t selfIntersectionCount(const std::vector<PlannerPoint> & points)
{
  std::size_t count = 0U;
  if (points.size() < 4U) {
    return count;
  }
  for (std::size_t i = 0; i + 1U < points.size(); ++i) {
    for (std::size_t j = i + 2U; j + 1U < points.size(); ++j) {
      if (segmentsCross(points[i], points[i + 1U], points[j], points[j + 1U])) {
        ++count;
      }
    }
  }
  return count;
}

double maxSegmentGap(const std::vector<PlannerPoint> & points)
{
  double max_gap = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    max_gap = std::max(max_gap, distance(points[i - 1U], points[i]));
  }
  return max_gap;
}

struct BoundaryOrderScore
{
  std::size_t self_intersections{0U};
  double max_gap{0.0};
  double loop_gap{0.0};
};

BoundaryOrderScore scoreBoundaryOrder(const std::vector<PlannerPoint> & points)
{
  BoundaryOrderScore score;
  score.self_intersections = selfIntersectionCount(points);
  score.max_gap = maxSegmentGap(points);
  if (points.size() >= 2U) {
    score.loop_gap = distance(points.front(), points.back());
  }
  return score;
}

bool betterBoundaryOrder(
  const BoundaryOrderScore & candidate,
  const BoundaryOrderScore & current)
{
  if (candidate.self_intersections != current.self_intersections) {
    return candidate.self_intersections < current.self_intersections;
  }
  if (std::abs(candidate.max_gap - current.max_gap) > kGeometryEpsilon) {
    return candidate.max_gap < current.max_gap;
  }
  return candidate.loop_gap < current.loop_gap;
}

double turnAngle(
  const PlannerPoint & previous,
  const PlannerPoint & current,
  const PlannerPoint & candidate)
{
  const auto incoming = subtract(current, previous);
  const auto outgoing = subtract(candidate, current);
  const double incoming_length = std::hypot(incoming.x, incoming.y);
  const double outgoing_length = std::hypot(outgoing.x, outgoing.y);
  if (incoming_length <= kGeometryEpsilon || outgoing_length <= kGeometryEpsilon) {
    return 0.0;
  }
  const double cosine = std::clamp(
    dot(incoming, outgoing) / (incoming_length * outgoing_length), -1.0, 1.0);
  return std::acos(cosine);
}

bool headingAwareGraphOrder(
  const std::vector<PlannerPoint> & input, std::size_t seed_index,
  double max_gap, std::size_t max_stranded_drops,
  std::vector<PlannerPoint> & ordered, std::string & reason)
{
  ordered.clear();
  if (input.empty()) {
    reason = "no boundary points";
    return false;
  }

  std::vector<bool> used(input.size(), false);
  std::size_t current = std::min(seed_index, input.size() - 1U);

  ordered.reserve(input.size());
  used[current] = true;
  ordered.push_back(input[current]);
  for (std::size_t count = 1; count < input.size(); ++count) {
    std::size_t next = input.size();
    double best_score = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < input.size(); ++i) {
      const double candidate_distance = distance(input[current], input[i]);
      if (used[i] || candidate_distance > max_gap) {
        continue;
      }
      double score = candidate_distance;
      if (ordered.size() >= 2U) {
        score += max_gap * turnAngle(ordered[ordered.size() - 2U], ordered.back(), input[i]);
      }
      if (score < best_score) {
        best_score = score;
        next = i;
      }
    }
    if (next == input.size()) {
      // Stranded: every unvisited cone is beyond max_gap from the chain head.
      // Observed live (map_20260720_233754): the heading penalty bypasses a
      // ghost landmark sitting between boundary lines, and once the walk has
      // consumed the rest of the ring the bypassed stray is unreachable — so a
      // whole valid lap was rejected for a couple of cones. A small unreachable
      // remainder is exactly that (ghosts or bypassed strays whose own gap the
      // ring already bridges); drop it and let the score gates judge the ring
      // that remains. A large remainder is a genuinely fragmented boundary and
      // still fails closed.
      if (input.size() - ordered.size() <= max_stranded_drops) {
        return true;
      }
      reason = "branch_jump";
      return false;
    }
    used[next] = true;
    ordered.push_back(input[next]);
    current = next;
  }
  return true;
}

double medianSegmentLength(const std::vector<PlannerPoint> & points)
{
  std::vector<double> segments;
  segments.reserve(points.size());
  for (std::size_t i = 1; i < points.size(); ++i) {
    segments.push_back(distance(points[i - 1U], points[i]));
  }
  if (segments.empty()) {
    return 0.0;
  }
  const std::size_t mid = segments.size() / 2U;
  std::nth_element(segments.begin(), segments.begin() + mid, segments.end());
  return segments[mid];
}

// The greedy walk must consume every cone, so a cone it bypasses (the heading
// penalty can prefer a straighter continuation past a tight kink) is appended
// at the end behind a jump far above the cone spacing. Those stragglers are
// real boundary cones: splice each one back at its cheapest-detour position
// instead of leaving a spur that folds the centerline at the seam. Stragglers
// that genuinely belong at the end re-insert there at zero cost, so a real
// sparse tail is left untouched.
void reinsertTailStragglers(std::vector<PlannerPoint> & ordered, double max_gap)
{
  if (ordered.size() < 4U) {
    return;
  }
  const double jump_limit =
    std::min(max_gap, std::max(2.5 * medianSegmentLength(ordered), 2.0));
  std::size_t jump_index = 0U;
  for (std::size_t i = 1; i < ordered.size(); ++i) {
    if (distance(ordered[i - 1U], ordered[i]) > jump_limit) {
      jump_index = i;
    }
  }
  if (jump_index == 0U) {
    return;
  }
  // A long tail behind a jump is a genuinely broken map, not a few bypassed
  // cones; leave it to the score gates below.
  const std::size_t tail_count = ordered.size() - jump_index;
  if (tail_count > ordered.size() / 4U) {
    return;
  }
  const std::vector<PlannerPoint> stragglers(ordered.begin() + jump_index, ordered.end());
  ordered.erase(ordered.begin() + jump_index, ordered.end());
  for (const auto & straggler : stragglers) {
    std::size_t best_position = ordered.size();
    double best_cost = distance(ordered.back(), straggler) +
      distance(straggler, ordered.front()) - distance(ordered.back(), ordered.front());
    for (std::size_t i = 1; i < ordered.size(); ++i) {
      const double cost = distance(ordered[i - 1U], straggler) +
        distance(straggler, ordered[i]) - distance(ordered[i - 1U], ordered[i]);
      if (cost < best_cost) {
        best_cost = cost;
        best_position = i;
      }
    }
    ordered.insert(ordered.begin() + best_position, straggler);
  }
}

// A ghost landmark consumed at the walk's very first or last step hangs off
// the ring as a spur: the loop gap it leaves is far above max_gap even though
// every interior step is fine, so the whole ring is rejected for one stray
// point. Pop whichever end actually shrinks the closing gap, only while the
// ring is open, so a genuinely open boundary (both ends real cones, trimming
// does not help) is left for the loop-gap gate to reject as before.
void trimOpenSeamSpurs(std::vector<PlannerPoint> & ordered, double max_gap, std::size_t budget)
{
  while (budget > 0U && ordered.size() > 4U &&
    distance(ordered.front(), ordered.back()) > max_gap)
  {
    const double open_gap = distance(ordered.front(), ordered.back());
    const double without_tail = distance(ordered.front(), ordered[ordered.size() - 2U]);
    const double without_head = distance(ordered[1], ordered.back());
    if (without_tail < open_gap && without_tail <= without_head) {
      ordered.pop_back();
    } else if (without_head < open_gap) {
      ordered.erase(ordered.begin());
    } else {
      return;
    }
    --budget;
  }
}

// Length-minimising 2-opt over the CLOSED ring (the loop edge back->front
// counts as an edge). Where a boundary runs beside itself closer than its own
// cone spacing — its_a_mess's dumbbell waist is two parallel blue rows 0.75 m
// apart with 1.0-1.5 m along-row spacing — the greedy walk's nearest-cone
// choice zigzags between the rows instead of running one row out and the
// other back, and every seed dies on the self-intersection or stranding
// gates. Even the ground-truth track only squeaked through the rescue tier;
// ordinary 10-17 cm SLAM noise killed every seed (map_20260722_043237).
// Flipping the span between two edges whenever the swap shortens the ring is
// the classical TSP repair. Length, not crossings, is the acceptance test:
// a crossing always shortens when flipped (triangle inequality), but the walk
// also leaves non-crossing switchback folds (observed beside a 0.86 m
// displaced cone on the same map) that a pure uncrossing pass cannot see,
// while both fold shapes waste length. Each accepted flip strictly shortens
// the ring, so the pass terminates; the budget is a backstop, not the
// expected exit. The repair only REORDERS points — it never invents or drops
// geometry — and the score and downstream width/heading/loop gates still
// judge the result, so a genuinely broken map is rejected exactly as before.
void twoOptRepairRing(std::vector<PlannerPoint> & ordered)
{
  const std::size_t n = ordered.size();
  if (n < 4U) {
    return;
  }
  constexpr double kMinImprovement = 1.0e-6;
  std::size_t budget = 16U * n;
  bool changed = true;
  while (changed && budget > 0U) {
    changed = false;
    for (std::size_t i = 0; i + 1U < n && !changed; ++i) {
      for (std::size_t j = i + 2U; j < n; ++j) {
        const std::size_t j_next = (j + 1U) % n;
        if (i == 0U && j_next == 0U) {
          continue;  // adjacent through the loop edge's shared vertex
        }
        const double current_length =
          distance(ordered[i], ordered[i + 1U]) + distance(ordered[j], ordered[j_next]);
        const double flipped_length =
          distance(ordered[i], ordered[j]) + distance(ordered[i + 1U], ordered[j_next]);
        if (flipped_length < current_length - kMinImprovement) {
          std::reverse(
            ordered.begin() + static_cast<std::ptrdiff_t>(i + 1U),
            ordered.begin() + static_cast<std::ptrdiff_t>(j + 1U));
          changed = true;
          --budget;
          break;
        }
      }
    }
  }
}

bool orderAttempt(
  const std::vector<PlannerPoint> & input, std::size_t seed_index, double max_gap,
  bool consider_input_order, std::size_t rescue_drops, bool uncross, bool allow_open_seam,
  std::vector<PlannerPoint> & ordered, std::string & reason)
{
  if (!headingAwareGraphOrder(input, seed_index, max_gap, rescue_drops, ordered, reason)) {
    return false;
  }
  const auto greedy_score = scoreBoundaryOrder(ordered);
  if (consider_input_order) {
    const auto input_score = scoreBoundaryOrder(input);
    if (input_score.max_gap <= max_gap && betterBoundaryOrder(input_score, greedy_score)) {
      ordered = input;
    }
  }
  reinsertTailStragglers(ordered, max_gap);
  // Spur trimming exists to shrink a closing gap that a stray end cone inflated.
  // Under allow_open_seam the closing gap is the very thing being measured, so
  // trimming would eat real end cones and mis-place the seam it reports.
  if (rescue_drops > 0U && !allow_open_seam) {
    trimOpenSeamSpurs(ordered, max_gap, rescue_drops);
  }
  if (uncross) {
    twoOptRepairRing(ordered);
  }
  const auto final_score = scoreBoundaryOrder(ordered);

  if (final_score.self_intersections > 0U) {
    reason = "self_intersection";
    return false;
  }
  if (final_score.max_gap > max_gap || (!allow_open_seam && final_score.loop_gap > max_gap)) {
    reason = "branch_jump";
    return false;
  }
  return true;
}

// The greedy walk is seed-sensitive: seeded at an awkward track phase it can
// strand cones behind a gap (branch_jump) or fold the seam even though the
// map itself is fine. Seeding only at the nearest-to-ego cone therefore made
// the global path's very existence depend on where the car happened to be
// when the map froze (observed live: 5 Hz rebuilds flapping between valid and
// branch_jump on an unchanged map). Bounded retries from deterministic seeds
// spread around the boundary; a genuinely broken map still fails from every
// seed, so the fail-closed contract and reported reason are unchanged.
constexpr std::size_t kMaxOrderingSeedAttempts = 24U;

bool orderBoundary(
  const std::vector<PlannerPoint> & input, const PlannerPoint & ego,
  double max_gap, bool allow_two_opt_repair, bool allow_open_seam,
  std::vector<PlannerPoint> & ordered, std::string & reason)
{
  if (input.empty()) {
    reason = "no boundary points";
    return false;
  }

  std::size_t ego_seed = 0U;
  double best_start_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < input.size(); ++i) {
    const double candidate_distance = distance(input[i], ego);
    if (candidate_distance < best_start_distance) {
      best_start_distance = candidate_distance;
      ego_seed = i;
    }
  }

  std::string first_reason;
  if (orderAttempt(input, ego_seed, max_gap, true, 0U, false, allow_open_seam, ordered,
      first_reason))
  {
    return true;
  }

  const std::size_t attempts = std::min(kMaxOrderingSeedAttempts, input.size());
  for (std::size_t i = 0; i < attempts; ++i) {
    const std::size_t seed = (i * input.size()) / attempts;
    if (seed == ego_seed) {
      continue;
    }
    std::string retry_reason;
    if (orderAttempt(input, seed, max_gap, false, 0U, false, allow_open_seam, ordered,
        retry_reason))
    {
      return true;
    }
  }

  // Every strict seed failed. Retry allowing a small unreachable remainder to
  // be dropped (and end spurs trimmed): a handful of ghost/bypassed cones must
  // not cost the mission its global path. 1/8 of a side keeps this a stray-
  // minority repair — well under the 1/4 fractions the map gates treat as
  // widespread — and leaves tiny fixture-sized boundaries (size < 8) fully
  // strict, so a genuinely broken map still fails from every seed with the
  // ego-seeded diagnosis below.
  const std::size_t rescue_drops = input.size() / 8U;
  if (rescue_drops > 0U) {
    for (std::size_t i = 0; i < attempts; ++i) {
      const std::size_t seed = (i * input.size()) / attempts;
      std::string retry_reason;
      if (orderAttempt(input, seed, max_gap, false, rescue_drops, false, allow_open_seam, ordered,
          retry_reason))
      {
        return true;
      }
    }
    // Final tier: rescue plus 2-opt ring repair, and only when the caller
    // opted in. The centerline builder enables it strictly AFTER every plain
    // attempt across all of ITS seeds failed, so any map the plain pipeline
    // already handled takes exactly the code path (and publishes exactly the
    // path) it did before this repair existed; only a map that would
    // otherwise publish nothing ever reaches it.
    if (allow_two_opt_repair) {
      for (std::size_t i = 0; i < attempts; ++i) {
        const std::size_t seed = (i * input.size()) / attempts;
        std::string retry_reason;
        if (orderAttempt(input, seed, max_gap, false, rescue_drops, true, allow_open_seam, ordered,
            retry_reason))
        {
          return true;
        }
      }
    }
  }

  ordered.clear();
  reason = first_reason;  // diagnose the ego-seeded attempt, as before
  return false;
}

bool alignBoundaryDirections(std::vector<PlannerPoint> & blue, std::vector<PlannerPoint> & yellow)
{
  if (blue.size() < 2U || yellow.size() < 2U) {
    return false;
  }
  // Vote with geometric correspondence: each yellow segment against the
  // direction of the blue segment nearest to it. Index-aligned sampling is
  // not sound here -- each ring is seeded at its own nearest-to-ego cone, so
  // equal indices sit at different track phases, and on a closed loop the
  // index-wise dot oscillates in sign, letting opposite traversal directions
  // survive the median vote.
  std::vector<double> direction_dots;
  direction_dots.reserve(yellow.size() - 1U);
  for (std::size_t i = 1; i < yellow.size(); ++i) {
    const auto yellow_segment = subtract(yellow[i], yellow[i - 1]);
    const double yellow_length = std::hypot(yellow_segment.x, yellow_segment.y);
    if (yellow_length <= kGeometryEpsilon) {
      continue;
    }
    const PlannerPoint yellow_mid{
      0.5 * (yellow[i].x + yellow[i - 1].x), 0.5 * (yellow[i].y + yellow[i - 1].y)};
    double best_d2 = std::numeric_limits<double>::max();
    PlannerPoint best_segment{0.0, 0.0};
    for (std::size_t j = 1; j < blue.size(); ++j) {
      const auto blue_segment = subtract(blue[j], blue[j - 1]);
      const double len2 = blue_segment.x * blue_segment.x + blue_segment.y * blue_segment.y;
      double t = 0.0;
      if (len2 > kGeometryEpsilon) {
        t = std::clamp(
          dot(subtract(yellow_mid, blue[j - 1]), blue_segment) / len2, 0.0, 1.0);
      }
      const PlannerPoint projection{
        blue[j - 1].x + t * blue_segment.x, blue[j - 1].y + t * blue_segment.y};
      const double dx = yellow_mid.x - projection.x;
      const double dy = yellow_mid.y - projection.y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_segment = blue_segment;
      }
    }
    const double blue_length = std::hypot(best_segment.x, best_segment.y);
    if (blue_length > kGeometryEpsilon) {
      direction_dots.push_back(
        dot(best_segment, yellow_segment) / (blue_length * yellow_length));
    }
  }
  const auto direction_dot_median = median(direction_dots);
  if (!direction_dot_median.has_value()) {
    return false;
  }
  if (direction_dot_median.value() < 0.0) {
    std::reverse(yellow.begin(), yellow.end());
  }
  return true;
}

std::optional<double> medianTrackSideSign(
  const std::vector<PlannerPoint> & blue, const std::vector<PlannerPoint> & yellow)
{
  const std::size_t sample_count = std::min(blue.size(), yellow.size());
  if (sample_count < 2U) {
    return std::nullopt;
  }
  const auto blue_samples = sampleNormalized(blue, sample_count);
  const auto yellow_samples = sampleNormalized(yellow, sample_count);
  std::vector<double> signs;
  signs.reserve(sample_count - 1U);
  for (std::size_t i = 1; i < sample_count; ++i) {
    const PlannerPoint previous_mid = scale(add(blue_samples[i - 1], yellow_samples[i - 1]), 0.5);
    const PlannerPoint current_mid = scale(add(blue_samples[i], yellow_samples[i]), 0.5);
    const PlannerPoint tangent = subtract(current_mid, previous_mid);
    const PlannerPoint side = subtract(yellow_samples[i], blue_samples[i]);
    const double tangent_length = std::hypot(tangent.x, tangent.y);
    const double side_length = std::hypot(side.x, side.y);
    if (tangent_length > kGeometryEpsilon && side_length > kGeometryEpsilon) {
      signs.push_back(cross2d(tangent, side) / (tangent_length * side_length));
    }
  }
  return median(signs);
}

// The pairing sweep downstream projects each blue sample onto the yellow ring
// through a short forward arc window, so it assumes both ring seams sit at the
// SAME track phase. The walk's seed retries break that: each colour keeps the
// first seed whose walk closes, and those seeds are independent — observed on
// map_20260720_233754, blue closed from the start line while yellow closed at
// a far hairpin, so every projection searched the wrong end of the arc and 40%
// of the pairs blew the width band. A rotation only relabels a closed ring's
// seam (the polygon and every interior step are unchanged, and the old seam
// gap becomes an interior step that already passed the same max_gap bound), so
// re-seam both rings at the cone nearest the caller's reference point.
void rotateRingSeamToReference(
  std::vector<PlannerPoint> & ring, const PlannerPoint & reference, double max_gap)
{
  if (ring.size() < 3U || distance(ring.front(), ring.back()) > max_gap) {
    return;  // an open chain's seam is real geometry, not a walk artefact
  }
  std::size_t nearest = 0U;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < ring.size(); ++i) {
    const double candidate = distance(ring[i], reference);
    if (candidate < best_distance) {
      best_distance = candidate;
      nearest = i;
    }
  }
  std::rotate(ring.begin(), ring.begin() + static_cast<std::ptrdiff_t>(nearest), ring.end());
}

bool orientTravelDirection(std::vector<PlannerPoint> & blue, std::vector<PlannerPoint> & yellow)
{
  auto sign = medianTrackSideSign(blue, yellow);
  if (!sign.has_value() || std::abs(sign.value()) <= kGeometryEpsilon) {
    return false;
  }
  if (sign.value() > 0.0) {
    std::reverse(blue.begin(), blue.end());
    std::reverse(yellow.begin(), yellow.end());
    sign = medianTrackSideSign(blue, yellow);
  }
  return sign.has_value() && sign.value() < -kGeometryEpsilon;
}

}

bool orderSlamBoundaries(
  const std::vector<PlannerPoint> & blue_points,
  const std::vector<PlannerPoint> & yellow_points,
  const PlannerPoint & ego,
  double max_gap,
  std::vector<PlannerPoint> & ordered_blue,
  std::vector<PlannerPoint> & ordered_yellow,
  std::string & reason,
  bool allow_two_opt_repair)
{
  if (!orderBoundary(blue_points, ego, max_gap, allow_two_opt_repair, false, ordered_blue,
      reason) ||
    !orderBoundary(yellow_points, ego, max_gap, allow_two_opt_repair, false, ordered_yellow,
      reason))
  {
    return false;
  }
  if (!alignBoundaryDirections(ordered_blue, ordered_yellow)) {
    reason = "could not align boundary directions";
    return false;
  }
  if (!orientTravelDirection(ordered_blue, ordered_yellow)) {
    reason = "inconsistent blue/yellow travel direction";
    return false;
  }
  rotateRingSeamToReference(ordered_blue, ego, max_gap);
  rotateRingSeamToReference(ordered_yellow, ego, max_gap);
  return true;
}

bool orderSlamBoundariesOpen(
  const std::vector<PlannerPoint> & blue_points,
  const std::vector<PlannerPoint> & yellow_points,
  const PlannerPoint & ego,
  double max_gap,
  std::vector<PlannerPoint> & ordered_blue,
  std::vector<PlannerPoint> & ordered_yellow,
  std::string & reason)
{
  // No direction alignment or seam rotation here. Both operate on CLOSED rings
  // (a rotation only relabels a closed ring's seam; on an open chain it would
  // move the very endpoints the caller is asking about), and the caller only
  // needs each chain's two ends.
  return orderBoundary(blue_points, ego, max_gap, false, true, ordered_blue, reason) &&
         orderBoundary(yellow_points, ego, max_gap, false, true, ordered_yellow, reason);
}

}
