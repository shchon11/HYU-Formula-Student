// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef HYU_LOCALIZATION__GATE_ANCHOR_HPP_
#define HYU_LOCALIZATION__GATE_ANCHOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace hyu_localization
{

// Orange-gate global anchor: recover an absolute SE2 pose from the start
// gate's big orange cones, independent of accumulated drift.
//
// Per-cone nearest-neighbour association can only close drift smaller than
// its gate (~1-3 m): cones are indistinguishable and ~3 m apart, so a wider
// gate aliases onto neighbouring cones. The big orange start-gate cones are
// the one exception — they exist nowhere else on the track — so matching the
// observed big-orange CONSTELLATION (two gate sides, a rigid 2-point body)
// against the mapped one yields a full SE2 fix whose validity does not
// depend on how far the pose has drifted. Correspondence ambiguity (left
// gate post vs right, and the 180-degree flip that comes with swapping
// them) is resolved by transforming ALL currently visible cones with each
// candidate and counting map inliers: only the true pose also explains the
// ordinary track cones around the gate.
//
// Header-only and dependency-free (mirrors slam_lifecycle_classifiers.hpp)
// so the geometry is unit-testable without g2o/ROS.

using GatePoint = std::array<double, 2>;

struct GateSe2
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

struct GateMatchParams
{
  // Observed big-orange cones within this radius collapse into one cluster
  // centroid: a gate SIDE is a pair of big cones ~0.4 m apart, and the two
  // sides (track width apart) are the two points the alignment needs.
  double cluster_radius_m{1.5};
  // Cross-check: the observed side-to-side distance must match the mapped
  // one within this tolerance before a candidate is even scored.
  double pair_tolerance_m{0.5};
  // Plausible gate widths; pairs outside are geometric nonsense (a lone
  // side seen twice, or clusters from different track features).
  double min_pair_separation_m{2.0};
  double max_pair_separation_m{9.0};
  // Verification: a candidate pose must place at least min_inliers of ALL
  // visible cones within inlier_distance of some map landmark.
  double inlier_distance_m{0.6};
  int min_inliers{4};
};

struct GateMatchResult
{
  GateSe2 pose{};
  int inliers{0};
  double gate_residual_m{0.0};
};

namespace gate_anchor_detail
{

inline double distance(const GatePoint & a, const GatePoint & b)
{
  return std::hypot(a[0] - b[0], a[1] - b[1]);
}

inline GatePoint transform(const GateSe2 & pose, const GatePoint & p)
{
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  return GatePoint{
    pose.x + c * p[0] - s * p[1],
    pose.y + s * p[0] + c * p[1]};
}

inline int countInliers(
  const std::vector<GatePoint> & observations_body,
  const std::vector<GatePoint> & landmarks,
  const GateSe2 & pose,
  double inlier_distance_m)
{
  int inliers = 0;
  const double inlier_sq = inlier_distance_m * inlier_distance_m;
  for (const GatePoint & obs : observations_body) {
    const GatePoint p = transform(pose, obs);
    double nearest_sq = std::numeric_limits<double>::max();
    for (const GatePoint & lm : landmarks) {
      const double dx = lm[0] - p[0];
      const double dy = lm[1] - p[1];
      const double d2 = dx * dx + dy * dy;
      nearest_sq = std::min(nearest_sq, d2);
    }
    if (nearest_sq < inlier_sq) {
      ++inliers;
    }
  }
  return inliers;
}

}  // namespace gate_anchor_detail

// Greedy centroid clustering: each point joins the first cluster whose
// running centroid is within radius, else founds a new one. Adequate for the
// handful of big-orange points involved.
inline std::vector<GatePoint> clusterCentroids(
  const std::vector<GatePoint> & points, double radius_m)
{
  std::vector<GatePoint> centroids;
  std::vector<std::size_t> counts;
  for (const GatePoint & p : points) {
    bool joined = false;
    for (std::size_t i = 0; i < centroids.size(); ++i) {
      if (gate_anchor_detail::distance(centroids[i], p) <= radius_m) {
        const double n = static_cast<double>(counts[i]);
        centroids[i][0] = (centroids[i][0] * n + p[0]) / (n + 1.0);
        centroids[i][1] = (centroids[i][1] * n + p[1]) / (n + 1.0);
        ++counts[i];
        joined = true;
        break;
      }
    }
    if (!joined) {
      centroids.push_back(p);
      counts.push_back(1U);
    }
  }
  return centroids;
}

// Match the observed big-orange constellation (body frame) against the
// mapped gate points and return the best verified vehicle pose. Candidates
// come from every ordered observed-cluster pair x mapped-cluster pair whose
// separations agree; each is scored by how many of all_observations_body it
// makes map inliers. Returns false when no candidate reaches min_inliers.
inline bool matchGateConstellation(
  const std::vector<GatePoint> & observed_gate_body,
  const std::vector<GatePoint> & gate_landmarks,
  const std::vector<GatePoint> & all_observations_body,
  const std::vector<GatePoint> & all_landmarks,
  const GateMatchParams & params,
  GateMatchResult * result)
{
  const std::vector<GatePoint> observed_sides =
    clusterCentroids(observed_gate_body, params.cluster_radius_m);
  const std::vector<GatePoint> mapped_sides =
    clusterCentroids(gate_landmarks, params.cluster_radius_m);
  if (observed_sides.size() < 2U || mapped_sides.size() < 2U) {
    return false;
  }

  bool found = false;
  GateMatchResult best;
  best.inliers = params.min_inliers - 1;

  for (std::size_t oi = 0; oi < observed_sides.size(); ++oi) {
    for (std::size_t oj = 0; oj < observed_sides.size(); ++oj) {
      if (oi == oj) {
        continue;
      }
      const GatePoint & o1 = observed_sides[oi];
      const GatePoint & o2 = observed_sides[oj];
      const double observed_sep = gate_anchor_detail::distance(o1, o2);
      if (observed_sep < params.min_pair_separation_m ||
        observed_sep > params.max_pair_separation_m)
      {
        continue;
      }
      // Unordered mapped pairs; the ordered observed loop already covers
      // both correspondences of each pairing.
      for (std::size_t mi = 0; mi + 1 < mapped_sides.size(); ++mi) {
        for (std::size_t mj = mi + 1; mj < mapped_sides.size(); ++mj) {
          const GatePoint & m1 = mapped_sides[mi];
          const GatePoint & m2 = mapped_sides[mj];
          const double mapped_sep = gate_anchor_detail::distance(m1, m2);
          if (std::abs(mapped_sep - observed_sep) > params.pair_tolerance_m) {
            continue;
          }

          // Two-point rigid alignment: rotation aligns the o1->o2 baseline
          // with m1->m2, translation pins o1 onto m1.
          const double theta =
            std::atan2(m2[1] - m1[1], m2[0] - m1[0]) -
            std::atan2(o2[1] - o1[1], o2[0] - o1[0]);
          GateSe2 candidate;
          candidate.theta = std::atan2(std::sin(theta), std::cos(theta));
          const double c = std::cos(candidate.theta);
          const double s = std::sin(candidate.theta);
          candidate.x = m1[0] - (c * o1[0] - s * o1[1]);
          candidate.y = m1[1] - (s * o1[0] + c * o1[1]);

          const int inliers = gate_anchor_detail::countInliers(
            all_observations_body, all_landmarks, candidate,
            params.inlier_distance_m);
          const double residual = std::abs(mapped_sep - observed_sep);
          if (inliers > best.inliers ||
            (inliers == best.inliers && found && residual < best.gate_residual_m))
          {
            best.pose = candidate;
            best.inliers = inliers;
            best.gate_residual_m = residual;
            found = true;
          }
        }
      }
    }
  }

  if (!found) {
    return false;
  }
  if (result != nullptr) {
    *result = best;
  }
  return true;
}

}  // namespace hyu_localization

#endif  // HYU_LOCALIZATION__GATE_ANCHOR_HPP_
