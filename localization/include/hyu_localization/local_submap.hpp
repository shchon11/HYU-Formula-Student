// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef HYU_LOCALIZATION__LOCAL_SUBMAP_HPP_
#define HYU_LOCALIZATION__LOCAL_SUBMAP_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <utility>
#include <vector>

#include "hyu_localization/gate_anchor.hpp"

namespace hyu_localization
{

// Rolling, odometry-consistent constellation of recent cone observations.
//
// Every consumer of scan matching in this stack (manual /initialpose, auto
// relocalization, seam closure) used to fit a SINGLE perception frame — 3-8
// cones — against the map. A 3-cone constellation aliases onto parallel
// track sections routinely (the autocross_kase2026 40 m mis-registration);
// a 20 m trail of cones, rigidly connected by short-horizon odometry, is
// close to unique on any FS track. Raw odometry is locally accurate (~1-2 %
// of distance), so over the submap span its internal distortion stays well
// below the matcher's inlier distance; the ABSOLUTE drift of the trail is
// exactly what the matcher solves for.
//
// Frames are stored with their raw-odometry pose; pointsInFrame() re-expresses
// everything in the body frame of a chosen reference pose (normally the raw
// pose of the newest frame) and merges repeated sightings of the same
// physical cone into per-color cluster centroids, so a cone seen 20 times
// counts once in inlier scoring.
//
// Header-only and dependency-free (mirrors gate_anchor.hpp) so the geometry
// is unit-testable without g2o/ROS.

// Color indices follow the frontend convention:
// 0 blue, 1 yellow, 2 orange, 3 big orange, 4+ unknown.
inline constexpr std::uint8_t kSubmapColorOrange = 2U;
inline constexpr std::uint8_t kSubmapColorBigOrange = 3U;
inline constexpr std::uint8_t kSubmapColorUnknown = 4U;

struct SubmapPoint
{
  double x{0.0};
  double y{0.0};
  std::uint8_t color{kSubmapColorUnknown};
};

inline GateSe2 composeSe2(const GateSe2 & a, const GateSe2 & b)
{
  const double c = std::cos(a.theta);
  const double s = std::sin(a.theta);
  GateSe2 out;
  out.x = a.x + c * b.x - s * b.y;
  out.y = a.y + s * b.x + c * b.y;
  out.theta = std::atan2(std::sin(a.theta + b.theta), std::cos(a.theta + b.theta));
  return out;
}

inline GateSe2 inverseSe2(const GateSe2 & a)
{
  const double c = std::cos(a.theta);
  const double s = std::sin(a.theta);
  GateSe2 out;
  out.x = -(c * a.x + s * a.y);
  out.y = -(-s * a.x + c * a.y);
  out.theta = -a.theta;
  return out;
}

inline SubmapPoint transformSubmapPoint(const GateSe2 & pose, const SubmapPoint & p)
{
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  return SubmapPoint{
    pose.x + c * p.x - s * p.y,
    pose.y + s * p.x + c * p.y,
    p.color};
}

// Orange and big-orange tolerate each other (the distinction is size-only and
// misclassifies at range); unknown wildcards. Blue-vs-yellow stays a hard
// mismatch — on parallel sections the lane colors mirror, which is exactly
// the aliasing this check exists to reject.
inline bool submapColorsCompatible(std::uint8_t a, std::uint8_t b)
{
  if (a >= kSubmapColorUnknown || b >= kSubmapColorUnknown) {
    return true;
  }
  if (a == b) {
    return true;
  }
  return (a == kSubmapColorOrange && b == kSubmapColorBigOrange) ||
         (a == kSubmapColorBigOrange && b == kSubmapColorOrange);
}

struct SubmapScore
{
  int inliers{0};
  double residual_sq{0.0};
};

// Count how many body-frame points land within inlier_distance of a
// color-compatible map target when the body is at `pose`.
inline SubmapScore scoreSubmapPose(
  const std::vector<SubmapPoint> & points_body,
  const std::vector<SubmapPoint> & targets_map,
  const GateSe2 & pose,
  double inlier_distance_m)
{
  SubmapScore score;
  const double inlier_sq = inlier_distance_m * inlier_distance_m;
  for (const SubmapPoint & p : points_body) {
    const SubmapPoint q = transformSubmapPoint(pose, p);
    double nearest_sq = std::numeric_limits<double>::max();
    for (const SubmapPoint & t : targets_map) {
      if (!submapColorsCompatible(p.color, t.color)) {
        continue;
      }
      const double dx = t.x - q.x;
      const double dy = t.y - q.y;
      nearest_sq = std::min(nearest_sq, dx * dx + dy * dy);
    }
    if (nearest_sq < inlier_sq) {
      ++score.inliers;
      score.residual_sq += nearest_sq;
    }
  }
  return score;
}

struct LocalSubmapParams
{
  // Travel span of history kept; drift inside it must stay below the
  // matcher's inlier distance (1-2 % odometry -> ~0.2-0.4 m over 20 m).
  double span_m{20.0};
  std::size_t max_frames{250};
  // A frame closer than this to the previous one REPLACES it, so a
  // stationary car keeps one fresh frame instead of flushing the history.
  double min_frame_travel_m{0.1};
  // Same-color sightings within this radius merge into one centroid.
  double dedup_radius_m{0.6};
  // Orange classes keep a tighter radius: the big-orange gate is REAL pairs
  // ~0.39 m apart, and merging a pair to one point costs the seam matcher
  // half its yaw evidence exactly where yaw matters most.
  double dedup_radius_orange_m{0.25};
};

class LocalConeSubmap
{
public:
  explicit LocalConeSubmap(LocalSubmapParams params = {})
  : params_(params) {}

  void reset() {frames_.clear();}

  std::size_t frameCount() const {return frames_.size();}

  void addFrame(
    const GateSe2 & raw_odom,
    std::vector<SubmapPoint> body_points,
    double traveled_m)
  {
    if (body_points.empty()) {
      return;
    }
    // Travel accounting restarted (reset / relocalize bookkeeping): the old
    // frames' traveled tags are meaningless now, drop them.
    if (!frames_.empty() && traveled_m < frames_.back().traveled_m) {
      frames_.clear();
    }
    if (!frames_.empty() &&
      traveled_m - frames_.back().traveled_m < params_.min_frame_travel_m)
    {
      frames_.back() = Frame{raw_odom, std::move(body_points), traveled_m};
    } else {
      frames_.push_back(Frame{raw_odom, std::move(body_points), traveled_m});
    }
    while (!frames_.empty() &&
      (frames_.size() > params_.max_frames ||
      traveled_m - frames_.front().traveled_m > params_.span_m))
    {
      frames_.pop_front();
    }
  }

  // All stored observations re-expressed in the body frame of `reference`
  // (odometry-relative, so this is exact up to short-horizon drift), with
  // repeated sightings of one physical cone merged into a per-color
  // centroid. Colors must match exactly to merge; a cone flickering between
  // a color and unknown yields two nearby points, which is harmless for
  // inlier scoring (both hit the same target).
  std::vector<SubmapPoint> pointsInFrame(const GateSe2 & reference) const
  {
    std::vector<SubmapPoint> centroids;
    std::vector<std::size_t> counts;
    const GateSe2 reference_inverse = inverseSe2(reference);
    const double radius_sq = params_.dedup_radius_m * params_.dedup_radius_m;
    const double orange_radius_sq =
      params_.dedup_radius_orange_m * params_.dedup_radius_orange_m;

    for (const Frame & frame : frames_) {
      const GateSe2 relative = composeSe2(reference_inverse, frame.raw_odom);
      for (const SubmapPoint & p : frame.points) {
        const SubmapPoint q = transformSubmapPoint(relative, p);
        bool joined = false;
        for (std::size_t i = 0; i < centroids.size(); ++i) {
          if (centroids[i].color != q.color) {
            continue;
          }
          const double dx = centroids[i].x - q.x;
          const double dy = centroids[i].y - q.y;
          const bool orange_class = q.color == kSubmapColorOrange ||
            q.color == kSubmapColorBigOrange;
          if (dx * dx + dy * dy > (orange_class ? orange_radius_sq : radius_sq)) {
            continue;
          }
          const double n = static_cast<double>(counts[i]);
          centroids[i].x = (centroids[i].x * n + q.x) / (n + 1.0);
          centroids[i].y = (centroids[i].y * n + q.y) / (n + 1.0);
          ++counts[i];
          joined = true;
          break;
        }
        if (!joined) {
          centroids.push_back(q);
          counts.push_back(1U);
        }
      }
    }
    return centroids;
  }

private:
  struct Frame
  {
    GateSe2 raw_odom;
    std::vector<SubmapPoint> points;
    double traveled_m;
  };

  LocalSubmapParams params_;
  std::deque<Frame> frames_;
};

}  // namespace hyu_localization

#endif  // HYU_LOCALIZATION__LOCAL_SUBMAP_HPP_
