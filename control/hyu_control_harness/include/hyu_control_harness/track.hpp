#pragma once

#include <optional>
#include <string>
#include <vector>

namespace hyu_control_harness
{

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

struct StartPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// Cone layout parsed from an eufs_tracks csv (tag,x,y,direction,...). Boundary
// vectors keep the file order, which the kase_layout generator writes in track
// order; the centerline builder relies on that ordering.
struct Track
{
  std::vector<Vec2> blue;
  std::vector<Vec2> yellow;
  std::vector<Vec2> orange;
  std::vector<Vec2> big_orange;
  StartPose start;
};

std::optional<Track> loadTrackCsv(const std::string & path);

// Reference centerline: midpoint of each blue cone and its nearest yellow cone,
// in blue order; half_width is half the pair distance, i.e. the cone line sits
// at |cte| == half_width. Closed tracks (the last midpoint loops back near the
// first) form a ring; an open corridor (acceleration / dlc: the end-to-start
// gap is a large fraction of the length) keeps no wrap segment, and
// projections from before the first or past the last midpoint are flagged
// beyond_ends so callers can exclude them from cone-line metrics -- the car
// legitimately starts before, and rolls out after, an open corridor.
struct CenterlinePoint
{
  Vec2 position;
  double half_width_m{0.0};
};

struct CenterlineProjection
{
  double cte_m{0.0};         // unsigned distance to the centerline
  double s_m{0.0};           // arc length of the projection, in [0, length)
  double half_width_m{0.0};  // interpolated half width at the projection
  bool beyond_ends{false};   // open centerline only: p lies before/past the ends
};

class Centerline
{
public:
  static Centerline build(const Track & track);

  bool valid() const { return points_.size() >= 3U; }
  bool closed() const { return closed_; }
  double length() const { return length_m_; }
  const std::vector<CenterlinePoint> & points() const { return points_; }

  CenterlineProjection project(const Vec2 & p) const;

  // Shortest signed arc-length step from s_prev to s_next on the ring; the
  // plain difference on an open centerline (there is nothing to wrap through).
  double wrappedDelta(double s_prev, double s_next) const;

private:
  std::vector<CenterlinePoint> points_;
  std::vector<double> cumulative_s_;  // per segment start
  double length_m_{0.0};
  bool closed_{true};
};

}  // namespace hyu_control_harness
