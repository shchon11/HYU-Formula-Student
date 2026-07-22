#include "hyu_control_harness/track.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace hyu_control_harness
{

namespace
{

std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

struct SegmentProjection
{
  double distance_sq{std::numeric_limits<double>::infinity()};
  double t{0.0};
};

SegmentProjection projectOntoSegment(const Vec2 & a, const Vec2 & b, const Vec2 & p)
{
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double len_sq = abx * abx + aby * aby;
  double t = 0.0;
  if (len_sq > 0.0) {
    t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / len_sq;
    t = std::max(0.0, std::min(1.0, t));
  }
  const double dx = p.x - (a.x + t * abx);
  const double dy = p.y - (a.y + t * aby);
  return {dx * dx + dy * dy, t};
}

}  // namespace

std::optional<Track> loadTrackCsv(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    return std::nullopt;
  }

  Track track;
  std::string line;
  bool first = true;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = splitCsvLine(line);
    if (first && !fields.empty() && fields[0] == "tag") {
      first = false;
      continue;
    }
    first = false;
    if (fields.size() < 4U) {
      continue;
    }
    Vec2 p;
    double direction = 0.0;
    try {
      p.x = std::stod(fields[1]);
      p.y = std::stod(fields[2]);
      direction = std::stod(fields[3]);
    } catch (const std::exception &) {
      return std::nullopt;
    }
    const std::string & tag = fields[0];
    if (tag == "blue") {
      track.blue.push_back(p);
    } else if (tag == "yellow") {
      track.yellow.push_back(p);
    } else if (tag == "orange") {
      track.orange.push_back(p);
    } else if (tag == "big_orange") {
      track.big_orange.push_back(p);
    } else if (tag == "car_start") {
      track.start = {p.x, p.y, direction};
    }
  }
  if (track.blue.empty() || track.yellow.empty()) {
    return std::nullopt;
  }
  return track;
}

Centerline Centerline::build(const Track & track)
{
  Centerline centerline;
  for (const Vec2 & b : track.blue) {
    double best_sq = std::numeric_limits<double>::infinity();
    const Vec2 * best = nullptr;
    for (const Vec2 & y : track.yellow) {
      const double dx = y.x - b.x;
      const double dy = y.y - b.y;
      const double d_sq = dx * dx + dy * dy;
      if (d_sq < best_sq) {
        best_sq = d_sq;
        best = &y;
      }
    }
    if (best == nullptr) {
      continue;
    }
    CenterlinePoint point;
    point.position = {0.5 * (b.x + best->x), 0.5 * (b.y + best->y)};
    point.half_width_m = 0.5 * std::sqrt(best_sq);
    centerline.points_.push_back(point);
  }

  // Open-corridor detection: on a closed track the last midpoint loops back
  // within roughly one cone spacing of the first; on an open corridor
  // (acceleration / dlc) the end-to-start gap IS the track length. Call it
  // open when the gap dominates the summed midpoint chain, and keep no wrap
  // segment: the ring's wrap edge would lie on top of the corridor and both
  // corrupt projections and double the length.
  double open_length = 0.0;
  for (std::size_t i = 0; i + 1U < centerline.points_.size(); ++i) {
    const Vec2 & a = centerline.points_[i].position;
    const Vec2 & b = centerline.points_[i + 1U].position;
    open_length += std::hypot(b.x - a.x, b.y - a.y);
  }
  if (centerline.points_.size() >= 2U) {
    const Vec2 & first = centerline.points_.front().position;
    const Vec2 & last = centerline.points_.back().position;
    const double gap = std::hypot(last.x - first.x, last.y - first.y);
    centerline.closed_ = gap < 0.25 * open_length;
  }

  centerline.cumulative_s_.resize(centerline.points_.size(), 0.0);
  double s = 0.0;
  for (std::size_t i = 0; i < centerline.points_.size(); ++i) {
    centerline.cumulative_s_[i] = s;
    if (i + 1U < centerline.points_.size() || centerline.closed_) {
      const Vec2 & a = centerline.points_[i].position;
      const Vec2 & b = centerline.points_[(i + 1U) % centerline.points_.size()].position;
      s += std::hypot(b.x - a.x, b.y - a.y);
    }
  }
  centerline.length_m_ = s;
  return centerline;
}

CenterlineProjection Centerline::project(const Vec2 & p) const
{
  CenterlineProjection best;
  best.cte_m = std::numeric_limits<double>::infinity();
  if (!valid()) {
    return best;
  }
  double best_sq = std::numeric_limits<double>::infinity();
  const std::size_t segment_count = closed_ ? points_.size() : points_.size() - 1U;
  for (std::size_t i = 0; i < segment_count; ++i) {
    const std::size_t j = (i + 1U) % points_.size();
    const auto projection = projectOntoSegment(points_[i].position, points_[j].position, p);
    if (projection.distance_sq < best_sq) {
      best_sq = projection.distance_sq;
      const Vec2 & a = points_[i].position;
      const Vec2 & b = points_[j].position;
      const double seg_len = std::hypot(b.x - a.x, b.y - a.y);
      best.cte_m = std::sqrt(projection.distance_sq);
      best.s_m = cumulative_s_[i] + projection.t * seg_len;
      best.half_width_m = points_[i].half_width_m +
        projection.t * (points_[j].half_width_m - points_[i].half_width_m);
      // Open corridor: a projection clamped to the very first or very last
      // midpoint means p lies before the corridor entry or past its end --
      // there is no track there to be off of.
      best.beyond_ends = !closed_ &&
        ((i == 0U && projection.t <= 0.0) ||
        (i + 1U == segment_count && projection.t >= 1.0));
    }
  }
  if (best.s_m >= length_m_) {
    best.s_m = closed_ ? best.s_m - length_m_ : length_m_;
  }
  return best;
}

double Centerline::wrappedDelta(double s_prev, double s_next) const
{
  double delta = s_next - s_prev;
  if (!closed_) {
    return delta;
  }
  while (delta > 0.5 * length_m_) {
    delta -= length_m_;
  }
  while (delta < -0.5 * length_m_) {
    delta += length_m_;
  }
  return delta;
}

}  // namespace hyu_control_harness
