#include "clcs_frenet_converter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

#include "geometry/clcs_exceptions.h"

namespace frenet_conversion
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinSegmentLength = 1.0e-9;

double distance(const ReferenceWaypoint & a, const ReferenceWaypoint & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

double cross(
  const ReferenceWaypoint & a,
  const ReferenceWaypoint & b,
  const ReferenceWaypoint & c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool rangesOverlap(double a_min, double a_max, double b_min, double b_max)
{
  if (a_min > a_max) {
    std::swap(a_min, a_max);
  }
  if (b_min > b_max) {
    std::swap(b_min, b_max);
  }
  return std::max(a_min, b_min) <= std::min(a_max, b_max);
}

bool segmentsIntersect(
  const ReferenceWaypoint & a,
  const ReferenceWaypoint & b,
  const ReferenceWaypoint & c,
  const ReferenceWaypoint & d)
{
  if (!rangesOverlap(a.x, b.x, c.x, d.x) || !rangesOverlap(a.y, b.y, c.y, d.y)) {
    return false;
  }

  const double c1 = cross(a, b, c);
  const double c2 = cross(a, b, d);
  const double c3 = cross(c, d, a);
  const double c4 = cross(c, d, b);
  return (c1 * c2 < 0.0) && (c3 * c4 < 0.0);
}

bool finiteWaypoint(const ReferenceWaypoint & waypoint)
{
  return std::isfinite(waypoint.x) && std::isfinite(waypoint.y) && std::isfinite(waypoint.s);
}

}  // namespace

ClcsFrenetConverter::Ptr ClcsFrenetConverter::create(
  const std::vector<ReferenceWaypoint> & waypoints,
  const ClcsFrenetConfig & config,
  const std::uint64_t path_version)
{
  const auto start = std::chrono::steady_clock::now();

  ClcsBuildStats stats;
  stats.path_version = path_version;
  stats.input_waypoint_count = waypoints.size();

  const auto reference_points = preprocessReferencePath(waypoints, config, stats);
  if (reference_points.size() < 3) {
    throw std::runtime_error("CLCS reference path requires at least 3 valid points");
  }

  stats.track_length = computePathLength(reference_points);
  if (stats.track_length < config.min_path_length) {
    throw std::runtime_error("CLCS reference path length is too short");
  }

  stats.waypoint_s_max_error = computeWaypointSMaxError(reference_points);
  stats.large_gap_count = countLargeGaps(reference_points, config.large_gap_factor);
  stats.self_intersection_count = countSelfIntersections(reference_points, config.closed_loop);

  auto clcs = std::make_shared<geometry::CurvilinearCoordinateSystem>(
    toEigenPolyline(reference_points),
    config.projection_domain_limit,
    config.projection_domain_epsilon,
    config.projection_domain_eps2,
    "off",
    config.projection_domain_method);

  stats.track_length = clcs->length();
  stats.reference_point_count = reference_points.size();
  const auto end = std::chrono::steady_clock::now();
  stats.build_time_ms =
    std::chrono::duration<double, std::milli>(end - start).count();

  return Ptr(new ClcsFrenetConverter(config, waypoints, reference_points, clcs, stats));
}

ClcsFrenetConverter::ClcsFrenetConverter(
  const ClcsFrenetConfig & config,
  const std::vector<ReferenceWaypoint> & source_waypoints,
  const std::vector<ReferenceWaypoint> & reference_points,
  std::shared_ptr<geometry::CurvilinearCoordinateSystem> clcs,
  const ClcsBuildStats & stats)
: config_(config),
  source_waypoints_(source_waypoints),
  reference_points_(reference_points),
  clcs_(std::move(clcs)),
  stats_(stats)
{
}

ClcsConversionResult ClcsFrenetConverter::convert(const ClcsConversionInput & input) const
{
  const auto start = std::chrono::steady_clock::now();

  ClcsConversionResult result;
  result.track_length = stats_.track_length;
  result.clcs_build_time_ms = stats_.build_time_ms;
  result.waypoint_s_max_error = stats_.waypoint_s_max_error;
  result.path_version = stats_.path_version;

  if (!std::isfinite(input.x) || !std::isfinite(input.y) || !std::isfinite(input.yaw)) {
    result.error_message = "non-finite Cartesian pose input";
    return result;
  }

  try {
    int segment_index = -1;
    const Eigen::Vector2d curvilinear =
      clcs_->convertToCurvilinearCoordsAndGetSegmentIdx(input.x, input.y, segment_index, false);

    if (!std::isfinite(curvilinear.x()) || !std::isfinite(curvilinear.y())) {
      result.error_message = "CLCS returned non-finite curvilinear coordinates";
      return result;
    }

    result.raw_s = curvilinear.x();
    result.s = normalizeS(curvilinear.x());
    result.d = curvilinear.y();
    result.segment_index = segment_index;

    if (std::isfinite(config_.max_projection_distance) &&
      config_.max_projection_distance > 0.0 &&
      std::abs(result.d) > config_.max_projection_distance)
    {
      result.error_message = "projection distance exceeds max_projection_distance";
      return result;
    }

    result.reference_yaw = referenceYaw(result.raw_s);
    result.heading_error = normalizeAngle(input.yaw - result.reference_yaw);

    double vx_map = input.linear_x;
    double vy_map = input.linear_y;
    if (config_.velocity_frame == VelocityFrame::kBody) {
      const double cos_yaw = std::cos(input.yaw);
      const double sin_yaw = std::sin(input.yaw);
      vx_map = cos_yaw * input.linear_x - sin_yaw * input.linear_y;
      vy_map = sin_yaw * input.linear_x + cos_yaw * input.linear_y;
    }

    if (config_.publish_frenet_velocity) {
      const double cos_ref = std::cos(result.reference_yaw);
      const double sin_ref = std::sin(result.reference_yaw);
      result.v_s = cos_ref * vx_map + sin_ref * vy_map;
      result.v_d = -sin_ref * vx_map + cos_ref * vy_map;
    } else {
      result.v_s = input.linear_x;
      result.v_d = input.linear_y;
    }
    result.yaw_rate = input.yaw_rate;

    const Eigen::Vector2d reconstructed =
      clcs_->convertToCartesianCoords(sForClcsQuery(result.raw_s), result.d, false);
    result.reconstruction_error =
      std::hypot(reconstructed.x() - input.x, reconstructed.y() - input.y);

    const auto end = std::chrono::steady_clock::now();
    result.conversion_time_us =
      std::chrono::duration<double, std::micro>(end - start).count();
    result.valid = std::isfinite(result.s) && std::isfinite(result.d) &&
      std::isfinite(result.reference_yaw) && std::isfinite(result.heading_error) &&
      std::isfinite(result.v_s) && std::isfinite(result.v_d);
    if (!result.valid) {
      result.error_message = "conversion result contains non-finite values";
    }
    return result;
  } catch (const geometry::ProjectionDomainError & e) {
    result.error_message = e.what();
  } catch (const std::exception & e) {
    result.error_message = e.what();
  }

  const auto end = std::chrono::steady_clock::now();
  result.conversion_time_us =
    std::chrono::duration<double, std::micro>(end - start).count();
  return result;
}

bool ClcsFrenetConverter::pathChanged(
  const std::vector<ReferenceWaypoint> & previous,
  const std::vector<ReferenceWaypoint> & current,
  const double tolerance)
{
  if (previous.size() != current.size()) {
    return true;
  }

  for (std::size_t i = 0; i < previous.size(); ++i) {
    if (std::hypot(previous[i].x - current[i].x, previous[i].y - current[i].y) > tolerance) {
      return true;
    }
  }
  return false;
}

double ClcsFrenetConverter::normalizeAngle(double angle)
{
  while (angle >= kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

std::vector<ReferenceWaypoint> ClcsFrenetConverter::preprocessReferencePath(
  const std::vector<ReferenceWaypoint> & waypoints,
  const ClcsFrenetConfig & config,
  ClcsBuildStats & stats)
{
  std::vector<ReferenceWaypoint> reference_points;
  reference_points.reserve(waypoints.size() + 1);

  for (const auto & waypoint : waypoints) {
    if (!finiteWaypoint(waypoint)) {
      ++stats.invalid_point_count;
      continue;
    }
    if (!reference_points.empty() &&
      distance(reference_points.back(), waypoint) <= config.duplicate_point_tolerance)
    {
      ++stats.removed_duplicate_count;
      continue;
    }
    reference_points.push_back(waypoint);
  }

  if (config.closed_loop && reference_points.size() >= 3) {
    if (distance(reference_points.front(), reference_points.back()) <=
      config.duplicate_point_tolerance)
    {
      reference_points.pop_back();
      ++stats.removed_duplicate_count;
    }

    if (distance(reference_points.front(), reference_points.back()) >
      config.duplicate_point_tolerance)
    {
      ReferenceWaypoint closing_point = reference_points.front();
      closing_point.s = computePathLength(reference_points) +
        distance(reference_points.back(), reference_points.front());
      reference_points.push_back(closing_point);
      stats.closed_by_appending_first_point = true;
    }
  }

  return reference_points;
}

geometry::EigenPolyline ClcsFrenetConverter::toEigenPolyline(
  const std::vector<ReferenceWaypoint> & reference_points)
{
  geometry::EigenPolyline polyline;
  polyline.reserve(reference_points.size());
  for (const auto & point : reference_points) {
    polyline.emplace_back(point.x, point.y);
  }
  return polyline;
}

double ClcsFrenetConverter::computePathLength(const std::vector<ReferenceWaypoint> & points)
{
  if (points.size() < 2) {
    return 0.0;
  }

  double length = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    length += distance(points[i], points[i + 1]);
  }
  return length;
}

double ClcsFrenetConverter::computeWaypointSMaxError(
  const std::vector<ReferenceWaypoint> & points)
{
  if (points.size() < 2) {
    return 0.0;
  }

  double cumulative_s = points.front().s;
  double max_error = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    cumulative_s += distance(points[i - 1], points[i]);
    max_error = std::max(max_error, std::abs(points[i].s - cumulative_s));
  }
  return max_error;
}

std::size_t ClcsFrenetConverter::countLargeGaps(
  const std::vector<ReferenceWaypoint> & points,
  const double large_gap_factor)
{
  if (points.size() < 3 || large_gap_factor <= 0.0) {
    return 0;
  }

  std::vector<double> lengths;
  lengths.reserve(points.size() - 1);
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double segment_length = distance(points[i], points[i + 1]);
    if (segment_length > kMinSegmentLength) {
      lengths.push_back(segment_length);
    }
  }

  if (lengths.empty()) {
    return 0;
  }

  std::sort(lengths.begin(), lengths.end());
  const double median = lengths[lengths.size() / 2];
  return static_cast<std::size_t>(
    std::count_if(lengths.begin(), lengths.end(), [median, large_gap_factor](const double length) {
      return length > median * large_gap_factor;
    }));
}

std::size_t ClcsFrenetConverter::countSelfIntersections(
  const std::vector<ReferenceWaypoint> & points,
  const bool closed_loop)
{
  if (points.size() < 4) {
    return 0;
  }

  std::size_t count = 0;
  const std::size_t segment_count = points.size() - 1;
  for (std::size_t i = 0; i < segment_count; ++i) {
    for (std::size_t j = i + 1; j < segment_count; ++j) {
      if (j == i + 1) {
        continue;
      }
      if (closed_loop && i == 0 && j + 1 == segment_count) {
        continue;
      }
      if (segmentsIntersect(points[i], points[i + 1], points[j], points[j + 1])) {
        ++count;
      }
    }
  }
  return count;
}

double ClcsFrenetConverter::normalizeS(const double s) const
{
  if (!config_.closed_loop || stats_.track_length <= 0.0) {
    return s;
  }

  double normalized = std::fmod(s, stats_.track_length);
  if (normalized < 0.0) {
    normalized += stats_.track_length;
  }
  if (normalized >= stats_.track_length) {
    normalized = 0.0;
  }
  return normalized;
}

double ClcsFrenetConverter::sForClcsQuery(const double s) const
{
  if (stats_.track_length <= 0.0) {
    return s;
  }

  const double eps = std::max(1.0e-6, std::min(config_.tangent_epsilon, stats_.track_length * 0.25));
  if (s <= 0.0) {
    return eps;
  }
  if (s >= stats_.track_length) {
    return stats_.track_length - eps;
  }
  return s;
}

double ClcsFrenetConverter::referenceYaw(const double raw_s) const
{
  const Eigen::Vector2d tangent = clcs_->tangent(sForClcsQuery(raw_s));
  return std::atan2(tangent.y(), tangent.x());
}

VelocityFrame parseVelocityFrame(const std::string & value)
{
  if (value == "map" || value == "Map" || value == "MAP") {
    return VelocityFrame::kMap;
  }
  return VelocityFrame::kBody;
}

std::string toString(const VelocityFrame value)
{
  return value == VelocityFrame::kMap ? "map" : "body";
}

}  // namespace frenet_conversion
