#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace hyu_global_planner
{

constexpr double kMinimumWaypointSpacingM = 0.05;

struct PlannerPoint
{
  double x{0.0};
  double y{0.0};
};

struct PlannerWaypoint
{
  double x{0.0};
  double y{0.0};
  double s{0.0};
  double psi{0.0};
  double kappa{0.0};
};

double distance(const PlannerPoint & a, const PlannerPoint & b);
PlannerPoint add(const PlannerPoint & a, const PlannerPoint & b);
PlannerPoint subtract(const PlannerPoint & a, const PlannerPoint & b);
PlannerPoint scale(const PlannerPoint & point, double factor);
double dot(const PlannerPoint & a, const PlannerPoint & b);
double cross2d(const PlannerPoint & a, const PlannerPoint & b);
double normalizeAngle(double angle);
bool isFinitePoint(const PlannerPoint & point);
std::optional<double> median(std::vector<double> values);
std::vector<double> cumulativeArcLengths(const std::vector<PlannerPoint> & points);
PlannerPoint interpolateAtArcLength(
  const std::vector<PlannerPoint> & points, const std::vector<double> & arc_lengths,
  double target_s);
std::vector<PlannerPoint> resampleBySpacing(
  const std::vector<PlannerPoint> & points, double spacing, double duplicate_tolerance);
std::vector<PlannerPoint> sampleNormalized(
  const std::vector<PlannerPoint> & points, std::size_t count);
void computeWaypointGeometry(std::vector<PlannerWaypoint> & waypoints);

// Curvature + friction speed profile (the raceline is otherwise driven at a
// flat speed). Corner speed is bounded by the friction circle
// v = sqrt(a_lat_max / |kappa|); a forward/backward pass then bounds
// longitudinal accel/decel so the profile is physically drivable.
struct VelocityProfileConfig
{
  double max_speed_mps{4.5};
  double min_speed_mps{1.5};
  double max_lateral_accel_mps2{6.0};
  double max_accel_mps2{3.0};
  double max_decel_mps2{5.0};
};

struct VelocityProfilePoint
{
  double vx{0.0};   // target speed [m/s]
  double ax{0.0};   // longitudinal accel toward the next point [m/s^2]
};

// One entry per input waypoint (uses each waypoint's s and kappa).
std::vector<VelocityProfilePoint> computeVelocityProfile(
  const std::vector<PlannerWaypoint> & waypoints, const VelocityProfileConfig & config);

}  // namespace hyu_global_planner
