#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace global_planner
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

}  // namespace global_planner
