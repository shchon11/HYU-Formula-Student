#pragma once

#include <vector>

#include "local_planner/local_path_builder.hpp"

namespace local_planner::internal
{

enum class BoundarySide
{
  kBlue,
  kYellow,
};

enum class TraversalFailure
{
  kNone,
  kTopologyGap,
  kHeadingJump,
  kUTurnBranchAmbiguous,
  kFoldBack,
};

struct TraversalResult
{
  std::vector<Point2> points;
  TraversalFailure failure{TraversalFailure::kNone};
};

double distance(const Point2 & first, const Point2 & second);
double normalizeAngle(double angle);
std::vector<Point2> cropToRoi(const std::vector<Point2> & input, const PlannerConfig & config);
std::vector<Point2> deduplicate(
  std::vector<Point2> points, double tolerance, bool exact_only = false);
std::vector<Point2> boundaryMidpoints(
  const std::vector<Point2> & blue, const std::vector<Point2> & yellow,
  const PlannerConfig & config);
TraversalResult forwardTraversalWithReason(
  const std::vector<Point2> & points, const PlannerConfig & config);
std::vector<Point2> forwardTraversal(const std::vector<Point2> & points, const PlannerConfig & config);
std::vector<Point2> offsetBoundary(
  const std::vector<Point2> & ordered, BoundarySide side, double offset);
BuildResult finishPath(
  const std::vector<Point2> & raw_path, double horizon, double speed, PathKind kind,
  const PlannerConfig & config);

}
