#pragma once

#include <vector>

#include "hyu_local_planner/local_path_builder.hpp"

namespace hyu_local_planner::internal
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
void classifyUnknownCones(
  std::vector<Point2> & blue, std::vector<Point2> & yellow,
  const std::vector<Point2> & unknown, const PlannerConfig & config);
// One classification pass over two colourless sources against a single
// snapshot of the labelled sets: `unknown` (colour drop-outs; their ego-side
// split is limited to config.unknown_geom_max_range_m) and `markers`
// (orange / big-orange gate cones; trusted boundary markers, unlimited split).
void classifyColorlessCones(
  std::vector<Point2> & blue, std::vector<Point2> & yellow,
  const std::vector<Point2> & unknown, const std::vector<Point2> & markers,
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
