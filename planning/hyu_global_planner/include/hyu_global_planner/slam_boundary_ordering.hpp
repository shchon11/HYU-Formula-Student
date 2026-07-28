#pragma once

#include <string>
#include <vector>

#include "hyu_global_planner/planner_geometry.hpp"

namespace hyu_global_planner
{

// allow_two_opt_repair adds one last per-boundary retry tier that runs a
// length-minimising 2-opt over the walked ring. Off by default so existing
// callers (and any map the walk already handles) keep byte-identical results;
// the centerline builder enables it only after every plain attempt failed.
bool orderSlamBoundaries(
  const std::vector<PlannerPoint> & blue_points,
  const std::vector<PlannerPoint> & yellow_points,
  const PlannerPoint & ego,
  double max_gap,
  std::vector<PlannerPoint> & ordered_blue,
  std::vector<PlannerPoint> & ordered_yellow,
  std::string & reason,
  bool allow_two_opt_repair = false);

// Walk both boundaries WITHOUT requiring either ring to close: every interior
// step still respects max_gap and the self-intersection gate still applies, but
// a closing gap above max_gap is returned instead of rejected. This is a
// diagnostic entry point, not a path source -- it exists so the centerline
// builder's seam-closure tier can see WHERE a genuinely open boundary is open.
// Nothing that publishes a path may use it directly: an open ring has no
// centerline, and the closure it enables is validated by the ordinary gates.
bool orderSlamBoundariesOpen(
  const std::vector<PlannerPoint> & blue_points,
  const std::vector<PlannerPoint> & yellow_points,
  const PlannerPoint & ego,
  double max_gap,
  std::vector<PlannerPoint> & ordered_blue,
  std::vector<PlannerPoint> & ordered_yellow,
  std::string & reason);

}
