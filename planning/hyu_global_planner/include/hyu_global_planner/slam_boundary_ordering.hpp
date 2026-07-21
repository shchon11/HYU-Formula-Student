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

}
