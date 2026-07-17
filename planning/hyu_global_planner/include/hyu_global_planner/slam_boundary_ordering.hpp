#pragma once

#include <string>
#include <vector>

#include "hyu_global_planner/planner_geometry.hpp"

namespace hyu_global_planner
{

bool orderSlamBoundaries(
  const std::vector<PlannerPoint> & blue_points,
  const std::vector<PlannerPoint> & yellow_points,
  const PlannerPoint & ego,
  double max_gap,
  std::vector<PlannerPoint> & ordered_blue,
  std::vector<PlannerPoint> & ordered_yellow,
  std::string & reason);

}
