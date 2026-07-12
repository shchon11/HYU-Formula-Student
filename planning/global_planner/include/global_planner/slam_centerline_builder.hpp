#pragma once

#include <string>
#include <vector>

#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "global_planner/planner_geometry.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace global_planner
{

struct SlamCenterlineConfig
{
  int min_cones_per_side{3};
  double max_boundary_gap_m{12.0};
  double min_track_width_m{2.0};
  double max_track_width_m{6.0};
  double close_loop_distance_m{5.0};
  double waypoint_spacing_m{0.5};
  double duplicate_point_tolerance{0.001};
};

bool buildCenterlineFromSlamMap(
  const eufs_msgs::msg::ConeArrayWithCovariance & cone_map,
  const nav_msgs::msg::Odometry & ego_odom,
  const SlamCenterlineConfig & config,
  std::vector<PlannerWaypoint> & waypoints,
  std::string & reason);

}  // namespace global_planner
