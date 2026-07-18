#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "rclcpp/logger.hpp"

namespace hyu_global_planner
{

struct TrajectoryPoint
{
  double s{0.0};
  double x{0.0};
  double y{0.0};
  double psi{0.0};
  double kappa{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  // Optional CSV columns 8/9 (d_left_m; d_right_m): distance to the track
  // boundary on each side of the travel direction. 0 = unknown.
  double d_left{0.0};
  double d_right{0.0};
};

struct TrajectoryValidationOptions
{
  double duplicate_point_tolerance{1.0e-4};
  int min_waypoint_count{3};
  bool recompute_s_if_invalid{true};
};

bool loadTrajectoryCsv(
  const std::filesystem::path & path,
  const rclcpp::Logger & logger,
  std::vector<TrajectoryPoint> & points,
  std::string & error_message);

bool validateTrajectory(
  std::vector<TrajectoryPoint> & points,
  const TrajectoryValidationOptions & options,
  const rclcpp::Logger & logger,
  std::string & error_message);

}  // namespace hyu_global_planner
