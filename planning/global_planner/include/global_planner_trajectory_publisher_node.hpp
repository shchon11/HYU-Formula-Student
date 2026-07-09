#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "eufs_msgs/msg/waypoint_array_stamped.hpp"

namespace global_planner
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
};

class GlobalPlannerTrajectoryPublisherNode : public rclcpp::Node
{
public:
  GlobalPlannerTrajectoryPublisherNode();

private:
  void declareParameters();
  void loadParameters();
  bool resolveTrajectoryPath(
    std::filesystem::path & resolved_path, std::string & error_message) const;
  bool loadTrajectoryCsv(
    const std::filesystem::path & path, std::vector<TrajectoryPoint> & points,
    std::string & error_message) const;
  bool validateTrajectory(
    std::vector<TrajectoryPoint> & points, std::string & error_message) const;
  eufs_msgs::msg::WaypointArrayStamped buildWaypointMessage(
    const std::vector<TrajectoryPoint> & points);
  void checkAndReloadTrajectory();
  void publishTrajectory();

  std::string trajectory_csv_path_;
  std::string output_root_;
  std::string map_name_;
  std::string trajectory_filename_;
  std::string global_waypoints_topic_;
  std::string frame_id_;
  double reload_period_sec_{1.0};
  double duplicate_point_tolerance_{1.0e-4};
  int min_waypoint_count_{3};
  bool recompute_s_if_invalid_{true};

  std::string resolved_path_log_;
  bool has_valid_trajectory_{false};
  bool has_last_loaded_write_time_{false};
  bool has_last_failed_write_time_{false};
  std::filesystem::file_time_type last_loaded_write_time_;
  std::filesystem::file_time_type last_failed_write_time_;
  std::vector<TrajectoryPoint> trajectory_points_;

  rclcpp::Publisher<eufs_msgs::msg::WaypointArrayStamped>::SharedPtr global_waypoints_pub_;
  rclcpp::TimerBase::SharedPtr reload_timer_;
};

}  // namespace global_planner
