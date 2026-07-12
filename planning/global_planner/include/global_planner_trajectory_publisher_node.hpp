#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "global_planner/trajectory_csv.hpp"
#include "rclcpp/rclcpp.hpp"

#include "eufs_msgs/msg/waypoint_array_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

namespace global_planner
{

class GlobalPlannerTrajectoryPublisherNode : public rclcpp::Node
{
public:
  GlobalPlannerTrajectoryPublisherNode();

private:
  void declareParameters();
  void loadParameters();
  bool resolveTrajectoryPath(
    std::filesystem::path & resolved_path, std::string & error_message) const;
  TrajectoryValidationOptions trajectoryValidationOptions() const;
  eufs_msgs::msg::WaypointArrayStamped buildWaypointMessage(
    const std::vector<TrajectoryPoint> & points);
  void setGlobalPathValid(bool valid);
  void publishValidityHeartbeat();
  void checkAndReloadTrajectory();
  void publishTrajectory();

  std::string trajectory_csv_path_;
  std::string output_root_;
  std::string map_name_;
  std::string trajectory_filename_;
  std::string global_waypoints_topic_;
  std::string global_path_valid_topic_;
  std::string frame_id_;
  double reload_period_sec_{1.0};
  double valid_heartbeat_hz_{5.0};
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
  bool global_path_is_valid_{false};
  bool true_heartbeat_ready_{true};

  rclcpp::Publisher<eufs_msgs::msg::WaypointArrayStamped>::SharedPtr global_waypoints_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr global_path_valid_pub_;
  rclcpp::TimerBase::SharedPtr reload_timer_;
  rclcpp::TimerBase::SharedPtr valid_heartbeat_timer_;
};

}  // namespace global_planner
