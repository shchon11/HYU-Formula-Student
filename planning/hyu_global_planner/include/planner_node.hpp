#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "eufs_msgs/msg/waypoint_array_stamped.hpp"
#include "hyu_global_planner/planner_geometry.hpp"
#include "hyu_global_planner/slam_centerline_builder.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace hyu_global_planner
{

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

private:
  void declareParameters();
  void loadParameters();
  void onConeMap(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);
  void onEgoOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGraphSlamStatus(const std_msgs::msg::String::SharedPtr msg);
  void onMapConverged(const std_msgs::msg::Bool::SharedPtr msg);
  void onHeartbeat();

  bool inputsAllowPlanning(std::string & reason) const;
  bool canHoldLastValidPath() const;
  SlamCenterlineConfig centerlineConfig() const;
  eufs_msgs::msg::WaypointArrayStamped buildWaypointMessage(
    const std::vector<PlannerWaypoint> & waypoints) const;
  void publishHeldPathValidity(const std::string & reason);
  void publishValidity(bool valid, const std::string & reason = "");
  void setInvalid(const std::string & reason);

  std::string cone_map_topic_;
  std::string ego_odom_topic_;
  std::string graph_slam_status_topic_;
  std::string graph_slam_map_converged_topic_;
  std::string global_waypoints_topic_;
  std::string global_path_valid_topic_;
  std::string global_path_reason_topic_;

  int min_cones_per_side_{3};
  double max_boundary_gap_m_{12.0};
  double min_track_width_m_{2.0};
  double max_track_width_m_{6.0};
  double close_loop_distance_m_{5.0};
  double waypoint_spacing_m_{0.5};
  double default_speed_mps_{4.0};
  // Curvature+friction speed profile (falls back to default_speed when the
  // path is too short to profile). See computeVelocityProfile.
  double max_speed_mps_{4.5};
  double min_speed_mps_{1.5};
  double max_lateral_accel_mps2_{6.0};
  double max_accel_mps2_{3.0};
  double max_decel_mps2_{5.0};
  double duplicate_point_tolerance_{0.001};
  double odom_timeout_sec_{0.5};
  double valid_heartbeat_hz_{5.0};
  bool hold_last_valid_path_{true};
  int raceline_smoothing_iterations_{0};
  double raceline_margin_m_{1.2};
  double raceline_alpha_{0.3};

  bool has_cone_map_{false};
  bool has_ego_odom_{false};
  bool has_graph_slam_status_{false};
  bool waypoint_spacing_valid_{true};
  bool path_valid_{false};
  bool warned_invalid_{false};
  std::uint64_t cone_map_version_{0U};
  std::uint64_t published_cone_map_version_{0U};
  // Content signature of the last accepted cone map. graph_slam keeps
  // republishing the frozen map in localization mode; identical content must
  // not count as a new map version (it would re-derive the path from the
  // current ego seed every tick and churn downstream consumers).
  std::uint64_t cone_map_signature_{0U};
  bool has_cone_map_signature_{false};

  eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr latest_cone_map_;
  nav_msgs::msg::Odometry latest_ego_odom_;
  rclcpp::Time latest_ego_odom_receive_time_;
  std::string graph_slam_status_;
  std::string waypoint_spacing_invalid_reason_;
  std::string last_invalid_reason_;

  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cone_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr graph_slam_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_converged_sub_;
  rclcpp::Publisher<eufs_msgs::msg::WaypointArrayStamped>::SharedPtr global_waypoints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_viz_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr global_path_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr global_path_reason_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace hyu_global_planner
