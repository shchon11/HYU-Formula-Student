#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "eufs_msgs/msg/waypoint_array_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "state_machine/global_path_readiness.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace state_machine
{

enum class PathSource
{
  LOCAL,
  GLOBAL_FULL,
  GLOBAL_FINAL_STOP,
  STOP
};

class LapTrackingPolicy
{
public:
  LapTrackingPolicy(double closure_tolerance_m, double closing_duplicate_tolerance_m);

  bool observeGraphSlamStatus(const std::string & status);
  bool acceptPath(const eufs_msgs::msg::WaypointArrayStamped & msg);
  bool observeFrenetSample(
    double s, double receive_time_sec, double freshness_timeout_sec, double cooldown_sec);
  void refresh(double current_time_sec, double freshness_timeout_sec);

  bool pathValid() const;
  bool armed() const;
  double pathLength() const;
  std::uint64_t acceptedPathGeneration() const;

private:
  void disarm();

  double closure_tolerance_m_{1.0};
  double closing_duplicate_tolerance_m_{0.05};
  bool discovery_lap_counted_{false};
  std::string previous_graph_slam_status_;
  bool path_valid_{false};
  bool armed_{false};
  bool has_previous_sample_{false};
  double path_length_{0.0};
  double previous_s_{0.0};
  double previous_sample_time_sec_{0.0};
  double last_count_time_sec_{0.0};
  bool has_count_time_{false};
  std::uint64_t accepted_path_generation_{0U};
};

// Lap counting and lap timing anchored to the physical start/finish gate:
// the pair of big-orange cone groups on either side of the track. A lap is
// the ego motion segment crossing the gate segment, in the direction of the
// first observed crossing, after the car has been at least arm_distance away
// from the gate since the previous count (so spawn-area jitter never counts).
class OrangeGateLapTracker
{
public:
  OrangeGateLapTracker(
    double cluster_tolerance_m, double min_gate_width_m, double max_gate_width_m,
    double arm_distance_m, double cooldown_sec);

  // Rebuild the gate from map-frame big-orange cone positions. A previously
  // valid gate is kept when the new set cannot form one (latched map hiccup).
  bool updateGate(const std::vector<std::array<double, 2>> & big_orange_xy);
  // Feed one map-frame ego position; returns true when a lap is counted.
  bool observeEgo(double x, double y, double time_sec);

  bool gateValid() const;
  bool armed() const;
  int countedLaps() const;
  bool hasLapTime() const;
  double lastLapSec() const;
  double bestLapSec() const;
  // Running time of the lap in progress; negative until the timer starts
  // (first start-line crossing).
  double currentLapElapsedSec(double now_sec) const;

private:
  double cluster_tolerance_m_;
  double min_gate_width_m_;
  double max_gate_width_m_;
  double arm_distance_m_;
  double cooldown_sec_;

  bool gate_valid_{false};
  double gate_ax_{0.0};
  double gate_ay_{0.0};
  double gate_bx_{0.0};
  double gate_by_{0.0};
  double gate_cx_{0.0};
  double gate_cy_{0.0};

  bool has_prev_{false};
  double prev_x_{0.0};
  double prev_y_{0.0};
  double prev_time_sec_{0.0};
  bool armed_{false};
  bool has_direction_{false};
  double direction_sign_{0.0};
  int counted_laps_{0};
  bool timer_started_{false};
  double lap_start_time_sec_{0.0};
  bool has_count_time_{false};
  double last_count_time_sec_{0.0};
  bool has_lap_time_{false};
  double last_lap_sec_{0.0};
  double best_lap_sec_{0.0};
};

class PlanningStateMachineNode : public rclcpp::Node
{
public:
  PlanningStateMachineNode();

private:
  void onFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGlobalWaypoints(const eufs_msgs::msg::WaypointArrayStamped::SharedPtr msg);
  void onGraphSlamStatus(const std_msgs::msg::String::SharedPtr msg);
  void onGlobalPathValid(const std_msgs::msg::Bool::SharedPtr msg);
  void onLocalPathValid(const std_msgs::msg::Bool::SharedPtr msg);
  void onGlobalHandoffReady(const std_msgs::msg::Bool::SharedPtr msg);
  void onCones(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);
  void onSlamConeMap(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);
  void onEgoOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onStopZoneSStart(const std_msgs::msg::Float64::SharedPtr msg);
  void onStopZoneSEnd(const std_msgs::msg::Float64::SharedPtr msg);
  void onStopZoneValid(const std_msgs::msg::Bool::SharedPtr msg);
  void onTimer();
  void updateState();
  void publishOutputs();

  std::string globalEntryReason(const rclcpp::Time & current_time) const;
  std::string stopRequestReason() const;
  bool isStoplineDetected() const;
  bool detectStartFinishGate() const;
  bool hasCrossedStartFinishGate() const;
  void updateLapCount();
  void recomputeLapCount();

  bool isFresh(const rclcpp::Time & stamp, double timeout_sec) const;
  bool hasFreshFrenetOdom() const;
  bool hasFreshLocalPathValid() const;
  bool hasFreshConeMap() const;
  bool hasFreshStopZone() const;
  bool isSInStopZone(double s) const;

  std::string frenet_odom_topic_;
  std::string global_waypoints_topic_;
  std::string graph_slam_status_topic_;
  std::string global_path_valid_topic_;
  std::string local_path_valid_topic_;
  std::string global_handoff_ready_topic_;
  std::string cone_map_topic_;
  std::string slam_cone_map_topic_;
  std::string ego_odom_topic_;
  std::string stop_zone_s_start_topic_;
  std::string stop_zone_s_end_topic_;
  std::string stop_zone_valid_topic_;

  int target_lap_count_{4};
  int initial_lap_count_{0};
  int final_lap_start_count_{3};
  // Published lap count = max of two INDEPENDENT estimators, so neither can
  // wedge the count if the other stalls (see recomputeLapCount):
  //  - the orange-gate tracker (accurate + the sole lap-time source), and
  //  - the fallback (frenet seam-wrap + the mapping->localization floor).
  int lap_count_{0};
  int fallback_lap_count_{0};

  double frenet_odom_timeout_sec_{0.5};
  double global_path_valid_timeout_sec_{0.5};
  double global_handoff_timeout_sec_{0.5};
  double global_entry_dwell_sec_{0.5};
  double cone_map_timeout_sec_{1.0};
  double stop_zone_timeout_sec_{1.0};
  double lap_path_closure_tolerance_m_{1.0};
  double lap_closing_duplicate_tolerance_m_{0.05};
  double lap_gate_cluster_tolerance_m_{2.0};
  double lap_gate_min_width_m_{2.0};
  double lap_gate_max_width_m_{7.0};
  double lap_gate_arm_distance_m_{12.0};
  double lap_gate_cooldown_sec_{10.0};
  double final_path_end_threshold_{2.0};
  double stop_zone_s_margin_{0.0};
  double max_abs_d_for_global_{2.0};

  int state_timer_period_ms_{50};
  bool enable_manual_lap_override_{false};
  bool global_requires_graph_slam_localization_{true};

  PlanningState state_{PlanningState::LOCAL};

  double current_s_{0.0};
  double current_d_{0.0};
  double stop_zone_s_start_{0.0};
  double stop_zone_s_end_{0.0};
  std::size_t blue_cone_count_{0U};
  std::size_t yellow_cone_count_{0U};
  std::size_t orange_cone_count_{0U};
  std::size_t big_orange_cone_count_{0U};
  std::size_t unknown_cone_count_{0U};

  bool has_frenet_odom_{false};
  bool has_local_path_valid_{false};
  bool local_path_valid_{false};
  bool has_cone_map_{false};
  bool has_stop_zone_s_start_{false};
  bool has_stop_zone_s_end_{false};
  bool has_stop_zone_valid_{false};
  bool stop_zone_valid_{false};

  std::string closest_segment_id_;
  std::string cone_frame_id_;
  std::string last_stop_request_reason_{"not_requested"};
  GlobalPathReadiness global_path_readiness_;
  std::unique_ptr<LapTrackingPolicy> lap_tracking_policy_;
  std::unique_ptr<OrangeGateLapTracker> gate_tracker_;

  rclcpp::Time last_frenet_odom_time_;
  rclcpp::Time last_local_path_valid_time_;
  rclcpp::Time last_cone_map_time_;
  rclcpp::Time last_stop_zone_s_start_time_;
  rclcpp::Time last_stop_zone_s_end_time_;
  rclcpp::Time last_stop_zone_valid_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_odom_sub_;
  rclcpp::Subscription<eufs_msgs::msg::WaypointArrayStamped>::SharedPtr global_waypoints_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr graph_slam_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_path_valid_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr local_path_valid_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_handoff_ready_sub_;
  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cone_map_sub_;
  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr slam_cone_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr stop_zone_s_start_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr stop_zone_s_end_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_zone_valid_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_source_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lap_count_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lap_time_last_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lap_time_best_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace state_machine
