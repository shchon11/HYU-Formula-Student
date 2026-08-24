#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hyu_msgs/msg/can_state.hpp"
#include "hyu_msgs/msg/cone_array_with_covariance.hpp"
#include "hyu_msgs/msg/waypoint_array_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hyu_state_machine/global_path_readiness.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace hyu_state_machine
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
  bool acceptPath(const hyu_msgs::msg::WaypointArrayStamped & msg);
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
  double gateAx() const {return gate_ax_;}
  double gateAy() const {return gate_ay_;}
  double gateBx() const {return gate_bx_;}
  double gateBy() const {return gate_by_;}
  bool hasLapTime() const;
  double lastLapSec() const;
  double bestLapSec() const;
  // Running time of the lap in progress; negative until the timer starts
  // (first start-line crossing).
  double currentLapElapsedSec(double now_sec) const;

private:
  // A refitted gate whose endpoints moved more than this is treated as a NEW
  // line: crossing history and arming reset (early-mapping gates jump around,
  // and a moving gate sweeping over a stationary ego "crosses" it).
  static constexpr double kGateMoveResetM = 0.75;
  // Between-sample ego jumps beyond this are pose snaps/teleports, not
  // driving; the spanned segment is never tested against the gate.
  static constexpr double kMaxPlausibleStepM = 2.0;

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

// Single authoritative lap count (pure, so it is unit-testable in isolation
// from ROS). laps COMPLETED, mapping lap = lap 1:
//   base   = 1 once the mapping->localization discovery lap is seen, else 0.
//   racing = the ORANGE GATE as primary — a DELTA from the gate count at the
//            discovery moment, so the gate's spawn-dependent race-start is
//            cancelled; the frenet seam-wrap is a FALLBACK, used only when the
//            gate is not available (no valid gate).
//   result = max(previous, initial, base + racing)  (monotonic, forward-only).
// max()-combining gate and frenet on EVERY sample was the bug (frenet drops
// and base mismatches then double- or under-counted: trackdrive 10 read 11,
// the "1 1 2 3…" double).
int authoritativeLapCount(
  int previous_lap_count,
  int initial_lap_count,
  bool discovery_lap_seen,
  int gate_counted_laps,
  int gate_laps_at_discovery,
  bool gate_available,
  int frenet_fallback_laps);

class PlanningStateMachineNode : public rclcpp::Node
{
public:
  PlanningStateMachineNode();

private:
  void onFrenetOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onGlobalWaypoints(const hyu_msgs::msg::WaypointArrayStamped::SharedPtr msg);
  void onGraphSlamStatus(const std_msgs::msg::String::SharedPtr msg);
  void onGlobalPathValid(const std_msgs::msg::Bool::SharedPtr msg);
  void onLocalPathValid(const std_msgs::msg::Bool::SharedPtr msg);
  void onGlobalHandoffReady(const std_msgs::msg::Bool::SharedPtr msg);
  void onCones(const hyu_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);
  void onSlamConeMap(const hyu_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);
  void onEgoOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onStopZoneSStart(const std_msgs::msg::Float64::SharedPtr msg);
  void onStopZoneSEnd(const std_msgs::msg::Float64::SharedPtr msg);
  void onStopZoneValid(const std_msgs::msg::Bool::SharedPtr msg);
  void onAsState(const hyu_msgs::msg::CanState::SharedPtr msg);
  void onTimer();
  void updateMissionFinished();
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
  // Autocross: stop from ANY state (including LOCAL) the moment lap_count
  // reaches target_lap_count. Off for trackdrive (stop belongs to the
  // GLOBAL_FINAL_STOP path) and for skidpad, where the gate line is crossed
  // on every circle and would end the mission mid-figure-eight.
  bool stop_on_target_laps_{false};
  // KASE 제35조③ USS: after the run's final stop the DS must report Finished
  // within 30 s or the run is scored DNF. mission_completed_ goes (latched)
  // true once the vehicle has DRIVEN and then been at rest for
  // finish_rest_duration_sec while either
  //   - state_ == STOP (trackdrive / autocross lap-target stop), or
  //   - finish_on_rest_ is set (local-only missions — acceleration, skidpad —
  //     where the controller brakes itself when the corridor ends and the
  //     state machine never enters STOP).
  // The flag is published on /vehicle/mission_completed, which the vehicle
  // state machine (sim plugin today, actuation bridge later) turns into
  // AS_FINISHED — and the DSSI then goes dark per 제20조.
  bool finish_on_rest_{false};
  double finish_rest_speed_mps_{0.15};
  double finish_rest_duration_sec_{3.0};
  bool has_vehicle_moved_{false};
  bool mission_finished_{false};
  rclcpp::Time last_motion_time_;
  bool has_last_motion_time_{false};

  // Lap counting is inhibited until the vehicle has actually entered
  // AS_DRIVING once. The car spawns ON the start/finish gate, and SLAM/INS
  // (re)initialisation transients can throw the ego estimate >3 m (arming
  // the gate) and then jitter it across the line — a fake lap counted while
  // the car is physically standing still. With the long pre-mission standby
  // of the 3-step flow that fake lap was reliably observed (and it instantly
  // "completes" a 1-lap autocross). Pre-mission laps are meaningless anyway.
  bool vehicle_driving_seen_{false};
  // Resolved at construction: <0 in the parameter means target_lap_count - 1.
  int final_lap_start_count_{-1};
  // Published lap count = max of two INDEPENDENT estimators, so neither can
  // wedge the count if the other stalls (see recomputeLapCount):
  //  - the orange-gate tracker (accurate + the sole lap-time source), and
  //  - the fallback (frenet seam-wrap + the mapping->localization floor).
  // Single authoritative count (see recomputeLapCount): mapping lap = lap 1
  // (discovery), then the gate counts each racing lap as a DELTA from the
  // discovery moment (spawn-independent); frenet fills in only when the gate
  // is unavailable. lap_count_ = laps COMPLETED (mapping = 1).
  int lap_count_{0};
  int fallback_lap_count_{0};        // frenet racing-wrap count (0-based)
  bool discovery_lap_seen_{false};   // mapping lap completed (= lap 1)
  int gate_laps_at_discovery_{0};    // gate count anchored at discovery

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
  // Ego samples slower than this never reach the gate tracker: standstill
  // SLAM pose snaps must not arm the gate or count crossings.
  double min_lap_count_speed_mps_{0.3};
  // Final-lap finish arming: the gate crossing that opens the final-lap
  // window and the frenet seam wrap race each other by a sample; one state
  // tick with lap_count already bumped but current_s still ~= path length
  // finished the mission a whole lap early (2026-08-24 lite). The finish may
  // only fire after the car has demonstrably LEFT the path-end zone on the
  // final lap.
  bool final_path_end_armed_{false};
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
  rclcpp::Subscription<hyu_msgs::msg::WaypointArrayStamped>::SharedPtr global_waypoints_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr graph_slam_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_path_valid_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr local_path_valid_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_handoff_ready_sub_;
  rclcpp::Subscription<hyu_msgs::msg::ConeArrayWithCovariance>::SharedPtr cone_map_sub_;
  rclcpp::Subscription<hyu_msgs::msg::ConeArrayWithCovariance>::SharedPtr slam_cone_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr stop_zone_s_start_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr stop_zone_s_end_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_zone_valid_sub_;
  rclcpp::Subscription<hyu_msgs::msg::CanState>::SharedPtr as_state_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr path_source_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lap_count_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lap_time_last_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lap_time_best_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_request_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_completed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hyu_state_machine
