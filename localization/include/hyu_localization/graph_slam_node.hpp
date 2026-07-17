// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SIZE_OK: This integration only preserves topic/config compatibility; splitting
// the legacy monolithic graph SLAM node declaration is outside this focused scope.

#ifndef HYU_LOCALIZATION__GRAPH_SLAM_NODE_HPP_
#define HYU_LOCALIZATION__GRAPH_SLAM_NODE_HPP_

#include <Eigen/Core>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d/vertex_se2.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "hyu_msgs/msg/car_state.hpp"
#include "hyu_msgs/msg/cone_array_with_covariance.hpp"
#include "hyu_msgs/msg/cone_with_covariance.hpp"
#include "hyu_localization/gate_anchor.hpp"
#include "hyu_localization/slam_lifecycle_classifiers.hpp"
#include "hyu_localization/tentative_track_frontend.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace hyu_localization
{

class GraphSlamNode : public rclcpp::Node
{
public:
  GraphSlamNode();

private:
  enum class ConeColor : std::uint8_t
  {
    Blue,
    Yellow,
    Orange,
    BigOrange,
    Unknown
  };

  struct PoseRecord
  {
    int graph_id;
    g2o::VertexSE2 * vertex;
    g2o::SE2 raw_odom;
    rclcpp::Time stamp;
  };

  struct LandmarkRecord
  {
    int graph_id;
    ConeColor color;
    g2o::VertexPointXY * vertex;
    Eigen::Matrix2d covariance;
    std::size_t observations;
    int consecutive_misses;
    // traveled_distance_ value when this landmark was last associated; drift
    // (and therefore the association gate inflation) grows with distance
    // driven, independent of keyframe density.
    double last_seen_traveled;
    std::array<std::uint16_t, 5> color_votes;
  };

  struct ConeObservation
  {
    Eigen::Vector2d measurement;
    Eigen::Matrix2d covariance;
    ConeColor color;
  };

  struct ObservationUpdate
  {
    std::size_t added_edges;
    std::size_t updated_landmarks;
    std::size_t deleted_landmarks;
    // Observations that associated with an EXISTING landmark this frame.
    // Zero while cones are visible is the lost signature in localization
    // mode (the frozen map creates no new landmarks to absorb them).
    std::size_t matched_landmarks;
  };

  void declareRecoveryParameters();
  void configureOptimizer();
  void resetGraph();

  void stateCallback(const hyu_msgs::msg::CarState::SharedPtr msg);
  void conesCallback(const hyu_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);
  void gnssOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void relocalizeTo(const g2o::SE2 & pose);
  // Re-anchor the trajectory at an already-verified pose (no scan-match).
  void relocalizeAt(const g2o::SE2 & pose);
  // Coarse-to-fine grid search of the latest cone observations against the
  // map around seed; reports the fine pass's inlier count for acceptance
  // gating (the manual /initialpose path applies the result regardless —
  // the operator asserted the neighbourhood — the automatic path must not).
  g2o::SE2 scanMatchNear(
    const g2o::SE2 & seed, double radius, double yaw_span,
    int * inliers_out) const;
  g2o::SE2 gridSearchPose(
    const g2o::SE2 & seed, double radius, double xy_step,
    double yaw_span, double yaw_step, double inlier_distance,
    int * best_inliers_out) const;
  // Lost detection + automatic recovery against the frozen map: cones are
  // visible but none associate for N consecutive frames -> re-localize via
  // (orange-gate seed, else escalating-radius grid search), gated on the
  // scan-match inlier count.
  void maybeAutoRelocalize(const ObservationUpdate & update, const rclcpp::Time & stamp);
  // Orange-gate constellation match: orange/big-orange observations (the
  // big/small distinction is size-only and misclassifies at range) against
  // the big-orange landmarks (optionally only those near the lap origin, to
  // exclude drift-era duplicate gates while mapping). Drift-magnitude
  // independent — see gate_anchor.hpp.
  bool matchGateFromObservations(
    const std::vector<ConeObservation> & observations,
    bool restrict_to_lap_origin,
    g2o::SE2 * pose_out,
    int * inliers_out) const;
  // Mapping-lap seam closure beyond the association gate: when the gate
  // match implies a correction too large for per-cone association to ever
  // recover, re-seed the current keyframe estimate so the optimizer closes
  // the loop instead of duplicating the map. Returns true if applied.
  bool maybeApplyGateAnchor(
    const std::vector<ConeObservation> & observations,
    const g2o::SE2 & observation_pose,
    const g2o::SE2 & keyframe_to_observation,
    const PoseRecord & pose);
  void suppressGnssPriors(double now_sec);
  g2o::SE2 latestRawOdom() const;

  g2o::SE2 poseFromCarState(const hyu_msgs::msg::CarState & msg) const;
  g2o::SE2 estimateFromRawOdometry(const g2o::SE2 & raw_odom) const;
  bool shouldCreateKeyframe(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp) const;
  void addInitialPose(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp);
  void addKeyframe(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp);
  // Anchor a keyframe's (x,y) to the latest GNSS absolute fix with a unary
  // EdgeSE2XYPrior, gated on the fix's freshness and covariance so the anchor
  // fades out smoothly as the GNSS/INS solution degrades (RTK dropout).
  void maybeAddGnssPrior(g2o::VertexSE2 * vertex, const rclcpp::Time & stamp);

  ObservationUpdate addConeObservations(
    const hyu_msgs::msg::ConeArrayWithCovariance & msg,
    bool force_process);
  std::vector<ConeObservation> extractConeObservations(
    const hyu_msgs::msg::ConeArrayWithCovariance & msg) const;
  Eigen::Matrix2d covarianceFromCone(
    const hyu_msgs::msg::ConeWithCovariance & cone) const;
  Eigen::Matrix2d covarianceInMapFrame(
    const g2o::SE2 & pose,
    const Eigen::Matrix2d & local_covariance) const;

  int findAssociatedLandmark(
    const Eigen::Vector2d & map_point,
    const Eigen::Matrix2d & map_covariance,
    ConeColor color,
    bool * ambiguous,
    const std::vector<bool> * claimed = nullptr) const;
  bool colorsCompatible(ConeColor observation_color, ConeColor landmark_color) const;
  static void voteLandmarkColor(LandmarkRecord & landmark, ConeColor observed_color);
  LandmarkRecord * addLandmark(
    const Eigen::Vector2d & map_point,
    const Eigen::Matrix2d & covariance,
    ConeColor color);
  bool updateLandmarkEstimate(
    LandmarkRecord & landmark,
    const Eigen::Vector2d & map_point,
    const Eigen::Matrix2d & covariance);
  void addObservationEdge(
    const ConeObservation & observation,
    g2o::VertexSE2 * pose_vertex,
    LandmarkRecord & landmark);
  std::size_t deleteMissedVisibleLandmarks(
    const PoseRecord & pose,
    const std::vector<std::size_t> & observed_landmark_indices);
  bool landmarkExpectedVisible(
    const PoseRecord & pose,
    const LandmarkRecord & landmark) const;
  bool removeLandmarkAt(std::size_t landmark_index);
  bool shouldUpdateLandmarkDeletion(const rclcpp::Time & stamp, bool force_update);
  std::size_t mergeCloseLandmarks();
  // Merge same-color landmark pairs within merge_distance. With
  // min_last_seen_gap_m > 0 only pairs whose last observations are separated
  // by at least that much TRAVEL merge (drift-era duplicates: association
  // missed under drift, so the stale twin stopped being observed when its
  // re-mapped sibling took over), and the recently-seen member's vertex is
  // kept. Real adjacent cones are both observed continuously and never meet
  // the gap, so a radius larger than the cone spacing stays safe.
  std::size_t mergeCloseLandmarks(double merge_distance, double min_last_seen_gap_m);

  void recordRawOdometry(double stamp_sec, const g2o::SE2 & raw_odom);
  g2o::SE2 rawOdomAt(double stamp_sec) const;

  void maybeOptimize();
  void onOptimizeTimer();
  bool optimizeGraph();
  void updateKeyframeSnapshot();
  void publishLiveEstimateFromSnapshot(const rclcpp::Time & stamp, const g2o::SE2 & raw_odom);
  std::size_t firstPublishedPoseIndex() const;
  bool shouldPublishVisuals(const rclcpp::Time & stamp);

  void publishEstimate();
  void publishGraphVisuals(const rclcpp::Time & stamp);
  void publishLiveEstimate(
    const rclcpp::Time & stamp,
    const g2o::SE2 & estimate,
    const g2o::SE2 & raw_odom);
  void publishMap(const rclcpp::Time & stamp);
  void publishPath(const rclcpp::Time & stamp);
  void publishOdometry(const rclcpp::Time & stamp, const g2o::SE2 & estimate);
  void publishMarkers(const rclcpp::Time & stamp);
  void publishTransform(
    const rclcpp::Time & stamp,
    const g2o::SE2 & estimate,
    const g2o::SE2 & raw_odom);

  void handleReset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleSaveGraph(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleSaveMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool saveMapCsv(const std::string & path, std::string * error) const;
  bool loadMapCsv(const std::string & path, std::string * error);
  static ConeColor colorFromTag(const std::string & tag);
  void handleLoadMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleStartMapping(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Trackdrive lifecycle: detect lap completion, freeze the map, and switch
  // to localization with a bounded sliding window of pose vertices.
  void maybeFinishMappingLap(const g2o::SE2 & current_estimate);
  void enterLocalizationMode(const std::string & reason);
  void prunePoseWindow();
  std::string saveMapTimestamped();
  MappingStopReason classifyMappingStopState();
  void publishLifecycleDiagnostics();
  void publishStatus();
  // Per-frame perception-to-estimate latency on ~/timing, schema-compatible
  // for external latency monitoring.
  void publishTiming(double elapsed_ms, const ObservationUpdate & update);

  static double normalizeAngle(double angle);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);
  static rclcpp::Time stampOrNow(
    const builtin_interfaces::msg::Time & stamp,
    const rclcpp::Clock::SharedPtr & clock);
  static std_msgs::msg::ColorRGBA colorToRgba(ConeColor color, double alpha);
  static std::string colorName(ConeColor color);

  g2o::SparseOptimizer optimizer_;

  rclcpp::Subscription<hyu_msgs::msg::CarState>::SharedPtr car_state_sub_;
  rclcpp::Subscription<hyu_msgs::msg::ConeArrayWithCovariance>::SharedPtr cones_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gnss_odom_sub_;
  rclcpp::Publisher<hyu_msgs::msg::ConeArrayWithCovariance>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lifecycle_diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr converged_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_graph_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_map_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mapping_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr optimize_timer_;
  rclcpp::CallbackGroup::SharedPtr state_callback_group_;
  rclcpp::CallbackGroup::SharedPtr graph_callback_group_;

  std::vector<PoseRecord> poses_;
  std::vector<LandmarkRecord> landmarks_;

  // Most recent cone observations (base frame), kept so a relocalization can
  // scan-match them against the fixed map near the clicked pose.
  std::vector<ConeObservation> last_observations_;

  Eigen::Matrix3d odom_information_;

  std::string car_state_topic_;
  std::string cones_topic_;
  std::string map_topic_;
  std::string slam_odom_topic_;
  std::string status_topic_;
  std::string lifecycle_diagnostics_topic_;
  std::string map_converged_topic_;
  std::string path_topic_;
  std::string marker_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string slam_base_frame_;
  std::string g2o_output_path_;
  std::string map_save_dir_;

  // GNSS global-anchor (unary prior) configuration.
  std::string gnss_prior_topic_;
  bool gnss_prior_enable_;
  double gnss_prior_max_position_sigma_;
  double gnss_prior_max_age_;
  double gnss_prior_robust_delta_;
  double gnss_prior_min_sigma_;
  // Pre-convergence anchor de-weighting (sigma multiplier): during mapping
  // the elastic graph has no innovation gate, and full-strength anchors
  // chasing the INS's time-correlated error shift landmarks under the
  // incoming observations until associations split (duplicate cones).
  double gnss_prior_mapping_sigma_scale_{4.0};
  double gnss_prior_innovation_max_residual_;
  double gnss_prior_min_interval_;
  double last_gnss_prior_stamp_sec_;
  // Manual relocalization (/initialpose) suppresses GNSS priors: the click is
  // a competing absolute reference, and an RTK prior would otherwise yank the
  // pose straight back. Priors re-arm only once GNSS agrees with the
  // cone-anchored pose again (or never, if the map is in a different frame).
  double gnss_prior_suppress_duration_;
  double gnss_prior_rearm_max_residual_;
  bool gnss_prior_suppressed_;
  double gnss_prior_suppress_until_sec_;

  double keyframe_distance_;
  double keyframe_yaw_;
  double keyframe_max_dt_;
  double association_max_distance_;
  double association_gate_chi2_;
  double association_ambiguity_ratio_;
  // Delayed data association frontend (tentative tracks): observations found
  // tracks outside the graph; only confirmed, converged tracks become
  // landmarks. Replaces the legacy founding heuristics (creation range cap,
  // near/far split rule, crowd radius, soft founding covariance).
  FrontendParams frontend_params_{};
  std::unique_ptr<TentativeTrackFrontend> frontend_;
  // Robust kernel on cone observation edges: "dcs" (Dynamic Covariance
  // Scaling, Agarwal ICRA'13 — the closed-form equivalent of switchable
  // constraints) lets the optimizer down-weight a wrong association instead
  // of letting it distort the map; "huber" keeps the legacy kernel; "none"
  // disables. Odometry edges are NEVER robustified (they are the gradient
  // backbone; see the robust-SLAM literature).
  std::string observation_robust_kernel_{"dcs"};
  double observation_dcs_phi_{1.0};
  double relocalize_search_radius_;
  double relocalize_search_yaw_;
  double relocalize_inlier_distance_;

  // Automatic relocalization (localization mode only). Association failure
  // is an absorbing state: once the pose is outside the gate nothing pulls
  // it back, so the node must notice and recover on its own.
  bool auto_relocalize_enable_{true};
  int auto_relocalize_min_visible_cones_{3};
  int auto_relocalize_lost_frames_{10};
  int auto_relocalize_min_inliers_{4};
  double auto_relocalize_search_radius_{4.0};
  double auto_relocalize_max_search_radius_{16.0};
  double auto_relocalize_cooldown_sec_{3.0};
  // Disarm auto-relocalization while GNSS priors are being ACCEPTED: a
  // corroborated pose with zero associations is a perception hiccup, not
  // "lost". 3 s = three missed 1 Hz anchor intervals before re-arming.
  double auto_relocalize_gnss_holdoff_sec_{3.0};
  int lost_frames_{0};
  // Consecutive frames with >= 2 associations. A SINGLE match while lost is
  // routinely an aliased cone (on a 10 m kidnap some cone always falls in
  // some wrong landmark's gate), so one match must neither clear the lost
  // counter nor de-escalate the search radius.
  int healthy_streak_{0};
  double auto_relocalize_current_radius_{4.0};
  double last_auto_relocalize_attempt_sec_{-1.0e18};
  std::size_t auto_relocalize_count_{0U};

  // Orange-gate global anchor (see gate_anchor.hpp).
  bool gate_anchor_enable_{true};
  double gate_anchor_cluster_radius_m_{1.5};
  double gate_anchor_pair_tolerance_m_{0.5};
  double gate_anchor_min_pair_separation_m_{2.0};
  double gate_anchor_max_pair_separation_m_{9.0};
  int gate_anchor_min_inliers_{4};
  // Below min the normal association gate is already closing the loop; above
  // max the match is more likely a mis-association than real drift.
  double gate_anchor_min_correction_m_{2.4};
  double gate_anchor_max_correction_m_{60.0};
  double gate_anchor_cooldown_travel_m_{5.0};
  double last_gate_anchor_traveled_m_{-1.0e18};
  std::size_t gate_anchor_count_{0U};
  double association_inflation_per_meter_;
  double association_max_inflation_;
  double landmark_merge_distance_;
  double map_trust_info_scale_;
  double min_observation_range_;
  double max_observation_range_;
  double default_observation_sigma_;
  double min_observation_variance_;
  double odom_translation_sigma_;
  double odom_yaw_sigma_;
  double robust_kernel_delta_;
  double marker_scale_;
  double landmark_delete_fov_;
  double landmark_delete_max_range_;
  double landmark_delete_max_abs_x_;
  double landmark_delete_max_abs_y_;
  double landmark_delete_min_interval_;
  double landmark_update_gain_;
  double landmark_update_process_variance_;

  int optimize_every_n_keyframes_;
  int optimization_iterations_;
  int landmark_min_observations_to_publish_;
  int max_landmarks_;
  int max_optimization_poses_;
  int path_max_poses_to_publish_;
  int landmark_missed_observations_to_delete_;
  int landmark_confirm_observations_;
  double loop_gap_distance_;
  int map_trust_loop_closures_required_;

  bool localization_mode_;
  std::string load_map_path_;

  // Trackdrive lifecycle: after the mapping lap closes near the start pose
  // (and the map has converged), freeze the map and localize with a bounded
  // pose window so the graph never outgrows real-time optimization.
  bool auto_localization_after_lap_;
  double lap_return_radius_;
  double lap_return_yaw_;
  int localization_window_poses_;
  double traveled_distance_;

  // Per-edge odometry trust: the motion source (SBG bridge / wheel odometry /
  // sim drift) reports its current noise level in CarState.pose.covariance;
  // odometry edges consume it so degraded modes weaken their constraints.
  // Written and read only on the state-callback thread.
  bool use_odom_covariance_;
  double latest_odom_sigma_trans_;
  double latest_odom_sigma_yaw_;

  // Latest body twist from the motion input, passed through on
  // /graph_slam/odom for downstream controllers. Atomics: written by the
  // state thread, read wherever the estimate is published.
  std::atomic<double> latest_twist_vx_{0.0};
  std::atomic<double> latest_twist_vy_{0.0};
  std::atomic<double> latest_twist_wz_{0.0};

  // Lap origin is captured a few meters into the drive so it sits on the
  // racing line (the spawn pose can be offset from it); each lap then passes
  // within ~1 m of this pose.
  double lap_origin_capture_distance_;
  bool lap_origin_captured_;
  g2o::SE2 lap_origin_;

  bool use_cone_covariance_;
  bool process_every_cone_message_;
  bool publish_tf_;
  bool delete_stale_landmarks_;
  bool update_existing_landmarks_;
  bool map_trust_after_loop_closure_;

  double optimize_min_interval_;
  double visual_publish_min_interval_;
  double tf_stamp_offset_;
  double last_optimization_time_sec_;
  double last_visual_publish_time_sec_;
  double last_landmark_delete_time_sec_;

  int next_vertex_id_;
  int next_edge_id_;
  int keyframes_since_last_optimization_;
  int last_cone_pose_graph_id_;

  // Once loop closures have been reconciled by this many optimization cycles,
  // map_converged_ turns on and confirmed-landmark observation edges are
  // trusted more so the pose conforms to the settled map.
  bool map_converged_;
  LoopConfirmationConfig loop_confirmation_config_;
  LoopConfirmationWindow loop_confirmation_window_;
  bool loop_confirmation_ready_for_optimize_;
  int loop_closure_optimize_cycles_;
  std::size_t loop_candidate_count_;
  std::size_t loop_confirmed_count_;
  std::size_t loop_rejected_count_;
  std::size_t loop_candidate_window_count_;
  LoopConfirmationReason last_loop_confirmation_reason_;
  bool optimizer_skipped_pose_limit_;
  double last_odom_stamp_sec_;
  double last_cone_stamp_sec_;
  double last_map_update_stamp_sec_;
  double last_live_odom_publish_stamp_sec_;
  bool lifecycle_map_saved_;
  bool lap_return_criteria_satisfied_;

  // Lap-finish trust gate: map_converged_ alone accepts ANY confirmed loop
  // (on a waisted track that can be a mid-lap mini-loop); these optionally
  // require seam evidence — candidates re-associating landmarks near the lap
  // origin — plus a bounded dwell so the seam accumulates constraints before
  // the map freezes. See LapFinishGate.
  bool require_lap_seam_loop_closure_{false};
  double lap_seam_landmark_radius_m_{10.0};
  int lap_seam_candidates_required_{2};
  double lap_finish_dwell_m_{0.0};
  LapFinishGate lap_finish_gate_{};
  std::size_t seam_loop_candidate_count_{0U};
  // Once the vehicle has verifiably returned to the lap origin, that geometry
  // independently corroborates a loop: optionally relax the confirmation
  // window's candidate threshold (never its residual gates). 0 disables.
  int loop_confirmation_required_candidates_on_lap_return_{0};
  bool loop_confirmation_relaxed_on_lap_return_{false};
  // The origin pose is captured while the car is standing on it, so the
  // return check is trivially satisfied in that same update; only travel
  // beyond this floor after capture counts as a LAP return. Guards both the
  // relaxation trigger and the finish gate.
  double lap_return_min_travel_m_{50.0};
  double lap_origin_capture_traveled_m_{0.0};
  // Freeze-time drift-duplicate sweep: same-color pairs within this radius
  // whose last observations are separated by >= loop_gap_distance of travel
  // merge into the recently-seen member before the map freezes. 0 disables.
  double freeze_merge_stale_distance_m_{0.0};

  g2o::SE2 latest_estimate_;
  bool has_latest_pose_;

  // Ring buffer of cone-frame processing times for the ~/timing publisher.
  std::vector<double> frame_times_ms_;

  // Raw odometry samples (stamp seconds, pose) for observation-time
  // interpolation between keyframes. Guarded by odom_buffer_mutex_: the
  // state callback writes while the cone/optimization thread interpolates.
  std::deque<std::pair<double, g2o::SE2>> raw_odom_buffer_;
  mutable std::mutex odom_buffer_mutex_;

  // Serializes g2o graph access (poses_, landmarks_, id counters, timing
  // state) between the state thread and the cone/optimization thread.
  std::mutex graph_mutex_;

  // Last keyframe pose (graph estimate + raw odometry) so the state callback
  // can dead-reckon and publish without the graph lock while an optimization
  // is running.
  struct KeyframeSnapshot
  {
    g2o::SE2 estimate;
    g2o::SE2 raw_odom;
    bool valid{false};
  };
  KeyframeSnapshot keyframe_snapshot_;
  std::mutex snapshot_mutex_;

  // Latest GNSS absolute fix (map/ENU frame) from the bridge's /localization/gnss_odom,
  // consumed as a unary prior on new keyframes. Guarded by gnss_mutex_ so the
  // GNSS callback can write while the keyframe/optimization thread reads.
  struct GnssFix
  {
    double stamp_sec{0.0};
    Eigen::Vector2d position{Eigen::Vector2d::Zero()};
    double sigma_x{0.0};
    double sigma_y{0.0};
    bool valid{false};
  };
  GnssFix latest_gnss_fix_;
  mutable std::mutex gnss_mutex_;
};

}  // namespace hyu_localization

#endif  // HYU_LOCALIZATION__GRAPH_SLAM_NODE_HPP_
