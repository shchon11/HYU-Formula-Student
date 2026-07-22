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
#include <set>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "hyu_msgs/msg/car_state.hpp"
#include "hyu_msgs/msg/cone_array_with_covariance.hpp"
#include "hyu_msgs/msg/cone_with_covariance.hpp"
#include "hyu_localization/correlation_grid.hpp"
#include "hyu_localization/local_submap.hpp"
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

namespace g2o
{
class EdgeSE2PointXY;
}  // namespace g2o

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
    // traveled_distance_ at founding: the seam matcher restricts its targets
    // to landmarks founded at least a loop_gap of travel ago, so the submap
    // is never matched against the landmarks it just created.
    double first_seen_traveled;
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

  void configureOptimizer();
  void resetGraph();

  void stateCallback(const hyu_msgs::msg::CarState::SharedPtr msg);
  void conesCallback(const hyu_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void relocalizeTo(const g2o::SE2 & pose);
  // Re-anchor the trajectory at an already-verified pose (no scan-match).
  // raw_reference is the raw-odometry pose the estimate refers to (the
  // newest submap frame's raw pose, NOT "now": at speed the difference is
  // the perception latency times the velocity, and recording "now" bakes
  // that offset permanently into the dead-reckoned trajectory).
  void relocalizeAt(const g2o::SE2 & pose, const g2o::SE2 & raw_reference);
  // Match points for scan matching: the deduped local submap (recent cone
  // observations rigidly connected by short-horizon odometry) when it is
  // populated, else the latest single frame.
  struct MatchPointSet
  {
    std::vector<SubmapPoint> points;
    bool from_submap{false};
  };
  MatchPointSet buildMatchPoints() const;
  // Landmark targets for scan matching, pre-filtered to a box around
  // `center` (radius <= 0 disables the box). max_first_seen_traveled >= 0
  // keeps only landmarks founded no later than that travel — the seam
  // matcher's "old map only" restriction.
  std::vector<SubmapPoint> landmarkMatchTargets(
    const Eigen::Vector2d & center, double radius,
    double max_first_seen_traveled) const;
  // Coarse-to-fine grid search of the match points against the targets
  // around seed; reports the fine pass's inlier count for acceptance
  // gating (the manual /initialpose path applies the result regardless —
  // the operator asserted the neighbourhood — the automatic path must not).
  g2o::SE2 scanMatchNear(
    const g2o::SE2 & seed, double radius, double yaw_span,
    const std::vector<SubmapPoint> & points,
    const std::vector<SubmapPoint> & targets,
    int * inliers_out) const;
  g2o::SE2 gridSearchPose(
    const g2o::SE2 & seed, double radius, double xy_step,
    double yaw_span, double yaw_step, double inlier_distance,
    const std::vector<SubmapPoint> & points,
    const std::vector<SubmapPoint> & targets,
    int * best_inliers_out) const;
  g2o::SE2 latestRawOdom() const;
  // Correlative registration (correlation_grid.hpp): the ONE re-registration
  // mechanism. Tracking mode keeps the pose glued to the map inside a tight
  // window; loop mode matches the submap constellation against the OLD map
  // (pre loop-gap landmarks) over a wide window to close the lap seam.
  // Returns true when the keyframe estimate was re-seeded.
  bool maybeCsmRegister(
    PoseRecord & pose, const g2o::SE2 & keyframe_to_observation);
  // Registered-but-unobserved reaper (mapping mode): a landmark well inside
  // the conservative visibility envelope that keeps missing is a ghost.
  // Misses count once per keyframe frame; big-orange gate cones are exempt.
  std::size_t reapUnobservedLandmarks(
    const g2o::SE2 & observation_pose,
    const std::vector<std::size_t> & observed_landmark_indices);
  bool removeLandmarkAt(std::size_t landmark_index);

  g2o::SE2 poseFromCarState(const hyu_msgs::msg::CarState & msg) const;
  g2o::SE2 estimateFromRawOdometry(const g2o::SE2 & raw_odom) const;
  bool shouldCreateKeyframe(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp) const;
  void addInitialPose(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp);
  void addKeyframe(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp);

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
  // as_map_repair: admit into a FROZEN map (localization-mode repair) — the
  // landmark is created fixed, like the loaded map.
  LandmarkRecord * addLandmark(
    const Eigen::Vector2d & map_point,
    const Eigen::Matrix2d & covariance,
    ConeColor color,
    bool as_map_repair = false);

  // Register a confirmed frontend promotion: twin-suppress, create the
  // landmark, replay its observation history against the keyframes.
  // recompute_position re-derives the position from the earliest surviving
  // keyframe (used when flushing promotions deferred across a seam
  // correction — the stored map position predates the correction).
  bool registerPromotion(
    const FrontendPromotion & promotion, bool recompute_position);
  // Promotions held while the seam was pending (registration near the
  // start-area map is deferred, not dropped: dropping them permanently
  // ate cones when the map froze on the first lap return).
  void flushDeferredPromotions(const char * reason);

  bool updateLandmarkEstimate(
    LandmarkRecord & landmark,
    const Eigen::Vector2d & map_point,
    const Eigen::Matrix2d & covariance);
  // loop_edge marks a stale-loop-candidate re-association (the landmark was
  // last seen >= loop_gap of travel ago). Such edges get HUBER even when the
  // config says DCS: DCS is redescending, so discarding a true loop edge
  // costs only ~2*phi — under meters of drift that is always cheaper than
  // bending the odometry chain, and the optimizer would rationally switch
  // the loop OFF and keep the drift (observed: a verified 9 m seam re-seed
  // reverted in one optimize call). The one edge class whose job is to force
  // the chain to bend must stay convex.
  void addObservationEdge(
    const ConeObservation & observation,
    g2o::VertexSE2 * pose_vertex,
    LandmarkRecord & landmark,
    bool loop_edge = false);
  // Attach the configured robust kernel (huber | none) to a cone
  // observation edge — one place, so merge-reanchored edges cannot drift out
  // of sync with newly created ones.
  void attachObservationKernel(g2o::EdgeSE2PointXY * edge, bool loop_edge) const;

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
  // Latch the lap-return evidence (and the optional loop-confirmation
  // relaxation). Called ONLY from the geometric return check (live-vertex
  // origin window): an anchor is a scan match, and a mis-match near the
  // startline must never certify "lap complete" and freeze a half-built
  // map.
  void markLapReturnObserved();
  // Lap origin in CURRENT map coordinates (live vertex estimate when the
  // origin keyframe still exists, else the capture-time snapshot).
  g2o::SE2 lapOrigin() const;
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
  std::vector<FrontendPromotion> deferred_promotions_;
  // Robust kernel on cone observation edges: "huber" tolerates a wrong
  // association without letting it distort the map; "none" disables.
  // Odometry edges are NEVER robustified (they are the gradient backbone).
  std::string observation_robust_kernel_{"huber"};
  double relocalize_search_radius_;
  double relocalize_search_yaw_;
  double relocalize_inlier_distance_;

  // Correlative scan matching (see maybeCsmRegister).
  bool csm_enable_{true};
  CorrelationGrid::Params csm_params_{};
  double csm_track_window_m_{2.0};
  double csm_track_window_theta_{0.17};
  double csm_loop_window_m_{6.0};
  double csm_loop_window_theta_{0.35};
  // Synthetic calibration: a unique match reports sigma 0.2-0.8 m (the
  // moment estimate integrates the smear width), a one-spacing alias on a
  // uniform straight reports ~4 m. The gap is wide; sit in the middle.
  double csm_max_sigma_xy_{1.2};
  double csm_max_sigma_theta_{0.05};
  // Unimodality: the best hypothesis must beat every out-of-basin hypothesis
  // by this response margin (synthetic: orange-gated seam wins by ~0.29, a
  // uniform straight's alias ties at ~0.00).
  double csm_peak_margin_{0.10};
  double csm_loop_min_response_{0.60};
  double csm_apply_cooldown_m_{2.0};
  bool landmark_delete_enable_{true};
  int landmark_delete_misses_{20};
  double landmark_delete_max_range_{10.0};
  double landmark_delete_fov_{1.5};
  // Frozen-map repair: the map stays writable in localization mode, but
  // under far stricter admission than mapping — the frozen map is the
  // trusted reference, so changing it needs overwhelming evidence.
  // 0 disables the respective path.
  int loc_map_repair_min_hits_{8};
  int loc_map_repair_delete_misses_{40};
  // The orange gate's own response at the matched pose (loop mode): the
  // seam certificate. Corridors may alias; the gate may not.
  // 0.8, not 0.5: a pair-interleaved gate fit (observed pair overlaid on
  // the WRONG map pair, the "| | |" seam) matches 2 of 4 gate cones and
  // scores ~0.5 -- only the full-rectangle overlay clears 0.8.
  double csm_loop_min_orange_response_{0.8};
  // Each corridor rail (blue, yellow) must fit at the refined seam pose:
  // a twisted seam's correlation optimum is winner-take-one-rail and the
  // splayed rail becomes a third row of ghosts.
  double csm_loop_min_rail_response_{0.35};
  double submap_dedup_orange_m_{0.25};
  // Observation-sigma bound for a cone to carry GATE credentials in the
  // submap: lidar-backed orange passes, vision-only ZNCC-depth orange
  // (meters of range variance) is demoted to unknown geometry.
  double csm_orange_max_obs_sigma_{0.45};
  double csm_min_interval_m_{1.0};
  double last_csm_travel_{-1.0e18};
  std::size_t csm_track_applied_{0U};
  std::size_t csm_loop_applied_{0U};


  // Local submap (see local_submap.hpp): the rolling constellation of recent
  // cone observations every scan match fits instead of a single frame.
  bool submap_enable_{true};
  double submap_span_m_{20.0};
  int submap_max_frames_{250};
  double submap_dedup_radius_m_{0.6};
  int submap_min_match_points_{6};
  LocalConeSubmap submap_{};
  // Raw-odometry pose of the newest submap frame: the body frame the match
  // points are expressed in, and the reference relocalizeAt() must re-anchor
  // at. Guarded by graph_mutex_ (cone thread only).
  g2o::SE2 submap_reference_;
  bool submap_reference_valid_{false};
  // Set by the state thread when the impossible-motion gate fires: the raw
  // odometry stream has a discontinuity, so the submap's frame-to-frame
  // rigidity is broken and its history must be dropped before the next
  // match. Consumed (cleared) on the cone thread.

  // Post-resume mapping quarantine. A blind odometry gap (INS free-inertial
  // fault -> bridge stops publishing -> stream resumes with the
  // re-acquisition pull-in transient still decaying) must not seed new
  // landmarks: mapping the pull-in error mints an offset copy of everything
  // in view (F2 fault-injection autopsy: 82 false cones from one 20 s
  // outage). While quarantined, association / loop candidates all keep
  // running — only track founding and promotion pause, so the map stays
  // clean through the transient.

  double association_inflation_per_meter_;
  double association_max_inflation_;
  double min_observation_range_;
  double max_observation_range_;
  // Sensor visibility model for the frontend's track-miss accounting: a
  // track only accrues misses while it should be observable from the
  // current pose (FOV/range exits must not eat the miss budget).
  double track_visible_max_range_{30.0};
  double track_visible_fov_{std::acos(-1.0)};
  double default_observation_sigma_;
  double min_observation_variance_;
  double odom_translation_sigma_;
  double odom_yaw_sigma_;
  double robust_kernel_delta_;
  double marker_scale_;
  double landmark_update_gain_;
  double landmark_update_process_variance_;

  int optimize_every_n_keyframes_;
  int optimization_iterations_;
  int landmark_min_observations_to_publish_;
  int max_landmarks_;
  int max_optimization_poses_;
  int path_max_poses_to_publish_;
  int landmark_confirm_observations_;
  double loop_gap_distance_;

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
  // Distance-proportional floor on the keyframe edge sigma: reported
  // covariance is per-sample, not per-edge (see addKeyframe). 0 disables.
  double odom_edge_sigma_per_meter_{0.0};
  double odom_edge_yaw_sigma_per_meter_{0.0};

  // Latest body twist from the motion input, passed through on
  // /graph_slam/odom for downstream controllers. Atomics: written by the
  // state thread, read wherever the estimate is published.
  std::atomic<double> latest_twist_vx_{0.0};
  std::atomic<double> latest_twist_vy_{0.0};
  std::atomic<double> latest_twist_wz_{0.0};

  // Output pose-jump gate. On a symmetric layout (skidpad's two circles) the
  // graph optimiser can momentarily converge to the mirror solution, flipping
  // the published pose ~180 deg / several metres in a single ~4 ms frame --
  // physically impossible, and it whipsaws the downstream local planner. Veto a
  // frame-to-frame step past physical limits and dead-reckon the last good pose
  // by the motion twist instead; a genuine step (real motion, converged
  // relocalisation across keyframes) stays under the limit and passes.

  // Freeze-time map ADMISSION check: freezing certifies the map as the
  // fixed reference for every remaining lap, so a polluted map must not
  // freeze just because the lap geometry closed. Measurable without ground
  // truth: (1) residual same-color duplicate pairs, (2) HOLES in the
  // blue/yellow chains along the driven lap (a stretch of track with no
  // cone on one side = a missing tooth that would break the planner's
  // corridor). While the check fails the car keeps mapping — an extra lap
  // fills holes and merges twins — up to a bounded number of extra returns.
  // Last admission result, published in the lifecycle diagnostics so a
  // delayed freeze is attributable from the topic alone.

  // Lap origin is captured a few meters into the drive so it sits on the
  // racing line (the spawn pose can be offset from it); each lap then passes
  // within ~1 m of this pose.
  double lap_origin_capture_distance_;
  bool lap_origin_captured_;
  g2o::SE2 lap_origin_;
  // The keyframe vertex captured as the lap origin. The snapshot above goes
  // stale the moment a loop closure bends the graph (a seam correction moves
  // the origin's map coordinates by the whole drift), so every origin
  // comparison must read the vertex's CURRENT estimate via lapOrigin().
  g2o::VertexSE2 * lap_origin_vertex_{nullptr};

  bool use_cone_covariance_;
  bool process_every_cone_message_;
  bool publish_tf_;
  bool update_existing_landmarks_;

  double optimize_min_interval_;
  double visual_publish_min_interval_;
  double tf_stamp_offset_;
  double last_optimization_time_sec_;
  double last_visual_publish_time_sec_;

  int next_vertex_id_;
  int next_edge_id_;
  int keyframes_since_last_optimization_;
  int last_cone_pose_graph_id_;

  // Once loop closures have been reconciled by this many optimization cycles,
  // map_converged_ turns on and confirmed-landmark observation edges are
  // trusted more so the pose conforms to the settled map.
  bool map_converged_;
  bool loop_confirmation_ready_for_optimize_;
  std::size_t loop_candidate_count_;
  std::size_t loop_confirmed_count_;
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
  // Once the vehicle has verifiably returned to the lap origin, that geometry
  // independently corroborates a loop: optionally relax the confirmation
  // window's candidate threshold (never its residual gates). 0 disables.
  // The origin pose is captured while the car is standing on it, so the
  // return check is trivially satisfied in that same update; only travel
  // beyond this floor after capture counts as a LAP return. Guards both the
  // relaxation trigger and the finish gate.
  double lap_return_min_travel_m_{50.0};
  // Freeze on the Nth certified lap return: lap 2 lays revisit loop edges
  // along the whole track so first-lap drift is ironed out before landmarks
  // fix (1 = freeze immediately, fastest global path, warp risk on long
  // tracks).
  int lap_returns_to_freeze_{2};
  int lap_returns_seen_{0};
  bool lap_return_window_active_{false};
  double lap_origin_capture_traveled_m_{0.0};
  // Freeze-time drift-duplicate sweep: same-color pairs within this radius
  // whose last observations are separated by >= loop_gap_distance of travel
  // merge into the recently-seen member before the map freezes. 0 disables.

  g2o::SE2 latest_estimate_;
  bool has_latest_pose_;

  // Translation the last optimizeGraph() call applied to the live pose.
  // While this is large the graph is mid-transient (a loop closure is still
  // being absorbed) and direct landmark writes must pause: the Kalman fusion
  // in updateLandmarkEstimate would bake the half-settled pose into landmark
  // positions the optimizer is about to move again.
  double last_optimize_correction_m_{0.0};

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
};

}  // namespace hyu_localization

#endif  // HYU_LOCALIZATION__GRAPH_SLAM_NODE_HPP_
