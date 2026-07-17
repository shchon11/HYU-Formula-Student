// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SIZE_OK: This integration only preserves topic/config compatibility; splitting
// the legacy monolithic graph SLAM node is outside this focused scope.

#include "hyu_localization/graph_slam_node.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>
#include <g2o/types/slam2d/edge_se2_xyprior.h>
#include <g2o/types/slam2d/types_slam2d.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace hyu_localization
{

GraphSlamNode::GraphSlamNode()
: Node("graph_slam_node"),
  keyframe_distance_(1.0),
  keyframe_yaw_(0.25),
  keyframe_max_dt_(1.0),
  association_max_distance_(1.2),
  association_gate_chi2_(5.991),
  association_ambiguity_ratio_(0.85),
  relocalize_search_radius_(2.0),
  relocalize_search_yaw_(0.5),
  relocalize_inlier_distance_(0.6),
  association_inflation_per_meter_(0.01),
  association_max_inflation_(4.0),
  landmark_merge_distance_(0.85),
  map_trust_info_scale_(3.0),
  min_observation_range_(0.2),
  max_observation_range_(30.0),
  default_observation_sigma_(0.25),
  min_observation_variance_(0.01),
  odom_translation_sigma_(0.05),
  odom_yaw_sigma_(0.03),
  robust_kernel_delta_(1.0),
  marker_scale_(0.3),
  landmark_delete_fov_(std::acos(-1.0)),
  landmark_delete_max_range_(30.0),
  landmark_delete_max_abs_x_(20.0),
  landmark_delete_max_abs_y_(20.0),
  landmark_delete_min_interval_(0.2),
  landmark_update_gain_(1.0),
  landmark_update_process_variance_(0.04),
  optimize_every_n_keyframes_(100),
  optimization_iterations_(3),
  landmark_min_observations_to_publish_(1),
  max_landmarks_(400),
  max_optimization_poses_(100),
  path_max_poses_to_publish_(1000),
  landmark_missed_observations_to_delete_(6),
  landmark_confirm_observations_(3),
  loop_gap_distance_(20.0),
  map_trust_loop_closures_required_(2),
  localization_mode_(false),
  auto_localization_after_lap_(true),
  lap_return_radius_(6.0),
  lap_return_yaw_(1.0),
  localization_window_poses_(100),
  traveled_distance_(0.0),
  use_odom_covariance_(true),
  latest_odom_sigma_trans_(0.0),
  latest_odom_sigma_yaw_(0.0),
  lap_origin_capture_distance_(15.0),
  lap_origin_captured_(false),
  use_cone_covariance_(true),
  process_every_cone_message_(false),
  publish_tf_(true),
  delete_stale_landmarks_(true),
  update_existing_landmarks_(true),
  map_trust_after_loop_closure_(true),
  optimize_min_interval_(10.0),
  visual_publish_min_interval_(0.5),
  tf_stamp_offset_(0.0),
  last_optimization_time_sec_(-1.0),
  last_visual_publish_time_sec_(-1.0),
  last_landmark_delete_time_sec_(-1.0),
  next_vertex_id_(0),
  next_edge_id_(0),
  keyframes_since_last_optimization_(0),
  last_cone_pose_graph_id_(-1),
  map_converged_(false),
  loop_confirmation_window_(loop_confirmation_config_),
  loop_confirmation_ready_for_optimize_(false),
  loop_closure_optimize_cycles_(0),
  loop_candidate_count_(0U),
  loop_confirmed_count_(0U),
  loop_rejected_count_(0U),
  loop_candidate_window_count_(0U),
  last_loop_confirmation_reason_(LoopConfirmationReason::PendingThreshold),
  optimizer_skipped_pose_limit_(false),
  last_odom_stamp_sec_(-1.0),
  last_cone_stamp_sec_(-1.0),
  last_map_update_stamp_sec_(-1.0),
  last_live_odom_publish_stamp_sec_(-1.0),
  lifecycle_map_saved_(false),
  lap_return_criteria_satisfied_(false)
{
  car_state_topic_ =
    declare_parameter<std::string>("car_state_topic", "/localization/wheel_odom");
  cones_topic_ = declare_parameter<std::string>("cones_topic", "/perception/cones");
  map_topic_ = declare_parameter<std::string>("map_topic", "/localization/cone_map");
  slam_odom_topic_ =
    declare_parameter<std::string>("slam_odom_topic", "/localization/ego_odom");
  status_topic_ = declare_parameter<std::string>("status_topic", "~/status");
  lifecycle_diagnostics_topic_ =
    declare_parameter<std::string>(
    "lifecycle_diagnostics_topic",
    "/localization/debug/lifecycle_diagnostics");
  map_converged_topic_ =
    declare_parameter<std::string>("map_converged_topic", "~/map_converged");
  path_topic_ = declare_parameter<std::string>("path_topic", "/localization/debug/path");
  marker_topic_ = declare_parameter<std::string>("marker_topic", "/localization/debug/markers");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  slam_base_frame_ = declare_parameter<std::string>("slam_base_frame", "base_footprint");
  g2o_output_path_ = declare_parameter<std::string>("g2o_output_path", "/tmp/hyu_localization.g2o");
  map_save_dir_ = declare_parameter<std::string>("map_save_dir", "/tmp");

  keyframe_distance_ = declare_parameter<double>("keyframe_distance", keyframe_distance_);
  keyframe_yaw_ = declare_parameter<double>("keyframe_yaw", keyframe_yaw_);
  keyframe_max_dt_ = declare_parameter<double>("keyframe_max_dt", keyframe_max_dt_);
  association_max_distance_ =
    declare_parameter<double>("association_max_distance", association_max_distance_);
  association_gate_chi2_ =
    declare_parameter<double>("association_gate_chi2", association_gate_chi2_);
  association_ambiguity_ratio_ =
    declare_parameter<double>("association_ambiguity_ratio", association_ambiguity_ratio_);
  // Tentative-track frontend (delayed data association).
  frontend_params_.heading_lever_sigma_rad = declare_parameter<double>(
    "heading_lever_sigma_rad", frontend_params_.heading_lever_sigma_rad);
  frontend_params_.track_gate_chi2 = declare_parameter<double>(
    "track_gate_chi2", frontend_params_.track_gate_chi2);
  frontend_params_.track_gate_max_distance_m = declare_parameter<double>(
    "track_gate_max_distance", frontend_params_.track_gate_max_distance_m);
  frontend_params_.promote_min_hits = declare_parameter<int>(
    "track_promote_min_hits", frontend_params_.promote_min_hits);
  frontend_params_.promote_max_position_sigma_m = declare_parameter<double>(
    "track_promote_max_sigma", frontend_params_.promote_max_position_sigma_m);
  frontend_params_.promote_hold_radius_m = declare_parameter<double>(
    "track_promote_hold_radius", frontend_params_.promote_hold_radius_m);
  frontend_params_.promote_hold_stale_travel_m = declare_parameter<double>(
    "track_promote_hold_stale_travel", frontend_params_.promote_hold_stale_travel_m);
  frontend_params_.kill_consecutive_misses = declare_parameter<int>(
    "track_kill_consecutive_misses", frontend_params_.kill_consecutive_misses);
  frontend_params_.kill_unpromoted_travel_m = declare_parameter<double>(
    "track_kill_unpromoted_travel", frontend_params_.kill_unpromoted_travel_m);
  // Robust kernel on observation edges (dcs | huber | none).
  observation_robust_kernel_ = declare_parameter<std::string>(
    "observation_robust_kernel", observation_robust_kernel_);
  observation_dcs_phi_ = declare_parameter<double>(
    "observation_dcs_phi", observation_dcs_phi_);
  relocalize_search_radius_ =
    declare_parameter<double>("relocalize_search_radius", relocalize_search_radius_);
  relocalize_search_yaw_ =
    declare_parameter<double>("relocalize_search_yaw", relocalize_search_yaw_);
  relocalize_inlier_distance_ =
    declare_parameter<double>("relocalize_inlier_distance", relocalize_inlier_distance_);

  declareRecoveryParameters();

  association_inflation_per_meter_ =
    declare_parameter<double>(
    "association_inflation_per_meter",
    association_inflation_per_meter_);
  association_max_inflation_ =
    declare_parameter<double>("association_max_inflation", association_max_inflation_);
  landmark_merge_distance_ =
    declare_parameter<double>("landmark_merge_distance", landmark_merge_distance_);
  min_observation_range_ =
    declare_parameter<double>("min_observation_range", min_observation_range_);
  max_observation_range_ =
    declare_parameter<double>("max_observation_range", max_observation_range_);
  default_observation_sigma_ =
    declare_parameter<double>("default_observation_sigma", default_observation_sigma_);
  min_observation_variance_ =
    declare_parameter<double>("min_observation_variance", min_observation_variance_);
  odom_translation_sigma_ =
    declare_parameter<double>("odom_translation_sigma", odom_translation_sigma_);
  odom_yaw_sigma_ = declare_parameter<double>("odom_yaw_sigma", odom_yaw_sigma_);
  robust_kernel_delta_ = declare_parameter<double>("robust_kernel_delta", robust_kernel_delta_);
  marker_scale_ = declare_parameter<double>("marker_scale", marker_scale_);
  landmark_delete_fov_ = declare_parameter<double>("landmark_delete_fov", landmark_delete_fov_);
  landmark_delete_max_range_ =
    declare_parameter<double>("landmark_delete_max_range", landmark_delete_max_range_);
  landmark_delete_max_abs_x_ =
    declare_parameter<double>("landmark_delete_max_abs_x", landmark_delete_max_abs_x_);
  landmark_delete_max_abs_y_ =
    declare_parameter<double>("landmark_delete_max_abs_y", landmark_delete_max_abs_y_);
  landmark_delete_min_interval_ =
    declare_parameter<double>("landmark_delete_min_interval", landmark_delete_min_interval_);
  landmark_update_gain_ =
    declare_parameter<double>("landmark_update_gain", landmark_update_gain_);
  landmark_update_process_variance_ =
    declare_parameter<double>(
    "landmark_update_process_variance",
    landmark_update_process_variance_);

  optimize_every_n_keyframes_ =
    declare_parameter<int>("optimize_every_n_keyframes", optimize_every_n_keyframes_);
  optimization_iterations_ =
    declare_parameter<int>("optimization_iterations", optimization_iterations_);
  landmark_min_observations_to_publish_ =
    declare_parameter<int>(
    "landmark_min_observations_to_publish",
    landmark_min_observations_to_publish_);
  max_landmarks_ = declare_parameter<int>("max_landmarks", max_landmarks_);
  max_optimization_poses_ =
    declare_parameter<int>("max_optimization_poses", max_optimization_poses_);
  path_max_poses_to_publish_ =
    declare_parameter<int>("path_max_poses_to_publish", path_max_poses_to_publish_);
  landmark_missed_observations_to_delete_ =
    declare_parameter<int>(
    "landmark_missed_observations_to_delete",
    landmark_missed_observations_to_delete_);
  landmark_confirm_observations_ =
    declare_parameter<int>(
    "landmark_confirm_observations",
    landmark_confirm_observations_);
  loop_gap_distance_ = declare_parameter<double>("loop_gap_distance", loop_gap_distance_);
  map_trust_loop_closures_required_ =
    declare_parameter<int>(
    "map_trust_loop_closures_required",
    map_trust_loop_closures_required_);
  map_trust_info_scale_ =
    declare_parameter<double>("map_trust_info_scale", map_trust_info_scale_);
  const int loop_confirmation_required_candidates =
    declare_parameter<int>(
    "loop_confirmation_required_candidates",
    static_cast<int>(loop_confirmation_config_.required_candidates));
  loop_confirmation_config_.required_candidates =
    static_cast<std::size_t>(std::max<int>(1, loop_confirmation_required_candidates));
  loop_confirmation_config_.min_travel_m =
    declare_parameter<double>(
    "loop_confirmation_min_travel_m",
    loop_confirmation_config_.min_travel_m);
  loop_confirmation_config_.min_elapsed_sec =
    declare_parameter<double>(
    "loop_confirmation_min_elapsed_sec",
    loop_confirmation_config_.min_elapsed_sec);
  loop_confirmation_config_.median_residual_max_m =
    declare_parameter<double>(
    "loop_confirmation_median_residual_max_m",
    loop_confirmation_config_.median_residual_max_m);
  loop_confirmation_config_.max_residual_m =
    declare_parameter<double>(
    "loop_confirmation_max_residual_m",
    loop_confirmation_config_.max_residual_m);

  optimize_min_interval_ =
    declare_parameter<double>("optimize_min_interval", optimize_min_interval_);
  visual_publish_min_interval_ =
    declare_parameter<double>("visual_publish_min_interval", visual_publish_min_interval_);
  tf_stamp_offset_ = declare_parameter<double>("tf_stamp_offset", tf_stamp_offset_);

  localization_mode_ = declare_parameter<bool>("localization_mode", localization_mode_);
  auto_localization_after_lap_ =
    declare_parameter<bool>("auto_localization_after_lap", auto_localization_after_lap_);
  require_lap_seam_loop_closure_ = declare_parameter<bool>(
    "require_lap_seam_loop_closure", require_lap_seam_loop_closure_);
  lap_seam_landmark_radius_m_ = declare_parameter<double>(
    "lap_seam_landmark_radius_m", lap_seam_landmark_radius_m_);
  lap_seam_candidates_required_ = declare_parameter<int>(
    "lap_seam_candidates_required", lap_seam_candidates_required_);
  lap_finish_dwell_m_ = declare_parameter<double>("lap_finish_dwell_m", lap_finish_dwell_m_);
  lap_return_min_travel_m_ = declare_parameter<double>(
    "lap_return_min_travel_m", lap_return_min_travel_m_);
  freeze_merge_stale_distance_m_ = declare_parameter<double>(
    "freeze_merge_stale_distance_m", freeze_merge_stale_distance_m_);
  loop_confirmation_required_candidates_on_lap_return_ = declare_parameter<int>(
    "loop_confirmation_required_candidates_on_lap_return",
    loop_confirmation_required_candidates_on_lap_return_);
  lap_return_radius_ = declare_parameter<double>("lap_return_radius", lap_return_radius_);
  lap_return_yaw_ = declare_parameter<double>("lap_return_yaw", lap_return_yaw_);
  localization_window_poses_ =
    declare_parameter<int>("localization_window_poses", localization_window_poses_);
  lap_origin_capture_distance_ =
    declare_parameter<double>(
    "lap_origin_capture_distance",
    lap_origin_capture_distance_);
  use_odom_covariance_ =
    declare_parameter<bool>("use_odom_covariance", use_odom_covariance_);
  load_map_path_ = declare_parameter<std::string>("load_map_path", "");
  use_cone_covariance_ = declare_parameter<bool>("use_cone_covariance", use_cone_covariance_);
  process_every_cone_message_ =
    declare_parameter<bool>("process_every_cone_message", process_every_cone_message_);
  publish_tf_ = declare_parameter<bool>("publish_tf", publish_tf_);
  delete_stale_landmarks_ =
    declare_parameter<bool>("delete_stale_landmarks", delete_stale_landmarks_);
  update_existing_landmarks_ =
    declare_parameter<bool>("update_existing_landmarks", update_existing_landmarks_);
  map_trust_after_loop_closure_ =
    declare_parameter<bool>("map_trust_after_loop_closure", map_trust_after_loop_closure_);

  // GNSS global anchor: add a unary EdgeSE2XYPrior on each keyframe from the
  // SBG bridge's /localization/gnss_odom absolute fix. gnss_prior_max_position_sigma gates
  // it so only trustworthy (mode-4 / RTK) fixes anchor the graph; degraded
  // fixes arrive with a huge covariance and are dropped automatically.
  gnss_prior_enable_ = declare_parameter<bool>("gnss_prior_enable", true);
  gnss_prior_topic_ = declare_parameter<std::string>("gnss_prior_topic", "/localization/gnss_odom");
  gnss_prior_max_position_sigma_ =
    declare_parameter<double>("gnss_prior_max_position_sigma", 0.5);
  gnss_prior_max_age_ = declare_parameter<double>("gnss_prior_max_age", 0.3);
  gnss_prior_robust_delta_ =
    declare_parameter<double>("gnss_prior_robust_delta", 1.0);
  gnss_prior_suppress_duration_ =
    declare_parameter<double>("gnss_prior_suppress_duration", 20.0);
  gnss_prior_rearm_max_residual_ =
    declare_parameter<double>("gnss_prior_rearm_max_residual", 2.0);
  gnss_prior_min_sigma_ = declare_parameter<double>("gnss_prior_min_sigma", 0.25);
  gnss_prior_mapping_sigma_scale_ =
    declare_parameter<double>("gnss_prior_mapping_sigma_scale", 4.0);
  gnss_prior_innovation_max_residual_ =
    declare_parameter<double>("gnss_prior_innovation_max_residual", 2.0);
  gnss_prior_min_interval_ =
    declare_parameter<double>("gnss_prior_min_interval", 1.0);
  gnss_prior_max_position_sigma_ = std::max(1e-3, gnss_prior_max_position_sigma_);
  gnss_prior_min_sigma_ = std::max(0.0, gnss_prior_min_sigma_);
  gnss_prior_suppressed_ = false;
  gnss_prior_suppress_until_sec_ = 0.0;
  last_gnss_prior_stamp_sec_ = -1.0e18;

  keyframe_distance_ = std::max(0.0, keyframe_distance_);
  keyframe_yaw_ = std::max(0.0, keyframe_yaw_);
  association_max_distance_ = std::max(0.05, association_max_distance_);
  association_gate_chi2_ = std::max(0.1, association_gate_chi2_);
  association_ambiguity_ratio_ = std::clamp(association_ambiguity_ratio_, 0.0, 1.0);
  relocalize_search_radius_ = std::max(0.0, relocalize_search_radius_);
  relocalize_search_yaw_ = std::max(0.0, relocalize_search_yaw_);
  relocalize_inlier_distance_ = std::max(0.05, relocalize_inlier_distance_);
  association_inflation_per_meter_ = std::max(0.0, association_inflation_per_meter_);
  association_max_inflation_ = std::max(0.0, association_max_inflation_);
  frontend_params_.association_max_distance_m = association_max_distance_;
  frontend_params_.association_gate_chi2 = association_gate_chi2_;
  frontend_params_.association_ambiguity_ratio = association_ambiguity_ratio_;
  frontend_params_.association_inflation_per_meter = association_inflation_per_meter_;
  frontend_params_.association_max_inflation = association_max_inflation_;
  frontend_ = std::make_unique<TentativeTrackFrontend>(frontend_params_);
  landmark_merge_distance_ = std::max(0.0, landmark_merge_distance_);
  landmark_confirm_observations_ = std::max(0, landmark_confirm_observations_);
  loop_gap_distance_ = std::max(0.0, loop_gap_distance_);
  loop_confirmation_config_.min_travel_m =
    std::max(0.0, loop_confirmation_config_.min_travel_m);
  loop_confirmation_config_.min_elapsed_sec =
    std::max(0.0, loop_confirmation_config_.min_elapsed_sec);
  loop_confirmation_config_.median_residual_max_m =
    std::max(0.0, loop_confirmation_config_.median_residual_max_m);
  loop_confirmation_config_.max_residual_m =
    std::max(
    loop_confirmation_config_.median_residual_max_m,
    loop_confirmation_config_.max_residual_m);
  loop_confirmation_window_ = LoopConfirmationWindow(loop_confirmation_config_);
  lap_seam_landmark_radius_m_ = std::max(1.0, lap_seam_landmark_radius_m_);
  lap_seam_candidates_required_ = std::max(1, lap_seam_candidates_required_);
  lap_finish_dwell_m_ = std::max(0.0, lap_finish_dwell_m_);
  lap_return_min_travel_m_ = std::max(0.0, lap_return_min_travel_m_);
  freeze_merge_stale_distance_m_ = std::max(0.0, freeze_merge_stale_distance_m_);
  loop_confirmation_required_candidates_on_lap_return_ =
    std::max(0, loop_confirmation_required_candidates_on_lap_return_);
  lap_finish_gate_ = LapFinishGate(
    LapFinishGateConfig{
      require_lap_seam_loop_closure_,
      static_cast<std::size_t>(lap_seam_candidates_required_),
      lap_finish_dwell_m_});
  lap_origin_capture_distance_ = std::max(1.0, lap_origin_capture_distance_);
  lap_return_radius_ = std::max(0.5, lap_return_radius_);
  lap_return_yaw_ = std::max(0.05, lap_return_yaw_);
  localization_window_poses_ = std::max(10, localization_window_poses_);
  map_trust_loop_closures_required_ = std::max(1, map_trust_loop_closures_required_);
  map_trust_info_scale_ = std::max(1.0, map_trust_info_scale_);
  min_observation_range_ = std::max(0.0, min_observation_range_);
  max_observation_range_ = std::max(min_observation_range_, max_observation_range_);
  default_observation_sigma_ = std::max(1e-3, default_observation_sigma_);
  min_observation_variance_ = std::max(1e-6, min_observation_variance_);
  odom_translation_sigma_ = std::max(1e-4, odom_translation_sigma_);
  odom_yaw_sigma_ = std::max(1e-4, odom_yaw_sigma_);
  optimize_every_n_keyframes_ = std::max(1, optimize_every_n_keyframes_);
  optimization_iterations_ = std::max(1, optimization_iterations_);
  landmark_min_observations_to_publish_ = std::max(1, landmark_min_observations_to_publish_);
  landmark_missed_observations_to_delete_ =
    std::max(1, landmark_missed_observations_to_delete_);
  landmark_delete_fov_ =
    std::clamp(landmark_delete_fov_, 0.0, 2.0 * std::acos(-1.0));
  landmark_delete_max_range_ =
    landmark_delete_max_range_ <= 0.0 ?
    max_observation_range_ :
    std::max(min_observation_range_, landmark_delete_max_range_);
  landmark_delete_max_abs_x_ = std::max(0.0, landmark_delete_max_abs_x_);
  landmark_delete_max_abs_y_ = std::max(0.0, landmark_delete_max_abs_y_);
  landmark_delete_min_interval_ = std::max(0.0, landmark_delete_min_interval_);
  landmark_update_gain_ = std::clamp(landmark_update_gain_, 0.0, 1.0);
  landmark_update_process_variance_ = std::max(0.0, landmark_update_process_variance_);
  optimize_min_interval_ = std::max(0.0, optimize_min_interval_);
  visual_publish_min_interval_ = std::max(0.0, visual_publish_min_interval_);
  tf_stamp_offset_ = std::max(0.0, tf_stamp_offset_);
  if (tf_stamp_offset_ > 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring tf_stamp_offset %.3f. Future-dated base TF can split RobotModel links; "
      "graph SLAM now publishes map->%s and %s->%s at the measurement stamp.",
      tf_stamp_offset_,
      odom_frame_.c_str(),
      odom_frame_.c_str(),
      slam_base_frame_.c_str());
    tf_stamp_offset_ = 0.0;
  }

  odom_information_.setZero();
  odom_information_(0, 0) = 1.0 / (odom_translation_sigma_ * odom_translation_sigma_);
  odom_information_(1, 1) = 1.0 / (odom_translation_sigma_ * odom_translation_sigma_);
  odom_information_(2, 2) = 1.0 / (odom_yaw_sigma_ * odom_yaw_sigma_);

  configureOptimizer();

  if (localization_mode_) {
    if (load_map_path_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "localization_mode is enabled but load_map_path is empty; "
        "no map to localize against");
    } else {
      std::string error;
      if (loadMapCsv(load_map_path_, &error)) {
        RCLCPP_INFO(
          get_logger(),
          "Localization mode: loaded %zu fixed landmarks from %s",
          landmarks_.size(),
          load_map_path_.c_str());
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to load map '%s' for localization: %s",
          load_map_path_.c_str(),
          error.c_str());
      }
    }
  }

  const auto transient_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  map_pub_ = create_publisher<hyu_msgs::msg::ConeArrayWithCovariance>(map_topic_, transient_qos);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(slam_odom_topic_, 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, transient_qos);
  marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, transient_qos);
  // Latched status so planning (local vs global) and RViz always see the
  // current SLAM lifecycle even when subscribing late.
  status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, transient_qos);
  timing_pub_ = create_publisher<std_msgs::msg::String>("/localization/debug/timing", 1);
  lifecycle_diagnostics_pub_ =
    create_publisher<std_msgs::msg::String>(lifecycle_diagnostics_topic_, transient_qos);
  converged_pub_ = create_publisher<std_msgs::msg::Bool>(map_converged_topic_, transient_qos);

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    RCLCPP_INFO(
      get_logger(),
      "Publishing graph SLAM TF '%s' -> '%s' -> '%s'; "
      "keep simulator publish_gt_tf disabled",
      map_frame_.c_str(),
      odom_frame_.c_str(),
      slam_base_frame_.c_str());
  }

  reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset",
    std::bind(&GraphSlamNode::handleReset, this, std::placeholders::_1, std::placeholders::_2));
  save_graph_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/save_graph",
    std::bind(&GraphSlamNode::handleSaveGraph, this, std::placeholders::_1, std::placeholders::_2));
  save_map_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/save_map",
    std::bind(&GraphSlamNode::handleSaveMap, this, std::placeholders::_1, std::placeholders::_2));
  load_map_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/load_map",
    std::bind(&GraphSlamNode::handleLoadMap, this, std::placeholders::_1, std::placeholders::_2));
  start_mapping_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/start_mapping",
    std::bind(
      &GraphSlamNode::handleStartMapping, this, std::placeholders::_1,
      std::placeholders::_2));

  // The state callback runs in its own group so a MultiThreadedExecutor can
  // keep publishing live odometry while cone processing or optimization
  // holds the graph. Cones and the optimization timer share a mutually
  // exclusive group, so they never run concurrently with each other.
  state_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  graph_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions state_options;
  state_options.callback_group = state_callback_group_;
  rclcpp::SubscriptionOptions graph_options;
  graph_options.callback_group = graph_callback_group_;

  car_state_sub_ = create_subscription<hyu_msgs::msg::CarState>(
    car_state_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&GraphSlamNode::stateCallback, this, std::placeholders::_1),
    state_options);
  cones_sub_ = create_subscription<hyu_msgs::msg::ConeArrayWithCovariance>(
    cones_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&GraphSlamNode::conesCallback, this, std::placeholders::_1),
    graph_options);
  // RViz "2D Pose Estimate" publishes here; used to relocalize when lost.
  initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose",
    rclcpp::QoS(1),
    std::bind(&GraphSlamNode::initialPoseCallback, this, std::placeholders::_1),
    graph_options);

  if (gnss_prior_enable_) {
    gnss_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      gnss_prior_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&GraphSlamNode::gnssOdomCallback, this, std::placeholders::_1),
      state_options);
  }

  optimize_timer_ = create_wall_timer(
    std::chrono::milliseconds(250),
    std::bind(&GraphSlamNode::onOptimizeTimer, this),
    graph_callback_group_);

  RCLCPP_INFO(
    get_logger(),
    "g2o graph SLAM listening to car state '%s' and cones '%s'",
    car_state_topic_.c_str(),
    cones_topic_.c_str());

  if (localization_mode_ && !landmarks_.empty()) {
    // Publish the loaded fixed map once so latched subscribers (RViz, GUI)
    // can preview it before the car has moved.
    const rclcpp::Time now = get_clock()->now();
    publishMap(now);
    publishMarkers(now);
    RCLCPP_INFO(
      get_logger(), "Localization mode active with a fixed map of %zu cones",
      landmarks_.size());
  }

  publishStatus();
}

void GraphSlamNode::declareRecoveryParameters()
{
  auto_relocalize_enable_ =
    declare_parameter<bool>("auto_relocalize_enable", auto_relocalize_enable_);
  auto_relocalize_min_visible_cones_ = declare_parameter<int>(
    "auto_relocalize_min_visible_cones", auto_relocalize_min_visible_cones_);
  auto_relocalize_lost_frames_ = declare_parameter<int>(
    "auto_relocalize_lost_frames", auto_relocalize_lost_frames_);
  auto_relocalize_min_inliers_ = declare_parameter<int>(
    "auto_relocalize_min_inliers", auto_relocalize_min_inliers_);
  auto_relocalize_search_radius_ = declare_parameter<double>(
    "auto_relocalize_search_radius", auto_relocalize_search_radius_);
  auto_relocalize_max_search_radius_ = declare_parameter<double>(
    "auto_relocalize_max_search_radius", auto_relocalize_max_search_radius_);
  auto_relocalize_cooldown_sec_ = declare_parameter<double>(
    "auto_relocalize_cooldown_sec", auto_relocalize_cooldown_sec_);
  auto_relocalize_gnss_holdoff_sec_ = declare_parameter<double>(
    "auto_relocalize_gnss_holdoff_sec", auto_relocalize_gnss_holdoff_sec_);

  gate_anchor_enable_ =
    declare_parameter<bool>("gate_anchor_enable", gate_anchor_enable_);
  gate_anchor_cluster_radius_m_ = declare_parameter<double>(
    "gate_anchor_cluster_radius_m", gate_anchor_cluster_radius_m_);
  gate_anchor_pair_tolerance_m_ = declare_parameter<double>(
    "gate_anchor_pair_tolerance_m", gate_anchor_pair_tolerance_m_);
  gate_anchor_min_pair_separation_m_ = declare_parameter<double>(
    "gate_anchor_min_pair_separation_m", gate_anchor_min_pair_separation_m_);
  gate_anchor_max_pair_separation_m_ = declare_parameter<double>(
    "gate_anchor_max_pair_separation_m", gate_anchor_max_pair_separation_m_);
  gate_anchor_min_inliers_ = declare_parameter<int>(
    "gate_anchor_min_inliers", gate_anchor_min_inliers_);
  gate_anchor_min_correction_m_ = declare_parameter<double>(
    "gate_anchor_min_correction_m", gate_anchor_min_correction_m_);
  gate_anchor_max_correction_m_ = declare_parameter<double>(
    "gate_anchor_max_correction_m", gate_anchor_max_correction_m_);
  gate_anchor_cooldown_travel_m_ = declare_parameter<double>(
    "gate_anchor_cooldown_travel_m", gate_anchor_cooldown_travel_m_);

  auto_relocalize_min_visible_cones_ = std::max(1, auto_relocalize_min_visible_cones_);
  auto_relocalize_lost_frames_ = std::max(1, auto_relocalize_lost_frames_);
  auto_relocalize_min_inliers_ = std::max(2, auto_relocalize_min_inliers_);
  auto_relocalize_search_radius_ = std::max(0.5, auto_relocalize_search_radius_);
  auto_relocalize_max_search_radius_ =
    std::max(auto_relocalize_search_radius_, auto_relocalize_max_search_radius_);
  auto_relocalize_cooldown_sec_ = std::max(0.0, auto_relocalize_cooldown_sec_);
  auto_relocalize_current_radius_ = auto_relocalize_search_radius_;
  gate_anchor_cluster_radius_m_ = std::max(0.1, gate_anchor_cluster_radius_m_);
  gate_anchor_pair_tolerance_m_ = std::max(0.05, gate_anchor_pair_tolerance_m_);
  gate_anchor_min_pair_separation_m_ = std::max(0.5, gate_anchor_min_pair_separation_m_);
  gate_anchor_max_pair_separation_m_ =
    std::max(gate_anchor_min_pair_separation_m_, gate_anchor_max_pair_separation_m_);
  gate_anchor_min_inliers_ = std::max(2, gate_anchor_min_inliers_);
  gate_anchor_min_correction_m_ = std::max(0.0, gate_anchor_min_correction_m_);
  gate_anchor_max_correction_m_ =
    std::max(gate_anchor_min_correction_m_, gate_anchor_max_correction_m_);
  gate_anchor_cooldown_travel_m_ = std::max(0.0, gate_anchor_cooldown_travel_m_);
}

void GraphSlamNode::configureOptimizer()
{
  optimizer_.setVerbose(false);

  using BlockSolverType = g2o::BlockSolver_3_2;
  auto linear_solver =
    std::make_unique<g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>>();
  auto block_solver = std::make_unique<BlockSolverType>(std::move(linear_solver));
  auto algorithm =
    std::make_unique<g2o::OptimizationAlgorithmLevenberg>(std::move(block_solver));

  optimizer_.setAlgorithm(algorithm.release());
}

void GraphSlamNode::resetGraph()
{
  optimizer_.clear();
  poses_.clear();
  landmarks_.clear();
  last_observations_.clear();
  if (frontend_) {
    frontend_->reset();
  }
  {
    std::lock_guard<std::mutex> lock(odom_buffer_mutex_);
    raw_odom_buffer_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    keyframe_snapshot_.valid = false;
  }

  next_vertex_id_ = 0;
  next_edge_id_ = 0;
  keyframes_since_last_optimization_ = 0;
  last_cone_pose_graph_id_ = -1;
  // Start GNSS priors suppressed after a reset. The graph re-anchors at the
  // car's current odometry, so the fresh map frame does not match the GNSS
  // ENU frame until (if ever) it is proven consistent. Re-arming immediately
  // lets a mode-4 fix drag the pose toward the ENU origin (~0,0) for the few
  // keyframes before the residual gate re-suppresses it — and any cones
  // observed during that slide get baked into the map at the wrong place.
  // maybeAddGnssPrior re-arms this only once the fix agrees within
  // gnss_prior_rearm_max_residual of the cone-anchored pose.
  gnss_prior_suppressed_ = true;
  gnss_prior_suppress_until_sec_ = 0.0;
  map_converged_ = false;
  // Full reconstruction, not reset(): the lap-return relaxation may have
  // lowered the candidate threshold, and a mapping restart must go back to
  // the strict configured window.
  loop_confirmation_window_ = LoopConfirmationWindow(loop_confirmation_config_);
  loop_confirmation_relaxed_on_lap_return_ = false;
  lap_finish_gate_.reset();
  seam_loop_candidate_count_ = 0U;
  loop_confirmation_ready_for_optimize_ = false;
  loop_closure_optimize_cycles_ = 0;
  loop_candidate_count_ = 0U;
  loop_confirmed_count_ = 0U;
  loop_rejected_count_ = 0U;
  loop_candidate_window_count_ = 0U;
  last_loop_confirmation_reason_ = LoopConfirmationReason::PendingThreshold;
  optimizer_skipped_pose_limit_ = false;
  last_odom_stamp_sec_ = -1.0;
  last_cone_stamp_sec_ = -1.0;
  last_map_update_stamp_sec_ = -1.0;
  last_live_odom_publish_stamp_sec_ = -1.0;
  lifecycle_map_saved_ = false;
  lap_return_criteria_satisfied_ = false;
  traveled_distance_ = 0.0;
  lap_origin_capture_traveled_m_ = 0.0;
  lap_origin_captured_ = false;
  lap_origin_ = g2o::SE2();
  last_optimization_time_sec_ = -1.0;
  last_visual_publish_time_sec_ = -1.0;
  last_landmark_delete_time_sec_ = -1.0;
  lost_frames_ = 0;
  healthy_streak_ = 0;
  auto_relocalize_current_radius_ = auto_relocalize_search_radius_;
  last_auto_relocalize_attempt_sec_ = -1.0e18;
  last_gate_anchor_traveled_m_ = -1.0e18;
}

void GraphSlamNode::stateCallback(const hyu_msgs::msg::CarState::SharedPtr msg)
{
  const rclcpp::Time stamp = stampOrNow(msg->header.stamp, get_clock());
  const g2o::SE2 raw_odom = poseFromCarState(*msg);
  // Physical-plausibility gate on the motion input: a raw-odometry step
  // faster than any FS car can move is upstream corruption, and one such
  // sample is enough to found landmarks hundreds of meters off track
  // (2026-07-18 autopsy: wild ghosts at 177-393 m survived every
  // pose-conditioned guard because the pose itself had jumped). Drop it
  // LOUDLY so the producer is identifiable from the log.
  {
    std::lock_guard<std::mutex> buffer_lock(odom_buffer_mutex_);
    if (!raw_odom_buffer_.empty()) {
      const double dt = stamp.seconds() - raw_odom_buffer_.back().first;
      const double dist =
        (raw_odom.translation() - raw_odom_buffer_.back().second.translation()).norm();
      if (dt > 0.0 && dt < 1.0 && dist / dt > 30.0) {
        RCLCPP_ERROR(
          get_logger(),
          "IMPOSSIBLE MOTION on %s: %.2f m in %.3f s (%.0f m/s) at stamp %.3f "
          "— dropping sample; upstream odometry is corrupt",
          car_state_topic_.c_str(), dist, dt, dist / dt, stamp.seconds());
        return;
      }
    }
  }
  recordRawOdometry(stamp.seconds(), raw_odom);
  last_odom_stamp_sec_ = stamp.seconds();

  // Motion-source trust and twist passthrough (see the member comments).
  latest_odom_sigma_trans_ =
    std::sqrt(std::max(0.0, std::max(msg->pose.covariance[0], msg->pose.covariance[7])));
  latest_odom_sigma_yaw_ = std::sqrt(std::max(0.0, msg->pose.covariance[35]));
  latest_twist_vx_.store(msg->twist.twist.linear.x);
  latest_twist_vy_.store(msg->twist.twist.linear.y);
  latest_twist_wz_.store(msg->twist.twist.angular.z);

  // Never wait for a running optimization: dead-reckon from the last
  // keyframe snapshot instead so live odometry keeps its input rate.
  std::unique_lock<std::mutex> lock(graph_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    publishLiveEstimateFromSnapshot(stamp, raw_odom);
    return;
  }

  if (poses_.empty()) {
    addInitialPose(raw_odom, stamp);
    publishEstimate();
    return;
  }

  if (!shouldCreateKeyframe(raw_odom, stamp)) {
    publishLiveEstimate(stamp, estimateFromRawOdometry(raw_odom), raw_odom);
    return;
  }

  addKeyframe(raw_odom, stamp);
  publishEstimate();
}

void GraphSlamNode::conesCallback(
  const hyu_msgs::msg::ConeArrayWithCovariance::SharedPtr msg)
{
  const auto tic = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(graph_mutex_);

  if (poses_.empty()) {
    return;
  }

  const ObservationUpdate update = addConeObservations(*msg, false);
  const rclcpp::Time stamp = stampOrNow(msg->header.stamp, get_clock());
  last_cone_stamp_sec_ = stamp.seconds();

  // Perception-driven optimization: correct the pose as soon as new cone
  // observations land, not only every N keyframes. optimize_min_interval
  // still rate-limits how often the solver actually runs. This keeps the
  // estimate continuously converging, so an initial pose that is slightly
  // off is pulled back quickly instead of lagging for many keyframes.
  if (update.added_edges > 0U) {
    maybeOptimize();
  }

  if (update.added_edges > 0U ||
    update.updated_landmarks > 0U ||
    update.deleted_landmarks > 0U)
  {
    last_map_update_stamp_sec_ = stamp.seconds();
    publishGraphVisuals(stamp);
  }

  maybeAutoRelocalize(update, stamp);

  // Perception-to-estimate latency JSON, consumed by the race harness
  // for latency scoring. Includes the
  // in-line optimizer time — that spike IS this backend's latency story.
  publishTiming(
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - tic).count(),
    update);
}

void GraphSlamNode::publishTiming(double elapsed_ms, const ObservationUpdate & update)
{
  if (!timing_pub_) {
    return;
  }
  frame_times_ms_.push_back(elapsed_ms);
  if (frame_times_ms_.size() > 400U) {
    frame_times_ms_.erase(frame_times_ms_.begin());
  }
  std::vector<double> sorted = frame_times_ms_;
  std::sort(sorted.begin(), sorted.end());
  const auto percentile = [&sorted](double p) {
      if (sorted.empty()) {return 0.0;}
      const std::size_t index = static_cast<std::size_t>(
        p * static_cast<double>(sorted.size() - 1U));
      return sorted[index];
    };

  std::ostringstream out;
  out << std::fixed << std::setprecision(2)
      << "{\"backend\":\"graph\""
      << ",\"mode\":\"" << (localization_mode_ ? "localization" : "mapping") << "\""
      << ",\"landmarks\":" << landmarks_.size()
      << ",\"poses\":" << poses_.size()
      << ",\"matched\":" << update.matched_landmarks
      << ",\"tracks\":" << (frontend_ ? frontend_->tracks().size() : 0U)
      << ",\"frame_ms\":" << elapsed_ms
      << ",\"frame_ms_p50\":" << percentile(0.5)
      << ",\"frame_ms_p99\":" << percentile(0.99)
      << ",\"traveled_m\":" << traveled_distance_ << "}";
  std_msgs::msg::String timing;
  timing.data = out.str();
  timing_pub_->publish(timing);
}

void GraphSlamNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  const g2o::SE2 pose(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    yawFromQuaternion(msg->pose.pose.orientation));

  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    RCLCPP_WARN(
      get_logger(),
      "2D pose estimate is in frame '%s' but the SLAM map frame is '%s'; "
      "relocalizing as if it were in '%s'",
      msg->header.frame_id.c_str(), map_frame_.c_str(), map_frame_.c_str());
  }

  std::lock_guard<std::mutex> lock(graph_mutex_);

  // The click is a manual absolute reference that may contradict GNSS (wrong
  // map frame, degraded GNSS). Suppress GNSS priors so they cannot yank the
  // pose back; they re-arm in maybeAddGnssPrior once GNSS agrees again.
  if (gnss_prior_enable_ && gnss_prior_suppress_duration_ > 0.0) {
    suppressGnssPriors(stampOrNow(msg->header.stamp, get_clock()).seconds());
    RCLCPP_INFO(
      get_logger(),
      "Manual relocalization: GNSS priors suppressed for %.0f s, then until "
      "GNSS agrees with the cone-anchored pose within %.1f m",
      gnss_prior_suppress_duration_, gnss_prior_rearm_max_residual_);
  }

  relocalizeTo(pose);
}

void GraphSlamNode::suppressGnssPriors(double now_sec)
{
  if (!(gnss_prior_enable_ && gnss_prior_suppress_duration_ > 0.0)) {
    return;
  }
  gnss_prior_suppressed_ = true;
  gnss_prior_suppress_until_sec_ = now_sec + gnss_prior_suppress_duration_;
}

bool GraphSlamNode::matchGateFromObservations(
  const std::vector<ConeObservation> & observations,
  bool restrict_to_lap_origin,
  g2o::SE2 * pose_out,
  int * inliers_out) const
{
  std::vector<GatePoint> gate_obs;
  std::vector<GatePoint> all_obs;
  all_obs.reserve(observations.size());
  for (const ConeObservation & obs : observations) {
    const GatePoint p{obs.measurement.x(), obs.measurement.y()};
    all_obs.push_back(p);
    // The observation side tolerates the big/small-orange misclassification
    // (same colour, size-only distinction — routine at range for vision): a
    // gate cone momentarily seen as Orange must not cost the anchor. The
    // LANDMARK side below stays BigOrange-only: map colours are majority-
    // voted (robust), and small-orange lane markings elsewhere on a track
    // must never form a false gate constellation.
    if (obs.color == ConeColor::BigOrange || obs.color == ConeColor::Orange) {
      gate_obs.push_back(p);
    }
  }
  if (gate_obs.size() < 2U) {
    return false;
  }

  std::vector<GatePoint> gate_landmarks;
  std::vector<GatePoint> all_landmarks;
  all_landmarks.reserve(landmarks_.size());
  for (const LandmarkRecord & lm : landmarks_) {
    const Eigen::Vector2d est = lm.vertex->estimate();
    all_landmarks.push_back(GatePoint{est.x(), est.y()});
    if (lm.color != ConeColor::BigOrange) {
      continue;
    }
    // While mapping, big-orange landmarks founded far from the lap origin
    // are drift-era duplicates of the gate itself; matching against them
    // would anchor the pose to the drift instead of closing it.
    if (restrict_to_lap_origin && lap_origin_captured_ &&
      (est - lap_origin_.translation()).norm() > lap_seam_landmark_radius_m_)
    {
      continue;
    }
    gate_landmarks.push_back(GatePoint{est.x(), est.y()});
  }
  if (gate_landmarks.size() < 2U) {
    return false;
  }

  GateMatchParams params;
  params.cluster_radius_m = gate_anchor_cluster_radius_m_;
  params.pair_tolerance_m = gate_anchor_pair_tolerance_m_;
  params.min_pair_separation_m = gate_anchor_min_pair_separation_m_;
  params.max_pair_separation_m = gate_anchor_max_pair_separation_m_;
  params.inlier_distance_m = relocalize_inlier_distance_;
  params.min_inliers = gate_anchor_min_inliers_;

  GateMatchResult result;
  if (!matchGateConstellation(
      gate_obs, gate_landmarks, all_obs, all_landmarks, params, &result))
  {
    return false;
  }
  if (pose_out != nullptr) {
    *pose_out = g2o::SE2(result.pose.x, result.pose.y, result.pose.theta);
  }
  if (inliers_out != nullptr) {
    *inliers_out = result.inliers;
  }
  return true;
}

bool GraphSlamNode::maybeApplyGateAnchor(
  const std::vector<ConeObservation> & observations,
  const g2o::SE2 & observation_pose,
  const g2o::SE2 & keyframe_to_observation,
  const PoseRecord & pose)
{
  if (!gate_anchor_enable_ || localization_mode_ || !lap_origin_captured_) {
    return false;
  }
  // Standing on the gate at capture trivially matches it; only a LAP return
  // counts (same floor the lap-finish gate uses).
  if (traveled_distance_ - lap_origin_capture_traveled_m_ < lap_return_min_travel_m_) {
    return false;
  }
  if (traveled_distance_ - last_gate_anchor_traveled_m_ < gate_anchor_cooldown_travel_m_) {
    return false;
  }

  g2o::SE2 gate_pose;
  int inliers = 0;
  if (!matchGateFromObservations(observations, true, &gate_pose, &inliers)) {
    return false;
  }

  const double correction_norm =
    (gate_pose.translation() - observation_pose.translation()).norm();
  const double yaw_correction = std::abs(
    normalizeAngle(
      gate_pose.rotation().angle() - observation_pose.rotation().angle()));
  if (correction_norm > gate_anchor_max_correction_m_) {
    RCLCPP_WARN(
      get_logger(),
      "Orange-gate anchor rejected: implied correction %.1f m exceeds the "
      "%.1f m sanity bound (mis-association more likely than drift)",
      correction_norm, gate_anchor_max_correction_m_);
    return false;
  }
  if (correction_norm < gate_anchor_min_correction_m_ && yaw_correction < 0.2) {
    // Inside the association gate's reach: the normal loop-closure path is
    // already handling it, and it does so with proper per-cone covariances.
    return false;
  }

  // Re-seed the current keyframe estimate onto the gate-aligned pose. The
  // odometry edge to the previous keyframe now carries the full drift as
  // residual, which is exactly what a loop closure is: the optimizer
  // redistributes it along the elastic chain (hundreds of odometry edges at
  // sigma ~2-5 cm absorb meters of drift for less chi2 than stretching a few
  // cone edges by the same amount). With the estimate corrected, THIS frame's
  // observations associate with the first-lap landmarks, so the seam
  // accumulates ordinary loop candidates and the existing confirmation /
  // freeze machinery proceeds unchanged.
  pose.vertex->setEstimate(gate_pose * keyframe_to_observation.inverse());
  last_gate_anchor_traveled_m_ = traveled_distance_;
  ++gate_anchor_count_;
  keyframes_since_last_optimization_ =
    std::max(keyframes_since_last_optimization_, optimize_every_n_keyframes_);
  RCLCPP_WARN(
    get_logger(),
    "Orange-gate anchor: closed %.1f m / %.2f rad of drift at the lap seam "
    "(%d cones verify the gate-aligned pose; anchor #%zu)",
    correction_norm, yaw_correction, inliers, gate_anchor_count_);
  return true;
}

void GraphSlamNode::maybeAutoRelocalize(
  const ObservationUpdate & update, const rclcpp::Time & stamp)
{
  // Only against a frozen map: while mapping, unmatched cones legitimately
  // found new landmarks, so "nothing associated" is not a lost signature.
  if (!auto_relocalize_enable_ || !localization_mode_ ||
    poses_.empty() || landmarks_.empty())
  {
    return;
  }
  if (last_observations_.size() <
    static_cast<std::size_t>(auto_relocalize_min_visible_cones_))
  {
    return;  // too few cones to judge either way
  }
  // A single match while lost is routinely an aliased cone; require two
  // before a frame counts as healthy, and require a STREAK of healthy
  // frames before the escalated search radius de-escalates.
  if (update.matched_landmarks >= 2U) {
    lost_frames_ = 0;
    if (++healthy_streak_ >= 5) {
      auto_relocalize_current_radius_ = auto_relocalize_search_radius_;
    }
    return;
  }
  healthy_streak_ = 0;
  if (update.matched_landmarks == 1U) {
    return;  // ambiguous: neither healthy nor further into lost
  }

  ++lost_frames_;
  if (lost_frames_ < auto_relocalize_lost_frames_) {
    return;
  }
  const double now = stamp.seconds();

  // A recently ACCEPTED GNSS prior (it passed the sigma gate and, post-
  // convergence, the innovation gate above) means the pose is externally
  // corroborated to within the innovation budget. A zero-association streak
  // under a healthy anchor is a perception hiccup or local map sparsity, not
  // "lost" — and a scan-match jump against noisy real-perception cones is
  // exactly how a healthy run acquires a 10+ m excursion (observed 8 false
  // relocalizations in one 300 s RTK lap set, 2026-07-17). Relocalization
  // stays armed for the case it exists for: anchors absent, rejected, or
  // suppressed — a real outage or a correlated INS excursion.
  if (gnss_prior_enable_ && !gnss_prior_suppressed_ &&
    now - last_gnss_prior_stamp_sec_ < auto_relocalize_gnss_holdoff_sec_)
  {
    return;
  }

  // Seed with the current estimate; a matched orange gate replaces it with a
  // drift-independent global fix (and then only needs a small refinement).
  // A gate sighting is rare and brief — one frame per lap — so it bypasses
  // the attempt cooldown instead of gambling that they coincide.
  g2o::SE2 seed = poses_.back().vertex->estimate();
  double radius = auto_relocalize_current_radius_;
  bool gate_seeded = false;
  g2o::SE2 gate_pose;
  if (gate_anchor_enable_ &&
    matchGateFromObservations(last_observations_, false, &gate_pose, nullptr))
  {
    seed = gate_pose;
    radius = std::min(radius, 2.0);
    gate_seeded = true;
  }
  if (!gate_seeded &&
    now - last_auto_relocalize_attempt_sec_ < auto_relocalize_cooldown_sec_)
  {
    return;
  }
  last_auto_relocalize_attempt_sec_ = now;

  // The acceptance bar cannot exceed what is visible: perception often
  // serves only 3-5 cones per frame, and demanding a fixed 4 inliers has
  // rejected poses where ALL visible cones fit. Require 3/4 of the visible
  // set, floored at 3, capped at the configured count.
  const int required_inliers = std::min(
    auto_relocalize_min_inliers_,
    std::max(
      3, static_cast<int>(std::ceil(
        0.75 *
        static_cast<double>(last_observations_.size())))));

  int inliers = 0;
  const g2o::SE2 refined = scanMatchNear(seed, radius, relocalize_search_yaw_, &inliers);
  if (inliers < required_inliers) {
    RCLCPP_WARN(
      get_logger(),
      "Auto relocalization attempt failed: %d/%zu inliers (< %d) at radius "
      "%.1f m%s; escalating",
      inliers, last_observations_.size(), required_inliers,
      radius, gate_seeded ? " (gate-seeded)" : "");
    auto_relocalize_current_radius_ = std::min(
      auto_relocalize_max_search_radius_, auto_relocalize_current_radius_ * 2.0);
    return;
  }

  // A recovered pose is a competing absolute reference, exactly like a
  // manual /initialpose click: keep GNSS priors from yanking it back until
  // they agree with the cone-anchored pose again.
  suppressGnssPriors(now);
  relocalizeAt(refined);
  ++auto_relocalize_count_;
  lost_frames_ = 0;
  healthy_streak_ = 0;
  auto_relocalize_current_radius_ = auto_relocalize_search_radius_;
  RCLCPP_WARN(
    get_logger(),
    "Auto relocalization #%zu: recovered with %d/%zu inliers%s",
    auto_relocalize_count_, inliers, last_observations_.size(),
    gate_seeded ? " from an orange-gate seed" : "");
}

g2o::SE2 GraphSlamNode::latestRawOdom() const
{
  std::lock_guard<std::mutex> lock(odom_buffer_mutex_);
  if (raw_odom_buffer_.empty()) {
    return g2o::SE2();
  }
  return raw_odom_buffer_.back().second;
}

g2o::SE2 GraphSlamNode::gridSearchPose(
  const g2o::SE2 & seed, double radius, double xy_step,
  double yaw_span, double yaw_step, double inlier_distance,
  int * best_inliers_out) const
{
  // Brute-force search over a window around the seed: for each candidate
  // pose, transform the latest cones into the map and count how many land
  // within inlier_distance of a fixed landmark. Best inlier count
  // (tie-broken by residual) wins.
  const int n_xy = xy_step > 0.0 ?
    static_cast<int>(std::ceil(radius / xy_step)) : 0;
  const int n_yaw = yaw_step > 0.0 ?
    static_cast<int>(std::ceil(yaw_span / yaw_step)) : 0;
  const double inlier_sq = inlier_distance * inlier_distance;

  g2o::SE2 best = seed;
  int best_inliers = -1;
  double best_residual = std::numeric_limits<double>::max();

  for (int ix = -n_xy; ix <= n_xy; ++ix) {
    for (int iy = -n_xy; iy <= n_xy; ++iy) {
      for (int iw = -n_yaw; iw <= n_yaw; ++iw) {
        const g2o::SE2 cand(
          seed.translation().x() + ix * xy_step,
          seed.translation().y() + iy * xy_step,
          normalizeAngle(seed.rotation().angle() + iw * yaw_step));

        int inliers = 0;
        double residual = 0.0;
        for (const ConeObservation & obs : last_observations_) {
          const Eigen::Vector2d p = cand * obs.measurement;
          double nearest_sq = std::numeric_limits<double>::max();
          for (const LandmarkRecord & lm : landmarks_) {
            const double d2 = (lm.vertex->estimate() - p).squaredNorm();
            if (d2 < nearest_sq) {
              nearest_sq = d2;
            }
          }
          if (nearest_sq < inlier_sq) {
            ++inliers;
            residual += nearest_sq;
          }
        }

        if (inliers > best_inliers ||
          (inliers == best_inliers && residual < best_residual))
        {
          best_inliers = inliers;
          best_residual = residual;
          best = cand;
        }
      }
    }
  }

  if (best_inliers_out != nullptr) {
    *best_inliers_out = std::max(0, best_inliers);
  }
  return best;
}

g2o::SE2 GraphSlamNode::scanMatchNear(
  const g2o::SE2 & seed, double radius, double yaw_span,
  int * inliers_out) const
{
  if (last_observations_.empty() || landmarks_.empty() || radius <= 0.0) {
    if (inliers_out != nullptr) {
      *inliers_out = 0;
    }
    return seed;
  }

  // Coarse pass finds the basin with a step-matched (widened) inlier ring so
  // the true pose cannot fall between grid points; the fine pass localizes
  // with the real inlier distance. Two stages keep a 16 m search tractable
  // where a single fine grid would be ~100x the work.
  const double coarse_step = std::max(0.4, radius / 8.0);
  const double coarse_yaw_step = std::max(0.05, yaw_span / 8.0);
  const double coarse_inlier =
    std::max(relocalize_inlier_distance_, 0.75 * coarse_step);
  const g2o::SE2 coarse = gridSearchPose(
    seed, radius, coarse_step, yaw_span, coarse_yaw_step, coarse_inlier,
    nullptr);

  int inliers = 0;
  const g2o::SE2 fine = gridSearchPose(
    coarse, 1.5 * coarse_step, 0.15, 1.5 * coarse_yaw_step, 0.03,
    relocalize_inlier_distance_, &inliers);

  if (inliers_out != nullptr) {
    *inliers_out = inliers;
  }
  RCLCPP_INFO(
    get_logger(),
    "Relocalization scan-match: %d/%zu cones fit the map at the best pose "
    "(radius %.1f m)",
    inliers, last_observations_.size(), radius);
  return fine;
}

void GraphSlamNode::relocalizeTo(const g2o::SE2 & click)
{
  // Scan-match the latest cones against the fixed map near the click to find
  // the best-fit starting pose. The operator asserts the car is near the
  // click, so the result is applied regardless of the inlier count (the
  // automatic path in maybeAutoRelocalize gates on it instead).
  relocalizeAt(scanMatchNear(click, relocalize_search_radius_, relocalize_search_yaw_, nullptr));
}

void GraphSlamNode::relocalizeAt(const g2o::SE2 & pose)
{
  // Drop the current pose trajectory (and its odometry/observation edges),
  // keeping the map. In localization mode this re-anchors the drifted
  // odometry to the fixed map.
  for (PoseRecord & record : poses_) {
    if (optimizer_.vertex(record.graph_id) == record.vertex) {
      optimizer_.removeVertex(record.vertex);
    }
  }
  poses_.clear();
  keyframes_since_last_optimization_ = 0;
  last_cone_pose_graph_id_ = -1;
  last_optimization_time_sec_ = -1.0;

  const bool can_refine = !last_observations_.empty() && !landmarks_.empty();

  auto * vertex = new g2o::VertexSE2();
  vertex->setId(next_vertex_id_++);
  vertex->setEstimate(pose);
  // (A) Leave the anchor free so cone observations to the fixed map refine it
  // to the local optimum; pin it only when there is nothing to refine against.
  vertex->setFixed(!can_refine);
  if (!optimizer_.addVertex(vertex)) {
    RCLCPP_ERROR(get_logger(), "Relocalization failed to add anchor pose vertex");
    delete vertex;
    return;
  }
  poses_.push_back(PoseRecord{vertex->id(), vertex, latestRawOdom(), get_clock()->now()});

  // Constrain the anchor with observation edges to the fixed map, then run a
  // local optimization so the pose snaps to the best fit near the click.
  std::size_t constrained = 0;
  if (can_refine) {
    std::vector<bool> claimed(landmarks_.size(), false);
    for (const ConeObservation & obs : last_observations_) {
      const Eigen::Vector2d map_point = pose * obs.measurement;
      const Eigen::Matrix2d map_covariance = covarianceInMapFrame(pose, obs.covariance);
      bool ambiguous = false;
      const int idx =
        findAssociatedLandmark(map_point, map_covariance, obs.color, &ambiguous, &claimed);
      if (idx >= 0) {
        claimed[static_cast<std::size_t>(idx)] = true;
        addObservationEdge(obs, vertex, landmarks_[static_cast<std::size_t>(idx)]);
        ++constrained;
      }
    }
    if (constrained >= 2U) {
      optimizeGraph();
    } else {
      // Not enough matches to solve for a full pose; keep the scan-match guess.
      vertex->setFixed(true);
    }
  }

  updateKeyframeSnapshot();
  publishEstimate();

  const g2o::SE2 result = vertex->estimate();
  // Lap accounting restarts from the asserted pose.
  traveled_distance_ = 0.0;
  lap_origin_captured_ = false;
  RCLCPP_INFO(
    get_logger(),
    "Relocalized to x=%.2f y=%.2f yaw=%.2f (%zu cones constrained the fit, "
    "%zu map landmarks kept)",
    result.translation().x(), result.translation().y(), result.rotation().angle(),
    constrained, landmarks_.size());
}

void GraphSlamNode::onOptimizeTimer()
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  maybeOptimize();
}

g2o::SE2 GraphSlamNode::poseFromCarState(const hyu_msgs::msg::CarState & msg) const
{
  return g2o::SE2(
    msg.pose.pose.position.x,
    msg.pose.pose.position.y,
    yawFromQuaternion(msg.pose.pose.orientation));
}

g2o::SE2 GraphSlamNode::estimateFromRawOdometry(const g2o::SE2 & raw_odom) const
{
  if (poses_.empty()) {
    return raw_odom;
  }

  const PoseRecord & previous = poses_.back();
  const g2o::SE2 odom_delta = previous.raw_odom.inverse() * raw_odom;
  return previous.vertex->estimate() * odom_delta;
}

bool GraphSlamNode::shouldCreateKeyframe(
  const g2o::SE2 & raw_odom,
  const rclcpp::Time & stamp) const
{
  if (poses_.empty()) {
    return true;
  }

  const PoseRecord & previous = poses_.back();
  const g2o::SE2 delta = previous.raw_odom.inverse() * raw_odom;
  const double distance = delta.translation().norm();
  const double yaw = std::abs(normalizeAngle(delta.rotation().angle()));
  const double dt = (stamp - previous.stamp).seconds();

  return distance >= keyframe_distance_ ||
         yaw >= keyframe_yaw_ ||
         (keyframe_max_dt_ > 0.0 && dt >= keyframe_max_dt_);
}

void GraphSlamNode::addInitialPose(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp)
{
  auto * vertex = new g2o::VertexSE2();
  vertex->setId(next_vertex_id_++);
  vertex->setEstimate(raw_odom);
  vertex->setFixed(true);

  if (!optimizer_.addVertex(vertex)) {
    RCLCPP_ERROR(get_logger(), "Failed to add initial g2o pose vertex");
    delete vertex;
    return;
  }

  poses_.push_back(PoseRecord{vertex->id(), vertex, raw_odom, stamp});
  updateKeyframeSnapshot();
  traveled_distance_ = 0.0;
  lap_origin_captured_ = false;
  RCLCPP_INFO(
    get_logger(),
    "Initialized graph SLAM at x=%.2f y=%.2f yaw=%.2f",
    raw_odom.translation().x(),
    raw_odom.translation().y(),
    raw_odom.rotation().angle());
}

void GraphSlamNode::addKeyframe(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp)
{
  if (poses_.empty()) {
    addInitialPose(raw_odom, stamp);
    return;
  }

  const PoseRecord previous = poses_.back();
  const g2o::SE2 odom_delta = previous.raw_odom.inverse() * raw_odom;
  const g2o::SE2 initial_estimate = previous.vertex->estimate() * odom_delta;

  auto * vertex = new g2o::VertexSE2();
  vertex->setId(next_vertex_id_++);
  vertex->setEstimate(initial_estimate);

  if (!optimizer_.addVertex(vertex)) {
    RCLCPP_ERROR(get_logger(), "Failed to add g2o pose vertex %d", vertex->id());
    delete vertex;
    return;
  }

  auto * edge = new g2o::EdgeSE2();
  edge->setId(next_edge_id_++);
  edge->setVertex(0, previous.vertex);
  edge->setVertex(1, vertex);
  edge->setMeasurement(odom_delta);

  // Per-edge trust from the motion source's reported noise: a bridge that
  // degrades (e.g. SBG mode tiers) weakens its own constraints instead of
  // being believed at the config sigma. Config sigmas act as the floor and a
  // zero/absent covariance falls back to them entirely.
  Eigen::Matrix3d information = odom_information_;
  if (use_odom_covariance_ && latest_odom_sigma_trans_ > 1e-6) {
    const double sigma_t =
      std::clamp(latest_odom_sigma_trans_, odom_translation_sigma_, 50.0);
    const double sigma_y = std::clamp(
      latest_odom_sigma_yaw_ > 1e-6 ? latest_odom_sigma_yaw_ : odom_yaw_sigma_,
      odom_yaw_sigma_, 10.0);
    information = Eigen::Matrix3d::Zero();
    information(0, 0) = 1.0 / (sigma_t * sigma_t);
    information(1, 1) = 1.0 / (sigma_t * sigma_t);
    information(2, 2) = 1.0 / (sigma_y * sigma_y);
  }
  edge->setInformation(information);

  if (!optimizer_.addEdge(edge)) {
    RCLCPP_ERROR(get_logger(), "Failed to add odometry edge");
    delete edge;
    return;
  }

  maybeAddGnssPrior(vertex, stamp);

  poses_.push_back(PoseRecord{vertex->id(), vertex, raw_odom, stamp});
  updateKeyframeSnapshot();
  ++keyframes_since_last_optimization_;

  traveled_distance_ += odom_delta.translation().norm();
  if (localization_mode_) {
    prunePoseWindow();
  } else {
    if (!lap_origin_captured_ && traveled_distance_ >= lap_origin_capture_distance_) {
      lap_origin_ = vertex->estimate();
      lap_origin_captured_ = true;
      lap_origin_capture_traveled_m_ = traveled_distance_;
      RCLCPP_INFO(
        get_logger(),
        "Lap origin captured on the racing line at x=%.2f y=%.2f (%.1f m in)",
        lap_origin_.translation().x(),
        lap_origin_.translation().y(),
        traveled_distance_);
    }
    maybeFinishMappingLap(vertex->estimate());
  }
}

void GraphSlamNode::gnssOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time stamp = stampOrNow(msg->header.stamp, get_clock());
  std::lock_guard<std::mutex> lock(gnss_mutex_);
  latest_gnss_fix_.stamp_sec = stamp.seconds();
  latest_gnss_fix_.position =
    Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
  // Bridge fills the (x, y) diagonal of the row-major 6x6 pose covariance.
  latest_gnss_fix_.sigma_x = std::sqrt(std::max(0.0, msg->pose.covariance[0]));
  latest_gnss_fix_.sigma_y = std::sqrt(std::max(0.0, msg->pose.covariance[7]));
  latest_gnss_fix_.valid = true;
}

void GraphSlamNode::maybeAddGnssPrior(
  g2o::VertexSE2 * vertex, const rclcpp::Time & stamp)
{
  if (!gnss_prior_enable_) {
    return;
  }

  GnssFix fix;
  {
    std::lock_guard<std::mutex> lock(gnss_mutex_);
    fix = latest_gnss_fix_;
  }
  if (!fix.valid) {
    return;
  }
  // Drop stale fixes and any whose reported accuracy is worse than the gate
  // (degraded modes arrive with a huge covariance and are filtered here).
  if (gnss_prior_max_age_ > 0.0 &&
    std::abs(stamp.seconds() - fix.stamp_sec) > gnss_prior_max_age_)
  {
    return;
  }
  if (fix.sigma_x <= 0.0 || fix.sigma_y <= 0.0 ||
    fix.sigma_x > gnss_prior_max_position_sigma_ ||
    fix.sigma_y > gnss_prior_max_position_sigma_)
  {
    return;
  }

  // The INS reports a BELIEVED accuracy that never reflects the realized
  // (time-correlated, Gauss-Markov) error — it can be confidently wrong.
  // Three defenses, tuned for that failure mode:
  // 1) De-correlate: consecutive fixes share the same GM error draw, so a
  //    prior on every 0.5 m keyframe multiplies one wrong measurement.
  if (gnss_prior_min_interval_ > 0.0 &&
    stamp.seconds() - last_gnss_prior_stamp_sec_ < gnss_prior_min_interval_)
  {
    return;
  }
  // 2) Innovation-gate once the map has converged: a fix that disagrees with
  //    the cone-anchored pose by more than the rearm budget is the INS being
  //    confidently wrong, not the map being off. Before convergence the graph
  //    is still elastic and the Huber kernel is the only sane defense.
  if (map_converged_) {
    const double innovation =
      (vertex->estimate().translation() - fix.position).norm();
    if (innovation > gnss_prior_innovation_max_residual_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "GNSS prior rejected: fix %.2f m from the cone-anchored pose "
        "(believed sigma %.2f m) — correlated INS excursion",
        innovation, fix.sigma_x);
      return;
    }
  }

  if (gnss_prior_suppressed_) {
    if (stamp.seconds() < gnss_prior_suppress_until_sec_) {
      return;
    }
    // Re-arm only when GNSS is consistent with the cone-anchored pose again.
    // A persistent disagreement means the map frame and the GNSS ENU frame
    // differ (e.g. an old local-frame map): keep the priors off and say so.
    const double residual = (vertex->estimate().translation() - fix.position).norm();
    if (residual > gnss_prior_rearm_max_residual_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "GNSS prior still suppressed: fix disagrees with the SLAM pose by "
        "%.1f m (map frame != GNSS ENU frame? set gnss_prior_enable:=false "
        "for non-georeferenced maps)",
        residual);
      return;
    }
    gnss_prior_suppressed_ = false;
    RCLCPP_INFO(
      get_logger(), "GNSS prior re-armed (residual %.2f m)", residual);
  }

  auto * prior = new g2o::EdgeSE2XYPrior();
  prior->setId(next_edge_id_++);
  prior->setVertex(0, vertex);
  prior->setMeasurement(fix.position);
  // 3) Sigma floor: an rtk_fixed report of 0.01 m would carry information
  //    1e4 and let one anchor out-vote the cone map; the floor keeps any
  //    single prior's pull bounded regardless of what the INS believes.
  // 4) Mapping-phase de-weighting: pre-convergence the graph is elastic and
  //    has no innovation gate, so a 1 Hz anchor stream whose realized error
  //    is time-correlated (Gauss-Markov, tens of seconds) drags the whole
  //    map sideways in optimizer-sized steps. Landmarks shift under the
  //    incoming observations, associations split, and duplicated cones are
  //    born — the probabilistic mapping failure observed in race runs
  //    (map 117-162 vs 71 GT, ego snapping 4 cm while stationary). During
  //    mapping the anchors' only real job is bounding gross odometry drift,
  //    which a weak pull does fine; full anchor strength returns with the
  //    frozen map, where the innovation gate stands guard.
  double mapping_scale = 1.0;
  if (!map_converged_) {
    mapping_scale = std::max(1.0, gnss_prior_mapping_sigma_scale_);
  }
  const double sigma_x = std::max(fix.sigma_x, gnss_prior_min_sigma_) * mapping_scale;
  const double sigma_y = std::max(fix.sigma_y, gnss_prior_min_sigma_) * mapping_scale;
  Eigen::Matrix2d information = Eigen::Matrix2d::Zero();
  information(0, 0) = 1.0 / (sigma_x * sigma_x);
  information(1, 1) = 1.0 / (sigma_y * sigma_y);
  prior->setInformation(information);
  last_gnss_prior_stamp_sec_ = stamp.seconds();
  if (gnss_prior_robust_delta_ > 0.0) {
    auto * kernel = new g2o::RobustKernelHuber();
    kernel->setDelta(gnss_prior_robust_delta_);
    prior->setRobustKernel(kernel);
  }

  if (!optimizer_.addEdge(prior)) {
    RCLCPP_ERROR(get_logger(), "Failed to add GNSS prior edge");
    delete prior;
  }
}

GraphSlamNode::ObservationUpdate GraphSlamNode::addConeObservations(
  const hyu_msgs::msg::ConeArrayWithCovariance & msg,
  bool force_process)
{
  if (poses_.empty()) {
    return ObservationUpdate{0U, 0U, 0U, 0U};
  }

  PoseRecord & pose = poses_.back();
  const bool add_edges =
    force_process ||
    process_every_cone_message_ ||
    last_cone_pose_graph_id_ != pose.graph_id;
  const bool update_landmarks = update_existing_landmarks_;
  const rclcpp::Time stamp = stampOrNow(msg.header.stamp, get_clock());
  const bool update_deletions = shouldUpdateLandmarkDeletion(stamp, add_edges);

  if (!add_edges && !update_landmarks && !update_deletions) {
    return ObservationUpdate{0U, 0U, 0U, 0U};
  }

  if (!add_edges &&
    !process_every_cone_message_ &&
    last_cone_pose_graph_id_ == pose.graph_id)
  {
    RCLCPP_DEBUG(
      get_logger(),
      "Skipping duplicate cone graph edges for pose %d; updating landmark deletion state",
      pose.graph_id);
  }

  const std::vector<ConeObservation> observations = extractConeObservations(msg);
  last_observations_ = observations;  // kept for relocalization scan-matching

  // Interpolate raw odometry at the cone stamp so measurements are expressed
  // relative to the keyframe vertex even when the message arrives between
  // keyframes; at speed the uncorrected offset reaches keyframe_distance.
  // A stamp OUTSIDE the buffer span cannot be interpolated — rawOdomAt
  // clamps to the buffer edge, which places the whole frame's cones at a
  // pose from a different time (the 2026-07-17 autopsy found promoted
  // landmarks hundreds of meters off track from exactly such frames).
  {
    std::lock_guard<std::mutex> buffer_lock(odom_buffer_mutex_);
    if (raw_odom_buffer_.empty() ||
      stamp.seconds() < raw_odom_buffer_.front().first - 0.5 ||
      stamp.seconds() > raw_odom_buffer_.back().first + 0.5)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping cone frame: stamp %.3f outside odometry buffer span",
        stamp.seconds());
      return ObservationUpdate{0U, 0U, 0U, 0U};
    }
  }
  const g2o::SE2 raw_at_observation = rawOdomAt(stamp.seconds());
  const g2o::SE2 keyframe_to_observation = pose.raw_odom.inverse() * raw_at_observation;
  g2o::SE2 observation_pose = pose.vertex->estimate() * keyframe_to_observation;

  // Orange-gate seam anchor: runs BEFORE association so that when it closes
  // a beyond-the-gate drift, this same frame's cones already associate with
  // the first-lap landmarks instead of founding another round of duplicates.
  if (add_edges &&
    maybeApplyGateAnchor(observations, observation_pose, keyframe_to_observation, pose))
  {
    observation_pose = pose.vertex->estimate() * keyframe_to_observation;
  }

  if (add_edges && !process_every_cone_message_) {
    last_cone_pose_graph_id_ = pose.graph_id;
  }

  std::size_t added_edges = 0U;
  std::size_t updated_landmarks = 0U;
  bool loop_closure_edge = false;
  std::vector<std::size_t> observed_landmark_indices;
  observed_landmark_indices.reserve(observations.size());

  // Delayed data association (tentative_track_frontend.hpp): observations
  // are matched frame-globally against confirmed landmarks and tentative
  // tracks; unmatched ones found tracks OUTSIDE the graph and only enter it
  // once confirmed and converged. Promotion replays the track's observation
  // history against the original keyframes, so the delay costs no
  // information. This replaces the legacy per-observation NN founding
  // pipeline and all of its creation heuristics.
  std::vector<FrontendObservation> frontend_observations;
  frontend_observations.reserve(observations.size());
  for (const ConeObservation & observation : observations) {
    FrontendObservation frontend_observation;
    frontend_observation.map_point = observation_pose * observation.measurement;
    frontend_observation.map_covariance =
      covarianceInMapFrame(observation_pose, observation.covariance);
    frontend_observation.keyframe_measurement =
      keyframe_to_observation * observation.measurement;
    frontend_observation.keyframe_covariance =
      covarianceInMapFrame(keyframe_to_observation, observation.covariance);
    frontend_observation.pose_graph_id = pose.graph_id;
    frontend_observation.range_m = observation.measurement.norm();
    frontend_observation.color = static_cast<std::uint8_t>(observation.color);
    frontend_observations.push_back(frontend_observation);
  }

  std::vector<FrontendConfirmedLandmark> confirmed_view;
  confirmed_view.reserve(landmarks_.size());
  for (const LandmarkRecord & existing : landmarks_) {
    confirmed_view.push_back(
      FrontendConfirmedLandmark{
        existing.vertex->estimate(), existing.covariance,
        existing.last_seen_traveled});
  }

  // Track misses only count while the track should be observable from the
  // CURRENT pose (occlusion cannot be modelled, but FOV/range exits must
  // not eat the miss budget).
  const auto track_expected_visible =
    [this, &observation_pose](const Eigen::Vector2d & point) {
      const Eigen::Vector2d relative = observation_pose.inverse() * point;
      const double range = relative.norm();
      if (range < min_observation_range_ || range > landmark_delete_max_range_) {
        return false;
      }
      if (landmark_delete_fov_ > 0.0 &&
        landmark_delete_fov_ < 2.0 * std::acos(-1.0) &&
        std::abs(std::atan2(relative.y(), relative.x())) > 0.5 * landmark_delete_fov_)
      {
        return false;
      }
      return true;
    };

  const FrontendFrameResult frame = frontend_->processFrame(
    frontend_observations, confirmed_view, traveled_distance_,
    track_expected_visible);
  const std::size_t matched_landmarks = frame.confirmed_matches.size();

  for (std::size_t i = 0U; i < frame.ambiguous_observations.size(); ++i) {
    // Two landmarks explain a cone almost equally well; fusing or spawning
    // a duplicate would corrupt the map, so the frontend dropped it.
    const auto decision = loop_confirmation_window_.rejectAmbiguousAssociation();
    ++loop_rejected_count_;
    loop_candidate_window_count_ = decision.candidate_count;
    last_loop_confirmation_reason_ = decision.reason;
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Loop confirmation rejected: reason=%s loop_candidates=%zu "
      "loop_confirmed=%zu loop_rejected=%zu",
      toString(last_loop_confirmation_reason_),
      loop_candidate_count_,
      loop_confirmed_count_,
      loop_rejected_count_);
    publishLifecycleDiagnostics();
  }

  for (const FrontendConfirmedMatch & match : frame.confirmed_matches) {
    const FrontendObservation & frontend_observation =
      frontend_observations[match.observation_index];
    const ConeObservation & observation = observations[match.observation_index];
    LandmarkRecord & landmark = landmarks_[match.landmark_index];
    voteLandmarkColor(landmark, observation.color);
    const bool stale_loop_candidate =
      add_edges && loop_gap_distance_ > 0.0 &&
      traveled_distance_ - landmark.last_seen_traveled >= loop_gap_distance_;
    if (stale_loop_candidate) {
      const double residual_m =
        (landmark.vertex->estimate() - frontend_observation.map_point).norm();
      // Seam evidence: this candidate re-associates a landmark that sits at
      // the lap origin, i.e. it is part of the loop that closes the LAP,
      // not a mid-lap mini-loop (a peanut waist). The lap-finish gate can
      // require this before freezing the map.
      if (lap_origin_captured_ &&
        residual_m <= loop_confirmation_config_.max_residual_m &&
        (landmark.vertex->estimate() - lap_origin_.translation()).norm() <=
        lap_seam_landmark_radius_m_)
      {
        ++seam_loop_candidate_count_;
      }
      const auto decision = loop_confirmation_window_.observeCandidate(
        LoopCandidate{traveled_distance_, stamp.seconds(), residual_m});
      ++loop_candidate_count_;
      loop_candidate_window_count_ = decision.candidate_count;
      last_loop_confirmation_reason_ = decision.reason;
      if (decision.reason != LoopConfirmationReason::PendingThreshold &&
        decision.reason != LoopConfirmationReason::Confirmed)
      {
        ++loop_rejected_count_;
      }
      if (decision.confirmed) {
        loop_closure_edge = true;
        loop_confirmation_ready_for_optimize_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Loop confirmation threshold reached: residual=%.3f m "
          "loop_candidates=%zu loop_candidate_window=%zu loop_confirmed=%zu "
          "loop_rejected=%zu; waiting for optimization",
          residual_m,
          loop_candidate_count_,
          loop_candidate_window_count_,
          loop_confirmed_count_,
          loop_rejected_count_);
      } else {
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "Loop confirmation %s: residual=%.3f m loop_candidates=%zu "
          "loop_candidate_window=%zu loop_confirmed=%zu loop_rejected=%zu",
          toString(last_loop_confirmation_reason_),
          residual_m,
          loop_candidate_count_,
          loop_candidate_window_count_,
          loop_confirmed_count_,
          loop_rejected_count_);
      }
      publishLifecycleDiagnostics();
    }
    landmark.last_seen_traveled = traveled_distance_;
    if (update_landmarks &&
      updateLandmarkEstimate(
        landmark, frontend_observation.map_point, frontend_observation.map_covariance))
    {
      ++updated_landmarks;
    }
    observed_landmark_indices.push_back(match.landmark_index);
    if (add_edges) {
      ConeObservation keyframe_observation = observation;
      keyframe_observation.measurement = frontend_observation.keyframe_measurement;
      keyframe_observation.covariance = frontend_observation.keyframe_covariance;
      addObservationEdge(keyframe_observation, pose.vertex, landmark);
      ++added_edges;
    }
  }

  for (const FrontendPromotion & promotion : frame.promotions) {
    // Geometric-impossibility veto: a cone we are supposedly observing
    // cannot be farther than the sensor range from the current pose. Such a
    // track was born from corrupted geometry; drop it instead of freezing
    // it into the map (it would fail the global planner's width check).
    if ((promotion.position - observation_pose.translation()).norm() >
      max_observation_range_ + 5.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "Vetoing impossible landmark promotion %.0f m from the pose",
        (promotion.position - observation_pose.translation()).norm());
      continue;
    }
    LandmarkRecord * landmark = addLandmark(
      promotion.position, promotion.covariance,
      static_cast<ConeColor>(promotion.color));
    if (landmark == nullptr) {
      continue;  // localization mode (frozen map) or landmark cap
    }
    // Replay the confirmed track's observation history into the graph
    // against the keyframes that actually saw it.
    for (const FrontendPendingObservation & pending : promotion.observations) {
      g2o::VertexSE2 * pending_vertex = nullptr;
      for (auto it = poses_.rbegin(); it != poses_.rend(); ++it) {
        if (it->graph_id == pending.pose_graph_id) {
          pending_vertex = it->vertex;
          break;
        }
        if (it->graph_id < pending.pose_graph_id) {
          break;
        }
      }
      if (pending_vertex == nullptr) {
        continue;  // keyframe pruned from the localization window
      }
      ConeObservation replay;
      replay.measurement = pending.keyframe_measurement;
      replay.covariance = pending.keyframe_covariance;
      replay.color = static_cast<ConeColor>(promotion.color);
      addObservationEdge(replay, pending_vertex, *landmark);
      ++added_edges;
    }
    observed_landmark_indices.push_back(landmarks_.size() - 1U);
  }

  if (loop_closure_edge) {
    if (keyframes_since_last_optimization_ < optimize_every_n_keyframes_) {
      keyframes_since_last_optimization_ = optimize_every_n_keyframes_;
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Loop closure confirmed by candidate gate; pulling graph optimization forward "
        "loop_candidates=%zu loop_confirmed=%zu loop_rejected=%zu",
        loop_candidate_count_,
        loop_confirmed_count_,
        loop_rejected_count_);
    }
  }

  const std::size_t deleted_landmarks = update_deletions ?
    deleteMissedVisibleLandmarks(pose, observed_landmark_indices) :
    0U;

  RCLCPP_DEBUG(
    get_logger(),
    "Added %zu cone observation edges, updated %zu landmarks from %zu visible cones; "
    "deleted %zu stale landmarks",
    added_edges,
    updated_landmarks,
    observations.size(),
    deleted_landmarks);

  return ObservationUpdate{added_edges, updated_landmarks, deleted_landmarks, matched_landmarks};
}

std::vector<GraphSlamNode::ConeObservation> GraphSlamNode::extractConeObservations(
  const hyu_msgs::msg::ConeArrayWithCovariance & msg) const
{
  std::vector<ConeObservation> observations;

  const auto append_cones =
    [this, &observations](
    const auto & cones,
    ConeColor color)
    {
      for (const hyu_msgs::msg::ConeWithCovariance & cone : cones) {
        if (!std::isfinite(cone.point.x) || !std::isfinite(cone.point.y)) {
          continue;
        }

        const double range = std::hypot(cone.point.x, cone.point.y);
        if (range < min_observation_range_ || range > max_observation_range_) {
          continue;
        }

        ConeObservation observation;
        observation.measurement = Eigen::Vector2d(cone.point.x, cone.point.y);
        observation.covariance = covarianceFromCone(cone);
        observation.color = color;
        observations.push_back(observation);
      }
    };

  append_cones(msg.blue_cones, ConeColor::Blue);
  append_cones(msg.yellow_cones, ConeColor::Yellow);
  append_cones(msg.orange_cones, ConeColor::Orange);
  append_cones(msg.big_orange_cones, ConeColor::BigOrange);
  append_cones(msg.unknown_color_cones, ConeColor::Unknown);

  return observations;
}

Eigen::Matrix2d GraphSlamNode::covarianceFromCone(
  const hyu_msgs::msg::ConeWithCovariance & cone) const
{
  const auto default_covariance =
    Eigen::Matrix2d::Identity() * default_observation_sigma_ * default_observation_sigma_;

  if (!use_cone_covariance_) {
    return default_covariance;
  }

  Eigen::Matrix2d covariance;
  covariance << cone.covariance[0], cone.covariance[1],
    cone.covariance[2], cone.covariance[3];

  if (!covariance.allFinite()) {
    return default_covariance;
  }

  covariance = (0.5 * (covariance + covariance.transpose())).eval();
  covariance(0, 0) = std::max(covariance(0, 0), min_observation_variance_);
  covariance(1, 1) = std::max(covariance(1, 1), min_observation_variance_);

  if (covariance.determinant() <= min_observation_variance_ * min_observation_variance_) {
    return default_covariance;
  }

  return covariance;
}

Eigen::Matrix2d GraphSlamNode::covarianceInMapFrame(
  const g2o::SE2 & pose,
  const Eigen::Matrix2d & local_covariance) const
{
  const double yaw = pose.rotation().angle();
  Eigen::Matrix2d rotation;
  rotation << std::cos(yaw), -std::sin(yaw),
    std::sin(yaw), std::cos(yaw);

  Eigen::Matrix2d covariance = rotation * local_covariance * rotation.transpose();
  covariance = (0.5 * (covariance + covariance.transpose())).eval();
  covariance(0, 0) = std::max(covariance(0, 0), min_observation_variance_);
  covariance(1, 1) = std::max(covariance(1, 1), min_observation_variance_);

  if (!covariance.allFinite() ||
    covariance.determinant() <= min_observation_variance_ * min_observation_variance_)
  {
    return Eigen::Matrix2d::Identity() * default_observation_sigma_ * default_observation_sigma_;
  }

  return covariance;
}

int GraphSlamNode::findAssociatedLandmark(
  const Eigen::Vector2d & map_point,
  const Eigen::Matrix2d & map_covariance,
  ConeColor color,
  bool * ambiguous,
  const std::vector<bool> * claimed) const
{
  if (ambiguous != nullptr) {
    *ambiguous = false;
  }


  double best_d2 = std::numeric_limits<double>::max();
  double second_d2 = std::numeric_limits<double>::max();
  double best_euclidean_sq = std::numeric_limits<double>::max();
  int best_index = -1;
  int second_index = -1;

  // Association is geometric only. Colour is tracked by majority vote and
  // deliberately not used as a gate: simulated perception mislabels cones,
  // and a colour gate would lock those errors in by rejecting the very
  // observations that could fix them. Cone spacing (>= 3 m) is far larger
  // than the association gate, so colour adds no discrimination here.
  (void)color;

  for (std::size_t i = 0; i < landmarks_.size(); ++i) {
    // In-frame exclusivity: a landmark already matched by another cone in
    // this frame cannot absorb a second one (two adjacent cones must not
    // collapse into a single landmark).
    if (claimed != nullptr && i < claimed->size() && (*claimed)[i]) {
      continue;
    }

    const LandmarkRecord & landmark = landmarks_[i];
    const Eigen::Vector2d diff = landmark.vertex->estimate() - map_point;

    // Pose drift since the landmark was last seen widens its gate; this is
    // what lets lap-closure re-associations succeed despite accumulated
    // odometry error. Drift grows with distance driven, so the inflation is
    // per meter (keyframe density must not change the gate).
    const double meters_unseen =
      std::max(0.0, traveled_distance_ - landmark.last_seen_traveled);
    const double inflation = std::min(
      association_max_inflation_,
      association_inflation_per_meter_ * meters_unseen);

    // Coarse pre-gate to skip the matrix inverse for distant landmarks.
    const double gate_radius = association_max_distance_ + 3.0 * std::sqrt(inflation);
    if (diff.squaredNorm() > gate_radius * gate_radius) {
      continue;
    }

    Eigen::Matrix2d innovation_covariance =
      landmark.covariance + map_covariance + Eigen::Matrix2d::Identity() * inflation;
    const double det = innovation_covariance.determinant();
    if (!innovation_covariance.allFinite() || det <= 0.0) {
      continue;
    }

    const double d2 = diff.dot(innovation_covariance.inverse() * diff);
    if (d2 < best_d2) {
      second_d2 = best_d2;
      second_index = best_index;
      best_d2 = d2;
      best_index = static_cast<int>(i);
      best_euclidean_sq = diff.squaredNorm();
    } else if (d2 < second_d2) {
      second_d2 = d2;
      second_index = static_cast<int>(i);
    }
  }

  if (best_index < 0) {
    return -1;
  }

  // Accept on the chi-square gate, with the legacy Euclidean radius as a
  // fallback so freshly-tracked landmarks with tight covariances still match.
  const bool chi2_ok = best_d2 <= association_gate_chi2_;
  const bool euclidean_ok =
    best_euclidean_sq <= association_max_distance_ * association_max_distance_;
  if (!chi2_ok && !euclidean_ok) {
    return -1;
  }

  if (ambiguous != nullptr &&
    second_d2 <= association_gate_chi2_ &&
    std::sqrt(best_d2) > association_ambiguity_ratio_ * std::sqrt(second_d2))
  {
    // Two landmarks explain this cone almost equally well. If they are close
    // to EACH OTHER, they are two duplicate landmarks of ONE physical cone
    // (drift spawned a second one), not two distinct cones — so this is not a
    // real ambiguity. Rejecting it would refuse every revisit near a duplicate
    // pair, which starves loop-closure confirmation (loop_rejected piles up,
    // loop_confirmed stays 0), the map never converges, mapping never ends,
    // and each extra lap adds more duplicates. Associate with the best instead
    // and let mergeCloseLandmarks collapse the pair after the next optimize.
    const double rival_separation = second_index < 0 ? std::numeric_limits<double>::max() :
      (landmarks_[best_index].vertex->estimate() -
      landmarks_[second_index].vertex->estimate()).norm();
    if (rival_separation > association_max_distance_) {
      *ambiguous = true;
      return -1;
    }
  }

  return best_index;
}

bool GraphSlamNode::colorsCompatible(ConeColor observation_color, ConeColor landmark_color) const
{
  return observation_color == ConeColor::Unknown ||
         landmark_color == ConeColor::Unknown ||
         observation_color == landmark_color;
}

void GraphSlamNode::voteLandmarkColor(LandmarkRecord & landmark, ConeColor observed_color)
{
  if (observed_color == ConeColor::Unknown) {
    return;
  }

  auto & votes = landmark.color_votes[static_cast<std::size_t>(observed_color)];
  if (votes < std::numeric_limits<std::uint16_t>::max()) {
    ++votes;
  }

  std::size_t best = 0U;
  for (std::size_t i = 1U; i < landmark.color_votes.size(); ++i) {
    if (landmark.color_votes[i] > landmark.color_votes[best]) {
      best = i;
    }
  }
  if (landmark.color_votes[best] > 0U) {
    landmark.color = static_cast<ConeColor>(best);
  }
}

GraphSlamNode::LandmarkRecord * GraphSlamNode::addLandmark(
  const Eigen::Vector2d & map_point,
  const Eigen::Matrix2d & covariance,
  ConeColor color)
{
  if (localization_mode_) {
    // Localization only: the map is fixed, never grow it.
    return nullptr;
  }

  if (max_landmarks_ > 0 &&
    landmarks_.size() >= static_cast<std::size_t>(max_landmarks_))
  {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "Maximum graph SLAM landmark count reached; ignoring new cone landmarks");
    return nullptr;
  }

  auto * vertex = new g2o::VertexPointXY();
  vertex->setId(next_vertex_id_++);
  vertex->setEstimate(map_point);
  vertex->setMarginalized(true);

  if (!optimizer_.addVertex(vertex)) {
    RCLCPP_ERROR(get_logger(), "Failed to add landmark vertex %d", vertex->id());
    delete vertex;
    return nullptr;
  }

  landmarks_.push_back(
    LandmarkRecord{vertex->id(), color, vertex, covariance, 0U, 0, traveled_distance_, {}});
  voteLandmarkColor(landmarks_.back(), color);
  return &landmarks_.back();
}

bool GraphSlamNode::updateLandmarkEstimate(
  LandmarkRecord & landmark,
  const Eigen::Vector2d & map_point,
  const Eigen::Matrix2d & covariance)
{
  // In localization mode the loaded map is fixed: setFixed only freezes the
  // g2o optimizer, but this Kalman fusion writes vertex->setEstimate directly,
  // so it must be skipped or a drifted observation would move the map.
  if (localization_mode_ || !update_existing_landmarks_ || landmark_update_gain_ <= 0.0) {
    return false;
  }

  Eigen::Matrix2d prior_covariance = landmark.covariance;
  prior_covariance = (0.5 * (prior_covariance + prior_covariance.transpose())).eval();
  prior_covariance(0, 0) = std::max(prior_covariance(0, 0), min_observation_variance_);
  prior_covariance(1, 1) = std::max(prior_covariance(1, 1), min_observation_variance_);

  if (landmark_update_process_variance_ > 0.0) {
    prior_covariance +=
      Eigen::Matrix2d::Identity() * landmark_update_process_variance_;
  }

  const Eigen::Matrix2d innovation_covariance = prior_covariance + covariance;
  if (!innovation_covariance.allFinite() ||
    innovation_covariance.determinant() <= min_observation_variance_ * min_observation_variance_)
  {
    return false;
  }

  const Eigen::Matrix2d kalman_gain = prior_covariance * innovation_covariance.inverse();
  const Eigen::Vector2d estimate = landmark.vertex->estimate();
  const Eigen::Vector2d update = landmark_update_gain_ * (kalman_gain * (map_point - estimate));
  const Eigen::Vector2d updated_estimate = estimate + update;

  if (!updated_estimate.allFinite()) {
    return false;
  }

  landmark.vertex->setEstimate(updated_estimate);

  const Eigen::Matrix2d identity = Eigen::Matrix2d::Identity();
  Eigen::Matrix2d updated_covariance =
    (identity - landmark_update_gain_ * kalman_gain) * prior_covariance;
  updated_covariance = (0.5 * (updated_covariance + updated_covariance.transpose())).eval();
  updated_covariance(0, 0) = std::max(updated_covariance(0, 0), min_observation_variance_);
  updated_covariance(1, 1) = std::max(updated_covariance(1, 1), min_observation_variance_);
  landmark.covariance = updated_covariance;

  return update.squaredNorm() > 1e-8;
}

void GraphSlamNode::addObservationEdge(
  const ConeObservation & observation,
  g2o::VertexSE2 * pose_vertex,
  LandmarkRecord & landmark)
{
  auto * edge = new g2o::EdgeSE2PointXY();
  edge->setId(next_edge_id_++);
  edge->setVertex(0, pose_vertex);
  edge->setVertex(1, landmark.vertex);
  edge->setMeasurement(observation.measurement);

  Eigen::Matrix2d information = observation.covariance.inverse();
  if (map_converged_ &&
    map_trust_info_scale_ > 1.0 &&
    landmark_confirm_observations_ > 0 &&
    landmark.observations >= static_cast<std::size_t>(landmark_confirm_observations_))
  {
    // The map has converged (loop closures reconciled) and this landmark is
    // confirmed. Trust its observation more so the pose conforms to the
    // settled map. The landmark already carries many edges, so it barely
    // moves; the pose is what gets pulled.
    information *= map_trust_info_scale_;
  }
  edge->setInformation(information);

  // DCS (default): the closed-form switchable constraint. A wrong
  // association's edge is smoothly down-weighted by s = min(1, 2*phi /
  // (phi + chi2)) instead of dragging the map; correct edges (chi2 <= phi)
  // are untouched. Requires a warm start, which the incremental optimize
  // cadence and the gate-anchor re-seed provide; the optimizer is Levenberg,
  // which tolerates DCS's redescending (negative-curvature) outlier region.
  // Odometry and GNSS-prior edges keep their own kernels — never DCS.
  if (observation_robust_kernel_ == "dcs" && observation_dcs_phi_ > 0.0) {
    auto * robust_kernel = new g2o::RobustKernelDCS();
    robust_kernel->setDelta(observation_dcs_phi_);
    edge->setRobustKernel(robust_kernel);
  } else if (observation_robust_kernel_ == "huber" && robust_kernel_delta_ > 0.0) {
    auto * robust_kernel = new g2o::RobustKernelHuber();
    robust_kernel->setDelta(robust_kernel_delta_);
    edge->setRobustKernel(robust_kernel);
  }

  if (!optimizer_.addEdge(edge)) {
    RCLCPP_ERROR(get_logger(), "Failed to add cone observation edge");
    delete edge;
    return;
  }

  ++landmark.observations;
  landmark.consecutive_misses = 0;
}

std::size_t GraphSlamNode::deleteMissedVisibleLandmarks(
  const PoseRecord & pose,
  const std::vector<std::size_t> & observed_landmark_indices)
{
  if (!delete_stale_landmarks_ || localization_mode_ || landmarks_.empty()) {
    return 0U;
  }

  std::vector<bool> observed(landmarks_.size(), false);
  for (const std::size_t landmark_index : observed_landmark_indices) {
    if (landmark_index < observed.size()) {
      observed[landmark_index] = true;
    }
  }

  std::vector<std::size_t> delete_indices;
  for (std::size_t i = 0; i < landmarks_.size(); ++i) {
    LandmarkRecord & landmark = landmarks_[i];
    if (observed[i]) {
      landmark.consecutive_misses = 0;
      continue;
    }

    if (!landmarkExpectedVisible(pose, landmark)) {
      continue;
    }

    // Confirmed landmarks survive occlusions and short perception dropouts
    // (deleting them would discard their loop-closure constraints), but a
    // 10x miss budget still clears drift-era ghost duplicates that are
    // never observed again.
    const bool confirmed = landmark_confirm_observations_ > 0 &&
      landmark.observations >= static_cast<std::size_t>(landmark_confirm_observations_);
    const int delete_threshold = confirmed ?
      10 * landmark_missed_observations_to_delete_ :
      landmark_missed_observations_to_delete_;

    ++landmark.consecutive_misses;
    if (landmark.consecutive_misses >= delete_threshold) {
      delete_indices.push_back(i);
    }
  }

  std::size_t deleted_landmarks = 0U;
  for (auto it = delete_indices.rbegin(); it != delete_indices.rend(); ++it) {
    if (removeLandmarkAt(*it)) {
      ++deleted_landmarks;
    }
  }

  if (deleted_landmarks > 0U) {
    RCLCPP_INFO(
      get_logger(),
      "Deleted %zu stale landmarks from the g2o graph; %zu landmarks remain",
      deleted_landmarks,
      landmarks_.size());
  }

  return deleted_landmarks;
}

bool GraphSlamNode::landmarkExpectedVisible(
  const PoseRecord & pose,
  const LandmarkRecord & landmark) const
{
  const Eigen::Vector2d relative =
    pose.vertex->estimate().inverse() * landmark.vertex->estimate();
  const double range = relative.norm();
  if (range < min_observation_range_ || range > landmark_delete_max_range_) {
    return false;
  }

  if (landmark_delete_max_abs_x_ > 0.0 &&
    std::abs(relative.x()) > landmark_delete_max_abs_x_)
  {
    return false;
  }
  if (landmark_delete_max_abs_y_ > 0.0 &&
    std::abs(relative.y()) > landmark_delete_max_abs_y_)
  {
    return false;
  }

  if (landmark_delete_fov_ > 0.0 && landmark_delete_fov_ < 2.0 * std::acos(-1.0)) {
    const double bearing = std::abs(std::atan2(relative.y(), relative.x()));
    if (bearing > 0.5 * landmark_delete_fov_) {
      return false;
    }
  }

  return true;
}

bool GraphSlamNode::removeLandmarkAt(std::size_t landmark_index)
{
  if (landmark_index >= landmarks_.size()) {
    return false;
  }

  LandmarkRecord & landmark = landmarks_[landmark_index];
  const int graph_id = landmark.graph_id;
  const std::size_t removed_edges = landmark.vertex->edges().size();

  if (optimizer_.vertex(graph_id) != landmark.vertex) {
    RCLCPP_ERROR(
      get_logger(),
      "Refusing to delete landmark vertex %d because optimizer bookkeeping is inconsistent",
      graph_id);
    return false;
  }

  if (!optimizer_.removeVertex(landmark.vertex)) {
    RCLCPP_ERROR(get_logger(), "Failed to delete landmark vertex %d from g2o graph", graph_id);
    return false;
  }

  landmarks_.erase(landmarks_.begin() + static_cast<std::ptrdiff_t>(landmark_index));

  RCLCPP_DEBUG(
    get_logger(),
    "Deleted landmark vertex %d and %zu connected observation edges from g2o graph",
    graph_id,
    removed_edges);
  return true;
}

bool GraphSlamNode::shouldUpdateLandmarkDeletion(const rclcpp::Time & stamp, bool force_update)
{
  if (!delete_stale_landmarks_) {
    return false;
  }

  if (force_update || landmark_delete_min_interval_ <= 0.0) {
    last_landmark_delete_time_sec_ = stamp.seconds();
    return true;
  }

  const double stamp_sec = stamp.seconds();
  if (last_landmark_delete_time_sec_ < 0.0 ||
    stamp_sec < last_landmark_delete_time_sec_ ||
    stamp_sec - last_landmark_delete_time_sec_ >= landmark_delete_min_interval_)
  {
    last_landmark_delete_time_sec_ = stamp_sec;
    return true;
  }

  return false;
}

std::size_t GraphSlamNode::mergeCloseLandmarks()
{
  return mergeCloseLandmarks(landmark_merge_distance_, 0.0);
}

std::size_t GraphSlamNode::mergeCloseLandmarks(
  const double merge_distance, const double min_last_seen_gap_m)
{
  if (localization_mode_ || merge_distance <= 0.0 || landmarks_.size() < 2U) {
    return 0U;
  }

  const double merge_distance_sq = merge_distance * merge_distance;
  std::size_t merged = 0U;

  for (std::size_t i = 0; i < landmarks_.size(); ++i) {
    for (std::size_t j = i + 1U; j < landmarks_.size(); ) {
      // Big orange start-line cones legitimately stand ~0.4 m apart in
      // pairs; merging them would destroy the start-line geometry.
      if (landmarks_[i].color == ConeColor::BigOrange ||
        landmarks_[j].color == ConeColor::BigOrange ||
        !colorsCompatible(landmarks_[i].color, landmarks_[j].color) ||
        (landmarks_[i].vertex->estimate() - landmarks_[j].vertex->estimate()).squaredNorm() >
        merge_distance_sq)
      {
        ++j;
        continue;
      }
      if (min_last_seen_gap_m > 0.0 &&
        std::abs(landmarks_[i].last_seen_traveled - landmarks_[j].last_seen_traveled) <
        min_last_seen_gap_m)
      {
        // Both members were observed recently: two real adjacent cones, not a
        // drift-era duplicate. Only the plain (small-radius) merge may touch
        // such pairs.
        ++j;
        continue;
      }

      if (min_last_seen_gap_m > 0.0) {
        // Drift duplicate: keep the RECENTLY seen member — its position
        // reflects the loop-closure-corrected pose, the stale twin's carries
        // the drift error that created the duplicate in the first place.
        if (landmarks_[j].last_seen_traveled > landmarks_[i].last_seen_traveled) {
          std::swap(landmarks_[i], landmarks_[j]);
        }
      } else if (landmarks_[j].observations > landmarks_[i].observations) {
        // Keep the landmark with more observations; it carries more edges.
        std::swap(landmarks_[i], landmarks_[j]);
      }
      LandmarkRecord & kept = landmarks_[i];
      LandmarkRecord & dropped = landmarks_[j];

      // Re-anchor the dropped landmark's observation edges onto the kept
      // vertex so its constraints survive the merge.
      const g2o::HyperGraph::EdgeSet dropped_edges = dropped.vertex->edges();
      for (g2o::HyperGraph::Edge * edge : dropped_edges) {
        auto * observation_edge = dynamic_cast<g2o::EdgeSE2PointXY *>(edge);
        if (observation_edge == nullptr) {
          continue;
        }
        auto * pose_vertex = dynamic_cast<g2o::VertexSE2 *>(observation_edge->vertex(0));
        if (pose_vertex == nullptr) {
          continue;
        }

        auto * new_edge = new g2o::EdgeSE2PointXY();
        new_edge->setId(next_edge_id_++);
        new_edge->setVertex(0, pose_vertex);
        new_edge->setVertex(1, kept.vertex);
        new_edge->setMeasurement(observation_edge->measurement());
        new_edge->setInformation(observation_edge->information());
        if (robust_kernel_delta_ > 0.0) {
          auto * robust_kernel = new g2o::RobustKernelHuber();
          robust_kernel->setDelta(robust_kernel_delta_);
          new_edge->setRobustKernel(robust_kernel);
        }
        if (!optimizer_.addEdge(new_edge)) {
          delete new_edge;
        }
      }

      kept.observations += dropped.observations;
      kept.consecutive_misses = 0;
      kept.last_seen_traveled =
        std::max(kept.last_seen_traveled, dropped.last_seen_traveled);
      for (std::size_t k = 0; k < kept.color_votes.size(); ++k) {
        const std::uint32_t total = static_cast<std::uint32_t>(kept.color_votes[k]) +
          static_cast<std::uint32_t>(dropped.color_votes[k]);
        kept.color_votes[k] = static_cast<std::uint16_t>(
          std::min<std::uint32_t>(total, std::numeric_limits<std::uint16_t>::max()));
      }
      std::size_t best_vote = 0U;
      for (std::size_t k = 1U; k < kept.color_votes.size(); ++k) {
        if (kept.color_votes[k] > kept.color_votes[best_vote]) {
          best_vote = k;
        }
      }
      if (kept.color_votes[best_vote] > 0U) {
        kept.color = static_cast<ConeColor>(best_vote);
      } else if (kept.color == ConeColor::Unknown) {
        kept.color = dropped.color;
      }

      if (removeLandmarkAt(j)) {
        ++merged;
      } else {
        ++j;
      }
    }
  }

  return merged;
}

void GraphSlamNode::recordRawOdometry(double stamp_sec, const g2o::SE2 & raw_odom)
{
  std::lock_guard<std::mutex> lock(odom_buffer_mutex_);

  if (!raw_odom_buffer_.empty() && stamp_sec < raw_odom_buffer_.back().first) {
    // Time went backwards (sim reset or bag loop); the buffer is stale.
    raw_odom_buffer_.clear();
  }

  raw_odom_buffer_.emplace_back(stamp_sec, raw_odom);

  const double horizon = std::max(5.0, 2.0 * keyframe_max_dt_);
  while (raw_odom_buffer_.size() > 1U &&
    raw_odom_buffer_.front().first < stamp_sec - horizon)
  {
    raw_odom_buffer_.pop_front();
  }
  while (raw_odom_buffer_.size() > 4000U) {
    raw_odom_buffer_.pop_front();
  }
}

g2o::SE2 GraphSlamNode::rawOdomAt(double stamp_sec) const
{
  std::lock_guard<std::mutex> lock(odom_buffer_mutex_);

  if (raw_odom_buffer_.empty()) {
    return poses_.empty() ? g2o::SE2() : poses_.back().raw_odom;
  }
  if (stamp_sec <= raw_odom_buffer_.front().first) {
    return raw_odom_buffer_.front().second;
  }
  if (stamp_sec >= raw_odom_buffer_.back().first) {
    return raw_odom_buffer_.back().second;
  }

  const auto upper = std::lower_bound(
    raw_odom_buffer_.begin(),
    raw_odom_buffer_.end(),
    stamp_sec,
    [](const std::pair<double, g2o::SE2> & sample, double value) {
      return sample.first < value;
    });
  const auto lower = std::prev(upper);

  const double t0 = lower->first;
  const double t1 = upper->first;
  if (t1 - t0 <= 1e-9) {
    return upper->second;
  }

  const double alpha = (stamp_sec - t0) / (t1 - t0);
  const Eigen::Vector2d translation =
    (1.0 - alpha) * lower->second.translation() + alpha * upper->second.translation();
  const double yaw0 = lower->second.rotation().angle();
  const double yaw = yaw0 +
    alpha * normalizeAngle(upper->second.rotation().angle() - yaw0);
  return g2o::SE2(translation.x(), translation.y(), normalizeAngle(yaw));
}

void GraphSlamNode::maybeOptimize()
{
  if (keyframes_since_last_optimization_ < optimize_every_n_keyframes_) {
    return;
  }

  if (max_optimization_poses_ > 0 &&
    poses_.size() > static_cast<std::size_t>(max_optimization_poses_))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Skipping graph optimization with %zu poses; max_optimization_poses is %d",
      poses_.size(),
      max_optimization_poses_);
    optimizer_skipped_pose_limit_ = true;
    publishLifecycleDiagnostics();
    keyframes_since_last_optimization_ = 0;
    return;
  }

  const double stamp_sec = poses_.empty() ? 0.0 : poses_.back().stamp.seconds();
  if (optimize_min_interval_ > 0.0 &&
    last_optimization_time_sec_ >= 0.0 &&
    stamp_sec >= last_optimization_time_sec_ &&
    stamp_sec - last_optimization_time_sec_ < optimize_min_interval_)
  {
    return;
  }

  if (!optimizeGraph()) {
    return;
  }
  optimizer_skipped_pose_limit_ = false;
  const std::size_t merged = mergeCloseLandmarks();
  if (merged > 0U) {
    RCLCPP_INFO(
      get_logger(),
      "Merged %zu duplicate landmarks after optimization; %zu landmarks remain",
      merged,
      landmarks_.size());
  }

  // Count optimization cycles that reconciled a loop closure. After enough of
  // them the map is treated as converged and confirmed-landmark observations
  // are trusted more (see addObservationEdge).
  if (map_trust_after_loop_closure_ &&
    !map_converged_ &&
    loop_confirmation_ready_for_optimize_)
  {
    loop_confirmation_ready_for_optimize_ = false;
    ++loop_confirmed_count_;
    ++loop_closure_optimize_cycles_;
    if (loop_closure_optimize_cycles_ >= map_trust_loop_closures_required_) {
      map_converged_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Map converged after %d loop-closure optimization cycle(s); "
        "boosting confirmed-landmark observation weight by %.1fx "
        "loop_candidates=%zu loop_confirmed=%zu loop_rejected=%zu",
        loop_closure_optimize_cycles_,
        map_trust_info_scale_,
        loop_candidate_count_,
        loop_confirmed_count_,
        loop_rejected_count_);
      publishStatus();
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Loop confirmed by optimization but waiting for map-trust threshold "
        "loop_candidates=%zu loop_confirmed=%zu loop_rejected=%zu required=%d",
        loop_candidate_count_,
        loop_confirmed_count_,
        loop_rejected_count_,
        map_trust_loop_closures_required_);
      publishLifecycleDiagnostics();
    }
  }

  keyframes_since_last_optimization_ = 0;
  last_optimization_time_sec_ = stamp_sec;

  // Optimization moved the keyframes: refresh the dead-reckoning snapshot
  // and let subscribers see the corrected map.
  updateKeyframeSnapshot();
  if (!poses_.empty()) {
    publishGraphVisuals(poses_.back().stamp);
  }
}

void GraphSlamNode::updateKeyframeSnapshot()
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  if (poses_.empty()) {
    keyframe_snapshot_.valid = false;
    return;
  }

  keyframe_snapshot_.estimate = poses_.back().vertex->estimate();
  keyframe_snapshot_.raw_odom = poses_.back().raw_odom;
  keyframe_snapshot_.valid = true;
}

void GraphSlamNode::publishLiveEstimateFromSnapshot(
  const rclcpp::Time & stamp,
  const g2o::SE2 & raw_odom)
{
  KeyframeSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot = keyframe_snapshot_;
  }
  if (!snapshot.valid) {
    return;
  }

  const g2o::SE2 live_estimate =
    snapshot.estimate * (snapshot.raw_odom.inverse() * raw_odom);
  publishLiveEstimate(stamp, live_estimate, raw_odom);
}

bool GraphSlamNode::optimizeGraph()
{
  if (poses_.size() < 2U || optimizer_.edges().empty()) {
    return false;
  }

  const g2o::SE2 pose_before = poses_.back().vertex->estimate();
  // A landmark whose observation edges the DCS kernel has all switched off
  // is effectively unconstrained: one LM step can fling its vertex hundreds
  // of meters (2026-07-18 autopsy: ghosts at 100-322 m with clean odometry
  // and no creation-guard hits). Snapshot positions so impossible per-step
  // jumps can be culled after the solve — no physical cone moves 10 m in
  // one optimization cycle (legit loop-closure corrections are meters).
  std::vector<Eigen::Vector2d> landmarks_before;
  landmarks_before.reserve(landmarks_.size());
  for (const LandmarkRecord & landmark : landmarks_) {
    landmarks_before.push_back(landmark.vertex->estimate());
  }
  optimizer_.initializeOptimization();
  const int completed_iterations = optimizer_.optimize(optimization_iterations_);
  for (std::size_t i = landmarks_.size(); i-- > 0U;) {
    if (i < landmarks_before.size() &&
      (landmarks_[i].vertex->estimate() - landmarks_before[i]).norm() > 10.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "Culling landmark flung %.0f m by one optimize step (unconstrained "
        "vertex — all edges likely switched off)",
        (landmarks_[i].vertex->estimate() - landmarks_before[i]).norm());
      removeLandmarkAt(i);
    }
  }
  // Correction-step autopsy: every optimizer run that moves the live pose by
  // centimeters is a step the planner (and every in-flight observation)
  // experiences as a jump. This is the number that explains "the map and the
  // car twitch" — keep it visible and correlate it with duplicate births.
  const double correction_m =
    (poses_.back().vertex->estimate().translation() - pose_before.translation()).norm();
  if (correction_m > 0.05) {
    RCLCPP_WARN(
      get_logger(),
      "optimizer moved the live pose %.3f m in one step "
      "(%d iters, %zu poses, %zu landmarks, converged=%d)",
      correction_m, completed_iterations, poses_.size(), landmarks_.size(),
      map_converged_ ? 1 : 0);
  }
  return completed_iterations > 0;
}

std::size_t GraphSlamNode::firstPublishedPoseIndex() const
{
  if (path_max_poses_to_publish_ <= 0 ||
    poses_.size() <= static_cast<std::size_t>(path_max_poses_to_publish_))
  {
    return 0U;
  }

  return poses_.size() - static_cast<std::size_t>(path_max_poses_to_publish_);
}

bool GraphSlamNode::shouldPublishVisuals(const rclcpp::Time & stamp)
{
  const double stamp_sec = stamp.seconds();
  if (visual_publish_min_interval_ <= 0.0 ||
    last_visual_publish_time_sec_ < 0.0 ||
    stamp_sec < last_visual_publish_time_sec_ ||
    stamp_sec - last_visual_publish_time_sec_ >= visual_publish_min_interval_)
  {
    last_visual_publish_time_sec_ = stamp_sec;
    return true;
  }

  return false;
}

void GraphSlamNode::publishEstimate()
{
  if (poses_.empty()) {
    return;
  }

  const rclcpp::Time stamp = poses_.back().stamp;
  const g2o::SE2 estimate = poses_.back().vertex->estimate();
  publishGraphVisuals(stamp);
  publishLiveEstimate(stamp, estimate, poses_.back().raw_odom);
}

void GraphSlamNode::publishGraphVisuals(const rclcpp::Time & stamp)
{
  if (shouldPublishVisuals(stamp)) {
    publishMap(stamp);
    publishPath(stamp);
    publishMarkers(stamp);
  }
}

void GraphSlamNode::publishLiveEstimate(
  const rclcpp::Time & stamp,
  const g2o::SE2 & estimate,
  const g2o::SE2 & raw_odom)
{
  last_live_odom_publish_stamp_sec_ = stamp.seconds();
  publishOdometry(stamp, estimate);
  publishTransform(stamp, estimate, raw_odom);
}

void GraphSlamNode::publishMap(const rclcpp::Time & stamp)
{
  hyu_msgs::msg::ConeArrayWithCovariance msg;
  msg.header.frame_id = map_frame_;
  msg.header.stamp = stamp;

  for (const LandmarkRecord & landmark : landmarks_) {
    if (landmark.observations <
      static_cast<std::size_t>(landmark_min_observations_to_publish_))
    {
      continue;
    }

    hyu_msgs::msg::ConeWithCovariance cone;
    const Eigen::Vector2d estimate = landmark.vertex->estimate();
    cone.point.x = estimate.x();
    cone.point.y = estimate.y();
    cone.point.z = 0.0;
    cone.covariance = {
      landmark.covariance(0, 0),
      landmark.covariance(0, 1),
      landmark.covariance(1, 0),
      landmark.covariance(1, 1)};

    switch (landmark.color) {
      case ConeColor::Blue:
        msg.blue_cones.push_back(cone);
        break;
      case ConeColor::Yellow:
        msg.yellow_cones.push_back(cone);
        break;
      case ConeColor::Orange:
        msg.orange_cones.push_back(cone);
        break;
      case ConeColor::BigOrange:
        msg.big_orange_cones.push_back(cone);
        break;
      case ConeColor::Unknown:
        msg.unknown_color_cones.push_back(cone);
        break;
    }
  }

  map_pub_->publish(msg);
}

void GraphSlamNode::publishPath(const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map_frame_;
  path.header.stamp = stamp;
  const std::size_t first_pose_index = firstPublishedPoseIndex();
  path.poses.reserve(poses_.size() - first_pose_index);

  for (std::size_t i = first_pose_index; i < poses_.size(); ++i) {
    const PoseRecord & pose_record = poses_[i];
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = pose_record.stamp;

    const g2o::SE2 estimate = pose_record.vertex->estimate();
    pose.pose.position.x = estimate.translation().x();
    pose.pose.position.y = estimate.translation().y();
    pose.pose.position.z = 0.0;
    pose.pose.orientation = quaternionFromYaw(estimate.rotation().angle());

    path.poses.push_back(pose);
  }

  path_pub_->publish(path);
}

void GraphSlamNode::publishOdometry(const rclcpp::Time & stamp, const g2o::SE2 & estimate)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = map_frame_;
  odom.header.stamp = stamp;
  odom.child_frame_id = slam_base_frame_;
  odom.pose.pose.position.x = estimate.translation().x();
  odom.pose.pose.position.y = estimate.translation().y();
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternionFromYaw(estimate.rotation().angle());

  // Honest, state-tiered pose covariance: tight once the pose is anchored to
  // a converged/loaded map, loose while mapping is still unconverged.
  // (Benign unsynchronized reads of the mode flags.)
  const bool anchored = localization_mode_ || map_converged_;
  const double sigma_xy = anchored ? 0.05 : 0.30;
  const double sigma_yaw = anchored ? 0.03 : 0.15;
  odom.pose.covariance[0] = sigma_xy * sigma_xy;
  odom.pose.covariance[7] = sigma_xy * sigma_xy;
  odom.pose.covariance[35] = sigma_yaw * sigma_yaw;

  // Body twist passthrough from the motion input for downstream controllers.
  odom.twist.twist.linear.x = latest_twist_vx_.load();
  odom.twist.twist.linear.y = latest_twist_vy_.load();
  odom.twist.twist.angular.z = latest_twist_wz_.load();
  odom.twist.covariance[0] = 0.01;
  odom.twist.covariance[7] = 0.01;
  odom.twist.covariance[35] = 0.02;

  odom_pub_->publish(odom);
}

void GraphSlamNode::publishMarkers(const rclcpp::Time & stamp)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear_marker);

  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = map_frame_;
  path_marker.header.stamp = stamp;
  path_marker.ns = "pose_graph";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = 0.06;
  path_marker.color.r = 1.0;
  path_marker.color.g = 1.0;
  path_marker.color.b = 1.0;
  path_marker.color.a = 0.85;

  for (std::size_t i = firstPublishedPoseIndex(); i < poses_.size(); ++i) {
    const PoseRecord & pose_record = poses_[i];
    geometry_msgs::msg::Point point;
    const g2o::SE2 estimate = pose_record.vertex->estimate();
    point.x = estimate.translation().x();
    point.y = estimate.translation().y();
    point.z = 0.05;
    path_marker.points.push_back(point);
  }
  markers.markers.push_back(path_marker);

  const std::array<ConeColor, 5> colors = {
    ConeColor::Blue,
    ConeColor::Yellow,
    ConeColor::Orange,
    ConeColor::BigOrange,
    ConeColor::Unknown};

  // Landmarks render as the same 3D cone meshes Gazebo uses on the track so
  // the SLAM map is visually comparable to the simulated world. Every mesh is
  // tinted with an explicit solid colour (embedded materials off) — the .dae
  // materials do not render reliably in RViz (most come out white).
  const auto meshForColor = [](ConeColor color) -> std::string {
      switch (color) {
        case ConeColor::Blue:
          return "package://eufs_tracks/meshes/cone_blue.dae";
        case ConeColor::Yellow:
          return "package://eufs_tracks/meshes/cone_yellow.dae";
        case ConeColor::BigOrange:
          return "package://eufs_tracks/meshes/cone_big.dae";
        case ConeColor::Orange:
        case ConeColor::Unknown:
        default:
          return "package://eufs_tracks/meshes/cone.dae";
      }
    };

  for (ConeColor color : colors) {
    const std::string mesh_resource = meshForColor(color);
    const std::string ns = colorName(color) + "_landmarks";
    int cone_id = 0;

    for (const LandmarkRecord & landmark : landmarks_) {
      if (landmark.color != color ||
        landmark.observations <
        static_cast<std::size_t>(landmark_min_observations_to_publish_))
      {
        continue;
      }

      visualization_msgs::msg::Marker landmark_marker;
      landmark_marker.header.frame_id = map_frame_;
      landmark_marker.header.stamp = stamp;
      landmark_marker.ns = ns;
      landmark_marker.id = cone_id++;
      landmark_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      landmark_marker.action = visualization_msgs::msg::Marker::ADD;
      landmark_marker.mesh_resource = mesh_resource;
      landmark_marker.mesh_use_embedded_materials = false;
      const Eigen::Vector2d estimate = landmark.vertex->estimate();
      landmark_marker.pose.position.x = estimate.x();
      landmark_marker.pose.position.y = estimate.y();
      landmark_marker.pose.position.z = 0.0;
      landmark_marker.pose.orientation.w = 1.0;
      landmark_marker.scale.x = 1.0;
      landmark_marker.scale.y = 1.0;
      landmark_marker.scale.z = 1.0;
      landmark_marker.color = colorToRgba(color, 0.95);
      markers.markers.push_back(landmark_marker);
    }
  }

  marker_pub_->publish(markers);
}

void GraphSlamNode::publishTransform(
  const rclcpp::Time & stamp,
  const g2o::SE2 & estimate,
  const g2o::SE2 & raw_odom)
{
  if (!publish_tf_ || !tf_broadcaster_) {
    return;
  }

  const auto make_transform =
    [&stamp](
    const std::string & parent_frame,
    const std::string & child_frame,
    const g2o::SE2 & transform)
    {
      geometry_msgs::msg::TransformStamped msg;
      msg.header.frame_id = parent_frame;
      msg.header.stamp = stamp;
      msg.child_frame_id = child_frame;
      msg.transform.translation.x = transform.translation().x();
      msg.transform.translation.y = transform.translation().y();
      msg.transform.translation.z = 0.0;
      msg.transform.rotation = GraphSlamNode::quaternionFromYaw(transform.rotation().angle());
      return msg;
    };

  const g2o::SE2 map_to_odom = estimate * raw_odom.inverse();
  const std::array<geometry_msgs::msg::TransformStamped, 2> transforms = {
    make_transform(map_frame_, odom_frame_, map_to_odom),
    make_transform(odom_frame_, slam_base_frame_, raw_odom)};
  tf_broadcaster_->sendTransform(transforms[0]);
  tf_broadcaster_->sendTransform(transforms[1]);
}

void GraphSlamNode::handleReset(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  resetGraph();
  publishStatus();
  response->success = true;
  response->message = "Graph SLAM state reset";
}

void GraphSlamNode::handleSaveGraph(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  if (g2o_output_path_.empty()) {
    response->success = false;
    response->message = "g2o_output_path parameter is empty";
    return;
  }

  std::lock_guard<std::mutex> lock(graph_mutex_);
  response->success = optimizer_.save(g2o_output_path_.c_str());
  response->message = response->success ?
    "Saved graph to " + g2o_output_path_ :
    "Failed to save graph to " + g2o_output_path_;
}

std::string GraphSlamNode::saveMapTimestamped()
{
  // Timestamped filename: map_YYYYmmdd_HHMMSS.csv in map_save_dir_.
  // Caller must hold graph_mutex_.
  const std::time_t now = std::time(nullptr);
  std::tm tm_buf{};
  localtime_r(&now, &tm_buf);
  std::ostringstream name;
  name << "map_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".csv";
  const std::string dir = map_save_dir_.empty() ? std::string("/tmp") : map_save_dir_;
  const std::string path = dir + "/" + name.str();

  std::string error;
  if (!saveMapCsv(path, &error)) {
    RCLCPP_ERROR(
      get_logger(), "Failed to save map to %s: %s", path.c_str(), error.c_str());
    return std::string();
  }
  lifecycle_map_saved_ = true;
  RCLCPP_INFO(get_logger(), "Saved cone map to %s", path.c_str());
  return path;
}

void GraphSlamNode::handleSaveMap(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  std::lock_guard<std::mutex> lock(graph_mutex_);
  const std::string path = saveMapTimestamped();
  response->success = !path.empty();
  response->message = response->success ?
    "Saved map to " + path :
    "Failed to save map (see node log)";
}

void GraphSlamNode::handleLoadMap(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  // The GUI sets the load_map_path parameter before calling this.
  const std::string path = get_parameter("load_map_path").as_string();
  if (path.empty()) {
    response->success = false;
    response->message = "load_map_path parameter is empty";
    return;
  }

  std::lock_guard<std::mutex> lock(graph_mutex_);
  resetGraph();
  localization_mode_ = true;
  load_map_path_ = path;
  std::string error;
  if (loadMapCsv(path, &error)) {
    const rclcpp::Time now = get_clock()->now();
    publishMap(now);
    publishMarkers(now);
    response->success = true;
    response->message = "Localizing against " + path;
    RCLCPP_INFO(
      get_logger(), "Switched to localization against %s (%zu cones)",
      path.c_str(), landmarks_.size());
  } else {
    localization_mode_ = false;
    response->success = false;
    response->message = "Failed to load map: " + error;
  }
  publishStatus();
}

void GraphSlamNode::handleStartMapping(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  resetGraph();
  localization_mode_ = false;
  response->success = true;
  response->message = "Mapping mode (SLAM) started";
  RCLCPP_INFO(get_logger(), "Switched to mapping (SLAM) mode");
  publishStatus();
}

void GraphSlamNode::maybeFinishMappingLap(const g2o::SE2 & current_estimate)
{
  if (!auto_localization_after_lap_ || !lap_origin_captured_ || landmarks_.empty()) {
    return;
  }

  // The origin pose is captured while the car is standing on it, so the
  // radius/yaw check is trivially satisfied in that same pose update. Only a
  // return after real travel counts as a lap.
  if (traveled_distance_ - lap_origin_capture_traveled_m_ < lap_return_min_travel_m_) {
    return;
  }

  // Arming requires the vehicle to be INSIDE the origin gates right now; once
  // armed, the finish is latched and the dwell below proceeds even as the car
  // drives back out of the radius.
  if (!lap_finish_gate_.armed()) {
    const double return_distance =
      (current_estimate.translation() - lap_origin_.translation()).norm();
    const double yaw_error = std::abs(
      normalizeAngle(
        current_estimate.rotation().angle() - lap_origin_.rotation().angle()));
    RCLCPP_DEBUG_THROTTLE(
      get_logger(),
      *get_clock(),
      10000,
      "Lap check: return=%.2f m (radius %.1f), yaw=%.2f rad",
      return_distance, lap_return_radius_, yaw_error);
    if (return_distance > lap_return_radius_ || yaw_error > lap_return_yaw_) {
      return;
    }
    if (!lap_return_criteria_satisfied_) {
      lap_return_criteria_satisfied_ = true;
      // The verified return is independent geometric evidence of a loop:
      // optionally require fewer co-located candidates to confirm one. The
      // residual gates are never relaxed.
      if (loop_confirmation_required_candidates_on_lap_return_ > 0 &&
        !loop_confirmation_relaxed_on_lap_return_)
      {
        loop_confirmation_relaxed_on_lap_return_ = true;
        loop_confirmation_window_.setRequiredCandidates(
          static_cast<std::size_t>(loop_confirmation_required_candidates_on_lap_return_));
        RCLCPP_INFO(
          get_logger(),
          "Lap return corroborates a loop closure: confirmation threshold "
          "relaxed %zu -> %d candidate(s)",
          loop_confirmation_config_.required_candidates,
          loop_confirmation_required_candidates_on_lap_return_);
      }
    }
  }

  const bool was_armed = lap_finish_gate_.armed();
  const LapFinishState state = lap_finish_gate_.evaluate(
    lap_return_criteria_satisfied_, map_converged_,
    seam_loop_candidate_count_, traveled_distance_);
  switch (state) {
    case LapFinishState::WaitingReturn:
      return;
    case LapFinishState::GatedByConvergence:
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Mapping lap return gated by loop confirmation: mapping_stop_reason=%s "
        "loop_candidates=%zu loop_confirmed=%zu loop_rejected=%zu last_loop_reason=%s",
        toString(classifyMappingStopState()),
        loop_candidate_count_,
        loop_confirmed_count_,
        loop_rejected_count_,
        toString(last_loop_confirmation_reason_));
      publishLifecycleDiagnostics();
      return;
    case LapFinishState::GatedBySeam:
      // Converged on the credentials of SOME loop (possibly a mid-lap
      // mini-loop); the freeze waits for the loop that closes the lap.
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Mapping lap return gated by seam closure: seam_candidates=%zu "
        "required=%d (loop_candidates=%zu loop_confirmed=%zu)",
        seam_loop_candidate_count_,
        lap_seam_candidates_required_,
        loop_candidate_count_,
        loop_confirmed_count_);
      publishLifecycleDiagnostics();
      return;
    case LapFinishState::Dwelling:
      if (!was_armed) {
        RCLCPP_INFO(
          get_logger(),
          "Mapping lap finish armed (seam_candidates=%zu); dwelling %.1f m to "
          "accumulate seam constraints before freezing",
          seam_loop_candidate_count_,
          lap_finish_dwell_m_);
      }
      return;
    case LapFinishState::Finished:
      RCLCPP_INFO(
        get_logger(),
        "Mapping lap complete: converged map, seam_candidates=%zu, dwell %.1f m",
        seam_loop_candidate_count_,
        lap_finish_dwell_m_);
      enterLocalizationMode("mapping lap completed");
      return;
  }
}

void GraphSlamNode::enterLocalizationMode(const std::string & reason)
{
  // Final clean-up before the map freezes. The lap-closing loop closure has
  // just pulled the drift-era duplicate landmarks (a physical cone re-mapped a
  // second time while the pose estimate was still drifting) close together, so
  // one more optimization to settle them followed by a merge pass removes the
  // overlapping/doubled cones from the map that is about to become the fixed
  // reference. Both are gated off by localization_mode_, so they must run now.
  optimizeGraph();
  // Drift-era duplicates first: pairs the lap-closing correction pulled to
  // 1-2 m apart — above the everyday merge radius, and distance alone cannot
  // separate them from REAL adjacent cones (tight corners pack same-color
  // cones down to ~1 m). The recency gate can: the stale twin stopped being
  // observed the moment association missed under drift.
  std::size_t merged_on_freeze = 0U;
  if (freeze_merge_stale_distance_m_ > 0.0) {
    merged_on_freeze +=
      mergeCloseLandmarks(freeze_merge_stale_distance_m_, loop_gap_distance_);
  }
  merged_on_freeze += mergeCloseLandmarks();
  if (merged_on_freeze > 0U) {
    RCLCPP_INFO(
      get_logger(),
      "Merged %zu duplicate landmark(s) while freezing the map; %zu remain",
      merged_on_freeze, landmarks_.size());
  }
  // Unpromoted tracks die with the mapping phase: the frozen map accepts no
  // new landmarks, so keeping them would only absorb observations that the
  // localization matcher needs.
  if (frontend_) {
    frontend_->reset();
  }

  // Freeze the map: landmarks become the fixed reference, mapping paths
  // (addLandmark / deletion / merge / Kalman updates) are disabled by the
  // localization_mode_ guards, and the pose window is bounded so the graph
  // no longer grows with laps.
  localization_mode_ = true;
  for (LandmarkRecord & landmark : landmarks_) {
    landmark.vertex->setFixed(true);
  }

  const std::string saved_path = saveMapTimestamped();
  prunePoseWindow();
  publishStatus();

  RCLCPP_INFO(
    get_logger(),
    "Entered localization mode (%s): %zu landmarks frozen, pose window %d%s%s",
    reason.c_str(),
    landmarks_.size(),
    localization_window_poses_,
    saved_path.empty() ? "" : ", map saved to ",
    saved_path.c_str());
}

void GraphSlamNode::prunePoseWindow()
{
  if (!localization_mode_ ||
    poses_.size() <= static_cast<std::size_t>(localization_window_poses_))
  {
    // Even without pruning, keep the oldest pose anchored so a window with
    // few cone matches cannot leave the solver without a gauge.
    if (localization_mode_ && !poses_.empty()) {
      poses_.front().vertex->setFixed(true);
    }
    return;
  }

  while (poses_.size() > static_cast<std::size_t>(localization_window_poses_)) {
    PoseRecord & oldest = poses_.front();
    if (optimizer_.vertex(oldest.graph_id) == oldest.vertex) {
      // Removes the vertex together with its odometry/observation/prior
      // edges; landmarks are fixed so no map information is lost.
      optimizer_.removeVertex(oldest.vertex);
    }
    poses_.erase(poses_.begin());
  }

  // The new oldest pose carries the discarded history as a hard anchor
  // (first-order marginalization).
  poses_.front().vertex->setFixed(true);
}

MappingStopReason GraphSlamNode::classifyMappingStopState()
{
  const double now_sec = get_clock()->now().seconds();
  const double freshness_sec = std::max(2.0, 2.0 * keyframe_max_dt_);
  const auto fresh =
    [now_sec, freshness_sec](double stamp_sec)
    {
      if (stamp_sec < 0.0) {
        return false;
      }
      if (now_sec <= 0.0 || stamp_sec <= 0.0 || stamp_sec > now_sec) {
        return true;
      }
      return now_sec - stamp_sec <= freshness_sec;
    };

  MappingStopClassifierInput input;
  input.localization_mode = localization_mode_;
  input.map_converged = map_converged_;
  input.map_saved = lifecycle_map_saved_;
  input.lap_return_criteria_satisfied = lap_return_criteria_satisfied_;
  input.loop_confirmed = loop_confirmed_count_ > 0U;
  input.loop_pending =
    !map_converged_ && (loop_candidate_window_count_ > 0U || lap_return_criteria_satisfied_);
  input.ambiguous_association =
    last_loop_confirmation_reason_ == LoopConfirmationReason::AmbiguousAssociation;
  input.stable_map_geometry = map_converged_ && !landmarks_.empty();
  input.optimizer_skipped_pose_limit = optimizer_skipped_pose_limit_;
  input.odometry_fresh = fresh(last_odom_stamp_sec_);
  input.cones_fresh = last_cone_stamp_sec_ < 0.0 || fresh(last_cone_stamp_sec_);
  input.live_odometry_published = last_live_odom_publish_stamp_sec_ >= 0.0;
  input.map_updates_stalled =
    last_map_update_stamp_sec_ >= 0.0 &&
    last_live_odom_publish_stamp_sec_ > last_map_update_stamp_sec_ + freshness_sec;
  return classifyMappingStop(input);
}

void GraphSlamNode::publishLifecycleDiagnostics()
{
  if (!lifecycle_diagnostics_pub_) {
    return;
  }

  const MappingStopReason reason = classifyMappingStopState();
  std_msgs::msg::String diagnostics;
  std::ostringstream out;
  out << "mapping_stop_reason=" << toString(reason)
      << " loop_candidates=" << loop_candidate_count_
      << " loop_candidate_window=" << loop_candidate_window_count_
      << " loop_confirmed=" << loop_confirmed_count_
      << " loop_rejected=" << loop_rejected_count_
      << " last_loop_reason=" << toString(last_loop_confirmation_reason_)
      << " map_converged=" << (map_converged_ ? "true" : "false")
      << " localization_mode=" << (localization_mode_ ? "true" : "false")
      << " optimizer_skipped_pose_limit="
      << (optimizer_skipped_pose_limit_ ? "true" : "false")
      << " lap_return_criteria_satisfied="
      << (lap_return_criteria_satisfied_ ? "true" : "false")
      << " odometry_fresh="
      << (classifyMappingStopState() == MappingStopReason::OdometryDropout ? "false" : "true")
      << " lost_frames=" << lost_frames_
      << " auto_relocalizations=" << auto_relocalize_count_
      << " gate_anchors=" << gate_anchor_count_;
  diagnostics.data = out.str();
  lifecycle_diagnostics_pub_->publish(diagnostics);
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    5000,
    "SLAM lifecycle diagnostics: %s",
    diagnostics.data.c_str());
}

void GraphSlamNode::publishStatus()
{
  if (!status_pub_ || !converged_pub_) {
    return;
  }

  std_msgs::msg::String status;
  if (localization_mode_) {
    status.data = "localization";
  } else if (map_converged_) {
    status.data = "mapping_converged";
  } else {
    status.data = "mapping";
  }
  status_pub_->publish(status);

  std_msgs::msg::Bool converged;
  converged.data = map_converged_ || (localization_mode_ && !landmarks_.empty());
  converged_pub_->publish(converged);
  publishLifecycleDiagnostics();
}

GraphSlamNode::ConeColor GraphSlamNode::colorFromTag(const std::string & tag)
{
  if (tag == "blue") {
    return ConeColor::Blue;
  }
  if (tag == "yellow") {
    return ConeColor::Yellow;
  }
  if (tag == "orange") {
    return ConeColor::Orange;
  }
  if (tag == "big_orange") {
    return ConeColor::BigOrange;
  }
  return ConeColor::Unknown;
}

bool GraphSlamNode::loadMapCsv(const std::string & path, std::string * error)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    if (error != nullptr) {
      *error = "could not open file for reading";
    }
    return false;
  }

  std::string line;
  std::getline(file, line);  // header
  std::size_t loaded = 0U;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    std::istringstream ss(line);
    std::string tag, sx, sy, sdir, svx, svy, sxy;
    std::getline(ss, tag, ',');
    std::getline(ss, sx, ',');
    std::getline(ss, sy, ',');
    std::getline(ss, sdir, ',');
    std::getline(ss, svx, ',');
    std::getline(ss, svy, ',');
    std::getline(ss, sxy, ',');
    if (tag == "car_start" || sx.empty() || sy.empty()) {
      continue;
    }

    double x = 0.0, y = 0.0, vx = default_observation_sigma_ * default_observation_sigma_;
    double vy = vx, xy = 0.0;
    try {
      x = std::stod(sx);
      y = std::stod(sy);
      if (!svx.empty()) {vx = std::stod(svx);}
      if (!svy.empty()) {vy = std::stod(svy);}
      if (!sxy.empty()) {xy = std::stod(sxy);}
    } catch (const std::exception &) {
      continue;
    }

    auto * vertex = new g2o::VertexPointXY();
    vertex->setId(next_vertex_id_++);
    vertex->setEstimate(Eigen::Vector2d(x, y));
    vertex->setFixed(true);  // the loaded map is the fixed reference
    if (!optimizer_.addVertex(vertex)) {
      delete vertex;
      continue;
    }

    Eigen::Matrix2d cov;
    cov << vx, xy, xy, vy;
    if (!cov.allFinite() || cov(0, 0) <= 0.0 || cov(1, 1) <= 0.0) {
      cov = Eigen::Matrix2d::Identity() * default_observation_sigma_ * default_observation_sigma_;
    }

    // Seed with enough observations to count as confirmed and be published.
    const std::size_t seed_obs =
      static_cast<std::size_t>(std::max(landmark_confirm_observations_, 1));
    LandmarkRecord record{vertex->id(), colorFromTag(tag), vertex, cov, seed_obs, 0, 0.0, {}};
    landmarks_.push_back(record);
    voteLandmarkColor(landmarks_.back(), colorFromTag(tag));
    ++loaded;
  }

  if (loaded == 0U) {
    if (error != nullptr) {
      *error = "no landmarks parsed from file";
    }
    return false;
  }
  return true;
}

bool GraphSlamNode::saveMapCsv(const std::string & path, std::string * error) const
{
  std::ofstream file(path);
  if (!file.is_open()) {
    if (error != nullptr) {
      *error = "could not open file for writing";
    }
    return false;
  }

  // Same columns as eufs_tracks CSVs so saved maps are interchangeable with
  // track files. direction is only meaningful for car_start.
  file << "tag,x,y,direction,x_variance,y_variance,xy_covariance\n";
  file << std::fixed << std::setprecision(6);

  // Record the map origin (SLAM initialises the map frame at the car start).
  file << "car_start,0.0,0.0,0.0,0.0,0.0,0.0\n";

  std::size_t written = 0U;
  for (const LandmarkRecord & landmark : landmarks_) {
    if (landmark.observations <
      static_cast<std::size_t>(landmark_min_observations_to_publish_))
    {
      continue;
    }
    const Eigen::Vector2d estimate = landmark.vertex->estimate();
    file << colorName(landmark.color) << ','
         << estimate.x() << ',' << estimate.y() << ",0.0,"
         << landmark.covariance(0, 0) << ','
         << landmark.covariance(1, 1) << ','
         << landmark.covariance(0, 1) << '\n';
    ++written;
  }

  file.close();
  if (!file) {
    if (error != nullptr) {
      *error = "write error while flushing";
    }
    return false;
  }
  RCLCPP_INFO(get_logger(), "Wrote %zu landmarks to map CSV", written);
  return true;
}

double GraphSlamNode::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double GraphSlamNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion GraphSlamNode::quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

rclcpp::Time GraphSlamNode::stampOrNow(
  const builtin_interfaces::msg::Time & stamp,
  const rclcpp::Clock::SharedPtr & clock)
{
  if (stamp.sec == 0 && stamp.nanosec == 0U) {
    return clock->now();
  }

  return rclcpp::Time(stamp, clock->get_clock_type());
}

std_msgs::msg::ColorRGBA GraphSlamNode::colorToRgba(ConeColor color, double alpha)
{
  std_msgs::msg::ColorRGBA rgba;
  rgba.a = static_cast<float>(alpha);

  switch (color) {
    case ConeColor::Blue:
      rgba.r = 0.1F;
      rgba.g = 0.35F;
      rgba.b = 1.0F;
      break;
    case ConeColor::Yellow:
      rgba.r = 1.0F;
      rgba.g = 0.85F;
      rgba.b = 0.1F;
      break;
    case ConeColor::Orange:
      rgba.r = 1.0F;
      rgba.g = 0.45F;
      rgba.b = 0.05F;
      break;
    case ConeColor::BigOrange:
      rgba.r = 1.0F;
      rgba.g = 0.2F;
      rgba.b = 0.0F;
      break;
    case ConeColor::Unknown:
      rgba.r = 0.8F;
      rgba.g = 0.8F;
      rgba.b = 0.8F;
      break;
  }

  return rgba;
}

std::string GraphSlamNode::colorName(ConeColor color)
{
  switch (color) {
    case ConeColor::Blue:
      return "blue";
    case ConeColor::Yellow:
      return "yellow";
    case ConeColor::Orange:
      return "orange";
    case ConeColor::BigOrange:
      return "big_orange";
    case ConeColor::Unknown:
      return "unknown";
  }

  return "unknown";
}

}  // namespace hyu_localization
