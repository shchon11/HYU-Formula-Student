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
#include <g2o/types/slam2d/edge_se2_prior.h>
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
  min_observation_range_(0.2),
  max_observation_range_(30.0),
  default_observation_sigma_(0.25),
  min_observation_variance_(0.01),
  odom_translation_sigma_(0.05),
  odom_yaw_sigma_(0.03),
  robust_kernel_delta_(1.0),
  marker_scale_(0.3),
  landmark_update_gain_(1.0),
  landmark_update_process_variance_(0.04),
  optimize_every_n_keyframes_(100),
  optimization_iterations_(3),
  landmark_min_observations_to_publish_(1),
  max_landmarks_(400),
  max_optimization_poses_(100),
  path_max_poses_to_publish_(1000),
  landmark_confirm_observations_(3),
  loop_gap_distance_(20.0),
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
  update_existing_landmarks_(true),
  optimize_min_interval_(10.0),
  visual_publish_min_interval_(0.5),
  tf_stamp_offset_(0.0),
  last_optimization_time_sec_(-1.0),
  last_visual_publish_time_sec_(-1.0),
  next_vertex_id_(0),
  next_edge_id_(0),
  keyframes_since_last_optimization_(0),
  last_cone_pose_graph_id_(-1),
  map_converged_(false),
  loop_confirmation_ready_for_optimize_(false),
  loop_candidate_count_(0U),
  loop_confirmed_count_(0U),
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
  frontend_params_.promote_far_min_hits = declare_parameter<int>(
    "track_promote_far_min_hits", frontend_params_.promote_far_min_hits);
  frontend_params_.promote_far_max_sigma_m = declare_parameter<double>(
    "track_promote_far_max_sigma", frontend_params_.promote_far_max_sigma_m);
  frontend_params_.promote_hold_radius_m = declare_parameter<double>(
    "track_promote_hold_radius", frontend_params_.promote_hold_radius_m);
  frontend_params_.promote_hold_stale_travel_m = declare_parameter<double>(
    "track_promote_hold_stale_travel", frontend_params_.promote_hold_stale_travel_m);
  frontend_params_.promote_hold_drift_stale_m = declare_parameter<double>(
    "track_promote_hold_drift_stale", frontend_params_.promote_hold_drift_stale_m);
  frontend_params_.kill_consecutive_misses = declare_parameter<int>(
    "track_kill_consecutive_misses", frontend_params_.kill_consecutive_misses);
  frontend_params_.kill_unpromoted_travel_m = declare_parameter<double>(
    "track_kill_unpromoted_travel", frontend_params_.kill_unpromoted_travel_m);
  // Robust kernel on observation edges (huber | none).
  observation_robust_kernel_ = declare_parameter<std::string>(
    "observation_robust_kernel", observation_robust_kernel_);
  relocalize_search_radius_ =
    declare_parameter<double>("relocalize_search_radius", relocalize_search_radius_);
  relocalize_search_yaw_ =
    declare_parameter<double>("relocalize_search_yaw", relocalize_search_yaw_);
  relocalize_inlier_distance_ =
    declare_parameter<double>("relocalize_inlier_distance", relocalize_inlier_distance_);

  association_inflation_per_meter_ =
    declare_parameter<double>(
    "association_inflation_per_meter",
    association_inflation_per_meter_);
  association_max_inflation_ =
    declare_parameter<double>("association_max_inflation", association_max_inflation_);
  min_observation_range_ =
    declare_parameter<double>("min_observation_range", min_observation_range_);
  max_observation_range_ =
    declare_parameter<double>("max_observation_range", max_observation_range_);
  track_visible_max_range_ = declare_parameter<double>(
    "track_visible_max_range", track_visible_max_range_);
  track_visible_fov_ = declare_parameter<double>(
    "track_visible_fov", track_visible_fov_);
  csm_enable_ = declare_parameter<bool>("csm_enable", csm_enable_);
  csm_params_.resolution = declare_parameter<double>(
    "csm_resolution", csm_params_.resolution);
  csm_params_.smear_sigma = declare_parameter<double>(
    "csm_smear_sigma", csm_params_.smear_sigma);
  csm_params_.coarse_response_min = declare_parameter<double>(
    "csm_coarse_response_min", csm_params_.coarse_response_min);
  csm_params_.fine_response_min = declare_parameter<double>(
    "csm_fine_response_min", csm_params_.fine_response_min);
  csm_params_.min_query_points = declare_parameter<int>(
    "csm_min_query_points", csm_params_.min_query_points);
  csm_track_window_m_ = declare_parameter<double>(
    "csm_track_window_m", csm_track_window_m_);
  csm_track_window_theta_ = declare_parameter<double>(
    "csm_track_window_theta", csm_track_window_theta_);
  csm_loop_window_m_ = declare_parameter<double>(
    "csm_loop_window_m", csm_loop_window_m_);
  csm_loop_window_theta_ = declare_parameter<double>(
    "csm_loop_window_theta", csm_loop_window_theta_);
  csm_max_sigma_xy_ = declare_parameter<double>(
    "csm_max_sigma_xy", csm_max_sigma_xy_);
  csm_max_sigma_theta_ = declare_parameter<double>(
    "csm_max_sigma_theta", csm_max_sigma_theta_);
  csm_min_interval_m_ = declare_parameter<double>(
    "csm_min_interval_m", csm_min_interval_m_);
  csm_peak_margin_ = declare_parameter<double>(
    "csm_peak_margin", csm_peak_margin_);
  csm_loop_min_orange_span_m_ = declare_parameter<double>(
    "csm_loop_min_orange_span_m", csm_loop_min_orange_span_m_);
  csm_loop_min_response_ = declare_parameter<double>(
    "csm_loop_min_response", csm_loop_min_response_);
  csm_apply_cooldown_m_ = declare_parameter<double>(
    "csm_apply_cooldown_m", csm_apply_cooldown_m_);
  csm_loop_min_orange_response_ = declare_parameter<double>(
    "csm_loop_min_orange_response", csm_loop_min_orange_response_);
  csm_loop_min_rail_response_ = declare_parameter<double>(
    "csm_loop_min_rail_response", csm_loop_min_rail_response_);
  submap_dedup_orange_m_ = declare_parameter<double>(
    "submap_dedup_orange_m", submap_dedup_orange_m_);
  csm_params_.orange_weight = declare_parameter<double>(
    "csm_orange_weight", csm_params_.orange_weight);
  csm_orange_max_obs_sigma_ = declare_parameter<double>(
    "csm_orange_max_obs_sigma", csm_orange_max_obs_sigma_);
  submap_enable_ = declare_parameter<bool>("submap_enable", submap_enable_);
  submap_span_m_ = declare_parameter<double>("submap_span_m", submap_span_m_);
  submap_max_frames_ = declare_parameter<int>(
    "submap_max_frames", submap_max_frames_);
  submap_dedup_radius_m_ = declare_parameter<double>(
    "submap_dedup_radius_m", submap_dedup_radius_m_);
  submap_min_match_points_ = declare_parameter<int>(
    "submap_min_match_points", submap_min_match_points_);
  {
    // The declared values must actually reach the submap (it was default-
    // constructed before this block existed).
    LocalSubmapParams submap_params;
    submap_params.span_m = submap_span_m_;
    submap_params.max_frames = static_cast<std::size_t>(
      std::max(1, submap_max_frames_));
    submap_params.dedup_radius_m = submap_dedup_radius_m_;
    submap_params.dedup_radius_orange_m = submap_dedup_orange_m_;
    submap_ = LocalConeSubmap(submap_params);
  }
  landmark_delete_enable_ = declare_parameter<bool>(
    "landmark_delete_enable", landmark_delete_enable_);
  landmark_delete_misses_ = std::max(
    3, static_cast<int>(declare_parameter<int>(
      "landmark_delete_misses", landmark_delete_misses_)));
  landmark_delete_max_range_ = declare_parameter<double>(
    "landmark_delete_max_range", landmark_delete_max_range_);
  landmark_delete_fov_ = declare_parameter<double>(
    "landmark_delete_fov", landmark_delete_fov_);
  loc_map_repair_min_hits_ = declare_parameter<int>(
    "loc_map_repair_min_hits", loc_map_repair_min_hits_);
  loc_map_repair_delete_misses_ = declare_parameter<int>(
    "loc_map_repair_delete_misses", loc_map_repair_delete_misses_);
  default_observation_sigma_ =
    declare_parameter<double>("default_observation_sigma", default_observation_sigma_);
  min_observation_variance_ =
    declare_parameter<double>("min_observation_variance", min_observation_variance_);
  odom_translation_sigma_ =
    declare_parameter<double>("odom_translation_sigma", odom_translation_sigma_);
  odom_yaw_sigma_ = declare_parameter<double>("odom_yaw_sigma", odom_yaw_sigma_);
  robust_kernel_delta_ = declare_parameter<double>("robust_kernel_delta", robust_kernel_delta_);
  marker_scale_ = declare_parameter<double>("marker_scale", marker_scale_);
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
  landmark_confirm_observations_ =
    declare_parameter<int>(
    "landmark_confirm_observations",
    landmark_confirm_observations_);
  loop_gap_distance_ = declare_parameter<double>("loop_gap_distance", loop_gap_distance_);

  optimize_min_interval_ =
    declare_parameter<double>("optimize_min_interval", optimize_min_interval_);
  visual_publish_min_interval_ =
    declare_parameter<double>("visual_publish_min_interval", visual_publish_min_interval_);
  tf_stamp_offset_ = declare_parameter<double>("tf_stamp_offset", tf_stamp_offset_);

  localization_mode_ = declare_parameter<bool>("localization_mode", localization_mode_);
  auto_localization_after_lap_ =
    declare_parameter<bool>("auto_localization_after_lap", auto_localization_after_lap_);
  lap_return_min_travel_m_ = declare_parameter<double>(
    "lap_return_min_travel_m", lap_return_min_travel_m_);
  lap_return_radius_ = declare_parameter<double>("lap_return_radius", lap_return_radius_);
  lap_return_yaw_ = declare_parameter<double>("lap_return_yaw", lap_return_yaw_);
  localization_window_poses_ =
    declare_parameter<int>("localization_window_poses", localization_window_poses_);
  lap_returns_to_freeze_ = std::max(
    1, static_cast<int>(declare_parameter<int>(
      "lap_returns_to_freeze", lap_returns_to_freeze_)));
  lap_origin_capture_distance_ =
    declare_parameter<double>(
    "lap_origin_capture_distance",
    lap_origin_capture_distance_);
  use_odom_covariance_ =
    declare_parameter<bool>("use_odom_covariance", use_odom_covariance_);
  odom_invalid_sigma_ = std::max(
    1.0, declare_parameter<double>("odom_invalid_sigma", odom_invalid_sigma_));
  odom_edge_sigma_per_meter_ = std::max(
    0.0, declare_parameter<double>(
      "odom_edge_sigma_per_meter", odom_edge_sigma_per_meter_));
  odom_edge_yaw_sigma_per_meter_ = std::max(
    0.0, declare_parameter<double>(
      "odom_edge_yaw_sigma_per_meter", odom_edge_yaw_sigma_per_meter_));
  load_map_path_ = declare_parameter<std::string>("load_map_path", "");
  use_cone_covariance_ = declare_parameter<bool>("use_cone_covariance", use_cone_covariance_);
  process_every_cone_message_ =
    declare_parameter<bool>("process_every_cone_message", process_every_cone_message_);
  publish_tf_ = declare_parameter<bool>("publish_tf", publish_tf_);
  update_existing_landmarks_ =
    declare_parameter<bool>("update_existing_landmarks", update_existing_landmarks_);

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
  landmark_confirm_observations_ = std::max(0, landmark_confirm_observations_);
  loop_gap_distance_ = std::max(0.0, loop_gap_distance_);
  lap_return_min_travel_m_ = std::max(0.0, lap_return_min_travel_m_);
  lap_origin_capture_distance_ = std::max(1.0, lap_origin_capture_distance_);
  lap_return_radius_ = std::max(0.5, lap_return_radius_);
  lap_return_yaw_ = std::max(0.05, lap_return_yaw_);
  localization_window_poses_ = std::max(10, localization_window_poses_);
  min_observation_range_ = std::max(0.0, min_observation_range_);
  max_observation_range_ = std::max(min_observation_range_, max_observation_range_);
  default_observation_sigma_ = std::max(1e-3, default_observation_sigma_);
  min_observation_variance_ = std::max(1e-6, min_observation_variance_);
  odom_translation_sigma_ = std::max(1e-4, odom_translation_sigma_);
  odom_yaw_sigma_ = std::max(1e-4, odom_yaw_sigma_);
  optimize_every_n_keyframes_ = std::max(1, optimize_every_n_keyframes_);
  optimization_iterations_ = std::max(1, optimization_iterations_);
  landmark_min_observations_to_publish_ = std::max(1, landmark_min_observations_to_publish_);
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
  deferred_promotions_.clear();
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
  map_converged_ = false;
  // Full reconstruction, not reset(): the lap-return relaxation may have
  // lowered the candidate threshold, and a mapping restart must go back to
  // the strict configured window.
  loop_confirmation_ready_for_optimize_ = false;
  loop_candidate_count_ = 0U;
  loop_confirmed_count_ = 0U;
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
  lap_origin_vertex_ = nullptr;
  last_optimization_time_sec_ = -1.0;
  last_visual_publish_time_sec_ = -1.0;
  submap_.reset();
  submap_reference_valid_ = false;
  last_csm_travel_ = -1.0e18;
  csm_track_applied_ = 0U;
  csm_loop_applied_ = 0U;
}

void GraphSlamNode::stateCallback(const hyu_msgs::msg::CarState::SharedPtr msg)
{
  const rclcpp::Time stamp = stampOrNow(msg->header.stamp, get_clock());
  const g2o::SE2 raw_odom = poseFromCarState(*msg);
  recordRawOdometry(stamp.seconds(), raw_odom);
  last_odom_stamp_sec_ = stamp.seconds();

  // Motion-source trust and twist passthrough (see the member comments).
  // A non-finite covariance is "not reported" (falls back to the config
  // sigma), never "zero" (which would read as maximally confident).
  const auto sigma_from_var =
    [](double var) {return std::isfinite(var) ? std::sqrt(std::max(0.0, var)) : 0.0;};
  latest_odom_sigma_trans_ = std::max(
    sigma_from_var(msg->pose.covariance[0]), sigma_from_var(msg->pose.covariance[7]));
  latest_odom_sigma_yaw_ = sigma_from_var(msg->pose.covariance[35]);
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
  relocalizeTo(pose);
}

g2o::SE2 GraphSlamNode::latestRawOdom() const
{
  std::lock_guard<std::mutex> lock(odom_buffer_mutex_);
  if (raw_odom_buffer_.empty()) {
    return g2o::SE2();
  }
  return raw_odom_buffer_.back().second;
}

namespace
{

GateSe2 toGateSe2(const g2o::SE2 & pose)
{
  return GateSe2{
    pose.translation().x(), pose.translation().y(), pose.rotation().angle()};
}

}  // namespace

GraphSlamNode::MatchPointSet GraphSlamNode::buildMatchPoints() const
{
  if (submap_enable_ && submap_reference_valid_) {
    std::vector<SubmapPoint> points =
      submap_.pointsInFrame(toGateSe2(submap_reference_));
    if (points.size() >= static_cast<std::size_t>(submap_min_match_points_)) {
      return MatchPointSet{std::move(points), true};
    }
  }
  MatchPointSet set;
  set.points.reserve(last_observations_.size());
  for (const ConeObservation & obs : last_observations_) {
    set.points.push_back(
      SubmapPoint{
        obs.measurement.x(), obs.measurement.y(),
        static_cast<std::uint8_t>(obs.color)});
  }
  return set;
}

std::vector<SubmapPoint> GraphSlamNode::landmarkMatchTargets(
  const Eigen::Vector2d & center, double radius,
  double max_first_seen_traveled) const
{
  std::vector<SubmapPoint> targets;
  targets.reserve(landmarks_.size());
  const double margin = radius + max_observation_range_ + submap_span_m_;
  const double margin_sq = margin * margin;
  for (const LandmarkRecord & landmark : landmarks_) {
    if (max_first_seen_traveled >= 0.0 &&
      landmark.first_seen_traveled > max_first_seen_traveled)
    {
      continue;
    }
    const Eigen::Vector2d estimate = landmark.vertex->estimate();
    if (radius > 0.0 && (estimate - center).squaredNorm() > margin_sq) {
      continue;
    }
    targets.push_back(
      SubmapPoint{
        estimate.x(), estimate.y(),
        static_cast<std::uint8_t>(landmark.color)});
  }
  return targets;
}

g2o::SE2 GraphSlamNode::gridSearchPose(
  const g2o::SE2 & seed, double radius, double xy_step,
  double yaw_span, double yaw_step, double inlier_distance,
  const std::vector<SubmapPoint> & points,
  const std::vector<SubmapPoint> & targets,
  int * best_inliers_out) const
{
  // Brute-force search over a window around the seed: for each candidate
  // pose, transform the match points into the map and count how many land
  // within inlier_distance of a color-compatible target. Best inlier count
  // (tie-broken by residual) wins.
  const int n_xy = xy_step > 0.0 ?
    static_cast<int>(std::ceil(radius / xy_step)) : 0;
  const int n_yaw = yaw_step > 0.0 ?
    static_cast<int>(std::ceil(yaw_span / yaw_step)) : 0;

  g2o::SE2 best = seed;
  int best_inliers = -1;
  double best_residual = std::numeric_limits<double>::max();

  for (int ix = -n_xy; ix <= n_xy; ++ix) {
    for (int iy = -n_xy; iy <= n_xy; ++iy) {
      for (int iw = -n_yaw; iw <= n_yaw; ++iw) {
        const GateSe2 cand{
          seed.translation().x() + ix * xy_step,
          seed.translation().y() + iy * xy_step,
          normalizeAngle(seed.rotation().angle() + iw * yaw_step)};

        const SubmapScore score =
          scoreSubmapPose(points, targets, cand, inlier_distance);
        if (score.inliers > best_inliers ||
          (score.inliers == best_inliers && score.residual_sq < best_residual))
        {
          best_inliers = score.inliers;
          best_residual = score.residual_sq;
          best = g2o::SE2(cand.x, cand.y, cand.theta);
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
  const std::vector<SubmapPoint> & points,
  const std::vector<SubmapPoint> & targets,
  int * inliers_out) const
{
  if (points.empty() || targets.empty() || radius <= 0.0) {
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
    points, targets, nullptr);

  int inliers = 0;
  const g2o::SE2 fine = gridSearchPose(
    coarse, 1.5 * coarse_step, 0.15, 1.5 * coarse_yaw_step, 0.03,
    relocalize_inlier_distance_, points, targets, &inliers);

  if (inliers_out != nullptr) {
    *inliers_out = inliers;
  }
  RCLCPP_INFO(
    get_logger(),
    "Relocalization scan-match: %d/%zu match points fit the map at the best "
    "pose (radius %.1f m)",
    inliers, points.size(), radius);
  return fine;
}

void GraphSlamNode::relocalizeTo(const g2o::SE2 & click)
{
  // Scan-match the recent cone constellation against the fixed map near the
  // click to find the best-fit starting pose. The operator asserts the car
  // is near the click, so the result is applied regardless of the inlier
  // count (the automatic path in maybeAutoRelocalize gates on it instead).
  const MatchPointSet match = buildMatchPoints();
  const std::vector<SubmapPoint> targets =
    landmarkMatchTargets(click.translation(), relocalize_search_radius_, -1.0);
  const g2o::SE2 raw_reference =
    (match.from_submap && submap_reference_valid_) ?
    submap_reference_ : latestRawOdom();
  relocalizeAt(
    scanMatchNear(
      click, relocalize_search_radius_, relocalize_search_yaw_,
      match.points, targets, nullptr),
    raw_reference);
}

void GraphSlamNode::relocalizeAt(const g2o::SE2 & pose, const g2o::SE2 & raw_reference)
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
  poses_.push_back(PoseRecord{vertex->id(), vertex, raw_reference, get_clock()->now()});

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
  // Lap accounting restarts from the asserted pose. The origin keyframe's
  // vertex was just removed with the rest of the trajectory.
  traveled_distance_ = 0.0;
  lap_origin_captured_ = false;
  lap_origin_vertex_ = nullptr;
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
  // Odometry dropout is otherwise invisible while it happens: every other
  // diagnostics publish is a lifecycle event (convergence, reset, save/load)
  // and ego_odom simply goes silent. Report the transition both ways.
  if (!poses_.empty() && last_odom_stamp_sec_ >= 0.0) {
    const bool dropout =
      classifyMappingStopState() == MappingStopReason::OdometryDropout;
    if (dropout != odom_dropout_reported_) {
      odom_dropout_reported_ = dropout;
      if (dropout) {
        RCLCPP_WARN(
          get_logger(),
          "Motion input stale (%.1f s since last CarState): ego_odom/TF frozen, "
          "cone frames dropped until it returns",
          get_clock()->now().seconds() - last_odom_stamp_sec_);
      } else {
        RCLCPP_INFO(get_logger(), "Motion input back after dropout");
      }
      publishLifecycleDiagnostics();
    }
  }
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

  // A frozen pose from a source that declares itself invalid (see
  // odom_invalid_sigma_) is not a place the car is known to be: minting a
  // keyframe on it every keyframe_max_dt would stack coincident vertices with
  // free edges under a car that may be moving. Real motion (a delta) still
  // mints -- that first post-gap keyframe carries the huge sigma on purpose.
  const bool frozen_invalid =
    latest_odom_sigma_trans_ >= odom_invalid_sigma_ && distance < 1e-3 && yaw < 1e-3;

  return distance >= keyframe_distance_ ||
         yaw >= keyframe_yaw_ ||
         (keyframe_max_dt_ > 0.0 && dt >= keyframe_max_dt_ && !frozen_invalid);
}

bool GraphSlamNode::maybeCsmRegister(
  PoseRecord & pose, const g2o::SE2 & keyframe_to_observation)
{
  (void)keyframe_to_observation;
  if (!csm_enable_) {
    return false;
  }
  if (traveled_distance_ - last_csm_travel_ < csm_min_interval_m_) {
    return false;
  }
  const MatchPointSet match_points = buildMatchPoints();
  if (static_cast<int>(match_points.points.size()) < csm_params_.min_query_points) {
    return false;
  }
  last_csm_travel_ = traveled_distance_;

  // The match points live in the body frame of the newest submap frame; its
  // map pose is the keyframe estimate carried forward by the raw-odometry
  // delta between them (at speed the difference is real).
  g2o::SE2 reference_estimate = pose.vertex->estimate();
  if (match_points.from_submap && submap_reference_valid_) {
    reference_estimate = pose.vertex->estimate() *
      (pose.raw_odom.inverse() * submap_reference_);
  }

  // LOOP mode once landmarks from at least a loop_gap of travel ago are in
  // reach: the constellation is matched against the OLD map only, over the
  // wide window, so the lap seam closes through a full-window search instead
  // of hoping per-cone NN survives the accumulated drift. TRACKING mode
  // otherwise (and always in localization mode): the full map, tight window,
  // keeps the pose continuously registered so drift never reaches the
  // association gate in the first place.
  bool loop_mode = false;
  std::vector<SubmapPoint> targets;
  // Gate-anchored seam: the loop registration is only attempted while the
  // orange gate is IN the query constellation. Blue/yellow corridors repeat
  // at uniform spacing and cannot certify a seam on their own (user-observed:
  // gate slightly out of view -> wrong seam matches); the gate is the one
  // unique feature, and the seam physically lives at the gate.
  std::vector<Eigen::Vector2d> orange_pts;
  for (const SubmapPoint & point : match_points.points) {
    if (point.color == kSubmapColorOrange ||
      point.color == kSubmapColorBigOrange)
    {
      orange_pts.emplace_back(point.x, point.y);
    }
  }
  const int orange_query_points = static_cast<int>(orange_pts.size());
  double orange_span = 0.0;
  for (std::size_t i = 0; i < orange_pts.size(); ++i) {
    for (std::size_t j = i + 1; j < orange_pts.size(); ++j) {
      orange_span = std::max(orange_span, (orange_pts[i] - orange_pts[j]).norm());
    }
  }
  // Loop mode exists to close the SEAM, and its lifetime is bounded by
  // geometry, not by flags: it arms only after a lap's worth of travel from
  // the origin (before that the submap tail still contains the DEPARTURE
  // gate, and matching it against the just-founded start cones "closes" a
  // fake seam at 25 m — measured, froze a 25% map), and it disarms after a
  // few applies (the seam is closed; old-only matching mid-lap is pure harm
  // — the csm4 thrash). map_converged is NOT the switch: a trivial stale
  // re-association sets it mid-lap-1, long before any real seam.
  const double travel_since_origin =
    traveled_distance_ - lap_origin_capture_traveled_m_;
  // Orange SPAN gate: >= 2 cones spanning at least csm_loop_min_orange_span_m.
  // A count alone is not enough — one gate side is a ~0.39 m pair that is
  // ambiguous with the other side and snaps to whichever is nearer, so a
  // single-side view can register the seam left/right mirrored. Requiring the
  // span means both sides are in view, which fixes the identity.
  if (!localization_mode_ && orange_query_points >= 2 &&
    orange_span >= csm_loop_min_orange_span_m_ &&
    lap_origin_captured_ && travel_since_origin >= lap_return_min_travel_m_ &&
    csm_loop_applied_ < 3U && loop_gap_distance_ > 0.0)
  {
    // Seam targets are the START-AREA map: cones founded within the first
    // 30 m after the origin. A rolling travel-minus-gap cutoff also admits
    // the trail just driven, and the loop then burns its apply budget on
    // trivial self-tail matches mid-lap (measured on peanut: 3 applies at
    // 100 m travel, none at the actual seam).
    targets = landmarkMatchTargets(
      reference_estimate.translation(), csm_loop_window_m_,
      lap_origin_capture_traveled_m_ + 30.0);
    loop_mode = static_cast<int>(targets.size()) >= csm_params_.min_query_points;
  }
  if (!loop_mode) {
    // Tracking runs ONLY against a FROZEN map: while mapping, a tracking
    // re-seed is an un-gated correction — the optimizer drags landmarks
    // after the re-seeded pose and the map is reshaped with zero orange
    // evidence (watched live on speedway: constant-curvature arcs alias
    // INSIDE the +-2 m window, where the second-peak check is blind, and
    // the map bent before the gate ever got its shot). Mapping is pure
    // odometry + founding; the gate seam is the one correction.
    if (!localization_mode_) {
      return false;
    }
    targets = landmarkMatchTargets(
      reference_estimate.translation(), csm_track_window_m_, -1.0);
    if (static_cast<int>(targets.size()) < csm_params_.min_query_points) {
      return false;
    }
  }
  const double window_xy = loop_mode ? csm_loop_window_m_ : csm_track_window_m_;
  const double window_theta =
    loop_mode ? csm_loop_window_theta_ : csm_track_window_theta_;

  const double half_extent = window_xy + submap_span_m_ + max_observation_range_;
  const CorrelationGrid grid(
    targets,
    reference_estimate.translation().x(),
    reference_estimate.translation().y(),
    half_extent, csm_params_);
  CsmMatch match;
  if (loop_mode) {
    // Two-stage gate-decided seam (dense tracks: 100+ corridor points
    // out-vote the gate even at 10x weight — measured on peanut, a 1.7 m /
    // 7.5 deg corridor-majority compromise passed the response floors).
    // Stage 1: the orange constellation ALONE fixes the pose over the wide
    // window; corridors get no vote. Stage 2: the full constellation may
    // refine it by at most a smear width — polish, never veto power.
    std::vector<SubmapPoint> orange_query;
    for (const SubmapPoint & point : match_points.points) {
      if (point.color == kSubmapColorOrange ||
        point.color == kSubmapColorBigOrange)
      {
        orange_query.push_back(point);
      }
    }
    CorrelationGrid::Params gate_params = csm_params_;
    gate_params.min_query_points =
      std::min<int>(3, static_cast<int>(orange_query.size()));
    // Gate stage scores CONJUNCTIVELY: every observed orange must fit or
    // the hypothesis earns ~nothing (see correlation_grid.hpp).
    gate_params.conjunctive = true;
    const CorrelationGrid gate_grid(
      targets,
      reference_estimate.translation().x(),
      reference_estimate.translation().y(),
      half_extent, gate_params);
    const CsmMatch gate_match = gate_grid.match(
      orange_query, toGateSe2(reference_estimate), window_xy, window_theta);
    // The gate rectangle is SYMMETRIC: its own constellation ties with the
    // 180-degree flip by construction (measured: response 1.00 vs second
    // 0.99, every attempt), so ambiguity here is expected, not disqualifying.
    // Both gate-approved hypotheses go to the corridor stage, where the
    // blue-left/yellow-right structure kills the flip instantly. Corridors
    // still only choose AMONG gate hypotheses — never override them.
    if (!gate_match.valid ||
      gate_match.response < csm_loop_min_orange_response_ ||
      gate_match.sigma_x > csm_max_sigma_xy_ ||
      gate_match.sigma_y > csm_max_sigma_xy_ ||
      gate_match.sigma_theta > csm_max_sigma_theta_)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "CSM loop: orange-gate stage did not lock (valid=%d response %.2f "
        "sigma %.2f/%.2f)",
        gate_match.valid ? 1 : 0, gate_match.response,
        gate_match.sigma_x, gate_match.sigma_y);
      return false;
    }
    std::vector<GateSe2> hypotheses{gate_match.pose};
    if (gate_match.second_peak >= csm_loop_min_orange_response_) {
      hypotheses.push_back(gate_match.second_pose);
    }
    // Corridors are a TIE-BREAKER, never a veto: the gate locked
    // conjunctively, so the seam WILL apply -- on a twisted ring the
    // corridor response is low by necessity and gating on it killed
    // every certified seam (run2: gate 0.99/0.95, zero seams, ghost 89%).
    // Pick whichever gate hypothesis the corridors prefer; if refinement
    // wanders off the gate or fails outright, apply the raw gate pose.
    double best_response = -1.0;
    bool have_refined = false;
    GateSe2 chosen = gate_match.pose;
    for (const GateSe2 & hypothesis : hypotheses) {
      const CsmMatch refined = grid.match(
        match_points.points, hypothesis, 1.0, 0.09);
      if (!refined.valid || refined.response <= best_response) {
        continue;
      }
      if (gate_grid.layerResponseAt(
          orange_query, refined.pose, 2) < csm_loop_min_orange_response_)
      {
        continue;  // refinement dragged off the gate: keep the gate pose
      }
      best_response = refined.response;
      chosen = refined.pose;
      have_refined = true;
      match = refined;
    }
    if (!have_refined) {
      match = gate_match;
      match.pose = chosen;
      RCLCPP_INFO(
        get_logger(),
        "CSM loop: corridors could not corroborate; applying the raw gate "
        "pose (gate response %.2f, %zu hypotheses)",
        gate_match.response, hypotheses.size());
    }
  } else {
    match = grid.match(
      match_points.points, toGateSe2(reference_estimate), window_xy, window_theta);
  }
  if (!match.valid) {
    return false;
  }
  // A seam re-seed moves the pose by the full drift, so it must be earned:
  // a barely-over-floor response means half the covered constellation
  // disagrees, which is an alias, not a seam.
  const double response_floor =
    loop_mode ? csm_loop_min_response_ : csm_params_.fine_response_min;
  // Loop mode: ambiguity was already owned by the gate-hypothesis vote (a
  // left/right or flip tie is EXPECTED there), so the unimodality margin
  // must not re-reject it. Tracking keeps the margin.
  const bool multimodal =
    !loop_mode && match.response - match.second_peak < csm_peak_margin_;
  if (match.response < response_floor ||
    match.sigma_x > csm_max_sigma_xy_ || match.sigma_y > csm_max_sigma_xy_ ||
    match.sigma_theta > csm_max_sigma_theta_ ||
    multimodal)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "CSM %s match rejected: response %.2f second %.2f "
      "sigma %.2f/%.2f m %.3f rad (degenerate or multimodal)",
      loop_mode ? "loop" : "tracking",
      match.response, match.second_peak,
      match.sigma_x, match.sigma_y, match.sigma_theta);
    return false;
  }
  const g2o::SE2 matched(match.pose.x, match.pose.y, match.pose.theta);
  const g2o::SE2 delta = matched * reference_estimate.inverse();
  const double delta_m = delta.translation().norm();
  const double delta_yaw = std::abs(delta.rotation().angle());
  if (delta_m < 0.15 && delta_yaw < 0.02) {
    if (loop_mode) {
      // Tracking (full-map, tight window) usually closes the seam BEFORE
      // the gate comes into view, so the gate stage finds nothing left to
      // correct — but a verified in-place gate match IS the seam
      // certificate the freeze waits for. Count it; there is just nothing
      // to re-seed.
      ++csm_loop_applied_;
      RCLCPP_INFO(
        get_logger(),
        "CSM loop: seam already registered (gate confirms in place, "
        "response %.2f)", match.response);
    }
    return false;
  }

  pose.vertex->setEstimate(delta * pose.vertex->estimate());
  // A LARGE apply moved the pose by the full drift, so every tentative
  // track's map-frame state (accumulated from pre-correction poses) is
  // offset by exactly that drift. Left alive they confirm into a ghost row
  // of cones beside the corrected map — drop them; re-observation rebuilds
  // the tracks from the corrected pose within a few frames. Routine
  // tracking applies (~0.15-0.2 m, sub-gate) don't reset: constant resets
  // would starve every track of the hits the frozen-map repair admission
  // requires.
  if (frontend_ && delta_m >= 0.5) {
    frontend_->reset();
  }
  // An apply re-linearizes the neighbourhood; give the optimizer room to
  // absorb it before the next correction can fire (rejections stay free).
  last_csm_travel_ = traveled_distance_ + csm_apply_cooldown_m_;
  if (loop_mode) {
    ++csm_loop_applied_;
    // A seam just registered: pull the optimization forward so the loop
    // edges the corrected association is about to add settle immediately.
    if (keyframes_since_last_optimization_ < optimize_every_n_keyframes_) {
      keyframes_since_last_optimization_ = optimize_every_n_keyframes_;
    }
  } else {
    ++csm_track_applied_;
  }
  RCLCPP_WARN(
    get_logger(),
    "CSM %s match APPLIED: response %.2f second %.2f (%zu pts vs %zu "
    "targets), delta %.2f m / %.1f deg, sigma %.2f/%.2f m %.3f rad",
    loop_mode ? "loop" : "tracking",
    match.response, match.second_peak,
    match_points.points.size(), targets.size(),
    delta_m, delta_yaw * 180.0 / M_PI,
    match.sigma_x, match.sigma_y, match.sigma_theta);
  return true;
}

bool GraphSlamNode::removeLandmarkAt(std::size_t landmark_index)
{
  if (landmark_index >= landmarks_.size()) {
    return false;
  }
  LandmarkRecord & landmark = landmarks_[landmark_index];
  if (landmark.vertex != nullptr &&
    optimizer_.vertex(landmark.graph_id) == landmark.vertex)
  {
    // Removes the vertex together with its observation edges.
    optimizer_.removeVertex(landmark.vertex);
  }
  landmarks_.erase(landmarks_.begin() + static_cast<std::ptrdiff_t>(landmark_index));
  return true;
}

std::size_t GraphSlamNode::reapUnobservedLandmarks(
  const g2o::SE2 & observation_pose,
  const std::vector<std::size_t> & observed_landmark_indices)
{
  std::vector<bool> seen(landmarks_.size(), false);
  for (const std::size_t index : observed_landmark_indices) {
    if (index < seen.size()) {
      seen[index] = true;
    }
  }
  std::vector<std::size_t> doomed;
  // Localization mode reaps too, but far more conservatively: the frozen
  // map is the trusted reference, so deleting from it takes a much longer
  // run of contradicting evidence than sweeping mapping-phase ghosts.
  const int miss_budget = localization_mode_ ?
    loc_map_repair_delete_misses_ : landmark_delete_misses_;
  for (std::size_t i = 0; i < landmarks_.size(); ++i) {
    LandmarkRecord & landmark = landmarks_[i];
    if (seen[i]) {
      landmark.consecutive_misses = 0;
      continue;
    }
    // Gate cones are the CSM seam anchor; a wrongly reaped gate would cost
    // far more than any ghost it could ever be. Orange classes are pooled —
    // never key on one label.
    if (landmark.color == ConeColor::BigOrange ||
      landmark.color == ConeColor::Orange)
    {
      continue;
    }
    // A miss is only evidence of absence when the cone sits WELL inside
    // what the sensor reliably sees (conservative envelope — a wide one
    // reaped real far cones during occlusions).
    const Eigen::Vector2d relative =
      observation_pose.inverse() * landmark.vertex->estimate();
    const double range = relative.norm();
    if (range < min_observation_range_ || range > landmark_delete_max_range_) {
      continue;
    }
    if (landmark_delete_fov_ > 0.0 &&
      landmark_delete_fov_ < 2.0 * std::acos(-1.0) &&
      std::abs(std::atan2(relative.y(), relative.x())) >
      0.5 * landmark_delete_fov_)
    {
      continue;
    }
    if (++landmark.consecutive_misses >= miss_budget) {
      doomed.push_back(i);
    }
  }
  std::size_t deleted = 0U;
  for (auto it = doomed.rbegin(); it != doomed.rend(); ++it) {
    if (removeLandmarkAt(*it)) {
      ++deleted;
    }
  }
  if (deleted > 0U) {
    RCLCPP_INFO(
      get_logger(),
      "Reaped %zu registered-but-unobserved landmark(s); %zu remain",
      deleted, landmarks_.size());
  }
  return deleted;
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
  // Either reported sigma engages the branch: a source may report a tight
  // translation with a stale-heading (huge) yaw sigma, and skipping the
  // branch on the translation alone would trust that heading at the floor.
  if (use_odom_covariance_ &&
    (latest_odom_sigma_trans_ > 1e-6 || latest_odom_sigma_yaw_ > 1e-6))
  {
    // The source reports PER-SAMPLE noise; a keyframe edge integrates many
    // samples plus unmodelled correlated error (slip, yaw leverage), so its
    // honest sigma grows with the distance covered. Believing the per-sample
    // number for the whole edge is what froze first-lap drift into the map
    // shape (the graph preferred keeping the warp to bending a 2 cm-sigma
    // chain).
    const double edge_distance = odom_delta.translation().norm();
    const double sigma_t = std::clamp(
      std::max(
        latest_odom_sigma_trans_ > 1e-6 ? latest_odom_sigma_trans_ : odom_translation_sigma_,
        odom_edge_sigma_per_meter_ * edge_distance),
      odom_translation_sigma_, 50.0);
    const double sigma_y = std::clamp(
      std::max(
        latest_odom_sigma_yaw_ > 1e-6 ? latest_odom_sigma_yaw_ : odom_yaw_sigma_,
        odom_edge_yaw_sigma_per_meter_ * edge_distance),
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

  poses_.push_back(PoseRecord{vertex->id(), vertex, raw_odom, stamp});
  updateKeyframeSnapshot();
  ++keyframes_since_last_optimization_;

  traveled_distance_ += odom_delta.translation().norm();
  if (localization_mode_) {
    prunePoseWindow();
  } else {
    if (!lap_origin_captured_ && traveled_distance_ >= lap_origin_capture_distance_) {
      lap_origin_ = vertex->estimate();
      lap_origin_vertex_ = vertex;
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

  if (!add_edges && !update_landmarks) {
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

  // The latest motion input declares its pose invalid (held through a source
  // gap, or the first message after a blind gap): the pose this frame would
  // attach to is unknown, so its cones cannot be placed. Drop the frame.
  if (use_odom_covariance_ && latest_odom_sigma_trans_ >= odom_invalid_sigma_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Dropping cone frame: motion input declares its pose invalid (sigma %.0f m)",
      latest_odom_sigma_trans_);
    return ObservationUpdate{0U, 0U, 0U, 0U};
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

  // Feed the local submap (raw-odometry frame, so scan matching can use the
  // whole recent constellation instead of this one frame).

  if (submap_enable_ && !observations.empty()) {
    std::vector<SubmapPoint> body_points;
    body_points.reserve(observations.size());
    for (const ConeObservation & observation : observations) {
      std::uint8_t color = static_cast<std::uint8_t>(observation.color);
      // Gate credentials require a TRUSTED observation: vision-only orange
      // (ZNCC-disparity depth) carries meters of range variance and a gate
      // anchored on it drags the seam. Demote high-sigma orange to unknown:
      // it still participates as geometry, it just cannot certify a gate.
      if (color == kSubmapColorOrange || color == kSubmapColorBigOrange) {
        const double sigma = std::sqrt(
          std::max(observation.covariance(0, 0), observation.covariance(1, 1)));
        if (sigma > csm_orange_max_obs_sigma_) {
          color = kSubmapColorUnknown;
        }
      }
      body_points.push_back(
        SubmapPoint{
          observation.measurement.x(), observation.measurement.y(), color});
    }
    submap_.addFrame(
      toGateSe2(raw_at_observation), std::move(body_points), traveled_distance_);
    submap_reference_ = raw_at_observation;
    submap_reference_valid_ = true;
  }

  // Correlative registration runs BEFORE association so that a corrected
  // pose lets this same frame's cones associate with the existing landmarks
  // instead of founding drift-offset duplicates.
  if (add_edges && maybeCsmRegister(pose, keyframe_to_observation)) {
    observation_pose = pose.vertex->estimate() * keyframe_to_observation;
  }

  if (add_edges && !process_every_cone_message_) {
    last_cone_pose_graph_id_ = pose.graph_id;
  }

  std::size_t added_edges = 0U;
  std::size_t updated_landmarks = 0U;
  std::size_t loop_edges_added = 0U;
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
      if (range < min_observation_range_ || range > track_visible_max_range_) {
        return false;
      }
      if (track_visible_fov_ > 0.0 &&
        track_visible_fov_ < 2.0 * std::acos(-1.0) &&
        std::abs(std::atan2(relative.y(), relative.x())) > 0.5 * track_visible_fov_)
      {
        return false;
      }
      return true;
    };

  const FrontendFrameResult frame = frontend_->processFrame(
    frontend_observations, confirmed_view, traveled_distance_,
    track_expected_visible);
  const std::size_t matched_landmarks = frame.confirmed_matches.size();

  for (const FrontendConfirmedMatch & match : frame.confirmed_matches) {
    const FrontendObservation & frontend_observation =
      frontend_observations[match.observation_index];
    const ConeObservation & observation = observations[match.observation_index];
    LandmarkRecord & landmark = landmarks_[match.landmark_index];
    voteLandmarkColor(landmark, observation.color);
    const bool stale_loop_candidate =
      add_edges && loop_gap_distance_ > 0.0 &&
      traveled_distance_ - landmark.last_seen_traveled >= loop_gap_distance_;
    if (stale_loop_candidate && !localization_mode_ && csm_loop_applied_ == 0U) {
      // No NN loop closure before the gate certifies the seam: a stale
      // re-association is a per-cone NN guess, and on uniform corridors it
      // locks onto neighbours and drags the map with no orange evidence in
      // sight. Until the CSM gate seam exists, drop the constraint entirely
      // -- registration is CSM's job, not NN's.
      continue;
    }
    if (stale_loop_candidate) {
      ++loop_candidate_count_;
      loop_closure_edge = true;
      loop_confirmation_ready_for_optimize_ = true;
    }
    landmark.last_seen_traveled = traveled_distance_;
    // No direct landmark writes mid-transient: while the optimizer is still
    // absorbing a loop closure, the pose this map_point was computed from is
    // about to move by meters.
    if (update_landmarks && last_optimize_correction_m_ < 0.5 &&
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
      addObservationEdge(
        keyframe_observation, pose.vertex, landmark, stale_loop_candidate);
      ++added_edges;
      if (stale_loop_candidate) {
        ++loop_edges_added;
      }
    }
  }

  // Registration near the start-area map is DEFERRED while the seam is
  // armed but not yet applied: the pose is offset from that map by the
  // full accumulated drift, so a promotion landing inside the seam-search
  // window around it is usually the OLD map re-observed at the drifted
  // pose, not a new cone (measured: gate pairs doubled ~2 m off). The
  // frontend's promote-hold covers the inflated association radius but
  // not the full CSM search window. Deferred, not dropped: the map can
  // freeze on the first lap return moments after the seam registers, and
  // dropped promotions became permanently missing cones. The flush
  // re-derives each position from its keyframes (the optimizer has
  // absorbed the seam by then) and twin-suppression eats the ghosts.
  const bool seam_pending = csm_enable_ && !localization_mode_ &&
    lap_origin_captured_ && csm_loop_applied_ == 0U &&
    loop_gap_distance_ > 0.0 &&
    traveled_distance_ - lap_origin_capture_traveled_m_ >=
    lap_return_min_travel_m_;
  // Not in the frame that applied the seam: the flush needs the optimizer
  // to have dragged the approach keyframes first, and the apply only
  // schedules that optimization.
  if (!seam_pending && !localization_mode_ && !deferred_promotions_.empty() &&
    keyframes_since_last_optimization_ == 0)
  {
    flushDeferredPromotions("seam registered");
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
    if (localization_mode_ &&
      (loc_map_repair_min_hits_ <= 0 ||
      promotion.hits < loc_map_repair_min_hits_))
    {
      // Frozen-map repair admission: the frozen map is the trusted
      // reference, so joining it takes far more corroboration than a
      // mapping-phase promotion (which needs only promote_min_hits).
      // Admitted repairs join FIXED, like the loaded map.
      continue;
    }
    if (seam_pending) {
      bool near_start_map = false;
      for (const LandmarkRecord & existing : landmarks_) {
        if (existing.first_seen_traveled >
          lap_origin_capture_traveled_m_ + 30.0)
        {
          continue;
        }
        if ((existing.vertex->estimate() - promotion.position).norm() <=
          csm_loop_window_m_)
        {
          near_start_map = true;
          break;
        }
      }
      if (near_start_map) {
        deferred_promotions_.push_back(promotion);
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Deferring landmark registration near the start-area map until "
          "the seam registers (%zu deferred)",
          deferred_promotions_.size());
        continue;
      }
    }
    if (registerPromotion(promotion, false)) {
      observed_landmark_indices.push_back(landmarks_.size() - 1U);
      ++added_edges;
    }
  }

  if (loop_closure_edge) {
    if (keyframes_since_last_optimization_ < optimize_every_n_keyframes_) {
      keyframes_since_last_optimization_ = optimize_every_n_keyframes_;
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Loop closure: stale re-association; pulling graph optimization forward "
        "loop_candidates=%zu loop_confirmed=%zu",
        loop_candidate_count_,
        loop_confirmed_count_);
    }
  }

const std::size_t deleted_landmarks =
    (landmark_delete_enable_ && add_edges &&
    (!localization_mode_ || loc_map_repair_delete_misses_ > 0)) ?
    reapUnobservedLandmarks(observation_pose, observed_landmark_indices) : 0U;

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
  double best_inflation = 0.0;
  double second_inflation = 0.0;

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
      second_inflation = best_inflation;
      best_d2 = d2;
      best_index = static_cast<int>(i);
      best_inflation = inflation;
      best_euclidean_sq = diff.squaredNorm();
    } else if (d2 < second_d2) {
      second_d2 = d2;
      second_index = static_cast<int>(i);
      second_inflation = inflation;
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
    // The duplicate-pair separation scales with the drift inflation the gate
    // itself admitted, capped below real cone spacing (see the frontend's
    // identical rule).
    const double duplicate_separation = std::min(
      frontend_params_.duplicate_pair_max_separation_m,
      association_max_distance_ +
      3.0 * std::sqrt(std::max(best_inflation, second_inflation)));
    if (rival_separation > duplicate_separation) {
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

bool GraphSlamNode::registerPromotion(
  const FrontendPromotion & promotion, bool recompute_position)
{
  Eigen::Vector2d position = promotion.position;
  if (recompute_position) {
    // The stored map position was accumulated from pre-correction poses;
    // the keyframe-relative history survives the correction, so re-derive
    // the position from the earliest keyframe that still exists.
    for (const FrontendPendingObservation & pending : promotion.observations) {
      g2o::VertexSE2 * vertex = nullptr;
      for (auto it = poses_.rbegin(); it != poses_.rend(); ++it) {
        if (it->graph_id == pending.pose_graph_id) {
          vertex = it->vertex;
          break;
        }
        if (it->graph_id < pending.pose_graph_id) {
          break;
        }
      }
      if (vertex != nullptr) {
        position = vertex->estimate() * pending.keyframe_measurement;
        break;
      }
    }
  }
  // Twin suppression: two tentative tracks of the SAME physical cone can
  // race to confirmation a frame apart (measured twins at 0.04-0.15 m).
  // A promotion landing on an existing same-color landmark is that race,
  // not a new cone. Big-orange gates are REAL pairs at ~0.35-0.40 m, so
  // orange classes use a radius below the physical pair spacing.
  const ConeColor promo_color = static_cast<ConeColor>(promotion.color);
  const bool orange_class = promo_color == ConeColor::Orange ||
    promo_color == ConeColor::BigOrange;
  const double twin_radius = orange_class ? 0.25 : 0.6;
  for (const LandmarkRecord & existing : landmarks_) {
    if (!colorsCompatible(promo_color, existing.color)) {
      continue;
    }
    if ((existing.vertex->estimate() - position).norm() < twin_radius) {
      return false;
    }
  }
  LandmarkRecord * landmark = addLandmark(
    position, promotion.covariance, promo_color, localization_mode_);
  if (landmark == nullptr) {
    return false;  // landmark cap
  }
  if (localization_mode_) {
    RCLCPP_INFO(
      get_logger(),
      "Frozen-map repair: admitted a cone (color %d) at (%.1f, %.1f) "
      "after %d hits",
      static_cast<int>(promo_color), position.x(), position.y(),
      promotion.hits);
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
    replay.color = promo_color;
    addObservationEdge(replay, pending_vertex, *landmark);
  }
  return true;
}

void GraphSlamNode::flushDeferredPromotions(const char * reason)
{
  if (deferred_promotions_.empty()) {
    return;
  }
  std::size_t registered = 0U;
  for (const FrontendPromotion & promotion : deferred_promotions_) {
    if (registerPromotion(promotion, true)) {
      ++registered;
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "Flushed deferred promotions (%s): %zu/%zu registered as new cones, "
    "the rest were the already-registered map re-observed",
    reason, registered, deferred_promotions_.size());
  deferred_promotions_.clear();
}

GraphSlamNode::LandmarkRecord * GraphSlamNode::addLandmark(
  const Eigen::Vector2d & map_point,
  const Eigen::Matrix2d & covariance,
  ConeColor color,
  bool as_map_repair)
{
  if (localization_mode_ && !as_map_repair) {
    // Localization only: the map is fixed; only the vetted repair path may
    // grow it.
    return nullptr;
  }

  if (max_landmarks_ > 0 &&
    landmarks_.size() >= static_cast<std::size_t>(max_landmarks_))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 30000,
      "Landmark cap max_landmarks=%d reached — MAPPING IS DROPPING CONES "
      "(hairpins autopsy: the map silently ended mid-track at the cap)",
      max_landmarks_);
    return nullptr;
  }

  auto * vertex = new g2o::VertexPointXY();
  vertex->setId(next_vertex_id_++);
  vertex->setEstimate(map_point);
  vertex->setMarginalized(true);
  if (as_map_repair) {
    vertex->setFixed(true);  // joins the frozen reference as-is
  }

  if (!optimizer_.addVertex(vertex)) {
    RCLCPP_ERROR(get_logger(), "Failed to add landmark vertex %d", vertex->id());
    delete vertex;
    return nullptr;
  }

  landmarks_.push_back(
    LandmarkRecord{
      vertex->id(), color, vertex, covariance, 0U, 0,
      traveled_distance_, traveled_distance_, {}});
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
  LandmarkRecord & landmark,
  bool loop_edge)
{
  auto * edge = new g2o::EdgeSE2PointXY();
  edge->setId(next_edge_id_++);
  edge->setVertex(0, pose_vertex);
  edge->setVertex(1, landmark.vertex);
  edge->setMeasurement(observation.measurement);

  Eigen::Matrix2d information = observation.covariance.inverse();
  edge->setInformation(information);

  attachObservationKernel(edge, loop_edge);

  if (!optimizer_.addEdge(edge)) {
    RCLCPP_ERROR(get_logger(), "Failed to add cone observation edge");
    delete edge;
    return;
  }

  ++landmark.observations;
  landmark.consecutive_misses = 0;
}

void GraphSlamNode::attachObservationKernel(
  g2o::EdgeSE2PointXY * edge, bool loop_edge) const
{
  if (observation_robust_kernel_ == "none") {
    return;
  }
  // Loop-closure edges are ALWAYS Huber (convex, cost keeps growing with the
  // residual): under DCS, discarding a true loop edge costs only ~2*phi,
  // which for meters of accumulated drift is far cheaper than bending the
  // odometry chain — the optimizer then rationally switches the loop OFF
  // and keeps the drifted double map (measured: a 460-edge chain absorbs a
  // 9 m closure for chi2 ~440 while DCS discards the loop for ~2/edge; the
  // crossover means DCS could never close more than ~2-3 m of drift).
  if (loop_edge && robust_kernel_delta_ > 0.0) {
    auto * robust_kernel = new g2o::RobustKernelHuber();
    robust_kernel->setDelta(robust_kernel_delta_);
    edge->setRobustKernel(robust_kernel);
    return;
  }
  if (observation_robust_kernel_ == "huber" && robust_kernel_delta_ > 0.0) {
    auto * robust_kernel = new g2o::RobustKernelHuber();
    robust_kernel->setDelta(robust_kernel_delta_);
    edge->setRobustKernel(robust_kernel);
  }
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
  // A large loop-closure correction (anchor re-seed) is not absorbed in one
  // Levenberg call: the Huber loop edges' influence is linear in the
  // residual, so the first call typically closes only part of the gap and
  // leaves the graph mid-swing (observed: a 15 m seam re-seed relaxed 10 m
  // in call one, then re-fired the anchor). Keep solving while the pose is
  // still moving by whole steps so one confirmed closure settles within one
  // cone frame instead of churning across anchor re-fires.
  for (int extra = 0; extra < 4; ++extra) {
    const g2o::SE2 before = poses_.back().vertex->estimate();
    if (!optimizeGraph()) {
      break;
    }
    if ((poses_.back().vertex->estimate().translation() - before.translation())
      .norm() < 0.5)
    {
      break;
    }
  }
  optimizer_skipped_pose_limit_ = false;

  if (!map_converged_ && loop_confirmation_ready_for_optimize_) {
    loop_confirmation_ready_for_optimize_ = false;
    ++loop_confirmed_count_;
    map_converged_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Map converged: a loop-closure edge was reconciled by optimization "
      "(loop_candidates=%zu)",
      loop_candidate_count_);
    publishStatus();
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
  optimizer_.initializeOptimization();
  const int completed_iterations = optimizer_.optimize(optimization_iterations_);
  // Correction-step autopsy: every optimizer run that moves the live pose by
  // centimeters is a step the planner (and every in-flight observation)
  // experiences as a jump. This is the number that explains "the map and the
  // car twitch" — keep it visible and correlate it with duplicate births.
  const double correction_m =
    (poses_.back().vertex->estimate().translation() - pose_before.translation()).norm();
  last_optimize_correction_m_ = correction_m;
  // Same staleness as a CSM re-seed: a large step means the optimizer
  // dragged the keyframes, and pending tracks still reference the
  // pre-correction geometry — promoting them would register duplicates
  // offset by the correction. 0.5 m matches the direct-update gate in
  // processCones.
  if (frontend_ && correction_m >= 0.5) {
    frontend_->reset();
  }
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
  double px = estimate.translation().x();
  double py = estimate.translation().y();
  double pyaw = estimate.rotation().angle();

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = map_frame_;
  odom.header.stamp = stamp;
  odom.child_frame_id = slam_base_frame_;
  odom.pose.pose.position.x = px;
  odom.pose.pose.position.y = py;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternionFromYaw(pyaw);

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

void GraphSlamNode::markLapReturnObserved()
{
  lap_return_criteria_satisfied_ = true;
}

g2o::SE2 GraphSlamNode::lapOrigin() const
{
  // The capture-time snapshot goes stale the moment a loop closure bends the
  // graph: a seam correction moves the origin's map coordinates by the whole
  // closed drift, and comparing against the snapshot then makes the car miss
  // its own lap-return window (observed: peanut/serpentine runs converging
  // but never freezing). The origin KEYFRAME's live estimate is the physical
  // racing-line point in current map coordinates.
  if (lap_origin_vertex_ != nullptr &&
    optimizer_.vertex(lap_origin_vertex_->id()) == lap_origin_vertex_)
  {
    return lap_origin_vertex_->estimate();
  }
  return lap_origin_;
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
  const g2o::SE2 origin = lapOrigin();
  const double return_distance =
    (current_estimate.translation() - origin.translation()).norm();
  const double yaw_error = std::abs(
    normalizeAngle(
      current_estimate.rotation().angle() - origin.rotation().angle()));
  if (return_distance <= 3.0 * lap_return_radius_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Lap return check: dist %.2f m (need <= %.1f) yaw %.2f rad (need <= "
      "%.2f) csm_loop=%zu loop_confirmed=%zu",
      return_distance, lap_return_radius_, yaw_error, lap_return_yaw_,
      csm_loop_applied_, loop_confirmed_count_);
  }
  if (return_distance > lap_return_radius_ || yaw_error > lap_return_yaw_) {
    lap_return_window_active_ = false;
    return;
  }
  // A lap is closed when the seam was REGISTERED, not when the estimate
  // happens to wander through the origin window: first-lap drift can brush
  // the window mid-lap (observed: freeze at ~60 m travel froze a half map).
  // Registration evidence is the CSM gate seam. (A reconciled vanilla loop
  // edge was accepted here briefly and froze boa's map before the seam was
  // actually registered — ATE 4.2 m in localization against it. The gate
  // seam is the only certificate that has never lied.)
  if (csm_enable_ && csm_loop_applied_ == 0U) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Lap geometry closed (return %.2f m) but no seam registration yet; "
      "continuing to map", return_distance);
    return;
  }
  if (!lap_return_window_active_) {
    lap_return_window_active_ = true;
    ++lap_returns_seen_;
    RCLCPP_INFO(
      get_logger(),
      "Lap return %d/%d (return %.2f m, yaw %.2f rad)",
      lap_returns_seen_, lap_returns_to_freeze_, return_distance, yaw_error);
  }
  // Freezing on the FIRST return bakes the first lap's drift into the map
  // (boa: 0.65 m cone warp, localization tracking then thrashed against
  // it). One more mapping lap lays revisit loop edges along the WHOLE
  // track and the optimizer irons the warp out before landmarks fix.
  if (lap_returns_seen_ < lap_returns_to_freeze_) {
    return;
  }
  markLapReturnObserved();
  RCLCPP_INFO(
    get_logger(),
    "Mapping lap complete (return %.2f m, yaw %.2f rad)",
    return_distance, yaw_error);
  enterLocalizationMode("mapping lap completed");
}

void GraphSlamNode::enterLocalizationMode(const std::string & reason)
{
  // Promotions deferred across the seam must land before the freeze — the
  // frozen map accepts no new landmarks, so anything still parked here
  // would become a permanently missing cone.
  flushDeferredPromotions("entering localization mode");
  // Unpromoted tracks die with the mapping phase: the frozen map accepts no
  // new landmarks, so keeping them would only absorb observations that the
  // localization matcher needs.
  if (frontend_) {
    frontend_->reset();
  }

  // Freeze the map: landmarks become the fixed reference, direct Kalman
  // updates are disabled by the localization_mode_ guards, and the pose
  // window is bounded so the graph no longer grows with laps. Repair stays
  // possible under the much stricter loc_map_repair_* admission (heavily
  // corroborated additions join fixed; deletion needs a far longer run of
  // contradicting evidence).
  localization_mode_ = true;
  for (LandmarkRecord & landmark : landmarks_) {
    landmark.vertex->setFixed(true);
    // Mapping-phase misses don't carry into the localization budget.
    landmark.consecutive_misses = 0;
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
    if (oldest.vertex == lap_origin_vertex_) {
      lap_origin_ = lap_origin_vertex_->estimate();  // keep the last live fix
      lap_origin_vertex_ = nullptr;
    }
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
    !map_converged_ && (loop_candidate_count_ > 0U || lap_return_criteria_satisfied_);
  input.ambiguous_association = false;
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
      << " loop_confirmed=" << loop_confirmed_count_
      << " map_converged=" << (map_converged_ ? "true" : "false")
      << " localization_mode=" << (localization_mode_ ? "true" : "false")
      << " lap_return_criteria_satisfied="
      << (lap_return_criteria_satisfied_ ? "true" : "false")
      << " odometry_fresh="
      << (reason == MappingStopReason::OdometryDropout ? "false" : "true")
      << " csm_track=" << csm_track_applied_
      << " csm_loop=" << csm_loop_applied_;
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
    LandmarkRecord record{
      vertex->id(), colorFromTag(tag), vertex, cov, seed_obs, 0, 0.0, 0.0, {}};
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
