// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "eufs_graph_slam/graph_slam_node.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>
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

namespace eufs_graph_slam
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
  association_inflation_per_keyframe_(0.01),
  association_max_inflation_(4.0),
  landmark_merge_distance_(0.6),
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
  loop_gap_keyframes_(30),
  map_trust_loop_closures_required_(2),
  map_trust_info_scale_(3.0),
  localization_mode_(false),
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
  loop_closure_seen_since_optimize_(false),
  loop_closure_optimize_cycles_(0)
{
  car_state_topic_ =
    declare_parameter<std::string>("car_state_topic", "/odometry_integration/car_state");
  cones_topic_ = declare_parameter<std::string>("cones_topic", "/cones");
  map_topic_ = declare_parameter<std::string>("map_topic", "/graph_slam/map");
  slam_odom_topic_ = declare_parameter<std::string>("slam_odom_topic", "/graph_slam/odom");
  path_topic_ = declare_parameter<std::string>("path_topic", "/graph_slam/path");
  marker_topic_ = declare_parameter<std::string>("marker_topic", "/graph_slam/markers");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  slam_base_frame_ = declare_parameter<std::string>("slam_base_frame", "base_footprint");
  g2o_output_path_ = declare_parameter<std::string>("g2o_output_path", "/tmp/eufs_graph_slam.g2o");
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
  relocalize_search_radius_ =
    declare_parameter<double>("relocalize_search_radius", relocalize_search_radius_);
  relocalize_search_yaw_ =
    declare_parameter<double>("relocalize_search_yaw", relocalize_search_yaw_);
  relocalize_inlier_distance_ =
    declare_parameter<double>("relocalize_inlier_distance", relocalize_inlier_distance_);
  association_inflation_per_keyframe_ =
    declare_parameter<double>(
    "association_inflation_per_keyframe",
    association_inflation_per_keyframe_);
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
  loop_gap_keyframes_ = declare_parameter<int>("loop_gap_keyframes", loop_gap_keyframes_);
  map_trust_loop_closures_required_ =
    declare_parameter<int>(
    "map_trust_loop_closures_required",
    map_trust_loop_closures_required_);
  map_trust_info_scale_ =
    declare_parameter<double>("map_trust_info_scale", map_trust_info_scale_);

  optimize_min_interval_ =
    declare_parameter<double>("optimize_min_interval", optimize_min_interval_);
  visual_publish_min_interval_ =
    declare_parameter<double>("visual_publish_min_interval", visual_publish_min_interval_);
  tf_stamp_offset_ = declare_parameter<double>("tf_stamp_offset", tf_stamp_offset_);

  localization_mode_ = declare_parameter<bool>("localization_mode", localization_mode_);
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

  keyframe_distance_ = std::max(0.0, keyframe_distance_);
  keyframe_yaw_ = std::max(0.0, keyframe_yaw_);
  association_max_distance_ = std::max(0.05, association_max_distance_);
  association_gate_chi2_ = std::max(0.1, association_gate_chi2_);
  association_ambiguity_ratio_ = std::clamp(association_ambiguity_ratio_, 0.0, 1.0);
  relocalize_search_radius_ = std::max(0.0, relocalize_search_radius_);
  relocalize_search_yaw_ = std::max(0.0, relocalize_search_yaw_);
  relocalize_inlier_distance_ = std::max(0.05, relocalize_inlier_distance_);
  association_inflation_per_keyframe_ = std::max(0.0, association_inflation_per_keyframe_);
  association_max_inflation_ = std::max(0.0, association_max_inflation_);
  landmark_merge_distance_ = std::max(0.0, landmark_merge_distance_);
  landmark_confirm_observations_ = std::max(0, landmark_confirm_observations_);
  loop_gap_keyframes_ = std::max(0, loop_gap_keyframes_);
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
  map_pub_ = create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>(map_topic_, transient_qos);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(slam_odom_topic_, 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, transient_qos);
  marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, transient_qos);

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

  car_state_sub_ = create_subscription<eufs_msgs::msg::CarState>(
    car_state_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&GraphSlamNode::stateCallback, this, std::placeholders::_1),
    state_options);
  cones_sub_ = create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
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
  loop_closure_seen_since_optimize_ = false;
  loop_closure_optimize_cycles_ = 0;
  last_optimization_time_sec_ = -1.0;
  last_visual_publish_time_sec_ = -1.0;
  last_landmark_delete_time_sec_ = -1.0;
}

void GraphSlamNode::stateCallback(const eufs_msgs::msg::CarState::SharedPtr msg)
{
  const rclcpp::Time stamp = stampOrNow(msg->header.stamp, get_clock());
  const g2o::SE2 raw_odom = poseFromCarState(*msg);
  recordRawOdometry(stamp.seconds(), raw_odom);

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
  const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);

  if (poses_.empty()) {
    return;
  }

  const ObservationUpdate update = addConeObservations(*msg, false);

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
    publishGraphVisuals(stampOrNow(msg->header.stamp, get_clock()));
  }
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

g2o::SE2 GraphSlamNode::scanMatchNearClick(const g2o::SE2 & click) const
{
  if (last_observations_.empty() || landmarks_.empty() ||
    relocalize_search_radius_ <= 0.0)
  {
    return click;
  }

  // Brute-force search over a window around the click: for each candidate
  // pose, transform the latest cones into the map and count how many land
  // within relocalize_inlier_distance of a fixed landmark. Best inlier count
  // (tie-broken by residual) wins.
  const int n_xy = 8;
  const int n_yaw = 8;
  const double dxy = relocalize_search_radius_ / n_xy;
  const double dyaw = relocalize_search_yaw_ > 0.0 ? relocalize_search_yaw_ / n_yaw : 0.0;
  const double inlier_sq = relocalize_inlier_distance_ * relocalize_inlier_distance_;

  g2o::SE2 best = click;
  int best_inliers = -1;
  double best_residual = std::numeric_limits<double>::max();

  for (int ix = -n_xy; ix <= n_xy; ++ix) {
    for (int iy = -n_xy; iy <= n_xy; ++iy) {
      for (int iw = -n_yaw; iw <= n_yaw; ++iw) {
        const g2o::SE2 cand(
          click.translation().x() + ix * dxy,
          click.translation().y() + iy * dxy,
          normalizeAngle(click.rotation().angle() + iw * dyaw));

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

        if (dyaw == 0.0) {
          break;
        }
      }
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "Relocalization scan-match: %d/%zu cones fit the map at the best pose",
    best_inliers, last_observations_.size());
  return best;
}

void GraphSlamNode::relocalizeTo(const g2o::SE2 & click)
{
  // Drop the current pose trajectory (and its odometry/observation edges),
  // keeping the map. In localization mode this re-anchors the drifted
  // odometry to the fixed map; the operator asserts the car is near the click.
  for (PoseRecord & record : poses_) {
    if (optimizer_.vertex(record.graph_id) == record.vertex) {
      optimizer_.removeVertex(record.vertex);
    }
  }
  poses_.clear();
  keyframes_since_last_optimization_ = 0;
  last_cone_pose_graph_id_ = -1;
  last_optimization_time_sec_ = -1.0;

  // (B) Scan-match the latest cones against the fixed map near the click to
  // find the best-fit starting pose.
  const g2o::SE2 pose = scanMatchNearClick(click);
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
    for (const ConeObservation & obs : last_observations_) {
      const Eigen::Vector2d map_point = pose * obs.measurement;
      const Eigen::Matrix2d map_covariance = covarianceInMapFrame(pose, obs.covariance);
      bool ambiguous = false;
      const int idx = findAssociatedLandmark(map_point, map_covariance, obs.color, &ambiguous);
      if (idx >= 0) {
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

g2o::SE2 GraphSlamNode::poseFromCarState(const eufs_msgs::msg::CarState & msg) const
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
  edge->setInformation(odom_information_);

  if (!optimizer_.addEdge(edge)) {
    RCLCPP_ERROR(get_logger(), "Failed to add odometry edge");
    delete edge;
    return;
  }

  poses_.push_back(PoseRecord{vertex->id(), vertex, raw_odom, stamp});
  updateKeyframeSnapshot();
  ++keyframes_since_last_optimization_;
}

GraphSlamNode::ObservationUpdate GraphSlamNode::addConeObservations(
  const eufs_msgs::msg::ConeArrayWithCovariance & msg,
  bool force_process)
{
  if (poses_.empty()) {
    return ObservationUpdate{0U, 0U, 0U};
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
    return ObservationUpdate{0U, 0U, 0U};
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
  const g2o::SE2 raw_at_observation = rawOdomAt(stamp.seconds());
  const g2o::SE2 keyframe_to_observation = pose.raw_odom.inverse() * raw_at_observation;
  const g2o::SE2 observation_pose = pose.vertex->estimate() * keyframe_to_observation;

  if (add_edges && !process_every_cone_message_) {
    last_cone_pose_graph_id_ = pose.graph_id;
  }

  const std::size_t keyframe_index = poses_.size() - 1U;
  std::size_t added_edges = 0U;
  std::size_t updated_landmarks = 0U;
  bool loop_closure_edge = false;
  std::vector<std::size_t> observed_landmark_indices;
  observed_landmark_indices.reserve(observations.size());

  for (const ConeObservation & observation : observations) {
    ConeObservation keyframe_observation = observation;
    keyframe_observation.measurement = keyframe_to_observation * observation.measurement;
    keyframe_observation.covariance =
      covarianceInMapFrame(keyframe_to_observation, observation.covariance);

    const Eigen::Vector2d map_point = observation_pose * observation.measurement;
    const Eigen::Matrix2d map_covariance =
      covarianceInMapFrame(observation_pose, observation.covariance);

    bool ambiguous = false;
    const int landmark_index =
      findAssociatedLandmark(map_point, map_covariance, observation.color, &ambiguous);
    if (ambiguous) {
      // Two landmarks explain this cone almost equally well; fusing or
      // spawning a duplicate would corrupt the map, so skip it this frame.
      continue;
    }

    LandmarkRecord * landmark = nullptr;
    std::size_t observed_index = 0U;
    bool has_observed_index = false;

    if (landmark_index >= 0) {
      observed_index = static_cast<std::size_t>(landmark_index);
      has_observed_index = true;
      landmark = &landmarks_[observed_index];
      voteLandmarkColor(*landmark, observation.color);
      if (add_edges && loop_gap_keyframes_ > 0 &&
        keyframe_index >= landmark->last_seen_keyframe_index +
        static_cast<std::size_t>(loop_gap_keyframes_))
      {
        loop_closure_edge = true;
      }
      landmark->last_seen_keyframe_index = keyframe_index;
      if (update_landmarks &&
        updateLandmarkEstimate(*landmark, map_point, map_covariance))
      {
        ++updated_landmarks;
      }
    } else if (add_edges) {
      landmark = addLandmark(map_point, map_covariance, observation.color);
      if (landmark != nullptr) {
        observed_index = landmarks_.size() - 1U;
        has_observed_index = true;
      }
    }

    if (landmark == nullptr) {
      continue;
    }

    if (has_observed_index) {
      observed_landmark_indices.push_back(observed_index);
    }

    if (add_edges) {
      addObservationEdge(keyframe_observation, pose.vertex, *landmark);
      ++added_edges;
    }
  }

  if (loop_closure_edge) {
    // Mark that a loop closure is pending reconciliation by the next
    // optimization; maybeOptimize() counts those cycles toward convergence.
    loop_closure_seen_since_optimize_ = true;

    if (keyframes_since_last_optimization_ < optimize_every_n_keyframes_) {
      // Re-observed a landmark unseen for many keyframes (lap closure).
      // Pull the next optimization forward so accumulated drift is corrected
      // promptly; optimize_min_interval still rate-limits back-to-back runs.
      keyframes_since_last_optimization_ = optimize_every_n_keyframes_;
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Loop closure: re-associated landmark(s) after >= %d keyframes; "
        "pulling graph optimization forward",
        loop_gap_keyframes_);
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

  return ObservationUpdate{added_edges, updated_landmarks, deleted_landmarks};
}

std::vector<GraphSlamNode::ConeObservation> GraphSlamNode::extractConeObservations(
  const eufs_msgs::msg::ConeArrayWithCovariance & msg) const
{
  std::vector<ConeObservation> observations;

  const auto append_cones =
    [this, &observations](
    const auto & cones,
    ConeColor color)
    {
      for (const eufs_msgs::msg::ConeWithCovariance & cone : cones) {
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
  const eufs_msgs::msg::ConeWithCovariance & cone) const
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
  bool * ambiguous) const
{
  if (ambiguous != nullptr) {
    *ambiguous = false;
  }

  const std::size_t current_keyframe = poses_.empty() ? 0U : poses_.size() - 1U;

  double best_d2 = std::numeric_limits<double>::max();
  double second_d2 = std::numeric_limits<double>::max();
  double best_euclidean_sq = std::numeric_limits<double>::max();
  int best_index = -1;

  // Association is geometric only. Colour is tracked by majority vote and
  // deliberately not used as a gate: simulated perception mislabels cones,
  // and a colour gate would lock those errors in by rejecting the very
  // observations that could fix them. Cone spacing (>= 3 m) is far larger
  // than the association gate, so colour adds no discrimination here.
  (void)color;

  for (std::size_t i = 0; i < landmarks_.size(); ++i) {
    const LandmarkRecord & landmark = landmarks_[i];
    const Eigen::Vector2d diff = landmark.vertex->estimate() - map_point;

    // Pose drift since the landmark was last seen widens its gate; this is
    // what lets lap-closure re-associations succeed despite accumulated
    // odometry error.
    const std::size_t keyframes_unseen =
      current_keyframe > landmark.last_seen_keyframe_index ?
      current_keyframe - landmark.last_seen_keyframe_index : 0U;
    const double inflation = std::min(
      association_max_inflation_,
      association_inflation_per_keyframe_ * static_cast<double>(keyframes_unseen));

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
      best_d2 = d2;
      best_index = static_cast<int>(i);
      best_euclidean_sq = diff.squaredNorm();
    } else if (d2 < second_d2) {
      second_d2 = d2;
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
    *ambiguous = true;
    return -1;
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

  const std::size_t keyframe_index = poses_.empty() ? 0U : poses_.size() - 1U;
  landmarks_.push_back(
    LandmarkRecord{vertex->id(), color, vertex, covariance, 0U, 0, keyframe_index, {}});
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

  if (robust_kernel_delta_ > 0.0) {
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
  if (localization_mode_ || landmark_merge_distance_ <= 0.0 || landmarks_.size() < 2U) {
    return 0U;
  }

  const double merge_distance_sq = landmark_merge_distance_ * landmark_merge_distance_;
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

      // Keep the landmark with more observations; it carries more edges.
      if (landmarks_[j].observations > landmarks_[i].observations) {
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
      kept.last_seen_keyframe_index =
        std::max(kept.last_seen_keyframe_index, dropped.last_seen_keyframe_index);
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

  optimizeGraph();
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
    loop_closure_seen_since_optimize_)
  {
    loop_closure_seen_since_optimize_ = false;
    if (++loop_closure_optimize_cycles_ >= map_trust_loop_closures_required_) {
      map_converged_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Map converged after %d loop-closure optimization cycle(s); "
        "boosting confirmed-landmark observation weight by %.1fx",
        loop_closure_optimize_cycles_,
        map_trust_info_scale_);
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

void GraphSlamNode::optimizeGraph()
{
  if (poses_.size() < 2U || optimizer_.edges().empty()) {
    return;
  }

  optimizer_.initializeOptimization();
  const int completed_iterations = optimizer_.optimize(optimization_iterations_);
  RCLCPP_DEBUG(
    get_logger(),
    "g2o optimization completed %d iterations for %zu poses and %zu landmarks",
    completed_iterations,
    poses_.size(),
    landmarks_.size());
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
  publishOdometry(stamp, estimate);
  publishTransform(stamp, estimate, raw_odom);
}

void GraphSlamNode::publishMap(const rclcpp::Time & stamp)
{
  eufs_msgs::msg::ConeArrayWithCovariance msg;
  msg.header.frame_id = map_frame_;
  msg.header.stamp = stamp;

  for (const LandmarkRecord & landmark : landmarks_) {
    if (landmark.observations <
      static_cast<std::size_t>(landmark_min_observations_to_publish_))
    {
      continue;
    }

    eufs_msgs::msg::ConeWithCovariance cone;
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
  odom.pose.covariance[0] = 0.05;
  odom.pose.covariance[7] = 0.05;
  odom.pose.covariance[35] = 0.05;

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

  int marker_id = 1;
  for (ConeColor color : colors) {
    visualization_msgs::msg::Marker landmark_marker;
    landmark_marker.header.frame_id = map_frame_;
    landmark_marker.header.stamp = stamp;
    landmark_marker.ns = colorName(color) + "_landmarks";
    landmark_marker.id = marker_id++;
    landmark_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    landmark_marker.action = visualization_msgs::msg::Marker::ADD;
    landmark_marker.pose.orientation.w = 1.0;
    landmark_marker.scale.x = marker_scale_;
    landmark_marker.scale.y = marker_scale_;
    landmark_marker.scale.z = marker_scale_;
    landmark_marker.color = colorToRgba(color, 0.95);

    for (const LandmarkRecord & landmark : landmarks_) {
      if (landmark.color != color ||
        landmark.observations <
        static_cast<std::size_t>(landmark_min_observations_to_publish_))
      {
        continue;
      }

      geometry_msgs::msg::Point point;
      const Eigen::Vector2d estimate = landmark.vertex->estimate();
      point.x = estimate.x();
      point.y = estimate.y();
      point.z = 0.15;
      landmark_marker.points.push_back(point);
    }

    markers.markers.push_back(landmark_marker);
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

void GraphSlamNode::handleSaveMap(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  // Timestamped filename: map_YYYYmmdd_HHMMSS.csv in map_save_dir_.
  const std::time_t now = std::time(nullptr);
  std::tm tm_buf{};
  localtime_r(&now, &tm_buf);
  std::ostringstream name;
  name << "map_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".csv";
  const std::string dir = map_save_dir_.empty() ? std::string("/tmp") : map_save_dir_;
  const std::string path = dir + "/" + name.str();

  std::string error;
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (saveMapCsv(path, &error)) {
    response->success = true;
    response->message = "Saved map to " + path;
    RCLCPP_INFO(get_logger(), "Saved cone map to %s", path.c_str());
  } else {
    response->success = false;
    response->message = "Failed to save map to " + path + ": " + error;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }
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
    LandmarkRecord record{vertex->id(), colorFromTag(tag), vertex, cov, seed_obs, 0, 0U, {}};
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

}  // namespace eufs_graph_slam
