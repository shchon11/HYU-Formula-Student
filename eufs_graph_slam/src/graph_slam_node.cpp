// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "eufs_graph_slam/graph_slam_node.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>
#include <g2o/types/slam2d/types_slam2d.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace eufs_graph_slam
{

namespace
{

time_alignment::Pose2d pose2dFromSe2(const g2o::SE2 & pose)
{
  return time_alignment::Pose2d{
    pose.translation().x(), pose.translation().y(), pose.rotation().angle()};
}

g2o::SE2 se2FromPose2d(const time_alignment::Pose2d & pose)
{
  return g2o::SE2(pose.x, pose.y, pose.yaw);
}

}  // namespace

GraphSlamNode::GraphSlamNode()
: Node("graph_slam_node"),
  keyframe_distance_(1.0),
  keyframe_yaw_(0.25),
  keyframe_max_dt_(1.0),
  pose_history_duration_(3.0),
  clock_rollback_threshold_(0.1),
  association_max_distance_(1.2),
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
  pose_history_max_samples_(1024),
  max_pending_cone_messages_(32),
  use_cone_covariance_(true),
  process_every_cone_message_(false),
  publish_tf_(true),
  delete_stale_landmarks_(true),
  update_existing_landmarks_(true),
  optimize_min_interval_(10.0),
  visual_publish_min_interval_(0.5),
  tf_stamp_offset_(0.0),
  last_optimization_time_sec_(-1.0),
  last_visual_publish_time_sec_(-1.0),
  last_landmark_delete_time_sec_(-1.0),
  next_vertex_id_(0),
  next_edge_id_(0),
  keyframes_since_last_optimization_(0),
  pose_history_(time_alignment::PoseHistoryOptions{3000000000LL, 1024U, 100000000LL}),
  latest_estimate_(),
  has_latest_pose_(false)
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

  keyframe_distance_ = declare_parameter<double>("keyframe_distance", keyframe_distance_);
  keyframe_yaw_ = declare_parameter<double>("keyframe_yaw", keyframe_yaw_);
  keyframe_max_dt_ = declare_parameter<double>("keyframe_max_dt", keyframe_max_dt_);
  pose_history_duration_ =
    declare_parameter<double>("pose_history_duration", pose_history_duration_);
  clock_rollback_threshold_ =
    declare_parameter<double>("clock_rollback_threshold", clock_rollback_threshold_);
  association_max_distance_ =
    declare_parameter<double>("association_max_distance", association_max_distance_);
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
  pose_history_max_samples_ =
    declare_parameter<int>("pose_history_max_samples", pose_history_max_samples_);
  max_pending_cone_messages_ =
    declare_parameter<int>("max_pending_cone_messages", max_pending_cone_messages_);

  optimize_min_interval_ =
    declare_parameter<double>("optimize_min_interval", optimize_min_interval_);
  visual_publish_min_interval_ =
    declare_parameter<double>("visual_publish_min_interval", visual_publish_min_interval_);
  tf_stamp_offset_ = declare_parameter<double>("tf_stamp_offset", tf_stamp_offset_);

  use_cone_covariance_ = declare_parameter<bool>("use_cone_covariance", use_cone_covariance_);
  process_every_cone_message_ =
    declare_parameter<bool>("process_every_cone_message", process_every_cone_message_);
  publish_tf_ = declare_parameter<bool>("publish_tf", publish_tf_);
  delete_stale_landmarks_ =
    declare_parameter<bool>("delete_stale_landmarks", delete_stale_landmarks_);
  update_existing_landmarks_ =
    declare_parameter<bool>("update_existing_landmarks", update_existing_landmarks_);

  keyframe_distance_ = std::max(0.0, keyframe_distance_);
  keyframe_yaw_ = std::max(0.0, keyframe_yaw_);
  pose_history_duration_ = std::max(0.1, pose_history_duration_);
  clock_rollback_threshold_ = std::max(1e-3, clock_rollback_threshold_);
  association_max_distance_ = std::max(0.05, association_max_distance_);
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
  pose_history_max_samples_ = std::max(2, pose_history_max_samples_);
  max_pending_cone_messages_ = std::max(1, max_pending_cone_messages_);
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

  pose_history_ = time_alignment::TimedPoseHistory(
    time_alignment::PoseHistoryOptions{
      static_cast<std::int64_t>(pose_history_duration_ * 1e9),
      static_cast<std::size_t>(pose_history_max_samples_),
      static_cast<std::int64_t>(clock_rollback_threshold_ * 1e9)});

  odom_information_.setZero();
  odom_information_(0, 0) = 1.0 / (odom_translation_sigma_ * odom_translation_sigma_);
  odom_information_(1, 1) = 1.0 / (odom_translation_sigma_ * odom_translation_sigma_);
  odom_information_(2, 2) = 1.0 / (odom_yaw_sigma_ * odom_yaw_sigma_);

  configureOptimizer();

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

  car_state_sub_ = create_subscription<eufs_msgs::msg::CarState>(
    car_state_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&GraphSlamNode::stateCallback, this, std::placeholders::_1));
  cones_sub_ = create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
    cones_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&GraphSlamNode::conesCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "g2o graph SLAM listening to car state '%s' and cones '%s'; "
    "retaining %.2f s/%d samples and deferring up to %d future cone frames",
    car_state_topic_.c_str(),
    cones_topic_.c_str(),
    pose_history_duration_,
    pose_history_max_samples_,
    max_pending_cone_messages_);
}

void GraphSlamNode::configureOptimizer()
{
  optimizer_.setVerbose(false);

  using BlockSolverType = g2o::BlockSolver_3_2;
  auto linear_solver =
    std::make_unique<g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>>();
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
  pose_history_.clear();
  pending_cone_messages_.clear();
  keyframe_stamps_ns_.clear();
  cone_edge_pose_graph_ids_.clear();

  next_vertex_id_ = 0;
  next_edge_id_ = 0;
  keyframes_since_last_optimization_ = 0;
  last_optimization_time_sec_ = -1.0;
  last_visual_publish_time_sec_ = -1.0;
  last_landmark_delete_time_sec_ = -1.0;
  has_latest_pose_ = false;
}

void GraphSlamNode::stateCallback(const eufs_msgs::msg::CarState::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const g2o::SE2 raw_odom = poseFromCarState(*msg);
  const time_alignment::TimedPose2d timed_pose{
    stamp.nanoseconds(), pose2dFromSe2(raw_odom)};
  const time_alignment::PushStatus push_status = pose_history_.push(timed_pose);
  const bool non_forward_sample =
    push_status == time_alignment::PushStatus::InsertedOutOfOrder ||
    push_status == time_alignment::PushStatus::ReplacedDuplicate;

  if (push_status == time_alignment::PushStatus::ClockRollback) {
    RCLCPP_WARN(
      get_logger(),
      "CarState clock rolled back by more than %.3f s; resetting graph, pose history, "
      "and pending cone observations",
      clock_rollback_threshold_);
    resetGraph();
    (void)pose_history_.push(timed_pose);
  } else if (push_status == time_alignment::PushStatus::IgnoredTooOld) {
    RCLCPP_DEBUG(get_logger(), "Ignoring CarState older than the retained pose-history window");
    return;
  } else if (non_forward_sample) {
    RCLCPP_DEBUG(
      get_logger(),
      "Stored non-forward CarState sample at %.9f without changing the live graph pose",
      stamp.seconds());
    return;
  }

  if (poses_.empty()) {
    addInitialPose(raw_odom, stamp);
    latest_estimate_ = poses_.empty() ? raw_odom : poses_.back().vertex->estimate();
    has_latest_pose_ = true;
    publishEstimate();
    drainPendingConeMessages();
    return;
  }

  const g2o::SE2 live_estimate = estimateFromRawOdometry(raw_odom);
  latest_estimate_ = live_estimate;
  has_latest_pose_ = true;

  if (!shouldCreateKeyframe(raw_odom, stamp)) {
    publishLiveEstimate(stamp, live_estimate, raw_odom);
    drainPendingConeMessages();
    return;
  }

  addKeyframe(raw_odom, stamp);
  latest_estimate_ = estimateFromRawOdometry(raw_odom);
  publishEstimate();
  drainPendingConeMessages();
}

void GraphSlamNode::conesCallback(
  const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg)
{
  if (!hasValidConeHeader(*msg)) {
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping cone frame with a zero timestamp; observation time cannot be fabricated");
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping cone frame '%s'; expected observations in '%s'",
        msg->header.frame_id.c_str(), slam_base_frame_.c_str());
    }
    return;
  }

  if (tryProcessConeMessage(msg) == ConeProcessingStatus::AwaitingFuture) {
    enqueuePendingConeMessage(msg);
  }
}

GraphSlamNode::ConeProcessingStatus GraphSlamNode::tryProcessConeMessage(
  const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr & msg)
{
  if (!hasValidConeHeader(*msg)) {
    return ConeProcessingStatus::Invalid;
  }

  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  const time_alignment::PoseLookup pose_lookup =
    pose_history_.interpolate(stamp.nanoseconds());
  if (pose_lookup.status == time_alignment::LookupStatus::Empty ||
    pose_lookup.status == time_alignment::LookupStatus::AwaitingFuture)
  {
    return ConeProcessingStatus::AwaitingFuture;
  }
  if (pose_lookup.status == time_alignment::LookupStatus::TooOld) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Dropping cone frame at %.9f because it is older than the retained CarState history",
      stamp.seconds());
    return ConeProcessingStatus::TooOld;
  }

  const std::optional<std::size_t> pose_index =
    time_alignment::latestIndexNotAfter(keyframe_stamps_ns_, stamp.nanoseconds());
  if (!pose_index.has_value() || *pose_index >= poses_.size()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Dropping cone frame at %.9f because no causal keyframe exists at or before it",
      stamp.seconds());
    return ConeProcessingStatus::TooOld;
  }

  ObservationUpdate update =
    addConeObservations(*msg, poses_[*pose_index], pose_lookup.pose);
  if (update.added_edges > 0U) {
    maybeOptimize();
  }
  if (update.added_edges > 0U ||
    update.updated_landmarks > 0U ||
    update.deleted_landmarks > 0U)
  {
    publishGraphVisuals(stamp);
  }
  return ConeProcessingStatus::Processed;
}

void GraphSlamNode::enqueuePendingConeMessage(
  const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr & msg)
{
  const std::int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  const auto insertion_point = std::upper_bound(
    pending_cone_messages_.begin(), pending_cone_messages_.end(), stamp_ns,
    [](std::int64_t candidate_stamp_ns, const auto & existing) {
      return candidate_stamp_ns < rclcpp::Time(existing->header.stamp).nanoseconds();
    });
  pending_cone_messages_.insert(insertion_point, msg);

  if (pending_cone_messages_.size() >
    static_cast<std::size_t>(max_pending_cone_messages_))
  {
    const rclcpp::Time dropped_stamp(pending_cone_messages_.back()->header.stamp);
    pending_cone_messages_.pop_back();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Future cone queue reached %d frames; dropping farthest frame at %.9f",
      max_pending_cone_messages_, dropped_stamp.seconds());
  }
}

void GraphSlamNode::drainPendingConeMessages()
{
  std::deque<eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr> pending =
    std::move(pending_cone_messages_);
  pending_cone_messages_.clear();

  for (const auto & msg : pending) {
    if (tryProcessConeMessage(msg) == ConeProcessingStatus::AwaitingFuture) {
      pending_cone_messages_.push_back(msg);
    }
  }
}

bool GraphSlamNode::hasValidConeHeader(
  const eufs_msgs::msg::ConeArrayWithCovariance & msg) const
{
  const bool has_stamp = msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0U;
  return has_stamp && msg.header.frame_id == slam_base_frame_;
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
  keyframe_stamps_ns_.push_back(stamp.nanoseconds());
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
  keyframe_stamps_ns_.push_back(stamp.nanoseconds());
  ++keyframes_since_last_optimization_;
}

GraphSlamNode::ObservationUpdate GraphSlamNode::addConeObservations(
  const eufs_msgs::msg::ConeArrayWithCovariance & msg,
  PoseRecord & pose,
  const time_alignment::Pose2d & raw_observation_pose)
{
  const bool add_edges =
    process_every_cone_message_ ||
    cone_edge_pose_graph_ids_.find(pose.graph_id) == cone_edge_pose_graph_ids_.end();
  const bool update_landmarks = update_existing_landmarks_;
  const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
  const std::vector<ConeObservation> observations = extractConeObservations(msg);
  const bool update_deletions =
    shouldUpdateLandmarkDeletion(stamp, add_edges && !observations.empty());

  if (!add_edges && !update_landmarks && !update_deletions) {
    return ObservationUpdate{0U, 0U, 0U};
  }

  if (!add_edges &&
    !process_every_cone_message_)
  {
    RCLCPP_DEBUG(
      get_logger(),
      "Skipping duplicate cone graph edges for pose %d; updating landmark deletion state",
      pose.graph_id);
  }

  const time_alignment::ObservationFrameAlignment frame_alignment =
    time_alignment::makeObservationFrameAlignment(
    pose2dFromSe2(pose.raw_odom),
    pose2dFromSe2(pose.vertex->estimate()),
    raw_observation_pose);
  const g2o::SE2 observation_pose = se2FromPose2d(frame_alignment.map_to_observation);

  std::size_t added_edges = 0U;
  std::size_t updated_landmarks = 0U;
  std::vector<std::size_t> observed_landmark_indices;
  observed_landmark_indices.reserve(observations.size());

  for (const ConeObservation & observation : observations) {
    const time_alignment::AlignedObservation2d aligned =
      time_alignment::alignObservation(
      frame_alignment, observation.measurement, observation.covariance);
    const ConeObservation keyframe_observation{
      aligned.keyframe_point, aligned.keyframe_covariance, observation.color};

    const int landmark_index = findAssociatedLandmark(aligned.map_point, observation.color);
    LandmarkRecord * landmark = nullptr;
    std::size_t observed_index = 0U;
    bool has_observed_index = false;

    if (landmark_index >= 0) {
      observed_index = static_cast<std::size_t>(landmark_index);
      has_observed_index = true;
      landmark = &landmarks_[observed_index];
      if (landmark->color == ConeColor::Unknown && observation.color != ConeColor::Unknown) {
        landmark->color = observation.color;
      }
      if (update_landmarks &&
        updateLandmarkEstimate(*landmark, aligned.map_point, aligned.map_covariance))
      {
        ++updated_landmarks;
      }
    } else if (add_edges) {
      landmark =
        addLandmark(
        aligned.map_point,
        aligned.map_covariance,
        observation.color);
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
      if (addObservationEdge(keyframe_observation, pose.vertex, *landmark)) {
        ++added_edges;
      }
    }
  }

  if (added_edges > 0U && !process_every_cone_message_) {
    cone_edge_pose_graph_ids_.insert(pose.graph_id);
  }

  const std::size_t deleted_landmarks = update_deletions ?
    deleteMissedVisibleLandmarks(observation_pose, observed_landmark_indices) :
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

        observations.push_back(
          ConeObservation{
          Eigen::Vector2d(cone.point.x, cone.point.y),
          covarianceFromCone(cone),
          color});
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

int GraphSlamNode::findAssociatedLandmark(
  const Eigen::Vector2d & map_point,
  ConeColor color) const
{
  const double max_distance_sq = association_max_distance_ * association_max_distance_;
  double best_distance_sq = max_distance_sq;
  int best_index = -1;

  for (std::size_t i = 0; i < landmarks_.size(); ++i) {
    const LandmarkRecord & landmark = landmarks_[i];
    if (!colorsCompatible(color, landmark.color)) {
      continue;
    }

    const double distance_sq = (landmark.vertex->estimate() - map_point).squaredNorm();
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_index = static_cast<int>(i);
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

GraphSlamNode::LandmarkRecord * GraphSlamNode::addLandmark(
  const Eigen::Vector2d & map_point,
  const Eigen::Matrix2d & covariance,
  ConeColor color)
{
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

  landmarks_.push_back(LandmarkRecord{vertex->id(), color, vertex, covariance, 0U, 0});
  return &landmarks_.back();
}

bool GraphSlamNode::updateLandmarkEstimate(
  LandmarkRecord & landmark,
  const Eigen::Vector2d & map_point,
  const Eigen::Matrix2d & covariance)
{
  if (!update_existing_landmarks_ || landmark_update_gain_ <= 0.0) {
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

bool GraphSlamNode::addObservationEdge(
  const ConeObservation & observation,
  g2o::VertexSE2 * pose_vertex,
  LandmarkRecord & landmark)
{
  auto * edge = new g2o::EdgeSE2PointXY();
  edge->setId(next_edge_id_++);
  edge->setVertex(0, pose_vertex);
  edge->setVertex(1, landmark.vertex);
  edge->setMeasurement(observation.measurement);
  edge->setInformation(observation.covariance.inverse());

  if (robust_kernel_delta_ > 0.0) {
    auto * robust_kernel = new g2o::RobustKernelHuber();
    robust_kernel->setDelta(robust_kernel_delta_);
    edge->setRobustKernel(robust_kernel);
  }

  if (!optimizer_.addEdge(edge)) {
    RCLCPP_ERROR(get_logger(), "Failed to add cone observation edge");
    delete edge;
    return false;
  }

  ++landmark.observations;
  landmark.consecutive_misses = 0;
  return true;
}

std::size_t GraphSlamNode::deleteMissedVisibleLandmarks(
  const g2o::SE2 & observation_pose,
  const std::vector<std::size_t> & observed_landmark_indices)
{
  if (!delete_stale_landmarks_ || landmarks_.empty()) {
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

    if (!landmarkExpectedVisible(observation_pose, landmark)) {
      continue;
    }

    ++landmark.consecutive_misses;
    if (landmark.consecutive_misses >= landmark_missed_observations_to_delete_) {
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
  const g2o::SE2 & observation_pose,
  const LandmarkRecord & landmark) const
{
  const Eigen::Vector2d relative =
    observation_pose.inverse() * landmark.vertex->estimate();
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
  keyframes_since_last_optimization_ = 0;
  last_optimization_time_sec_ = stamp_sec;
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

  response->success = optimizer_.save(g2o_output_path_.c_str());
  response->message = response->success ?
    "Saved graph to " + g2o_output_path_ :
    "Failed to save graph to " + g2o_output_path_;
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
