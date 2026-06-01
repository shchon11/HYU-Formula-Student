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
#include <functional>
#include <limits>
#include <memory>
#include <utility>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace eufs_graph_slam
{

GraphSlamNode::GraphSlamNode()
: Node("graph_slam_node"),
  keyframe_distance_(0.5),
  keyframe_yaw_(0.25),
  keyframe_max_dt_(1.0),
  association_max_distance_(1.2),
  min_observation_range_(0.2),
  max_observation_range_(30.0),
  default_observation_sigma_(0.25),
  min_observation_variance_(0.01),
  odom_translation_sigma_(0.05),
  odom_yaw_sigma_(0.03),
  robust_kernel_delta_(1.0),
  marker_scale_(0.3),
  optimize_every_n_keyframes_(3),
  optimization_iterations_(10),
  landmark_min_observations_to_publish_(1),
  max_landmarks_(400),
  max_optimization_poses_(1500),
  path_max_poses_to_publish_(1000),
  use_cone_covariance_(true),
  process_every_cone_message_(false),
  publish_tf_(true),
  optimize_min_interval_(1.0),
  last_optimization_time_sec_(-1.0),
  next_vertex_id_(0),
  next_edge_id_(0),
  keyframes_since_last_optimization_(0),
  last_cone_pose_graph_id_(-1)
{
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/ground_truth/odom");
  cones_topic_ = declare_parameter<std::string>("cones_topic", "/cones");
  map_topic_ = declare_parameter<std::string>("map_topic", "/graph_slam/map");
  slam_odom_topic_ = declare_parameter<std::string>("slam_odom_topic", "/graph_slam/odom");
  path_topic_ = declare_parameter<std::string>("path_topic", "/graph_slam/path");
  marker_topic_ = declare_parameter<std::string>("marker_topic", "/graph_slam/markers");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  slam_base_frame_ = declare_parameter<std::string>("slam_base_frame", "base_footprint");
  g2o_output_path_ = declare_parameter<std::string>("g2o_output_path", "/tmp/eufs_graph_slam.g2o");

  keyframe_distance_ = declare_parameter<double>("keyframe_distance", keyframe_distance_);
  keyframe_yaw_ = declare_parameter<double>("keyframe_yaw", keyframe_yaw_);
  keyframe_max_dt_ = declare_parameter<double>("keyframe_max_dt", keyframe_max_dt_);
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

  optimize_min_interval_ =
    declare_parameter<double>("optimize_min_interval", optimize_min_interval_);

  use_cone_covariance_ = declare_parameter<bool>("use_cone_covariance", use_cone_covariance_);
  process_every_cone_message_ =
    declare_parameter<bool>("process_every_cone_message", process_every_cone_message_);
  publish_tf_ = declare_parameter<bool>("publish_tf", publish_tf_);

  keyframe_distance_ = std::max(0.0, keyframe_distance_);
  keyframe_yaw_ = std::max(0.0, keyframe_yaw_);
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
  optimize_min_interval_ = std::max(0.0, optimize_min_interval_);

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
      "Publishing graph SLAM TF '%s' -> '%s'; keep simulator publish_gt_tf disabled",
      map_frame_.c_str(),
      slam_base_frame_.c_str());
  }

  reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "reset",
    std::bind(&GraphSlamNode::handleReset, this, std::placeholders::_1, std::placeholders::_2));
  save_graph_srv_ = create_service<std_srvs::srv::Trigger>(
    "save_graph",
    std::bind(&GraphSlamNode::handleSaveGraph, this, std::placeholders::_1, std::placeholders::_2));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&GraphSlamNode::odomCallback, this, std::placeholders::_1));
  cones_sub_ = create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
    cones_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&GraphSlamNode::conesCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "g2o graph SLAM listening to odom '%s' and cones '%s'",
    odom_topic_.c_str(),
    cones_topic_.c_str());
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

  next_vertex_id_ = 0;
  next_edge_id_ = 0;
  keyframes_since_last_optimization_ = 0;
  last_cone_pose_graph_id_ = -1;
  last_optimization_time_sec_ = -1.0;
}

void GraphSlamNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time stamp = stampOrNow(msg->header.stamp, get_clock());
  const g2o::SE2 raw_odom = poseFromOdometry(*msg);

  if (poses_.empty()) {
    addInitialPose(raw_odom, stamp);
    publishEstimate();
    return;
  }

  if (!shouldCreateKeyframe(raw_odom, stamp)) {
    return;
  }

  addKeyframe(raw_odom, stamp);
  publishEstimate();
}

void GraphSlamNode::conesCallback(
  const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg)
{
  if (poses_.empty()) {
    return;
  }

  const std::size_t added_edges = addConeObservations(*msg, false);
  if (added_edges > 0U) {
    maybeOptimize();
    publishEstimate();
  }
}

g2o::SE2 GraphSlamNode::poseFromOdometry(const nav_msgs::msg::Odometry & msg) const
{
  return g2o::SE2(
    msg.pose.pose.position.x,
    msg.pose.pose.position.y,
    yawFromOdometry(msg));
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
  ++keyframes_since_last_optimization_;
}

std::size_t GraphSlamNode::addConeObservations(
  const eufs_msgs::msg::ConeArrayWithCovariance & msg,
  bool force_process)
{
  if (poses_.empty()) {
    return 0U;
  }

  PoseRecord & pose = poses_.back();
  if (!force_process &&
    !process_every_cone_message_ &&
    last_cone_pose_graph_id_ == pose.graph_id)
  {
    return 0U;
  }

  const std::vector<ConeObservation> observations = extractConeObservations(msg);
  if (observations.empty()) {
    return 0U;
  }

  if (!process_every_cone_message_) {
    last_cone_pose_graph_id_ = pose.graph_id;
  }

  std::size_t added_edges = 0U;

  for (const ConeObservation & observation : observations) {
    const Eigen::Vector2d map_point = pose.vertex->estimate() * observation.measurement;

    const int landmark_index = findAssociatedLandmark(map_point, observation.color);
    LandmarkRecord * landmark = nullptr;

    if (landmark_index >= 0) {
      landmark = &landmarks_[static_cast<std::size_t>(landmark_index)];
      if (landmark->color == ConeColor::Unknown && observation.color != ConeColor::Unknown) {
        landmark->color = observation.color;
      }
    } else {
      landmark = addLandmark(map_point, observation.color);
    }

    if (landmark == nullptr) {
      continue;
    }

    addObservationEdge(observation, pose.vertex, *landmark);
    ++added_edges;
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Added %zu cone observation edges from %zu visible cones",
    added_edges,
    observations.size());

  return added_edges;
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

  landmarks_.push_back(LandmarkRecord{vertex->id(), color, vertex, 0U});
  return &landmarks_.back();
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
  edge->setInformation(observation.covariance.inverse());

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

void GraphSlamNode::publishEstimate()
{
  if (poses_.empty()) {
    return;
  }

  const rclcpp::Time stamp = poses_.back().stamp;
  publishMap(stamp);
  publishPath(stamp);
  publishOdometry(stamp);
  publishMarkers(stamp);
  publishTransform(stamp);
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
    cone.covariance = {0.0, 0.0, 0.0, 0.0};

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

void GraphSlamNode::publishOdometry(const rclcpp::Time & stamp)
{
  const PoseRecord & pose_record = poses_.back();
  const g2o::SE2 estimate = pose_record.vertex->estimate();

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

void GraphSlamNode::publishTransform(const rclcpp::Time & stamp)
{
  if (!publish_tf_ || !tf_broadcaster_ || poses_.empty()) {
    return;
  }

  const g2o::SE2 estimate = poses_.back().vertex->estimate();

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = map_frame_;
  transform.header.stamp = stamp;
  transform.child_frame_id = slam_base_frame_;
  transform.transform.translation.x = estimate.translation().x();
  transform.transform.translation.y = estimate.translation().y();
  transform.transform.translation.z = 0.0;
  transform.transform.rotation = quaternionFromYaw(estimate.rotation().angle());

  tf_broadcaster_->sendTransform(transform);
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

double GraphSlamNode::yawFromOdometry(const nav_msgs::msg::Odometry & msg)
{
  const auto & q = msg.pose.pose.orientation;
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
