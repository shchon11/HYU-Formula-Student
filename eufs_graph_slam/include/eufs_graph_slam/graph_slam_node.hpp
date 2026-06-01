// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef EUFS_GRAPH_SLAM__GRAPH_SLAM_NODE_HPP_
#define EUFS_GRAPH_SLAM__GRAPH_SLAM_NODE_HPP_

#include <Eigen/Core>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d/vertex_se2.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "eufs_msgs/msg/car_state.hpp"
#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "eufs_msgs/msg/cone_with_covariance.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace eufs_graph_slam
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
    std::size_t observations;
    int consecutive_misses;
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
    std::size_t deleted_landmarks;
  };

  void configureOptimizer();
  void resetGraph();

  void stateCallback(const eufs_msgs::msg::CarState::SharedPtr msg);
  void conesCallback(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);

  g2o::SE2 poseFromCarState(const eufs_msgs::msg::CarState & msg) const;
  g2o::SE2 estimateFromRawOdometry(const g2o::SE2 & raw_odom) const;
  bool shouldCreateKeyframe(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp) const;
  void addInitialPose(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp);
  void addKeyframe(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp);

  ObservationUpdate addConeObservations(
    const eufs_msgs::msg::ConeArrayWithCovariance & msg,
    bool force_process);
  std::vector<ConeObservation> extractConeObservations(
    const eufs_msgs::msg::ConeArrayWithCovariance & msg) const;
  Eigen::Matrix2d covarianceFromCone(
    const eufs_msgs::msg::ConeWithCovariance & cone) const;

  int findAssociatedLandmark(
    const Eigen::Vector2d & map_point,
    ConeColor color) const;
  bool colorsCompatible(ConeColor observation_color, ConeColor landmark_color) const;
  LandmarkRecord * addLandmark(const Eigen::Vector2d & map_point, ConeColor color);
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

  void maybeOptimize();
  void optimizeGraph();
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

  static double normalizeAngle(double angle);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);
  static rclcpp::Time stampOrNow(
    const builtin_interfaces::msg::Time & stamp,
    const rclcpp::Clock::SharedPtr & clock);
  static std_msgs::msg::ColorRGBA colorToRgba(ConeColor color, double alpha);
  static std::string colorName(ConeColor color);

  g2o::SparseOptimizer optimizer_;

  rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr car_state_sub_;
  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cones_sub_;
  rclcpp::Publisher<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_graph_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<PoseRecord> poses_;
  std::vector<LandmarkRecord> landmarks_;

  Eigen::Matrix3d odom_information_;

  std::string car_state_topic_;
  std::string cones_topic_;
  std::string map_topic_;
  std::string slam_odom_topic_;
  std::string path_topic_;
  std::string marker_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string slam_base_frame_;
  std::string g2o_output_path_;

  double keyframe_distance_;
  double keyframe_yaw_;
  double keyframe_max_dt_;
  double association_max_distance_;
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

  int optimize_every_n_keyframes_;
  int optimization_iterations_;
  int landmark_min_observations_to_publish_;
  int max_landmarks_;
  int max_optimization_poses_;
  int path_max_poses_to_publish_;
  int landmark_missed_observations_to_delete_;

  bool use_cone_covariance_;
  bool process_every_cone_message_;
  bool publish_tf_;
  bool delete_stale_landmarks_;

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
};

}  // namespace eufs_graph_slam

#endif  // EUFS_GRAPH_SLAM__GRAPH_SLAM_NODE_HPP_
