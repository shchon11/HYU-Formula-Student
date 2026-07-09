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
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "eufs_msgs/msg/car_state.hpp"
#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "eufs_msgs/msg/cone_with_covariance.hpp"
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
  };

  void configureOptimizer();
  void resetGraph();

  void stateCallback(const eufs_msgs::msg::CarState::SharedPtr msg);
  void conesCallback(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg);
  void gnssOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void relocalizeTo(const g2o::SE2 & pose);
  g2o::SE2 scanMatchNearClick(const g2o::SE2 & click) const;
  g2o::SE2 latestRawOdom() const;

  g2o::SE2 poseFromCarState(const eufs_msgs::msg::CarState & msg) const;
  g2o::SE2 estimateFromRawOdometry(const g2o::SE2 & raw_odom) const;
  bool shouldCreateKeyframe(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp) const;
  void addInitialPose(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp);
  void addKeyframe(const g2o::SE2 & raw_odom, const rclcpp::Time & stamp);
  // Anchor a keyframe's (x,y) to the latest GNSS absolute fix with a unary
  // EdgeSE2XYPrior, gated on the fix's freshness and covariance so the anchor
  // fades out smoothly as the GNSS/INS solution degrades (RTK dropout).
  void maybeAddGnssPrior(g2o::VertexSE2 * vertex, const rclcpp::Time & stamp);

  ObservationUpdate addConeObservations(
    const eufs_msgs::msg::ConeArrayWithCovariance & msg,
    bool force_process);
  std::vector<ConeObservation> extractConeObservations(
    const eufs_msgs::msg::ConeArrayWithCovariance & msg) const;
  Eigen::Matrix2d covarianceFromCone(
    const eufs_msgs::msg::ConeWithCovariance & cone) const;
  Eigen::Matrix2d covarianceInMapFrame(
    const g2o::SE2 & pose,
    const Eigen::Matrix2d & local_covariance) const;

  int findAssociatedLandmark(
    const Eigen::Vector2d & map_point,
    const Eigen::Matrix2d & map_covariance,
    ConeColor color,
    bool * ambiguous) const;
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

  void recordRawOdometry(double stamp_sec, const g2o::SE2 & raw_odom);
  g2o::SE2 rawOdomAt(double stamp_sec) const;

  void maybeOptimize();
  void onOptimizeTimer();
  void optimizeGraph();
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
  void publishStatus();

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
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gnss_odom_sub_;
  rclcpp::Publisher<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
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
  double relocalize_search_radius_;
  double relocalize_search_yaw_;
  double relocalize_inlier_distance_;
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
  double lap_min_distance_;
  double lap_return_radius_;
  double lap_return_yaw_;
  int localization_window_poses_;
  double traveled_distance_;
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
  bool loop_closure_seen_since_optimize_;
  int loop_closure_optimize_cycles_;

  g2o::SE2 latest_estimate_;
  bool has_latest_pose_;

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

  // Latest GNSS absolute fix (map/ENU frame) from the bridge's /gnss/odom,
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

}  // namespace eufs_graph_slam

#endif  // EUFS_GRAPH_SLAM__GRAPH_SLAM_NODE_HPP_
