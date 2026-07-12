#include "planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>

#include "eufs_msgs/msg/waypoint.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace global_planner
{

PlannerNode::PlannerNode()
: Node("planner_node")
{
  declareParameters();
  loadParameters();

  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
  const auto valid_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  cone_map_sub_ = create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
    cone_map_topic_, latched_qos,
    std::bind(&PlannerNode::onConeMap, this, std::placeholders::_1));
  ego_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    ego_odom_topic_, odom_qos,
    std::bind(&PlannerNode::onEgoOdom, this, std::placeholders::_1));
  graph_slam_status_sub_ = create_subscription<std_msgs::msg::String>(
    graph_slam_status_topic_, latched_qos,
    std::bind(&PlannerNode::onGraphSlamStatus, this, std::placeholders::_1));
  if (!graph_slam_map_converged_topic_.empty()) {
    map_converged_sub_ = create_subscription<std_msgs::msg::Bool>(
      graph_slam_map_converged_topic_, latched_qos,
      std::bind(&PlannerNode::onMapConverged, this, std::placeholders::_1));
  }

  global_waypoints_pub_ = create_publisher<eufs_msgs::msg::WaypointArrayStamped>(
    global_waypoints_topic_, latched_qos);
  // RViz-friendly mirror of the waypoint message (nav_msgs/Path, latched).
  global_path_viz_pub_ = create_publisher<nav_msgs::msg::Path>(
    global_waypoints_topic_ + "/path", latched_qos);
  global_path_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
    global_path_valid_topic_, valid_qos);

  const double heartbeat_hz = std::max(0.1, valid_heartbeat_hz_);
  heartbeat_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / heartbeat_hz)),
    std::bind(&PlannerNode::onHeartbeat, this));

  RCLCPP_INFO(
    get_logger(),
    "planner_node started: map=%s odom=%s status=%s -> waypoints=%s valid=%s",
    cone_map_topic_.c_str(), ego_odom_topic_.c_str(), graph_slam_status_topic_.c_str(),
    global_waypoints_topic_.c_str(), global_path_valid_topic_.c_str());
}

void PlannerNode::declareParameters()
{
  declare_parameter<std::string>("cone_map_topic", "/localization/cone_map");
  declare_parameter<std::string>("ego_odom_topic", "/localization/ego_odom");
  declare_parameter<std::string>("graph_slam_status_topic", "/graph_slam/status");
  declare_parameter<std::string>("graph_slam_map_converged_topic", "/graph_slam/map_converged");
  declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
  declare_parameter<std::string>("global_path_valid_topic", "/planning/global_path_valid");
  declare_parameter<double>("valid_heartbeat_hz", 5.0);
  declare_parameter<int>("min_cones_per_side", 3);
  declare_parameter<double>("max_boundary_gap_m", 12.0);
  declare_parameter<double>("min_track_width_m", 2.0);
  declare_parameter<double>("max_track_width_m", 6.0);
  declare_parameter<double>("close_loop_distance_m", 5.0);
  declare_parameter<double>("waypoint_spacing_m", 0.5);
  declare_parameter<double>("default_speed_mps", 4.0);
  declare_parameter<double>("max_speed_mps", 4.5);
  declare_parameter<double>("min_speed_mps", 1.5);
  declare_parameter<double>("max_lateral_accel_mps2", 6.0);
  declare_parameter<double>("max_accel_mps2", 3.0);
  declare_parameter<double>("max_decel_mps2", 5.0);
  declare_parameter<double>("duplicate_point_tolerance", 0.001);
  declare_parameter<double>("odom_timeout_sec", 0.5);
}

void PlannerNode::loadParameters()
{
  cone_map_topic_ = get_parameter("cone_map_topic").as_string();
  ego_odom_topic_ = get_parameter("ego_odom_topic").as_string();
  graph_slam_status_topic_ = get_parameter("graph_slam_status_topic").as_string();
  graph_slam_map_converged_topic_ = get_parameter("graph_slam_map_converged_topic").as_string();
  global_waypoints_topic_ = get_parameter("global_waypoints_topic").as_string();
  global_path_valid_topic_ = get_parameter("global_path_valid_topic").as_string();
  valid_heartbeat_hz_ = get_parameter("valid_heartbeat_hz").as_double();
  min_cones_per_side_ = get_parameter("min_cones_per_side").as_int();
  max_boundary_gap_m_ = get_parameter("max_boundary_gap_m").as_double();
  min_track_width_m_ = get_parameter("min_track_width_m").as_double();
  max_track_width_m_ = get_parameter("max_track_width_m").as_double();
  close_loop_distance_m_ = get_parameter("close_loop_distance_m").as_double();
  waypoint_spacing_m_ = get_parameter("waypoint_spacing_m").as_double();
  default_speed_mps_ = get_parameter("default_speed_mps").as_double();
  max_speed_mps_ = get_parameter("max_speed_mps").as_double();
  min_speed_mps_ = get_parameter("min_speed_mps").as_double();
  max_lateral_accel_mps2_ = get_parameter("max_lateral_accel_mps2").as_double();
  max_accel_mps2_ = get_parameter("max_accel_mps2").as_double();
  max_decel_mps2_ = get_parameter("max_decel_mps2").as_double();
  duplicate_point_tolerance_ = get_parameter("duplicate_point_tolerance").as_double();
  odom_timeout_sec_ = get_parameter("odom_timeout_sec").as_double();

  min_cones_per_side_ = std::max(1, min_cones_per_side_);
  max_boundary_gap_m_ = std::max(0.0, max_boundary_gap_m_);
  min_track_width_m_ = std::max(0.0, min_track_width_m_);
  max_track_width_m_ = std::max(min_track_width_m_, max_track_width_m_);
  duplicate_point_tolerance_ = std::isfinite(duplicate_point_tolerance_) ?
    std::max(0.0, duplicate_point_tolerance_) : 0.0;
  waypoint_spacing_valid_ = std::isfinite(waypoint_spacing_m_) &&
    waypoint_spacing_m_ >= kMinimumWaypointSpacingM;
  if (!waypoint_spacing_valid_) {
    waypoint_spacing_invalid_reason_ = "waypoint_spacing_m must be finite and >= 0.05 m";
    RCLCPP_WARN(
      get_logger(),
      "%s; planner will not publish global waypoints.",
      waypoint_spacing_invalid_reason_.c_str());
  }
  odom_timeout_sec_ = std::max(0.0, odom_timeout_sec_);
  default_speed_mps_ = std::max(0.0, default_speed_mps_);
}

void PlannerNode::onConeMap(const eufs_msgs::msg::ConeArrayWithCovariance::SharedPtr msg)
{
  latest_cone_map_ = msg;
  has_cone_map_ = true;
  ++cone_map_version_;
}

void PlannerNode::onEgoOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_ego_odom_ = *msg;
  latest_ego_odom_receive_time_ = now();
  has_ego_odom_ = true;
}

void PlannerNode::onGraphSlamStatus(const std_msgs::msg::String::SharedPtr msg)
{
  graph_slam_status_ = msg->data;
  has_graph_slam_status_ = true;
}

void PlannerNode::onMapConverged(const std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_DEBUG(
    get_logger(), "Graph SLAM map_converged diagnostic: %s",
    msg->data ? "true" : "false");
}

namespace
{

nav_msgs::msg::Path buildPathMessage(const eufs_msgs::msg::WaypointArrayStamped & msg)
{
  nav_msgs::msg::Path path;
  path.header = msg.header;
  path.poses.reserve(msg.waypoints.size());
  for (const auto & wp : msg.waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = wp.x_m;
    pose.pose.position.y = wp.y_m;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.z = std::sin(0.5 * wp.psi_rad);
    pose.pose.orientation.w = std::cos(0.5 * wp.psi_rad);
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace

void PlannerNode::onHeartbeat()
{
  std::string reason;
  if (!inputsAllowPlanning(reason)) {
    setInvalid(reason);
    publishValidity(false);
    return;
  }

  std::vector<PlannerWaypoint> waypoints;
  if (!buildCenterlineFromSlamMap(*latest_cone_map_, latest_ego_odom_, centerlineConfig(), waypoints, reason)) {
    setInvalid(reason);
    publishValidity(false);
    return;
  }

  if (!path_valid_ || published_cone_map_version_ != cone_map_version_) {
    auto waypoint_msg = buildWaypointMessage(waypoints);
    global_waypoints_pub_->publish(waypoint_msg);
    global_path_viz_pub_->publish(buildPathMessage(waypoint_msg));
    published_cone_map_version_ = cone_map_version_;
    RCLCPP_INFO(
      get_logger(),
      "Published %zu SLAM-map global waypoint(s), frame=%s",
      waypoint_msg.waypoints.size(), waypoint_msg.header.frame_id.c_str());
  }

  path_valid_ = true;
  warned_invalid_ = false;
  last_invalid_reason_.clear();
  publishValidity(true);
}

bool PlannerNode::inputsAllowPlanning(std::string & reason) const
{
  if (!waypoint_spacing_valid_) {
    reason = waypoint_spacing_invalid_reason_;
    return false;
  }
  if (!has_graph_slam_status_) {
    reason = "waiting for graph slam status";
    return false;
  }
  if (graph_slam_status_ != "localization") {
    reason = "graph slam status is " + graph_slam_status_;
    return false;
  }
  if (!has_cone_map_ || !latest_cone_map_) {
    reason = "waiting for cone map";
    return false;
  }
  if (!has_ego_odom_) {
    reason = "waiting for ego odom";
    return false;
  }

  const double odom_age = (now() - latest_ego_odom_receive_time_).seconds();
  if (odom_age > odom_timeout_sec_) {
    std::ostringstream stream;
    stream << "ego odom stale: age=" << odom_age << " sec";
    reason = stream.str();
    return false;
  }
  return true;
}

SlamCenterlineConfig PlannerNode::centerlineConfig() const
{
  return SlamCenterlineConfig{
    min_cones_per_side_,
    max_boundary_gap_m_,
    min_track_width_m_,
    max_track_width_m_,
    close_loop_distance_m_,
    waypoint_spacing_m_,
    duplicate_point_tolerance_};
}

eufs_msgs::msg::WaypointArrayStamped PlannerNode::buildWaypointMessage(
  const std::vector<PlannerWaypoint> & waypoints) const
{
  eufs_msgs::msg::WaypointArrayStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = latest_cone_map_->header.frame_id.empty() ?
    std::string("map") : latest_cone_map_->header.frame_id;
  msg.waypoints.reserve(waypoints.size());

  VelocityProfileConfig vp;
  vp.max_speed_mps = max_speed_mps_;
  vp.min_speed_mps = min_speed_mps_;
  vp.max_lateral_accel_mps2 = max_lateral_accel_mps2_;
  vp.max_accel_mps2 = max_accel_mps2_;
  vp.max_decel_mps2 = max_decel_mps2_;
  const auto profile = computeVelocityProfile(waypoints, vp);

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const auto & point = waypoints[i];
    // Curvature/friction profile when available; flat default otherwise.
    const double vx = i < profile.size() ? profile[i].vx : default_speed_mps_;
    const double ax = i < profile.size() ? profile[i].ax : 0.0;
    eufs_msgs::msg::Waypoint waypoint;
    waypoint.position.x = point.x;
    waypoint.position.y = point.y;
    waypoint.position.z = 0.0;
    waypoint.speed = vx;
    waypoint.suggested_steering = 0.0;
    waypoint.x_m = point.x;
    waypoint.y_m = point.y;
    waypoint.s_m = point.s;
    waypoint.psi_rad = point.psi;
    waypoint.kappa_radpm = point.kappa;
    waypoint.vx_mps = vx;
    waypoint.ax_mps2 = ax;
    msg.waypoints.push_back(waypoint);
  }

  return msg;
}

void PlannerNode::publishValidity(bool valid)
{
  std_msgs::msg::Bool msg;
  msg.data = valid;
  global_path_valid_pub_->publish(msg);
}

void PlannerNode::setInvalid(const std::string & reason)
{
  path_valid_ = false;
  if (!warned_invalid_ || reason != last_invalid_reason_) {
    RCLCPP_WARN(get_logger(), "Global path invalid: %s", reason.c_str());
    warned_invalid_ = true;
    last_invalid_reason_ = reason;
  }
}

}  // namespace global_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<global_planner::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
