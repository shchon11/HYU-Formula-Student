#include "hyu_global_planner_trajectory_publisher_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <sstream>
#include <system_error>
#include <utility>

#include "hyu_msgs/msg/waypoint.hpp"

namespace hyu_global_planner
{

GlobalPlannerTrajectoryPublisherNode::GlobalPlannerTrajectoryPublisherNode()
: Node("hyu_global_planner_trajectory_publisher_node")
{
  declareParameters();
  loadParameters();

  const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
  global_waypoints_pub_ =
    create_publisher<hyu_msgs::msg::WaypointArrayStamped>(
    global_waypoints_topic_,
    latched_qos);
  const auto heartbeat_qos = rclcpp::QoS(1).reliable();
  global_path_valid_pub_ =
    create_publisher<std_msgs::msg::Bool>(global_path_valid_topic_, heartbeat_qos);

  RCLCPP_INFO(get_logger(), "global_waypoints_topic=%s", global_waypoints_topic_.c_str());
  RCLCPP_INFO(get_logger(), "global_path_valid_topic=%s", global_path_valid_topic_.c_str());

  publishValidityHeartbeat();

  checkAndReloadTrajectory();

  const auto period = std::chrono::duration<double>(
    std::max(0.1, reload_period_sec_));
  reload_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GlobalPlannerTrajectoryPublisherNode::checkAndReloadTrajectory, this));

  const auto heartbeat_period = std::chrono::duration<double>(
    1.0 / std::max(0.1, valid_heartbeat_hz_));
  valid_heartbeat_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(heartbeat_period),
    std::bind(&GlobalPlannerTrajectoryPublisherNode::publishValidityHeartbeat, this));
}

void GlobalPlannerTrajectoryPublisherNode::declareParameters()
{
  declare_parameter<std::string>("trajectory_csv_path", "");
  declare_parameter<std::string>("output_root", "");
  declare_parameter<std::string>("map_name", "");
  declare_parameter<std::string>("trajectory_filename", "traj_race_cl.csv");
  declare_parameter<std::string>("global_waypoints_topic", "/planning/global_waypoints");
  declare_parameter<std::string>("global_path_valid_topic", "/planning/global_path_valid");
  declare_parameter<std::string>("frame_id", "map");
  declare_parameter<double>("reload_period_sec", 1.0);
  declare_parameter<double>("valid_heartbeat_hz", 5.0);
  declare_parameter<double>("duplicate_point_tolerance", 1.0e-4);
  declare_parameter<int>("min_waypoint_count", 3);
  declare_parameter<bool>("recompute_s_if_invalid", true);
}

void GlobalPlannerTrajectoryPublisherNode::loadParameters()
{
  trajectory_csv_path_ = get_parameter("trajectory_csv_path").as_string();
  output_root_ = get_parameter("output_root").as_string();
  map_name_ = get_parameter("map_name").as_string();
  trajectory_filename_ = get_parameter("trajectory_filename").as_string();
  global_waypoints_topic_ = get_parameter("global_waypoints_topic").as_string();
  global_path_valid_topic_ = get_parameter("global_path_valid_topic").as_string();
  frame_id_ = get_parameter("frame_id").as_string();
  reload_period_sec_ = get_parameter("reload_period_sec").as_double();
  valid_heartbeat_hz_ = get_parameter("valid_heartbeat_hz").as_double();
  duplicate_point_tolerance_ = get_parameter("duplicate_point_tolerance").as_double();
  min_waypoint_count_ = get_parameter("min_waypoint_count").as_int();
  recompute_s_if_invalid_ = get_parameter("recompute_s_if_invalid").as_bool();

  if (min_waypoint_count_ < 1) {
    RCLCPP_WARN(get_logger(), "min_waypoint_count must be >= 1. Clamping to 1.");
    min_waypoint_count_ = 1;
  }
  if (duplicate_point_tolerance_ < 0.0) {
    RCLCPP_WARN(get_logger(), "duplicate_point_tolerance must be >= 0. Clamping to 0.");
    duplicate_point_tolerance_ = 0.0;
  }
  if (valid_heartbeat_hz_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "valid_heartbeat_hz must be > 0. Clamping to 0.1.");
    valid_heartbeat_hz_ = 0.1;
  }
}

bool GlobalPlannerTrajectoryPublisherNode::resolveTrajectoryPath(
  std::filesystem::path & resolved_path,
  std::string & error_message) const
{
  if (!trajectory_csv_path_.empty()) {
    resolved_path = std::filesystem::path(trajectory_csv_path_);
    return true;
  }

  if (output_root_.empty() || map_name_.empty()) {
    error_message =
      "trajectory_csv_path is empty and output_root/map_name parameters are incomplete";
    return false;
  }

  resolved_path =
    std::filesystem::path(output_root_) /
    std::filesystem::path(map_name_) /
    std::filesystem::path(trajectory_filename_);
  return true;
}

TrajectoryValidationOptions GlobalPlannerTrajectoryPublisherNode::trajectoryValidationOptions() const
{
  return TrajectoryValidationOptions{
    duplicate_point_tolerance_, min_waypoint_count_, recompute_s_if_invalid_};
}

hyu_msgs::msg::WaypointArrayStamped GlobalPlannerTrajectoryPublisherNode::buildWaypointMessage(
  const std::vector<TrajectoryPoint> & points)
{
  hyu_msgs::msg::WaypointArrayStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  msg.waypoints.reserve(points.size());

  for (const auto & point : points) {
    hyu_msgs::msg::Waypoint waypoint;
    waypoint.position.x = point.x;
    waypoint.position.y = point.y;
    waypoint.position.z = 0.0;
    waypoint.speed = point.velocity;
    waypoint.suggested_steering = 0.0;
    waypoint.s_m = point.s;
    waypoint.x_m = point.x;
    waypoint.y_m = point.y;
    waypoint.psi_rad = point.psi;
    waypoint.kappa_radpm = point.kappa;
    waypoint.vx_mps = point.velocity;
    waypoint.ax_mps2 = point.acceleration;
    waypoint.d_left_m = point.d_left;
    waypoint.d_right_m = point.d_right;
    msg.waypoints.push_back(waypoint);
  }

  return msg;
}

void GlobalPlannerTrajectoryPublisherNode::setGlobalPathValid(bool valid)
{
  if (global_path_is_valid_ == valid) {
    return;
  }

  global_path_is_valid_ = valid;
  if (valid) {
    true_heartbeat_ready_ = false;
  } else {
    true_heartbeat_ready_ = true;
    publishValidityHeartbeat();
  }
  RCLCPP_INFO(
    get_logger(), "Global path validity changed: %s", valid ? "true" : "false");
}

void GlobalPlannerTrajectoryPublisherNode::publishValidityHeartbeat()
{
  if (global_path_is_valid_ && !true_heartbeat_ready_) {
    true_heartbeat_ready_ = true;
  } else {
    std_msgs::msg::Bool msg;
    msg.data = global_path_is_valid_;
    global_path_valid_pub_->publish(msg);
  }

  // Re-latch the raceline alongside the heartbeat. global_waypoints is otherwise
  // published only on a CSV file change, so a consumer that dropped its
  // reference on a TRANSIENT validity blip (frenet_odom invalidates on a single
  // false/stale heartbeat, then waits for a fresh snapshot "after the latest
  // invalidation") would never get one again and stalls forever -- no
  // frenet_odom means no current_d, which hard-blocks the GLOBAL entry gate and
  // strands the car in LOCAL/Pure-Pursuit for the whole run (TMPC never drives).
  // Consumers dedup by path content (and frenet clears its cache on
  // invalidation), so this rebuilds nothing unless they actually need it.
  if (global_path_is_valid_ && has_valid_trajectory_) {
    publishTrajectory();
  }
}

void GlobalPlannerTrajectoryPublisherNode::checkAndReloadTrajectory()
{
  std::filesystem::path trajectory_path;
  std::string error_message;
  if (!resolveTrajectoryPath(trajectory_path, error_message)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "%s", error_message.c_str());
    setGlobalPathValid(false);
    return;
  }

  if (resolved_path_log_ != trajectory_path.string()) {
    resolved_path_log_ = trajectory_path.string();
    RCLCPP_INFO(get_logger(), "Resolved trajectory CSV path: %s", resolved_path_log_.c_str());
  }

  std::error_code exists_error;
  if (!std::filesystem::exists(trajectory_path, exists_error) || exists_error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Trajectory CSV does not exist yet: %s", trajectory_path.string().c_str());
    setGlobalPathValid(false);
    return;
  }

  std::error_code time_error;
  const auto write_time = std::filesystem::last_write_time(trajectory_path, time_error);
  if (time_error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Failed to read last_write_time for %s: %s",
      trajectory_path.string().c_str(),
      time_error.message().c_str());
    setGlobalPathValid(false);
    return;
  }

  if ((has_last_loaded_write_time_ && write_time == last_loaded_write_time_) ||
    (has_last_failed_write_time_ && write_time == last_failed_write_time_))
  {
    return;
  }

  std::vector<TrajectoryPoint> candidate_points;
  if (!loadTrajectoryCsv(trajectory_path, get_logger(), candidate_points, error_message) ||
    !validateTrajectory(candidate_points, trajectoryValidationOptions(), get_logger(), error_message))
  {
    last_failed_write_time_ = write_time;
    has_last_failed_write_time_ = true;
    RCLCPP_ERROR(get_logger(), "Trajectory reload failed: %s", error_message.c_str());
    if (has_valid_trajectory_) {
      RCLCPP_WARN(get_logger(), "Keeping previous valid trajectory.");
    }
    setGlobalPathValid(false);
    return;
  }

  trajectory_points_ = std::move(candidate_points);
  last_loaded_write_time_ = write_time;
  has_last_loaded_write_time_ = true;
  has_last_failed_write_time_ = false;
  has_valid_trajectory_ = true;

  const double total_length = trajectory_points_.empty() ? 0.0 : trajectory_points_.back().s;
  RCLCPP_INFO(
    get_logger(),
    "Trajectory load success: waypoints=%zu total_length=%.3f m",
    trajectory_points_.size(),
    total_length);

  publishTrajectory();
  setGlobalPathValid(true);
}

void GlobalPlannerTrajectoryPublisherNode::publishTrajectory()
{
  if (!has_valid_trajectory_ || trajectory_points_.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "No valid trajectory loaded; not publishing /planning/global_waypoints.");
    return;
  }

  global_waypoints_pub_->publish(buildWaypointMessage(trajectory_points_));
  // Throttled: this is now also called on every validity heartbeat to re-latch
  // the raceline, so an unthrottled line would flood the log.
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Published %zu global waypoint(s) to %s",
    trajectory_points_.size(),
    global_waypoints_topic_.c_str());
}

}  // namespace hyu_global_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hyu_global_planner::GlobalPlannerTrajectoryPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
