#include "global_planner_trajectory_publisher_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <sstream>
#include <system_error>
#include <utility>

#include "eufs_msgs/msg/waypoint.hpp"

namespace global_planner
{
namespace
{

constexpr std::size_t kExpectedColumnCount = 7;
constexpr std::array<const char *, kExpectedColumnCount> kExpectedHeader = {
  "s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2"};

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string removeUtf8Bom(std::string value)
{
  if (value.size() >= 3U &&
    static_cast<unsigned char>(value[0]) == 0xEF &&
    static_cast<unsigned char>(value[1]) == 0xBB &&
    static_cast<unsigned char>(value[2]) == 0xBF)
  {
    value.erase(0, 3);
  }
  return value;
}

std::vector<std::string> splitSemicolon(const std::string & line)
{
  std::vector<std::string> tokens;
  std::stringstream stream(line);
  std::string token;
  while (std::getline(stream, token, ';')) {
    tokens.push_back(trim(token));
  }
  return tokens;
}

bool parseFiniteDouble(const std::string & token, double & output)
{
  try {
    std::size_t parsed_chars = 0;
    const double value = std::stod(token, &parsed_chars);
    if (parsed_chars != token.size() || !std::isfinite(value)) {
      return false;
    }
    output = value;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool isExpectedHeader(const std::vector<std::string> & tokens)
{
  if (tokens.size() != kExpectedColumnCount) {
    return false;
  }
  for (std::size_t i = 0; i < kExpectedColumnCount; ++i) {
    if (tokens[i] != kExpectedHeader[i]) {
      return false;
    }
  }
  return true;
}

double pointDistance(const TrajectoryPoint & a, const TrajectoryPoint & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

}  // namespace

GlobalPlannerTrajectoryPublisherNode::GlobalPlannerTrajectoryPublisherNode()
: Node("global_planner_trajectory_publisher_node")
{
  declareParameters();
  loadParameters();

  const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
  global_waypoints_pub_ =
    create_publisher<eufs_msgs::msg::WaypointArrayStamped>(
    global_waypoints_topic_,
    latched_qos);

  RCLCPP_INFO(get_logger(), "global_waypoints_topic=%s", global_waypoints_topic_.c_str());

  checkAndReloadTrajectory();

  const auto period = std::chrono::duration<double>(
    std::max(0.1, reload_period_sec_));
  reload_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GlobalPlannerTrajectoryPublisherNode::checkAndReloadTrajectory, this));
}

void GlobalPlannerTrajectoryPublisherNode::declareParameters()
{
  declare_parameter<std::string>("trajectory_csv_path", "");
  declare_parameter<std::string>("output_root", "");
  declare_parameter<std::string>("map_name", "");
  declare_parameter<std::string>("trajectory_filename", "traj_race_cl.csv");
  declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
  declare_parameter<std::string>("frame_id", "map");
  declare_parameter<double>("reload_period_sec", 1.0);
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
  frame_id_ = get_parameter("frame_id").as_string();
  reload_period_sec_ = get_parameter("reload_period_sec").as_double();
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

bool GlobalPlannerTrajectoryPublisherNode::loadTrajectoryCsv(
  const std::filesystem::path & path,
  std::vector<TrajectoryPoint> & points,
  std::string & error_message) const
{
  std::ifstream file(path);
  if (!file.is_open()) {
    error_message = "failed to open trajectory CSV: " + path.string();
    return false;
  }

  points.clear();

  bool header_seen = false;
  std::string line;
  std::size_t line_number = 0;
  std::size_t malformed_rows = 0;

  while (std::getline(file, line)) {
    ++line_number;
    line = trim(removeUtf8Bom(line));
    if (line.empty()) {
      continue;
    }

    if (!line.empty() && line.front() == '#') {
      line = trim(line.substr(1));
      if (line.empty()) {
        continue;
      }
    }

    const auto tokens = splitSemicolon(line);
    if (!header_seen) {
      if (isExpectedHeader(tokens)) {
        header_seen = true;
        continue;
      }

      if (tokens.size() == kExpectedColumnCount && tokens.front() == kExpectedHeader.front()) {
        error_message = "CSV header invalid at line " + std::to_string(line_number);
        return false;
      }

      continue;
    }

    if (tokens.size() != kExpectedColumnCount) {
      ++malformed_rows;
      RCLCPP_WARN(
        get_logger(),
        "Skipping malformed trajectory row %zu: expected %zu columns, got %zu",
        line_number,
        kExpectedColumnCount,
        tokens.size());
      continue;
    }

    TrajectoryPoint point;
    if (!parseFiniteDouble(tokens[0], point.s) ||
      !parseFiniteDouble(tokens[1], point.x) ||
      !parseFiniteDouble(tokens[2], point.y) ||
      !parseFiniteDouble(tokens[3], point.psi) ||
      !parseFiniteDouble(tokens[4], point.kappa) ||
      !parseFiniteDouble(tokens[5], point.velocity) ||
      !parseFiniteDouble(tokens[6], point.acceleration))
    {
      ++malformed_rows;
      RCLCPP_WARN(
        get_logger(),
        "Skipping malformed trajectory row %zu: numeric conversion failed or non-finite value",
        line_number);
      continue;
    }

    points.push_back(point);
  }

  if (!header_seen) {
    error_message = "CSV header invalid or missing";
    return false;
  }

  if (malformed_rows > 0U) {
    RCLCPP_WARN(get_logger(), "Skipped %zu malformed trajectory row(s).", malformed_rows);
  }
  return true;
}

bool GlobalPlannerTrajectoryPublisherNode::validateTrajectory(
  std::vector<TrajectoryPoint> & points,
  std::string & error_message) const
{
  if (points.size() < static_cast<std::size_t>(min_waypoint_count_)) {
    error_message =
      "valid waypoint count is below min_waypoint_count: " + std::to_string(points.size());
    return false;
  }

  std::vector<TrajectoryPoint> deduplicated;
  deduplicated.reserve(points.size());
  deduplicated.push_back(points.front());

  std::size_t duplicate_count = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    if (!std::isfinite(points[i].s) || !std::isfinite(points[i].x) || !std::isfinite(points[i].y)) {
      error_message = "trajectory contains non-finite s/x/y after parsing";
      return false;
    }

    if (pointDistance(deduplicated.back(), points[i]) <= duplicate_point_tolerance_) {
      ++duplicate_count;
      continue;
    }
    deduplicated.push_back(points[i]);
  }

  if (duplicate_count > 0U) {
    RCLCPP_WARN(
      get_logger(),
      "Removed %zu duplicate trajectory waypoint(s); remaining=%zu",
      duplicate_count,
      deduplicated.size());
  }

  if (deduplicated.size() < static_cast<std::size_t>(min_waypoint_count_)) {
    error_message =
      "waypoint count after duplicate removal is below min_waypoint_count: " +
      std::to_string(deduplicated.size());
    return false;
  }

  bool s_is_strictly_increasing = true;
  for (std::size_t i = 1; i < deduplicated.size(); ++i) {
    if (!(deduplicated[i].s > deduplicated[i - 1].s)) {
      s_is_strictly_increasing = false;
      break;
    }
  }

  if (!s_is_strictly_increasing) {
    if (!recompute_s_if_invalid_) {
      error_message = "trajectory s_m is not strictly increasing";
      return false;
    }

    deduplicated.front().s = 0.0;
    for (std::size_t i = 1; i < deduplicated.size(); ++i) {
      deduplicated[i].s = deduplicated[i - 1].s + pointDistance(deduplicated[i - 1], deduplicated[i]);
    }
    RCLCPP_WARN(get_logger(), "Invalid s_m sequence detected; recomputed s from x/y distance.");
  }

  points = std::move(deduplicated);
  return true;
}

eufs_msgs::msg::WaypointArrayStamped GlobalPlannerTrajectoryPublisherNode::buildWaypointMessage(
  const std::vector<TrajectoryPoint> & points)
{
  eufs_msgs::msg::WaypointArrayStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;
  msg.waypoints.reserve(points.size());

  for (const auto & point : points) {
    eufs_msgs::msg::Waypoint waypoint;
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
    msg.waypoints.push_back(waypoint);
  }

  return msg;
}

void GlobalPlannerTrajectoryPublisherNode::checkAndReloadTrajectory()
{
  std::filesystem::path trajectory_path;
  std::string error_message;
  if (!resolveTrajectoryPath(trajectory_path, error_message)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "%s", error_message.c_str());
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
    return;
  }

  if ((has_last_loaded_write_time_ && write_time == last_loaded_write_time_) ||
    (has_last_failed_write_time_ && write_time == last_failed_write_time_))
  {
    return;
  }

  std::vector<TrajectoryPoint> candidate_points;
  if (!loadTrajectoryCsv(trajectory_path, candidate_points, error_message) ||
    !validateTrajectory(candidate_points, error_message))
  {
    last_failed_write_time_ = write_time;
    has_last_failed_write_time_ = true;
    RCLCPP_ERROR(get_logger(), "Trajectory reload failed: %s", error_message.c_str());
    if (has_valid_trajectory_) {
      RCLCPP_WARN(get_logger(), "Keeping previous valid trajectory.");
    }
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
}

void GlobalPlannerTrajectoryPublisherNode::publishTrajectory()
{
  if (!has_valid_trajectory_ || trajectory_points_.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "No valid trajectory loaded; not publishing /global_waypoints.");
    return;
  }

  global_waypoints_pub_->publish(buildWaypointMessage(trajectory_points_));
  RCLCPP_INFO(
    get_logger(),
    "Published %zu global waypoint(s) to %s",
    trajectory_points_.size(),
    global_waypoints_topic_.c_str());
}

}  // namespace global_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<global_planner::GlobalPlannerTrajectoryPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
