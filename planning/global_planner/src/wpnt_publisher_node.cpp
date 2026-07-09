#include "wpnt_publisher_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace wpnt_publisher
{

WpntPublisher::WpntPublisher()
: Node("wpnt_publisher")
{
  global_waypoints_topic_ =
    declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
  global_path_valid_topic_ =
    declare_parameter<std::string>("global_path_valid_topic", "/planning/global_path_valid");
  global_path_valid_timeout_sec_ =
    declare_parameter<double>("global_path_valid_timeout_sec", 0.5);
  frenet_odom_topic_ =
    declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
  path_waypoints_topic_ =
    declare_parameter<std::string>("path_waypoints_topic", "/path_waypoints");
  path_topic_ = declare_parameter<std::string>("path_topic", "/path_waypoints/path");
  waypoint_num_ = declare_parameter<int>("waypoint_num", 50);
  closed_loop_ = declare_parameter<bool>("closed_loop", true);
  closing_duplicate_tolerance_ =
    declare_parameter<double>("closing_duplicate_tolerance", 1.0e-3);
  check_segment_index_ = declare_parameter<bool>("check_segment_index", true);

  if (waypoint_num_ < 1) {
    RCLCPP_WARN(
      get_logger(), "waypoint_num (%d) must be >= 1; clamping to 1.", waypoint_num_);
    waypoint_num_ = 1;
  }
  if (global_path_valid_timeout_sec_ <= 0.0) {
    RCLCPP_WARN(
      get_logger(), "global_path_valid_timeout_sec (%.3f) must be > 0; clamping to 0.1.",
      global_path_valid_timeout_sec_);
    global_path_valid_timeout_sec_ = 0.1;
  }
  validity_gate_.setTimeout(global_path_valid_timeout_sec_);

  auto global_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto valid_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  global_sub_ = create_subscription<eufs_msgs::msg::WaypointArrayStamped>(
    global_waypoints_topic_, global_qos,
    std::bind(&WpntPublisher::onGlobalWaypoints, this, std::placeholders::_1));
  global_path_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
    global_path_valid_topic_, valid_qos,
    std::bind(&WpntPublisher::onGlobalPathValid, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    frenet_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
    std::bind(&WpntPublisher::onOdom, this, std::placeholders::_1));

  path_waypoints_pub_ = create_publisher<eufs_msgs::msg::WaypointArrayStamped>(
    path_waypoints_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    path_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  const auto watchdog_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(std::max(0.05, 0.5 * global_path_valid_timeout_sec_)));
  validity_watchdog_timer_ = create_wall_timer(
    watchdog_period, std::bind(&WpntPublisher::onValidityWatchdog, this));

  RCLCPP_INFO(
    get_logger(),
    "wpnt_publisher started: global=%s odom=%s -> path_waypoints=%s path=%s "
    "valid=%s timeout=%.3fs waypoint_num=%d closed_loop=%s",
    global_waypoints_topic_.c_str(), frenet_odom_topic_.c_str(),
    path_waypoints_topic_.c_str(), path_topic_.c_str(), global_path_valid_topic_.c_str(),
    global_path_valid_timeout_sec_, waypoint_num_, closed_loop_ ? "true" : "false");
}

void WpntPublisher::onGlobalWaypoints(
  const eufs_msgs::msg::WaypointArrayStamped::SharedPtr msg)
{
  const auto result = makePathSnapshot(
    *msg, PathSnapshotOptions{closed_loop_, closing_duplicate_tolerance_});
  if (!result.snapshot) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "%s", result.error_message.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    validity_gate_.onSnapshot(result.snapshot, now());
  }

  RCLCPP_INFO(
    get_logger(),
    "Accepted global path snapshot: input=%zu unique=%zu closing_duplicate=%s "
    "track_length=%.3f m frame=%s",
    result.input_count, result.unique_count,
    result.closing_duplicate_removed ? "removed" : "none",
    result.snapshot->track_length, result.snapshot->frame_id.c_str());
}

void WpntPublisher::onGlobalPathValid(const std_msgs::msg::Bool::SharedPtr msg)
{
  ValidityEventResult result;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    result = validity_gate_.onValidity(msg->data, now());
  }

  if (result.invalidated) {
    if (result.stale_invalidated) {
      RCLCPP_WARN(
        get_logger(),
        "Global path invalidated by stale validity heartbeat before accepting recovered true.");
    } else {
      RCLCPP_WARN(get_logger(), "Global path invalidated by false validity heartbeat.");
    }
  } else if (result.activated) {
    RCLCPP_INFO(get_logger(), "Global path re-enabled after fresh validity heartbeat.");
  }
}

void WpntPublisher::onValidityWatchdog()
{
  bool invalidated = false;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    invalidated = validity_gate_.invalidateIfStale(now());
  }

  if (invalidated) {
    RCLCPP_WARN(
      get_logger(), "Global path invalidated by stale validity heartbeat; waiting for %s.",
      global_path_valid_topic_.c_str());
  }
}

void WpntPublisher::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  OdomGateResult gate_result;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    gate_result = validity_gate_.readForOdom(now());
  }

  if (!gate_result.has_validity || !gate_result.validity_true) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for true global path validity on %s.", global_path_valid_topic_.c_str());
    return;
  }
  if (!gate_result.snapshot) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for fresh global validity and a newer global waypoint snapshot after invalidation.");
    return;
  }

  double s = msg->pose.pose.position.x;
  if (!std::isfinite(s)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Frenet odom s is non-finite; skipping (projection failure?).");
    return;
  }

  const auto & snap = *gate_result.snapshot;
  const std::size_t n = snap.s.size();
  if (closed_loop_ && snap.track_length > 0.0) {
    s = std::fmod(s, snap.track_length);
    if (s < 0.0) {
      s += snap.track_length;
    }
  }

  std::size_t start =
    static_cast<std::size_t>(std::upper_bound(snap.s.begin(), snap.s.end(), s) - snap.s.begin());
  if (start == n) {
    if (closed_loop_) {
      start = 0;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Vehicle past the end of an open path; nothing ahead to publish.");
      return;
    }
  }

  const std::size_t requested = static_cast<std::size_t>(waypoint_num_);
  const std::size_t count =
    closed_loop_ ? std::min(requested, n) : std::min(requested, n - start);

  eufs_msgs::msg::WaypointArrayStamped out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = snap.frame_id;
  out.waypoints.reserve(count);
  for (std::size_t k = 0; k < count; ++k) {
    out.waypoints.push_back(snap.waypoints[(start + k) % n]);
  }
  path_waypoints_pub_->publish(out);
  path_pub_->publish(buildPathMessage(snap, start, count, out.header));

  if (check_segment_index_) {
    crossCheckSegment(msg->child_frame_id, start, n);
  }
}

void WpntPublisher::crossCheckSegment(
  const std::string & child_frame_id, std::size_t start, std::size_t n)
{
  const auto seg = parseIndex(child_frame_id);
  if (!seg.has_value() || seg.value() < 0) {
    return;
  }

  const int closest_by_s = static_cast<int>((start + n - 1) % n);
  int diff = std::abs(seg.value() - closest_by_s);
  diff = std::min(diff, static_cast<int>(n) - diff);
  if (diff >= 3) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Frenet segment index (%d) disagrees with s-based closest (%d), circular diff=%d. "
      "Check frenet_odom_node projection_domain_eps2 / duplicate_point_tolerance.",
      seg.value(), closest_by_s, diff);
  }
}

}  // namespace wpnt_publisher

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wpnt_publisher::WpntPublisher>());
  rclcpp::shutdown();
  return 0;
}
