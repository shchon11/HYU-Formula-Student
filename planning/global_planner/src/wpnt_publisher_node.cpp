#include "wpnt_publisher_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace wpnt_publisher
{
namespace
{

// eufs_msgs/Waypoint carries both the geometric pose (position) and the offline
// CSV echo fields (x_m/y_m). Prefer x_m/y_m; fall back to position only when the
// CSV fields are left unset (both exactly zero) but position is populated.
void resolveXY(const eufs_msgs::msg::Waypoint & w, double & x, double & y)
{
  x = w.x_m;
  y = w.y_m;
  if (x == 0.0 && y == 0.0 && (w.position.x != 0.0 || w.position.y != 0.0)) {
    x = w.position.x;
    y = w.position.y;
  }
}

// Parse an optionally-signed integer string (e.g. frenet child_frame_id).
// Returns nullopt for empty / non-numeric input such as "-1" handling upstream.
std::optional<int> parseIndex(const std::string & s)
{
  std::size_t i = 0;
  std::size_t j = s.size();
  while (i < j && std::isspace(static_cast<unsigned char>(s[i]))) {
    ++i;
  }
  while (j > i && std::isspace(static_cast<unsigned char>(s[j - 1]))) {
    --j;
  }
  if (i >= j) {
    return std::nullopt;
  }

  std::string t = s.substr(i, j - i);
  std::size_t k = 0;
  if (t[0] == '+' || t[0] == '-') {
    k = 1;
  }
  if (k == t.size()) {
    return std::nullopt;
  }
  for (; k < t.size(); ++k) {
    if (!std::isdigit(static_cast<unsigned char>(t[k]))) {
      return std::nullopt;
    }
  }
  try {
    return std::stoi(t);
  } catch (...) {
    return std::nullopt;
  }
}

}  // namespace

WpntPublisher::WpntPublisher()
: Node("wpnt_publisher")
{
  global_waypoints_topic_ =
    declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
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

  // /global_waypoints is latched (the global publisher uses transient_local);
  // the subscription MUST match or a late-starting node never receives it.
  auto global_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  global_sub_ = create_subscription<eufs_msgs::msg::WaypointArrayStamped>(
    global_waypoints_topic_, global_qos,
    std::bind(&WpntPublisher::onGlobalWaypoints, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    frenet_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
    std::bind(&WpntPublisher::onOdom, this, std::placeholders::_1));

  path_waypoints_pub_ = create_publisher<eufs_msgs::msg::WaypointArrayStamped>(
    path_waypoints_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    path_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  RCLCPP_INFO(
    get_logger(),
    "wpnt_publisher started: global=%s odom=%s -> path_waypoints=%s path=%s "
    "waypoint_num=%d closed_loop=%s",
    global_waypoints_topic_.c_str(), frenet_odom_topic_.c_str(),
    path_waypoints_topic_.c_str(), path_topic_.c_str(), waypoint_num_,
    closed_loop_ ? "true" : "false");
}

void WpntPublisher::onGlobalWaypoints(
  const eufs_msgs::msg::WaypointArrayStamped::SharedPtr msg)
{
  const auto & wps = msg->waypoints;
  if (wps.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received %zu global waypoint(s); need at least 2.", wps.size());
    return;
  }

  double fx = 0.0;
  double fy = 0.0;
  double bx = 0.0;
  double by = 0.0;
  resolveXY(wps.front(), fx, fy);
  resolveXY(wps.back(), bx, by);
  const double front_back = std::hypot(bx - fx, by - fy);
  const bool drop_back = (front_back <= closing_duplicate_tolerance_);

  const std::size_t count = drop_back ? wps.size() - 1 : wps.size();
  if (count < 2) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Fewer than 2 unique waypoints after closing-duplicate removal.");
    return;
  }

  auto snap = std::make_shared<PathSnapshot>();
  snap->frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
  snap->waypoints.reserve(count);
  snap->xs.reserve(count);
  snap->ys.reserve(count);
  snap->s.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    double x = 0.0;
    double y = 0.0;
    resolveXY(wps[i], x, y);
    snap->waypoints.push_back(wps[i]);
    snap->xs.push_back(x);
    snap->ys.push_back(y);
    snap->s.push_back(wps[i].s_m);
  }

  // Reject a non-monotone s_m sequence instead of recomputing it: the global
  // publisher already guarantees strictly-increasing s_m, so a violation is an
  // upstream bug, and silently rewriting s here would desync from the s that
  // frenet_odom_node reports on /car_state/frenet/odom.
  for (std::size_t i = 1; i < count; ++i) {
    if (!(snap->s[i] > snap->s[i - 1])) {
      RCLCPP_ERROR(
        get_logger(),
        "Global waypoints s_m not strictly increasing at index %zu (%.6f <= %.6f); "
        "keeping previous snapshot.",
        i, snap->s[i], snap->s[i - 1]);
      return;
    }
  }

  if (drop_back) {
    // s_m of the dropped closing point equals the full loop length.
    snap->track_length = wps.back().s_m;
  } else if (closed_loop_) {
    snap->track_length = wps.back().s_m + front_back;
  } else {
    snap->track_length = wps.back().s_m;
  }

  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path_ = snap;
  }

  RCLCPP_INFO(
    get_logger(),
    "Stored global path: input=%zu unique=%zu closing_duplicate=%s "
    "track_length=%.3f m frame=%s",
    wps.size(), count, drop_back ? "removed" : "none",
    snap->track_length, snap->frame_id.c_str());
}

void WpntPublisher::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::shared_ptr<const PathSnapshot> snap;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    snap = path_;
  }
  if (!snap) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for global waypoints on %s.", global_waypoints_topic_.c_str());
    return;
  }

  double s = msg->pose.pose.position.x;
  if (!std::isfinite(s)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Frenet odom s is non-finite; skipping (projection failure?).");
    return;
  }

  const std::size_t n = snap->s.size();
  if (closed_loop_ && snap->track_length > 0.0) {
    s = std::fmod(s, snap->track_length);
    if (s < 0.0) {
      s += snap->track_length;
    }
  }

  // First waypoint strictly ahead of the current s.
  std::size_t start =
    static_cast<std::size_t>(std::upper_bound(snap->s.begin(), snap->s.end(), s) - snap->s.begin());
  if (start == n) {
    if (closed_loop_) {
      start = 0;  // past the finish line -> first point of the next lap
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
  out.header.stamp = msg->header.stamp;  // correlate with the driving odom sample
  out.header.frame_id = snap->frame_id;
  out.waypoints.reserve(count);
  for (std::size_t k = 0; k < count; ++k) {
    out.waypoints.push_back(snap->waypoints[(start + k) % n]);
  }
  path_waypoints_pub_->publish(out);

  path_pub_->publish(buildPath(*snap, start, count, out.header));

  if (check_segment_index_) {
    crossCheckSegment(msg->child_frame_id, start, n);
  }
}

nav_msgs::msg::Path WpntPublisher::buildPath(
  const PathSnapshot & snap, std::size_t start, std::size_t count,
  const std_msgs::msg::Header & header) const
{
  const std::size_t n = snap.s.size();
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(count);

  double prev_yaw = 0.0;
  for (std::size_t k = 0; k < count; ++k) {
    const std::size_t gi = (start + k) % n;

    geometry_msgs::msg::PoseStamped ps;
    ps.header = header;
    ps.pose.position.x = snap.xs[gi];
    ps.pose.position.y = snap.ys[gi];
    ps.pose.position.z = 0.0;

    // Orientation from the heading to the next window point. psi_rad is NOT
    // used: the CSV heading convention is not guaranteed to equal ROS yaw.
    double yaw = prev_yaw;
    if (k + 1 < count) {
      const std::size_t gj = (start + k + 1) % n;
      const double dx = snap.xs[gj] - snap.xs[gi];
      const double dy = snap.ys[gj] - snap.ys[gi];
      if (std::hypot(dx, dy) >= 1.0e-9) {
        yaw = std::atan2(dy, dx);
      }
    }
    prev_yaw = yaw;

    ps.pose.orientation.z = std::sin(0.5 * yaw);
    ps.pose.orientation.w = std::cos(0.5 * yaw);
    path.poses.push_back(std::move(ps));
  }
  return path;
}

void WpntPublisher::crossCheckSegment(
  const std::string & child_frame_id, std::size_t start, std::size_t n)
{
  const auto seg = parseIndex(child_frame_id);
  if (!seg.has_value() || seg.value() < 0) {
    return;  // e.g. "-1" on projection failure
  }

  const int closest_by_s = static_cast<int>((start + n - 1) % n);
  int diff = std::abs(seg.value() - closest_by_s);
  diff = std::min(diff, static_cast<int>(n) - diff);  // circular distance
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
