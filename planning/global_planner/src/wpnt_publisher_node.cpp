#include "wpnt_publisher_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace wpnt_publisher
{

WpntPublisher::WpntPublisher(const rclcpp::NodeOptions & options)
: Node("wpnt_publisher", options)
{
  global_waypoints_topic_ =
    declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
  global_path_valid_topic_ =
    declare_parameter<std::string>("global_path_valid_topic", "/planning/global_path_valid");
  global_path_valid_timeout_sec_ =
    declare_parameter<double>("global_path_valid_timeout_sec", 0.5);
  frenet_odom_topic_ =
    declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
  ego_odom_topic_ =
    declare_parameter<std::string>("ego_odom_topic", "/localization/ego_odom");
  path_source_topic_ =
    declare_parameter<std::string>("path_source_topic", "/planning/path_source");
  lap_count_topic_ =
    declare_parameter<std::string>("lap_count_topic", "/planning/lap_count");
  path_waypoints_topic_ =
    declare_parameter<std::string>("path_waypoints_topic", "/path_waypoints");
  path_topic_ = declare_parameter<std::string>("path_topic", "/path_waypoints/path");
  tmpc_performance_trajectory_topic_ = declare_parameter<std::string>(
    "tmpc_performance_trajectory_topic", "/tmpc/trajectory_performance");
  tmpc_emergency_trajectory_topic_ = declare_parameter<std::string>(
    "tmpc_emergency_trajectory_topic", "/tmpc/trajectory_emergency");
  waypoint_num_ = declare_parameter<int>("waypoint_num", 50);
  closed_loop_ = declare_parameter<bool>("closed_loop", true);
  closing_duplicate_tolerance_ =
    declare_parameter<double>("closing_duplicate_tolerance", 1.0e-3);
  check_segment_index_ = declare_parameter<bool>("check_segment_index", true);
  tmpc_publish_rate_hz_ = declare_parameter<double>("tmpc_publish_rate_hz", 20.0);
  tmpc_input_timeout_sec_ = declare_parameter<double>("tmpc_input_timeout_sec", 0.5);

  const double tmpc_foresight_time_sec =
    declare_parameter<double>("tmpc_foresight_time_sec", 3.0);
  const double tmpc_min_horizon_m = declare_parameter<double>("tmpc_min_horizon_m", 10.0);
  const double tmpc_max_horizon_m = declare_parameter<double>("tmpc_max_horizon_m", 30.0);
  const double tmpc_start_behind_m = declare_parameter<double>("tmpc_start_behind_m", 0.5);
  const double tmpc_banking_rad = declare_parameter<double>("tmpc_banking_rad", 0.0);
  const double tmpc_tube_l_m = declare_parameter<double>("tmpc_tube_l_m", 0.1);
  const double tmpc_tube_r_m = declare_parameter<double>("tmpc_tube_r_m", 0.1);
  const double tmpc_emergency_decel_scale =
    declare_parameter<double>("tmpc_emergency_decel_scale", 0.95);
  const double tmpc_emergency_final_speed_mps =
    declare_parameter<double>("tmpc_emergency_final_speed_mps", 0.5);
  const double tmpc_min_sample_distance_m =
    declare_parameter<double>("tmpc_min_sample_distance_m", 1.0e-4);
  const double tmpc_drag_coefficient =
    declare_parameter<double>("tmpc_drag_coefficient", 1.4);
  const double tmpc_air_density_kgpm3 =
    declare_parameter<double>("tmpc_air_density_kgpm3", 1.22);
  const double tmpc_vehicle_mass_kg =
    declare_parameter<double>("tmpc_vehicle_mass_kg", 225.0);
  const double tmpc_acceleration_utilization_limit =
    declare_parameter<double>("tmpc_acceleration_utilization_limit", 1.0);
  const double tmpc_performance_speed_cap_mps =
    declare_parameter<double>("tmpc_performance_speed_cap_mps", 0.0);
  const std::string tmpc_input_heading_convention = declare_parameter<std::string>(
    "tmpc_input_heading_convention", "ros_yaw");

  const std::string default_ggv_path =
    ament_index_cpp::get_package_share_directory("global_planner") +
    "/config/veh_dyn_info/ggv.csv";
  const std::string tmpc_ggv_csv_path =
    declare_parameter<std::string>("tmpc_ggv_csv_path", default_ggv_path);
  const std::vector<double> tmpc_ggv_speed_mps =
    declare_parameter<std::vector<double>>("tmpc_ggv_speed_mps", {0.0, 8.5});
  const std::vector<double> tmpc_ggv_ax_max_mps2 =
    declare_parameter<std::vector<double>>("tmpc_ggv_ax_max_mps2", {11.772, 11.772});
  const std::vector<double> tmpc_ggv_ay_max_mps2 = declare_parameter<std::vector<double>>(
    "tmpc_ggv_ay_max_mps2", {20.6428571429, 20.6428571429});
  const std::vector<double> tmpc_ggv_ax_decel_max_mps2 =
    declare_parameter<std::vector<double>>("tmpc_ggv_ax_decel_max_mps2", {9.81, 9.81});

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
  if (!std::isfinite(tmpc_publish_rate_hz_) || tmpc_publish_rate_hz_ < 10.0) {
    throw std::invalid_argument("tmpc_publish_rate_hz must be finite and at least 10 Hz");
  }
  if (!std::isfinite(tmpc_input_timeout_sec_) || tmpc_input_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("tmpc_input_timeout_sec must be finite and positive");
  }
  validity_gate_.setTimeout(global_path_valid_timeout_sec_);

  global_planner::tmpc::BuilderConfig builder_config;
  builder_config.foresight_time_sec = tmpc_foresight_time_sec;
  builder_config.min_horizon_m = tmpc_min_horizon_m;
  builder_config.max_horizon_m = tmpc_max_horizon_m;
  builder_config.start_behind_m = tmpc_start_behind_m;
  builder_config.banking_rad = tmpc_banking_rad;
  builder_config.tube_l_m = tmpc_tube_l_m;
  builder_config.tube_r_m = tmpc_tube_r_m;
  builder_config.emergency_decel_scale = tmpc_emergency_decel_scale;
  builder_config.emergency_final_speed_mps = tmpc_emergency_final_speed_mps;
  builder_config.minimum_adjacent_distance_m = tmpc_min_sample_distance_m;
  builder_config.drag_coefficient = tmpc_drag_coefficient;
  builder_config.air_density_kgpm3 = tmpc_air_density_kgpm3;
  builder_config.vehicle_mass_kg = tmpc_vehicle_mass_kg;
  builder_config.acceleration_utilization_limit = tmpc_acceleration_utilization_limit;
  builder_config.performance_speed_cap_mps = tmpc_performance_speed_cap_mps;
  if (tmpc_input_heading_convention == "ros_yaw") {
    builder_config.heading_convention =
      global_planner::tmpc::HeadingConvention::kRosEastZero;
  } else if (tmpc_input_heading_convention == "formula_north_zero") {
    builder_config.heading_convention =
      global_planner::tmpc::HeadingConvention::kFormulaNorthZero;
  } else {
    throw std::invalid_argument(
            "tmpc_input_heading_convention must be 'ros_yaw' or 'formula_north_zero'");
  }

  std::string ggv_error;
  std::optional<global_planner::tmpc::GgvTable> ggv_table;
  if (tmpc_ggv_csv_path.empty()) {
    ggv_table = global_planner::tmpc::GgvTable::fromVectors(
      tmpc_ggv_speed_mps, tmpc_ggv_ax_max_mps2, tmpc_ggv_ay_max_mps2,
      tmpc_ggv_ax_decel_max_mps2, &ggv_error);
  } else {
    ggv_table = global_planner::tmpc::GgvTable::loadCsv(tmpc_ggv_csv_path, &ggv_error);
  }
  if (!ggv_table) {
    throw std::invalid_argument("Invalid TMPC GGV configuration: " + ggv_error);
  }

  tmpc_builder_ = std::make_unique<global_planner::tmpc::TmpcTrajectoryBuilder>(
    builder_config, std::move(*ggv_table));
  std::string builder_error;
  if (!tmpc_builder_->valid(&builder_error)) {
    throw std::invalid_argument("Invalid TMPC trajectory builder configuration: " + builder_error);
  }

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
  ego_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    ego_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
    std::bind(&WpntPublisher::onEgoOdom, this, std::placeholders::_1));
  path_source_sub_ = create_subscription<std_msgs::msg::String>(
    path_source_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&WpntPublisher::onPathSource, this, std::placeholders::_1));
  lap_count_sub_ = create_subscription<std_msgs::msg::Int32>(
    lap_count_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    std::bind(&WpntPublisher::onLapCount, this, std::placeholders::_1));

  path_waypoints_pub_ = create_publisher<eufs_msgs::msg::WaypointArrayStamped>(
    path_waypoints_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    path_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  const auto tmpc_output_qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  tmpc_performance_pub_ =
    create_publisher<hyu_formular_control_msgs::msg::TumTrajectory>(
    tmpc_performance_trajectory_topic_, tmpc_output_qos);
  tmpc_emergency_pub_ =
    create_publisher<hyu_formular_control_msgs::msg::TumTrajectory>(
    tmpc_emergency_trajectory_topic_, tmpc_output_qos);

  const auto watchdog_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(std::max(0.05, 0.5 * global_path_valid_timeout_sec_)));
  validity_watchdog_timer_ = create_wall_timer(
    watchdog_period, std::bind(&WpntPublisher::onValidityWatchdog, this));
  const auto tmpc_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / tmpc_publish_rate_hz_));
  tmpc_timer_ = create_wall_timer(tmpc_period, std::bind(&WpntPublisher::onTmpcTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "wpnt_publisher started: global=%s odom=%s -> path_waypoints=%s path=%s "
    "valid=%s timeout=%.3fs waypoint_num=%d closed_loop=%s",
    global_waypoints_topic_.c_str(), frenet_odom_topic_.c_str(),
    path_waypoints_topic_.c_str(), path_topic_.c_str(), global_path_valid_topic_.c_str(),
    global_path_valid_timeout_sec_, waypoint_num_, closed_loop_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(),
    "Formula TMPC trajectory output: performance=%s emergency=%s rate=%.1fHz "
    "ego=%s source=%s lap=%s input_timeout=%.3fs",
    tmpc_performance_trajectory_topic_.c_str(), tmpc_emergency_trajectory_topic_.c_str(),
    tmpc_publish_rate_hz_, ego_odom_topic_.c_str(), path_source_topic_.c_str(),
    lap_count_topic_.c_str(), tmpc_input_timeout_sec_);
}

void WpntPublisher::onGlobalWaypoints(
  const eufs_msgs::msg::WaypointArrayStamped::SharedPtr msg)
{
  const auto result = makePathSnapshot(
    *msg, PathSnapshotOptions{closed_loop_, closing_duplicate_tolerance_});
  if (!result.snapshot) {
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      latest_global_snapshot_valid_ = false;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "%s", result.error_message.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    validity_gate_.onSnapshot(result.snapshot, now());
    latest_global_snapshot_valid_ = true;
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
    latest_frenet_s_m_ = msg->pose.pose.position.x;
    latest_frenet_odom_time_ = now();
    has_frenet_odom_ = true;
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

void WpntPublisher::onEgoOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  std::lock_guard<std::mutex> lock(path_mutex_);
  latest_ego_speed_mps_ = speed;
  latest_ego_odom_time_ = now();
  has_ego_odom_ = true;
}

void WpntPublisher::onPathSource(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  latest_path_source_ = msg->data;
  latest_path_source_time_ = now();
  has_path_source_ = true;
}

void WpntPublisher::onLapCount(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  latest_lap_count_ = msg->data;
  latest_lap_count_time_ = now();
  has_lap_count_ = true;
}

bool WpntPublisher::inputFresh(
  const rclcpp::Time & stamp, const rclcpp::Time & current_time) const
{
  const double age_sec = (current_time - stamp).seconds();
  return std::isfinite(age_sec) && age_sec >= 0.0 && age_sec <= tmpc_input_timeout_sec_;
}

void WpntPublisher::onTmpcTimer()
{
  std::unique_lock<std::mutex> cycle_lock(tmpc_cycle_mutex_, std::try_to_lock);
  if (!cycle_lock.owns_lock()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Previous TMPC trajectory build is still running; skipping this cycle.");
    return;
  }

  const auto current_time = now();
  std::shared_ptr<const PathSnapshot> snapshot;
  bool snapshot_input_valid = false;
  bool has_frenet = false;
  bool has_ego = false;
  bool has_source = false;
  bool has_lap = false;
  double current_s_m = 0.0;
  double current_speed_mps = 0.0;
  std::string source;
  std::int32_t lap_count = 0;
  rclcpp::Time frenet_time(0, 0, RCL_ROS_TIME);
  rclcpp::Time ego_time(0, 0, RCL_ROS_TIME);
  rclcpp::Time source_time(0, 0, RCL_ROS_TIME);
  rclcpp::Time lap_time(0, 0, RCL_ROS_TIME);
  OdomGateResult gate_result;

  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    gate_result = validity_gate_.readForOdom(current_time);
    snapshot = gate_result.snapshot;
    snapshot_input_valid = latest_global_snapshot_valid_;
    has_frenet = has_frenet_odom_;
    has_ego = has_ego_odom_;
    has_source = has_path_source_;
    has_lap = has_lap_count_;
    current_s_m = latest_frenet_s_m_;
    current_speed_mps = latest_ego_speed_mps_;
    source = latest_path_source_;
    lap_count = latest_lap_count_;
    frenet_time = latest_frenet_odom_time_;
    ego_time = latest_ego_odom_time_;
    source_time = latest_path_source_time_;
    lap_time = latest_lap_count_time_;
  }

  if (!gate_result.has_validity || !gate_result.validity_true || !snapshot ||
    !snapshot_input_valid)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TMPC inactive: waiting for a valid global path snapshot and fresh validity heartbeat.");
    return;
  }
  if (!has_source || !inputFresh(source_time, current_time) ||
    (source != "GLOBAL_FULL" && source != "GLOBAL_FINAL_STOP"))
  {
    return;
  }
  if (!has_frenet || !has_ego || !has_lap ||
    !inputFresh(frenet_time, current_time) || !inputFresh(ego_time, current_time) ||
    !inputFresh(lap_time, current_time))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TMPC inactive: Frenet odometry, ego odometry, or lap count is missing/stale.");
    return;
  }
  if (!std::isfinite(current_s_m) || !std::isfinite(current_speed_mps) ||
    current_speed_mps < 0.0)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TMPC inactive: Frenet s or ego speed is invalid.");
    return;
  }

  const std::uint32_t next_counter =
    trajectory_counter_ == std::numeric_limits<std::uint32_t>::max() ?
    1U : trajectory_counter_ + 1U;
  global_planner::tmpc::BuildInput build_input;
  build_input.path = snapshot.get();
  build_input.current_s_m = current_s_m;
  build_input.current_speed_mps = current_speed_mps;
  build_input.lap_cnt = static_cast<std::uint32_t>(std::max<std::int32_t>(0, lap_count));
  build_input.traj_cnt = next_counter;

  const auto result = tmpc_builder_->build(build_input);
  if (!result.success) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TMPC trajectory build rejected: %s", result.error.c_str());
    return;
  }
  if (result.ggv_endpoint_clamped) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "TMPC trajectory speed is outside the configured GGV range; endpoint limits were used.");
  }

  trajectory_counter_ = next_counter;
  tmpc_performance_pub_->publish(result.performance);
  tmpc_emergency_pub_->publish(result.emergency);
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
