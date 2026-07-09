#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "clcs_frenet_converter.hpp"
#include "eufs_msgs/msg/waypoint_array_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace frenet_conversion
{
namespace
{

enum class ProjectionFailurePolicy
{
  kDropMessage,
  kPublishLastValid,
  kPublishNan
};

ProjectionFailurePolicy parseProjectionFailurePolicy(const std::string & value)
{
  if (value == "publish_last_valid") {
    return ProjectionFailurePolicy::kPublishLastValid;
  }
  if (value == "publish_nan") {
    return ProjectionFailurePolicy::kPublishNan;
  }
  return ProjectionFailurePolicy::kDropMessage;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

std::vector<ReferenceWaypoint> toReferenceWaypoints(
  const eufs_msgs::msg::WaypointArrayStamped & msg)
{
  std::vector<ReferenceWaypoint> waypoints;
  waypoints.reserve(msg.waypoints.size());
  for (const auto & waypoint : msg.waypoints) {
    // eufs_msgs/Waypoint carries both the geometric pose (position) and the
    // offline-CSV echo fields (x_m/y_m). The HYU global_planner fills both, so
    // x_m/y_m are the primary source. Fall back to position when the CSV fields
    // are left unset (both exactly zero) so any /global_waypoints producer that
    // only populates position still yields a usable reference path.
    double x = waypoint.x_m;
    double y = waypoint.y_m;
    if (x == 0.0 && y == 0.0 &&
      (waypoint.position.x != 0.0 || waypoint.position.y != 0.0))
    {
      x = waypoint.position.x;
      y = waypoint.position.y;
    }
    waypoints.push_back({x, y, waypoint.s_m});
  }
  return waypoints;
}

void clearCovariance(nav_msgs::msg::Odometry & odom)
{
  std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
  std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
}

}  // namespace

class FrenetOdomNode : public rclcpp::Node
{
public:
  FrenetOdomNode()
  : Node("frenet_odom_node")
  {
    declareParameters();
    loadParameters();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      std::bind(&FrenetOdomNode::odomCallback, this, std::placeholders::_1));

    waypoint_sub_ = create_subscription<eufs_msgs::msg::WaypointArrayStamped>(
      waypoint_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&FrenetOdomNode::waypointsCallback, this, std::placeholders::_1));

    frenet_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      frenet_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

    if (publish_debug_) {
      debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        debug_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    }

    if (enable_path_smoothing_ || enable_curvature_reduction_ || reference_resample_step_ > 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "CommonRoad-CLCS C++ core does not expose the Python preprocessing pipeline used by "
        "commonroad_clcs.clcs. Only finite filtering, duplicate removal, and closed-loop "
        "closing are applied in this node.");
    }

    RCLCPP_INFO(
      get_logger(),
      "CLCS frenet odom started: odom=%s waypoints=%s output=%s closed_loop=%s "
      "child_frame_id=closest_segment_index velocity_frame=%s failure_policy=%s",
      odom_topic_.c_str(), waypoint_topic_.c_str(), frenet_odom_topic_.c_str(),
      config_.closed_loop ? "true" : "false",
      toString(config_.velocity_frame).c_str(), projection_failure_policy_name_.c_str());
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
    declare_parameter<std::string>("waypoint_topic", "/global_waypoints");
    declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
    declare_parameter<std::string>("debug_topic", "/car_state/frenet/debug");
    declare_parameter<std::string>("frenet_frame_id", "frenet");
    declare_parameter<std::string>("projection_failure_policy", "drop_message");
    declare_parameter<std::string>("velocity_frame", "body");
    declare_parameter<std::string>("covariance_mode", "zero");

    declare_parameter<bool>("closed_loop", true);
    declare_parameter<bool>("publish_debug", true);
    declare_parameter<bool>("debug_timing", true);
    declare_parameter<bool>("publish_heading_error", true);
    declare_parameter<bool>("publish_frenet_velocity", true);
    declare_parameter<bool>("compatibility_mode", false);
    declare_parameter<bool>("use_path_preprocessing", true);
    declare_parameter<bool>("enable_path_smoothing", false);
    declare_parameter<bool>("enable_curvature_reduction", false);

    declare_parameter<double>("path_change_tolerance", 1.0e-4);
    declare_parameter<double>("duplicate_point_tolerance", 1.0e-3);
    declare_parameter<double>("reference_resample_step", 0.0);
    declare_parameter<double>("tangent_epsilon", 0.05);
    declare_parameter<double>("max_projection_distance", 20.0);
    declare_parameter<double>("projection_domain_limit", 20.0);
    declare_parameter<double>("projection_domain_epsilon", 0.1);
    declare_parameter<double>("projection_domain_eps2", 0.0);
    declare_parameter<double>("min_path_length", 0.5);
    declare_parameter<double>("large_gap_factor", 5.0);
    declare_parameter<int>("projection_domain_method", 1);
  }

  void loadParameters()
  {
    odom_topic_ = get_parameter("odom_topic").as_string();
    waypoint_topic_ = get_parameter("waypoint_topic").as_string();
    frenet_odom_topic_ = get_parameter("frenet_odom_topic").as_string();
    debug_topic_ = get_parameter("debug_topic").as_string();
    frenet_frame_id_ = get_parameter("frenet_frame_id").as_string();
    projection_failure_policy_name_ = get_parameter("projection_failure_policy").as_string();
    projection_failure_policy_ =
      parseProjectionFailurePolicy(projection_failure_policy_name_);
    covariance_mode_ = get_parameter("covariance_mode").as_string();

    config_.closed_loop = get_parameter("closed_loop").as_bool();
    publish_debug_ = get_parameter("publish_debug").as_bool();
    debug_timing_ = get_parameter("debug_timing").as_bool();
    publish_heading_error_ = get_parameter("publish_heading_error").as_bool();
    config_.publish_frenet_velocity = get_parameter("publish_frenet_velocity").as_bool();
    compatibility_mode_ = get_parameter("compatibility_mode").as_bool();
    use_path_preprocessing_ = get_parameter("use_path_preprocessing").as_bool();
    enable_path_smoothing_ = get_parameter("enable_path_smoothing").as_bool();
    enable_curvature_reduction_ = get_parameter("enable_curvature_reduction").as_bool();

    config_.path_change_tolerance = get_parameter("path_change_tolerance").as_double();
    config_.duplicate_point_tolerance =
      get_parameter("duplicate_point_tolerance").as_double();
    reference_resample_step_ = get_parameter("reference_resample_step").as_double();
    config_.tangent_epsilon = get_parameter("tangent_epsilon").as_double();
    config_.max_projection_distance = get_parameter("max_projection_distance").as_double();
    config_.projection_domain_limit = get_parameter("projection_domain_limit").as_double();
    config_.projection_domain_epsilon = get_parameter("projection_domain_epsilon").as_double();
    config_.projection_domain_eps2 = get_parameter("projection_domain_eps2").as_double();
    config_.min_path_length = get_parameter("min_path_length").as_double();
    config_.large_gap_factor = get_parameter("large_gap_factor").as_double();
    config_.projection_domain_method = get_parameter("projection_domain_method").as_int();
    config_.velocity_frame = parseVelocityFrame(get_parameter("velocity_frame").as_string());

    if (!use_path_preprocessing_) {
      RCLCPP_WARN(
        get_logger(),
        "use_path_preprocessing=false requested, but safety filtering of invalid and duplicate "
        "points remains enabled because CLCS rejects degenerate paths.");
    }
  }

  void waypointsCallback(const eufs_msgs::msg::WaypointArrayStamped::SharedPtr msg)
  {
    const auto raw_waypoints = toReferenceWaypoints(*msg);
    if (raw_waypoints.size() < 3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Received %zu waypoint(s); CLCS requires at least 3 valid reference points.",
        raw_waypoints.size());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(converter_mutex_);
      if (!ClcsFrenetConverter::pathChanged(
          last_raw_waypoints_, raw_waypoints, config_.path_change_tolerance))
      {
        return;
      }
    }

    const std::uint64_t next_version = path_version_ + 1;
    ClcsFrenetConverter::Ptr new_converter;
    try {
      new_converter = ClcsFrenetConverter::create(raw_waypoints, config_, next_version);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to build CommonRoad-CLCS reference path. Keeping previous valid CLCS if any: %s",
        e.what());
      return;
    }

    const auto & stats = new_converter->stats();
    {
      std::lock_guard<std::mutex> lock(converter_mutex_);
      converter_ = new_converter;
      last_raw_waypoints_ = raw_waypoints;
      path_version_ = stats.path_version;
    }

    RCLCPP_INFO(
      get_logger(),
      "Built CLCS path version=%lu input=%zu reference=%zu track_length=%.3f m "
      "build_time=%.3f ms removed_duplicates=%zu invalid=%zu",
      static_cast<unsigned long>(stats.path_version), stats.input_waypoint_count,
      stats.reference_point_count, stats.track_length, stats.build_time_ms,
      stats.removed_duplicate_count, stats.invalid_point_count);

    if (stats.large_gap_count > 0) {
      RCLCPP_WARN(
        get_logger(), "Reference path has %zu unusually large waypoint gap(s).",
        stats.large_gap_count);
    }
    if (stats.self_intersection_count > 0) {
      RCLCPP_WARN(
        get_logger(), "Reference path has %zu possible self-intersection(s).",
        stats.self_intersection_count);
    }
    if (stats.waypoint_s_max_error > 0.1) {
      RCLCPP_WARN(
        get_logger(),
        "Waypoint s_m differs from CLCS geometric arc length. max_error=%.3f m. "
        "Published s uses CLCS geometric arc length.",
        stats.waypoint_s_max_error);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    ClcsFrenetConverter::ConstPtr converter;
    {
      std::lock_guard<std::mutex> lock(converter_mutex_);
      converter = converter_;
    }

    if (!converter) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for a valid CLCS reference path from %s.", waypoint_topic_.c_str());
      return;
    }

    ClcsConversionInput input;
    input.x = odom->pose.pose.position.x;
    input.y = odom->pose.pose.position.y;
    input.yaw = yawFromQuaternion(odom->pose.pose.orientation);
    input.linear_x = odom->twist.twist.linear.x;
    input.linear_y = odom->twist.twist.linear.y;
    input.yaw_rate = odom->twist.twist.angular.z;

    const auto conversion = converter->convert(input);
    if (!conversion.valid) {
      handleProjectionFailure(*odom, conversion);
      return;
    }

    auto output = buildOutputOdometry(*odom, conversion);
    last_valid_odom_ = output;
    has_last_valid_odom_ = true;
    frenet_pub_->publish(output);
    publishDebug(conversion, true);
  }

  nav_msgs::msg::Odometry buildOutputOdometry(
    const nav_msgs::msg::Odometry & input,
    const ClcsConversionResult & conversion) const
  {
    nav_msgs::msg::Odometry output = input;
    output.header.frame_id = frenet_frame_id_;
    output.child_frame_id = std::to_string(conversion.segment_index);
    output.pose.pose.position.x = conversion.s;
    output.pose.pose.position.y = conversion.d;
    output.pose.pose.position.z = 0.0;

    if (!compatibility_mode_ && publish_heading_error_) {
      output.pose.pose.orientation = quaternionFromYaw(conversion.heading_error);
    }

    if (!compatibility_mode_ && config_.publish_frenet_velocity) {
      output.twist.twist.linear.x = conversion.v_s;
      output.twist.twist.linear.y = conversion.v_d;
      output.twist.twist.angular.z = conversion.yaw_rate;
    }

    if (!compatibility_mode_ && covariance_mode_ != "preserve") {
      clearCovariance(output);
    }
    return output;
  }

  void handleProjectionFailure(
    const nav_msgs::msg::Odometry & input,
    const ClcsConversionResult & conversion)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "CLCS projection failed: %s", conversion.error_message.c_str());

    publishDebug(conversion, false);

    if (projection_failure_policy_ == ProjectionFailurePolicy::kDropMessage) {
      return;
    }

    if (projection_failure_policy_ == ProjectionFailurePolicy::kPublishLastValid) {
      if (!has_last_valid_odom_) {
        return;
      }
      auto output = last_valid_odom_;
      output.header.stamp = input.header.stamp;
      frenet_pub_->publish(output);
      return;
    }

    nav_msgs::msg::Odometry output = input;
    output.header.frame_id = frenet_frame_id_;
    output.child_frame_id = "-1";
    output.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
    output.pose.pose.position.y = std::numeric_limits<double>::quiet_NaN();
    output.pose.pose.position.z = 0.0;
    if (!compatibility_mode_ && covariance_mode_ != "preserve") {
      clearCovariance(output);
    }
    frenet_pub_->publish(output);
  }

  void publishDebug(const ClcsConversionResult & conversion, const bool projection_valid)
  {
    if (!debug_pub_) {
      return;
    }

    std_msgs::msg::Float64MultiArray debug;
    debug.data = {
      conversion.s,
      conversion.d,
      conversion.reference_yaw,
      conversion.heading_error,
      conversion.v_s,
      conversion.v_d,
      conversion.track_length,
      projection_valid ? 1.0 : 0.0,
      debug_timing_ ? conversion.conversion_time_us : 0.0,
      conversion.clcs_build_time_ms,
      conversion.waypoint_s_max_error,
      static_cast<double>(conversion.path_version),
      conversion.reconstruction_error,
      static_cast<double>(conversion.segment_index)};
    debug_pub_->publish(debug);
  }

  ClcsFrenetConfig config_;
  bool publish_debug_{true};
  bool debug_timing_{true};
  bool publish_heading_error_{true};
  bool compatibility_mode_{false};
  bool use_path_preprocessing_{true};
  bool enable_path_smoothing_{false};
  bool enable_curvature_reduction_{false};
  bool has_last_valid_odom_{false};
  double reference_resample_step_{0.0};
  std::uint64_t path_version_{0};

  std::string odom_topic_;
  std::string waypoint_topic_;
  std::string frenet_odom_topic_;
  std::string debug_topic_;
  std::string frenet_frame_id_;
  std::string projection_failure_policy_name_;
  std::string covariance_mode_;
  ProjectionFailurePolicy projection_failure_policy_{ProjectionFailurePolicy::kDropMessage};

  std::mutex converter_mutex_;
  ClcsFrenetConverter::ConstPtr converter_;
  std::vector<ReferenceWaypoint> last_raw_waypoints_;
  nav_msgs::msg::Odometry last_valid_odom_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<eufs_msgs::msg::WaypointArrayStamped>::SharedPtr waypoint_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr frenet_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
};

}  // namespace frenet_conversion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<frenet_conversion::FrenetOdomNode>());
  rclcpp::shutdown();
  return 0;
}
