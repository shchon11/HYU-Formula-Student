#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "clcs_frenet_converter.hpp"
#include "hyu_msgs/msg/waypoint_array_stamped.hpp"
#include "hyu_frenet_conversion/clcs_build_diagnostics.hpp"
#include "hyu_frenet_conversion/frenet_odom_messages.hpp"
#include "hyu_frenet_conversion/frenet_odom_parameters.hpp"
#include "hyu_frenet_conversion/global_path_validity_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace hyu_frenet_conversion
{
class FrenetOdomNode : public rclcpp::Node
{
public:
  FrenetOdomNode()
  : Node("frenet_odom_node")
  {
    declareFrenetOdomParameters(*this);
    parameters_ = loadFrenetOdomParameters(*this);
    global_path_validity_.setTimeout(parameters_.global_path_valid_timeout_sec);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      parameters_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      std::bind(&FrenetOdomNode::odomCallback, this, std::placeholders::_1));

    waypoint_sub_ = create_subscription<hyu_msgs::msg::WaypointArrayStamped>(
      parameters_.waypoint_topic, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&FrenetOdomNode::waypointsCallback, this, std::placeholders::_1));

    global_path_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      parameters_.global_path_valid_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      std::bind(&FrenetOdomNode::globalPathValidCallback, this, std::placeholders::_1));

    validity_watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&FrenetOdomNode::validityWatchdogCallback, this));

    frenet_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      parameters_.frenet_odom_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

    if (parameters_.publish_debug) {
      debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        parameters_.debug_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    }

    if (parameters_.enable_path_smoothing || parameters_.enable_curvature_reduction ||
      parameters_.reference_resample_step > 0.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "CommonRoad-CLCS C++ core does not expose the Python preprocessing pipeline used by "
        "commonroad_clcs.clcs. Only finite filtering, duplicate removal, and closed-loop "
        "closing are applied in this node.");
    }

    RCLCPP_INFO(
      get_logger(),
      "CLCS frenet odom started: odom=%s waypoints=%s output=%s closed_loop=%s "
      "child_frame_id=closest_segment_index velocity_frame=%s failure_policy=%s "
      "global_path_valid=%s timeout=%.3f s",
      parameters_.odom_topic.c_str(), parameters_.waypoint_topic.c_str(),
      parameters_.frenet_odom_topic.c_str(),
      parameters_.config.closed_loop ? "true" : "false",
      toString(parameters_.config.velocity_frame).c_str(),
      parameters_.projection_failure_policy_name.c_str(),
      parameters_.global_path_valid_topic.c_str(), parameters_.global_path_valid_timeout_sec);
  }

private:
  void waypointsCallback(const hyu_msgs::msg::WaypointArrayStamped::SharedPtr msg)
  {
    const auto raw_waypoints = toReferenceWaypoints(*msg);
    if (raw_waypoints.size() < 3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Received %zu waypoint(s); CLCS requires at least 3 valid reference points.",
        raw_waypoints.size());
      return;
    }

    std::uint64_t waypoint_invalidation_generation = 0U;
    std::uint64_t next_version = 0U;
    {
      std::lock_guard<std::mutex> lock(converter_mutex_);
      if (!ClcsFrenetConverter::pathChanged(
          last_raw_waypoints_, raw_waypoints, parameters_.config.path_change_tolerance))
      {
        return;
      }
      waypoint_invalidation_generation = global_path_validity_.invalidationGeneration();
      next_version = path_version_ + 1;
    }

    ClcsFrenetConverter::Ptr new_converter;
    try {
      new_converter = ClcsFrenetConverter::create(
        raw_waypoints, parameters_.config, next_version);
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
      if (!global_path_validity_.isCurrentGeneration(waypoint_invalidation_generation)) {
        RCLCPP_WARN(
          get_logger(),
          "Discarding CLCS build because global path validity changed while building it.");
        return;
      }
      converter_ = new_converter;
      last_raw_waypoints_ = raw_waypoints;
      path_version_ = stats.path_version;
      global_path_validity_.acceptWaypoints();
    }

    logClcsBuildStats(get_logger(), stats);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    ClcsFrenetConverter::ConstPtr converter;
    std::string wait_reason;
    std::string wait_topic;
    {
      std::lock_guard<std::mutex> lock(converter_mutex_);
      const auto now = get_clock()->now();
      invalidateIfStaleLocked(now);
      if (!hasFreshValidHeartbeatLocked(now)) {
        wait_reason = "a fresh true global path validity heartbeat";
        wait_topic = parameters_.global_path_valid_topic;
      } else if (!global_path_validity_.hasAcceptedWaypointsForCurrentGeneration()) {
        wait_reason = "a /global_waypoints snapshot after the latest invalidation";
        wait_topic = parameters_.waypoint_topic;
      } else if (!converter_) {
        wait_reason = "a valid CLCS reference path";
        wait_topic = parameters_.waypoint_topic;
      }
      converter = converter_;
    }

    if (!wait_reason.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for %s from %s before publishing Frenet odom.",
        wait_reason.c_str(), wait_topic.c_str());
      return;
    }

    const auto input = toConversionInput(*odom);
    const auto conversion = converter->convert(input);
    if (!conversion.valid) {
      handleProjectionFailure(*odom, conversion);
      return;
    }

    auto output = buildFrenetOdometry(*odom, conversion, parameters_.outputOptions());
    last_valid_odom_ = output;
    has_last_valid_odom_ = true;
    frenet_pub_->publish(output);
    publishDebug(conversion, true);
  }

  void globalPathValidCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const auto now = get_clock()->now();
    std::lock_guard<std::mutex> lock(converter_mutex_);
    if (!msg->data) {
      invalidateGlobalPathLocked(now, "global path validity heartbeat is false");
      return;
    }

    invalidateIfStaleLocked(now);
    const auto last_valid_heartbeat_time_ = now;
    global_path_validity_.recordValidHeartbeat(last_valid_heartbeat_time_);
  }

  void validityWatchdogCallback()
  {
    const auto now = get_clock()->now();
    std::lock_guard<std::mutex> lock(converter_mutex_);
    invalidateIfStaleLocked(now);
  }

  bool hasFreshValidHeartbeatLocked(const rclcpp::Time & now) const
  {
    return global_path_validity_.hasFreshValidHeartbeat(now);
  }

  void invalidateIfStaleLocked(const rclcpp::Time & now)
  {
    applyInvalidationLocked(global_path_validity_.invalidateIfStale(now));
  }

  void invalidateGlobalPathLocked(const rclcpp::Time & now, const std::string & reason)
  {
    applyInvalidationLocked(global_path_validity_.invalidate(now, reason));
  }

  void applyInvalidationLocked(const GlobalPathInvalidation & invalidation)
  {
    if (!invalidation.invalidated) {
      return;
    }
    converter_.reset();
    last_raw_waypoints_.clear();
    has_last_valid_odom_ = false;
    path_version_ = 0U;
    RCLCPP_WARN(get_logger(), "Cleared CLCS reference path: %s.", invalidation.reason.c_str());
  }

  void handleProjectionFailure(
    const nav_msgs::msg::Odometry & input,
    const ClcsConversionResult & conversion)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "CLCS projection failed: %s", conversion.error_message.c_str());

    publishDebug(conversion, false);

    if (parameters_.projection_failure_policy == ProjectionFailurePolicy::kDropMessage) {
      return;
    }

    if (parameters_.projection_failure_policy == ProjectionFailurePolicy::kPublishLastValid) {
      if (!has_last_valid_odom_) {
        return;
      }
      auto output = last_valid_odom_;
      output.header.stamp = input.header.stamp;
      frenet_pub_->publish(output);
      return;
    }

    auto output = buildNanFrenetOdometry(input, parameters_.outputOptions());
    frenet_pub_->publish(output);
  }

  void publishDebug(const ClcsConversionResult & conversion, const bool projection_valid)
  {
    if (!debug_pub_) {
      return;
    }

    const auto debug = buildFrenetDebugMessage(
      conversion, projection_valid, parameters_.debug_timing);
    debug_pub_->publish(debug);
  }

  bool has_last_valid_odom_{false};
  std::uint64_t path_version_{0};

  std::mutex converter_mutex_;
  ClcsFrenetConverter::ConstPtr converter_;
  std::vector<ReferenceWaypoint> last_raw_waypoints_;
  nav_msgs::msg::Odometry last_valid_odom_;
  FrenetOdomParameters parameters_;
  GlobalPathValidityState global_path_validity_{0.5};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<hyu_msgs::msg::WaypointArrayStamped>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_path_valid_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr frenet_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr validity_watchdog_timer_;
};

}  // namespace hyu_frenet_conversion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hyu_frenet_conversion::FrenetOdomNode>());
  rclcpp::shutdown();
  return 0;
}
