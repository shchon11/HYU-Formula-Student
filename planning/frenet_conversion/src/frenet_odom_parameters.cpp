#include "frenet_conversion/frenet_odom_parameters.hpp"

namespace frenet_conversion
{
namespace
{

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

}

FrenetOutputOptions FrenetOdomParameters::outputOptions() const
{
  return {
    frenet_frame_id,
    compatibility_mode,
    publish_heading_error,
    config.publish_frenet_velocity,
    covariance_mode != "preserve"};
}

void declareFrenetOdomParameters(rclcpp::Node & node)
{
  node.declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
  node.declare_parameter<std::string>("waypoint_topic", "/global_waypoints");
  node.declare_parameter<std::string>("global_path_valid_topic", "/planning/global_path_valid");
  node.declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
  node.declare_parameter<std::string>("debug_topic", "/car_state/frenet/debug");
  node.declare_parameter<std::string>("frenet_frame_id", "frenet");
  node.declare_parameter<std::string>("projection_failure_policy", "drop_message");
  node.declare_parameter<std::string>("velocity_frame", "body");
  node.declare_parameter<std::string>("covariance_mode", "zero");
  node.declare_parameter<bool>("closed_loop", true);
  node.declare_parameter<bool>("publish_debug", true);
  node.declare_parameter<bool>("debug_timing", true);
  node.declare_parameter<bool>("publish_heading_error", true);
  node.declare_parameter<bool>("publish_frenet_velocity", true);
  node.declare_parameter<bool>("compatibility_mode", false);
  node.declare_parameter<bool>("use_path_preprocessing", true);
  node.declare_parameter<bool>("enable_path_smoothing", false);
  node.declare_parameter<bool>("enable_curvature_reduction", false);
  node.declare_parameter<double>("path_change_tolerance", 1.0e-4);
  node.declare_parameter<double>("duplicate_point_tolerance", 1.0e-3);
  node.declare_parameter<double>("reference_resample_step", 0.0);
  node.declare_parameter<double>("tangent_epsilon", 0.05);
  node.declare_parameter<double>("max_projection_distance", 20.0);
  node.declare_parameter<double>("projection_domain_limit", 20.0);
  node.declare_parameter<double>("projection_domain_epsilon", 0.1);
  node.declare_parameter<double>("projection_domain_eps2", 0.0);
  node.declare_parameter<double>("min_path_length", 0.5);
  node.declare_parameter<double>("large_gap_factor", 5.0);
  node.declare_parameter<double>("global_path_valid_timeout_sec", 0.5);
  node.declare_parameter<int>("projection_domain_method", 1);
}

FrenetOdomParameters loadFrenetOdomParameters(rclcpp::Node & node)
{
  FrenetOdomParameters parameters;
  parameters.odom_topic = node.get_parameter("odom_topic").as_string();
  parameters.waypoint_topic = node.get_parameter("waypoint_topic").as_string();
  parameters.global_path_valid_topic =
    node.get_parameter("global_path_valid_topic").as_string();
  parameters.frenet_odom_topic = node.get_parameter("frenet_odom_topic").as_string();
  parameters.debug_topic = node.get_parameter("debug_topic").as_string();
  parameters.frenet_frame_id = node.get_parameter("frenet_frame_id").as_string();
  parameters.projection_failure_policy_name =
    node.get_parameter("projection_failure_policy").as_string();
  parameters.projection_failure_policy =
    parseProjectionFailurePolicy(parameters.projection_failure_policy_name);
  parameters.covariance_mode = node.get_parameter("covariance_mode").as_string();
  parameters.config.closed_loop = node.get_parameter("closed_loop").as_bool();
  parameters.publish_debug = node.get_parameter("publish_debug").as_bool();
  parameters.debug_timing = node.get_parameter("debug_timing").as_bool();
  parameters.publish_heading_error = node.get_parameter("publish_heading_error").as_bool();
  parameters.config.publish_frenet_velocity =
    node.get_parameter("publish_frenet_velocity").as_bool();
  parameters.compatibility_mode = node.get_parameter("compatibility_mode").as_bool();
  parameters.use_path_preprocessing = node.get_parameter("use_path_preprocessing").as_bool();
  parameters.enable_path_smoothing = node.get_parameter("enable_path_smoothing").as_bool();
  parameters.enable_curvature_reduction =
    node.get_parameter("enable_curvature_reduction").as_bool();
  parameters.config.path_change_tolerance =
    node.get_parameter("path_change_tolerance").as_double();
  parameters.config.duplicate_point_tolerance =
    node.get_parameter("duplicate_point_tolerance").as_double();
  parameters.reference_resample_step =
    node.get_parameter("reference_resample_step").as_double();
  parameters.config.tangent_epsilon = node.get_parameter("tangent_epsilon").as_double();
  parameters.config.max_projection_distance =
    node.get_parameter("max_projection_distance").as_double();
  parameters.config.projection_domain_limit =
    node.get_parameter("projection_domain_limit").as_double();
  parameters.config.projection_domain_epsilon =
    node.get_parameter("projection_domain_epsilon").as_double();
  parameters.config.projection_domain_eps2 =
    node.get_parameter("projection_domain_eps2").as_double();
  parameters.config.min_path_length = node.get_parameter("min_path_length").as_double();
  parameters.config.large_gap_factor = node.get_parameter("large_gap_factor").as_double();
  parameters.global_path_valid_timeout_sec =
    node.get_parameter("global_path_valid_timeout_sec").as_double();
  parameters.config.projection_domain_method =
    node.get_parameter("projection_domain_method").as_int();
  parameters.config.velocity_frame =
    parseVelocityFrame(node.get_parameter("velocity_frame").as_string());

  if (!parameters.use_path_preprocessing) {
    RCLCPP_WARN(
      node.get_logger(),
      "use_path_preprocessing=false requested, but safety filtering of invalid and duplicate "
      "points remains enabled because CLCS rejects degenerate paths.");
  }
  if (parameters.global_path_valid_timeout_sec <= 0.0) {
    RCLCPP_WARN(
      node.get_logger(),
      "global_path_valid_timeout_sec must be positive; using 0.5 seconds.");
    parameters.global_path_valid_timeout_sec = 0.5;
  }
  return parameters;
}

}
