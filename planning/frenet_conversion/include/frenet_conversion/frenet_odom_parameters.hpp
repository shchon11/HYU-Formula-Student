#pragma once

#include <string>

#include "clcs_frenet_converter.hpp"
#include "frenet_conversion/frenet_odom_messages.hpp"
#include "rclcpp/rclcpp.hpp"

namespace frenet_conversion
{

enum class ProjectionFailurePolicy
{
  kDropMessage,
  kPublishLastValid,
  kPublishNan
};

struct FrenetOdomParameters
{
  ClcsFrenetConfig config;
  bool publish_debug{true};
  bool debug_timing{true};
  bool publish_heading_error{true};
  bool compatibility_mode{false};
  bool use_path_preprocessing{true};
  bool enable_path_smoothing{false};
  bool enable_curvature_reduction{false};
  double reference_resample_step{0.0};
  double global_path_valid_timeout_sec{0.5};
  std::string odom_topic;
  std::string waypoint_topic;
  std::string global_path_valid_topic;
  std::string frenet_odom_topic;
  std::string debug_topic;
  std::string frenet_frame_id;
  std::string projection_failure_policy_name;
  std::string covariance_mode;
  ProjectionFailurePolicy projection_failure_policy{ProjectionFailurePolicy::kDropMessage};

  FrenetOutputOptions outputOptions() const;
};

void declareFrenetOdomParameters(rclcpp::Node & node);
FrenetOdomParameters loadFrenetOdomParameters(rclcpp::Node & node);

}
