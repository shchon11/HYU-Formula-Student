#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "local_planner/input_policy.hpp"
#include "local_planner/local_path_builder.hpp"
#include "local_planner/ros_inputs.hpp"
#include "local_planner/ros_outputs.hpp"

namespace local_planner
{

class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void processLivePair(const LiveInputPair & input);
  void processSlamMap(const SlamMapInput & input);

  SourceMode source_mode_;
  PlannerConfig planner_config_;
  double max_stamp_skew_sec_{0.1};
  double max_input_age_sec_{0.5};
  double heartbeat_hz_{10.0};
  std::unique_ptr<LocalPlannerOutput> output_;
  std::unique_ptr<LocalPlannerInputs> inputs_;
};

}
