#include "state_machine/planning_state_machine_node.hpp"

#include "state_machine/planning_state_debug.hpp"

namespace state_machine
{

void PlanningStateMachineNode::publishOutputs()
{
  const PathSource source = selectPathSource(state_, lap_count_, final_lap_start_count_);
  const bool stop_request = state_ == PlanningState::STOP;

  std_msgs::msg::String state_msg;
  state_msg.data = planningStateToString(state_);
  state_pub_->publish(state_msg);

  std_msgs::msg::String path_source_msg;
  path_source_msg.data = pathSourceToString(source);
  path_source_pub_->publish(path_source_msg);

  std_msgs::msg::Int32 lap_count_msg;
  lap_count_msg.data = lap_count_;
  lap_count_pub_->publish(lap_count_msg);

  std_msgs::msg::Bool stop_request_msg;
  stop_request_msg.data = stop_request;
  stop_request_pub_->publish(stop_request_msg);

  std_msgs::msg::String debug_msg;
  debug_msg.data = makePlanningStateDebugString({
    state_,
    source,
    lap_count_,
    current_s_,
    current_d_,
    global_path_readiness_.pathLength(),
    hasFreshFrenetOdom(),
    global_path_readiness_.graphSlamStatus(),
    global_path_readiness_.graphSlamLocalized(),
    global_path_readiness_.pathValid(),
    global_path_readiness_.hasFreshValidity(now(), global_path_valid_timeout_sec_),
    global_path_readiness_.acceptedWaypointGeneration(),
    global_path_readiness_.invalidationGeneration(),
    global_path_readiness_.hasWaypoints() ?
    global_path_readiness_.lastWaypointsTime().seconds() : 0.0,
    global_path_readiness_.acceptedWaypointCount(),
    hasFreshConeMap(),
    hasFreshStopZone(),
    stop_zone_valid_,
    stop_zone_s_start_,
    stop_zone_s_end_,
    isGlobalPathReady(),
    stop_request,
    closest_segment_id_,
    cone_frame_id_,
    blue_cone_count_,
    yellow_cone_count_,
    orange_cone_count_,
    big_orange_cone_count_,
    unknown_cone_count_});
  debug_pub_->publish(debug_msg);
}

}
