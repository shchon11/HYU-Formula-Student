#include "state_machine/planning_state_debug.hpp"

#include <sstream>

namespace state_machine
{

PathSource selectPathSource(
  PlanningState state,
  int lap_count,
  int final_lap_start_count)
{
  if (state == PlanningState::STOP) {
    return PathSource::STOP;
  }

  if (state == PlanningState::LOCAL) {
    return PathSource::LOCAL;
  }

  if (lap_count >= final_lap_start_count) {
    return PathSource::GLOBAL_FINAL_STOP;
  }

  return PathSource::GLOBAL_FULL;
}

std::string planningStateToString(PlanningState state)
{
  switch (state) {
    case PlanningState::LOCAL:
      return "LOCAL";
    case PlanningState::GLOBAL:
      return "GLOBAL";
    case PlanningState::STOP:
      return "STOP";
  }

  return "LOCAL";
}

std::string pathSourceToString(PathSource source)
{
  switch (source) {
    case PathSource::LOCAL:
      return "LOCAL";
    case PathSource::GLOBAL_FULL:
      return "GLOBAL_FULL";
    case PathSource::GLOBAL_FINAL_STOP:
      return "GLOBAL_FINAL_STOP";
    case PathSource::STOP:
      return "STOP";
  }

  return "LOCAL";
}

std::string makePlanningStateDebugString(const PlanningStateDebugSnapshot & snapshot)
{
  std::ostringstream ss;
  ss << "state=" << planningStateToString(snapshot.state)
     << " path_source=" << pathSourceToString(snapshot.path_source)
     << " lap_count=" << snapshot.lap_count
     << " current_s=" << snapshot.current_s
     << " current_d=" << snapshot.current_d
     << " global_path_length=" << snapshot.global_path_length
     << " frenet_fresh=" << (snapshot.frenet_fresh ? "true" : "false")
     << " graph_slam_status=" << snapshot.graph_slam_status
     << " graph_slam_localized=" << (snapshot.graph_slam_localized ? "true" : "false")
     << " global_path_valid=" << (snapshot.global_path_valid ? "true" : "false")
     << " global_path_valid_fresh=" << (
      snapshot.global_path_valid_fresh ? "true" : "false")
     << " accepted_waypoint_generation=" << snapshot.accepted_waypoint_generation
     << " invalidation_generation=" << snapshot.invalidation_generation
     << " accepted_waypoints_time_sec=" << snapshot.accepted_waypoints_time_sec
     << " accepted_waypoint_count=" << snapshot.accepted_waypoint_count
     << " cone_map_fresh=" << (snapshot.cone_map_fresh ? "true" : "false")
     << " stop_zone_fresh=" << (snapshot.stop_zone_fresh ? "true" : "false")
     << " stop_zone_valid=" << (snapshot.stop_zone_valid ? "true" : "false")
     << " stop_zone_s_start=" << snapshot.stop_zone_s_start
     << " stop_zone_s_end=" << snapshot.stop_zone_s_end
     << " global_path_ready=" << (snapshot.global_path_ready ? "true" : "false")
     << " global_requires_graph_slam_localization=" << (
      snapshot.global_requires_graph_slam_localization ? "true" : "false")
     << " global_readiness_reason=" << snapshot.global_readiness_reason
     << " global_entry_reason=" << snapshot.global_entry_reason
     << " stop_request=" << (snapshot.stop_request ? "true" : "false")
     << " stop_request_reason=" << snapshot.stop_request_reason
     << " closest_segment_id=" << snapshot.closest_segment_id
     << " cone_frame_id=" << snapshot.cone_frame_id
     << " cones_blue=" << snapshot.blue_cone_count
     << " cones_yellow=" << snapshot.yellow_cone_count
     << " cones_orange=" << snapshot.orange_cone_count
     << " cones_big_orange=" << snapshot.big_orange_cone_count
     << " cones_unknown=" << snapshot.unknown_cone_count
     << " local_path_valid_received=" << (
      snapshot.local_path_valid_received ? "true" : "false")
     << " local_path_valid=" << (snapshot.local_path_valid ? "true" : "false")
     << " local_path_valid_fresh=" << (
      snapshot.local_path_valid_fresh ? "true" : "false")
     << " local_path_valid_time_sec=" << snapshot.local_path_valid_time_sec
     << " global_handoff_received=" << (
      snapshot.global_handoff_received ? "true" : "false")
     << " global_handoff_ready=" << (snapshot.global_handoff_ready ? "true" : "false")
     << " global_handoff_fresh=" << (
      snapshot.global_handoff_fresh ? "true" : "false")
     << " global_handoff_dwell_ready=" << (
      snapshot.global_handoff_dwell_ready ? "true" : "false")
     << " global_handoff_time_sec=" << snapshot.global_handoff_time_sec
     << " lap_path_valid=" << (snapshot.lap_path_valid ? "true" : "false")
     << " lap_armed=" << (snapshot.lap_armed ? "true" : "false")
     << " lap_path_length=" << snapshot.lap_path_length
     << " lap_path_generation=" << snapshot.lap_path_generation
     << " lap_gate_valid=" << (snapshot.lap_gate_valid ? "true" : "false")
     << " lap_gate_armed=" << (snapshot.lap_gate_armed ? "true" : "false")
     << " lap_time_last=" << snapshot.lap_time_last
     << " lap_time_best=" << snapshot.lap_time_best
     << " lap_elapsed=" << snapshot.lap_elapsed;

  return ss.str();
}

}
