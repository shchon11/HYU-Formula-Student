#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "state_machine/planning_state_machine_node.hpp"

namespace state_machine
{

struct PlanningStateDebugSnapshot
{
  PlanningState state{PlanningState::LOCAL};
  PathSource path_source{PathSource::LOCAL};
  int lap_count{0};
  double current_s{0.0};
  double current_d{0.0};
  double global_path_length{0.0};
  bool frenet_fresh{false};
  std::string graph_slam_status;
  bool graph_slam_localized{false};
  bool global_path_valid{false};
  bool global_path_valid_fresh{false};
  std::uint64_t accepted_waypoint_generation{0U};
  std::uint64_t invalidation_generation{0U};
  double accepted_waypoints_time_sec{0.0};
  std::size_t accepted_waypoint_count{0U};
  bool cone_map_fresh{false};
  bool stop_zone_fresh{false};
  bool stop_zone_valid{false};
  double stop_zone_s_start{0.0};
  double stop_zone_s_end{0.0};
  bool global_path_ready{false};
  bool global_requires_graph_slam_localization{true};
  std::string global_readiness_reason{"missing_global_path"};
  std::string global_entry_reason{"missing_global_path"};
  bool stop_request{false};
  std::string stop_request_reason{"not_requested"};
  std::string closest_segment_id;
  std::string cone_frame_id;
  std::size_t blue_cone_count{0U};
  std::size_t yellow_cone_count{0U};
  std::size_t orange_cone_count{0U};
  std::size_t big_orange_cone_count{0U};
  std::size_t unknown_cone_count{0U};
  bool local_path_valid_received{false};
  bool local_path_valid{false};
  bool local_path_valid_fresh{false};
  double local_path_valid_time_sec{0.0};
  bool global_handoff_received{false};
  bool global_handoff_ready{false};
  bool global_handoff_fresh{false};
  bool global_handoff_dwell_ready{false};
  double global_handoff_time_sec{0.0};
  bool lap_path_valid{false};
  bool lap_armed{false};
  double lap_path_length{0.0};
  std::uint64_t lap_path_generation{0U};
  bool lap_gate_valid{false};
  bool lap_gate_armed{false};
  double lap_time_last{0.0};
  double lap_time_best{0.0};
  double lap_elapsed{-1.0};
};

PathSource selectPathSource(PlanningState state, int lap_count, int final_lap_start_count);
std::string planningStateToString(PlanningState state);
std::string pathSourceToString(PathSource source);
std::string makePlanningStateDebugString(const PlanningStateDebugSnapshot & snapshot);

}
