#pragma once

#include <optional>

#include "hyu_msgs/msg/waypoint_array_stamped.hpp"

namespace hyu_path_selector
{

struct TransitionBlendConfig
{
  // Distance the car travels while the output cross-fades LOCAL -> GLOBAL.
  double blend_length_m{10.0};
  double waypoint_spacing_m{0.5};
  // Speed profiling mirrors hyu_local_planner's finishPath():
  // v = max(min_speed, min(cap, sqrt(max_lateral_accel / |kappa|))).
  double max_lateral_accel_mps2{3.0};
  double min_speed_mps{1.0};
};

// Builds the LOCAL->GLOBAL transition path: a smoothstep cross-fade from the
// frozen local path (what the car was tracking at the flip) onto the global
// path, followed by the untouched global tail (raceline speed profile kept).
// Both inputs must already be trimmed at the ego nearest point.
// blend_progress_m is the distance travelled since the flip; the cross-fade
// weight starts at smoothstep(progress/length) so successive calls converge
// onto the global path as the car advances.
// Returns nullopt when no blend segment can be built (window exhausted or
// degenerate inputs) — the caller then publishes the pure global path.
std::optional<hyu_msgs::msg::WaypointArrayStamped> buildTransitionPath(
  const hyu_msgs::msg::WaypointArrayStamped & local_trimmed,
  const hyu_msgs::msg::WaypointArrayStamped & global_trimmed,
  double blend_progress_m,
  const TransitionBlendConfig & config);

}
