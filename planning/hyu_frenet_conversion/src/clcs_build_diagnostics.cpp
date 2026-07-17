#include "hyu_frenet_conversion/clcs_build_diagnostics.hpp"

#include "rclcpp/rclcpp.hpp"

namespace hyu_frenet_conversion
{

void logClcsBuildStats(const rclcpp::Logger & logger, const ClcsBuildStats & stats)
{
  RCLCPP_INFO(
    logger,
    "Built CLCS path version=%lu input=%zu reference=%zu track_length=%.3f m "
    "build_time=%.3f ms removed_duplicates=%zu invalid=%zu",
    static_cast<unsigned long>(stats.path_version), stats.input_waypoint_count,
    stats.reference_point_count, stats.track_length, stats.build_time_ms,
    stats.removed_duplicate_count, stats.invalid_point_count);

  if (stats.large_gap_count > 0) {
    RCLCPP_WARN(
      logger, "Reference path has %zu unusually large waypoint gap(s).",
      stats.large_gap_count);
  }
  if (stats.self_intersection_count > 0) {
    RCLCPP_WARN(
      logger, "Reference path has %zu possible self-intersection(s).",
      stats.self_intersection_count);
  }
  if (stats.waypoint_s_max_error > 0.1) {
    RCLCPP_WARN(
      logger,
      "Waypoint s_m differs from CLCS geometric arc length. max_error=%.3f m. "
      "Published s uses CLCS geometric arc length.",
      stats.waypoint_s_max_error);
  }
}

}
