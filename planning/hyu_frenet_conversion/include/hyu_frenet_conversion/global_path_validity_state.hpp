#pragma once

#include <cstdint>
#include <string>

#include "rclcpp/time.hpp"

namespace hyu_frenet_conversion
{

struct GlobalPathInvalidation
{
  bool invalidated{false};
  std::uint64_t generation{0U};
  std::string reason;
};

class GlobalPathValidityState
{
public:
  explicit GlobalPathValidityState(double timeout_sec);

  void setTimeout(double timeout_sec);
  double timeout() const { return timeout_sec_; }

  GlobalPathInvalidation invalidate(const rclcpp::Time & now, const std::string & reason);
  GlobalPathInvalidation invalidateIfStale(const rclcpp::Time & now);

  void recordValidHeartbeat(const rclcpp::Time & now);
  bool hasFreshValidHeartbeat(const rclcpp::Time & now) const;

  std::uint64_t invalidationGeneration() const { return invalidation_generation_; }
  bool isCurrentGeneration(std::uint64_t generation) const;

  void acceptWaypoints();
  bool hasAcceptedWaypointsForCurrentGeneration() const;

private:
  bool hasStaleHeartbeat(const rclcpp::Time & now) const;

  double timeout_sec_{0.5};
  std::uint64_t invalidation_generation_{0U};
  std::uint64_t accepted_waypoint_generation_{0U};
  bool global_path_valid_{false};
  bool has_valid_heartbeat_{false};
  rclcpp::Time latest_invalidation_time_;
  rclcpp::Time last_valid_heartbeat_time_;
};

}
