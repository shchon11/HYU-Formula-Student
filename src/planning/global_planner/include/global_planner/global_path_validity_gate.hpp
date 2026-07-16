#pragma once

#include <cstdint>
#include <memory>

#include "global_planner/path_snapshot.hpp"
#include "rclcpp/time.hpp"

namespace wpnt_publisher
{

struct ValidityEventResult
{
  bool activated{false};
  bool invalidated{false};
  bool stale_invalidated{false};
};

struct OdomGateResult
{
  std::shared_ptr<const PathSnapshot> snapshot;
  bool has_validity{false};
  bool validity_true{false};
};

class GlobalPathValidityGate
{
public:
  explicit GlobalPathValidityGate(double timeout_sec);

  void setTimeout(double timeout_sec);
  void onSnapshot(std::shared_ptr<const PathSnapshot> snapshot, const rclcpp::Time & current_time);
  ValidityEventResult onValidity(bool valid, const rclcpp::Time & current_time);
  bool invalidateIfStale(const rclcpp::Time & current_time);
  OdomGateResult readForOdom(const rclcpp::Time & current_time);

private:
  bool validityFresh(const rclcpp::Time & current_time) const;
  bool invalidate();
  bool activatePending();

  double timeout_sec_{0.5};
  std::shared_ptr<const PathSnapshot> path_;
  std::shared_ptr<const PathSnapshot> pending_path_;
  bool has_global_path_valid_{false};
  bool global_path_valid_{false};
  bool has_invalidation_{false};
  rclcpp::Time last_global_path_valid_time_;
  std::uint64_t invalidation_generation_{0U};
  std::uint64_t pending_path_generation_{0U};
};

}  // namespace wpnt_publisher
