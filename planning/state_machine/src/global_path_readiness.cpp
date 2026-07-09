#include "state_machine/global_path_readiness.hpp"

namespace state_machine
{

bool GlobalPathReadiness::onWaypoints(
  const eufs_msgs::msg::WaypointArrayStamped & msg,
  const rclcpp::Time & current_time)
{
  ++waypoint_message_generation_;

  if (msg.waypoints.empty()) {
    return false;
  }

  has_global_waypoints_ = true;
  global_path_ready_ = true;
  accepted_waypoint_generation_ = waypoint_message_generation_;
  accepted_waypoint_count_ = msg.waypoints.size();
  last_global_waypoints_time_ = current_time;
  global_path_length_ = msg.waypoints.back().s_m;
  return true;
}

void GlobalPathReadiness::onGraphSlamStatus(const std::string & status)
{
  graph_slam_status_ = status;
  has_graph_slam_status_ = true;
}

void GlobalPathReadiness::onValidity(
  bool valid,
  const rclcpp::Time & current_time,
  double timeout_sec)
{
  refreshValidity(current_time, timeout_sec);
  has_global_path_valid_ = true;
  global_path_valid_ = valid;
  last_global_path_valid_time_ = current_time;

  if (!global_path_valid_) {
    invalidate();
  }
}

void GlobalPathReadiness::refreshValidity(
  const rclcpp::Time & current_time,
  double timeout_sec)
{
  if (!has_global_path_valid_ || !global_path_valid_) {
    return;
  }

  if ((current_time - last_global_path_valid_time_).seconds() <= timeout_sec) {
    return;
  }

  global_path_valid_ = false;
  invalidate();
}

bool GlobalPathReadiness::ready(
  const rclcpp::Time & current_time,
  double timeout_sec) const
{
  return has_global_waypoints_ &&
         global_path_ready_ &&
         accepted_waypoint_count_ > 0U &&
         accepted_waypoint_generation_ > invalidation_generation_ &&
         hasFreshValidity(current_time, timeout_sec) &&
         graphSlamLocalized();
}

bool GlobalPathReadiness::hasFreshValidity(
  const rclcpp::Time & current_time,
  double timeout_sec) const
{
  return has_global_path_valid_ &&
         global_path_valid_ &&
         (current_time - last_global_path_valid_time_).seconds() <= timeout_sec;
}

bool GlobalPathReadiness::graphSlamLocalized() const
{
  return has_graph_slam_status_ && graph_slam_status_ == "localization";
}

double GlobalPathReadiness::pathLength() const
{
  return global_path_length_;
}

bool GlobalPathReadiness::pathValid() const
{
  return global_path_valid_;
}

bool GlobalPathReadiness::hasWaypoints() const
{
  return has_global_waypoints_;
}

const rclcpp::Time & GlobalPathReadiness::lastWaypointsTime() const
{
  return last_global_waypoints_time_;
}

const std::string & GlobalPathReadiness::graphSlamStatus() const
{
  return graph_slam_status_;
}

std::uint64_t GlobalPathReadiness::acceptedWaypointGeneration() const
{
  return accepted_waypoint_generation_;
}

std::uint64_t GlobalPathReadiness::invalidationGeneration() const
{
  return invalidation_generation_;
}

std::size_t GlobalPathReadiness::acceptedWaypointCount() const
{
  return accepted_waypoint_count_;
}

void GlobalPathReadiness::invalidate()
{
  invalidation_generation_ = waypoint_message_generation_;
  has_global_waypoints_ = false;
  global_path_ready_ = false;
  global_path_length_ = 0.0;
  accepted_waypoint_count_ = 0U;
}

}
