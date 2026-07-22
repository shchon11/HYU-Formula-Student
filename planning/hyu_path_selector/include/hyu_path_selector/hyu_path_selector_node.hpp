#pragma once

#include <chrono>
#include <mutex>
#include <optional>
#include <string>

#include "hyu_msgs/msg/waypoint_array_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "hyu_path_selector/continuity_check.hpp"
#include "hyu_path_selector/selection_policy.hpp"
#include "hyu_path_selector/transition_blend.hpp"

namespace hyu_path_selector
{

class PathSelectorNode : public rclcpp::Node
{
public:
  PathSelectorNode();

private:
  struct NodeState
  {
    std::optional<std::string> requested_source;
    double source_receive_time_sec{0.0};
    CandidateInput local;
    CandidateInput global;
    OdometryInput odometry;
    bool global_entry_handoff_consumed{false};
  };

  void onPathSource(const std_msgs::msg::String::SharedPtr message);
  void onLocalPath(const hyu_msgs::msg::WaypointArrayStamped::SharedPtr message);
  void onGlobalPath(const hyu_msgs::msg::WaypointArrayStamped::SharedPtr message);
  void onLocalValidity(const std_msgs::msg::Bool::SharedPtr message);
  void onGlobalValidity(const std_msgs::msg::Bool::SharedPtr message);
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void publishHeartbeat();

  static double steadyNowSec();
  std::string makeDiagnostic(
    const NodeState & state,
    const SelectionDecision & decision,
    const ContinuityResult & continuity,
    bool selected_valid) const;

  void resetBlend();

  std::string path_source_topic_;
  std::string local_path_topic_;
  std::string local_validity_topic_;
  std::string global_path_topic_;
  std::string global_validity_topic_;
  std::string odometry_topic_;
  std::string selected_path_topic_;
  std::string selected_path_viz_topic_;
  std::string selected_validity_topic_;
  std::string handoff_ready_topic_;
  std::string debug_topic_;

  SelectionPolicy selection_policy_;
  ContinuityCheck continuity_check_;

  // LOCAL->GLOBAL transition blend. All of this state is only touched from
  // the heartbeat timer thread, so it needs no locking.
  bool blend_enabled_{true};
  TransitionBlendConfig blend_config_;
  bool blend_active_{false};
  double blend_travelled_m_{0.0};
  bool has_blend_last_ego_{false};
  double blend_last_ego_x_{0.0};
  double blend_last_ego_y_{0.0};
  std::optional<hyu_msgs::msg::WaypointArrayStamped> blend_frozen_local_;
  std::optional<hyu_msgs::msg::WaypointArrayStamped> last_local_trimmed_;
  double last_local_trimmed_time_sec_{0.0};

  std::mutex state_mutex_;
  NodeState state_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_source_sub_;
  rclcpp::Subscription<hyu_msgs::msg::WaypointArrayStamped>::SharedPtr local_path_sub_;
  rclcpp::Subscription<hyu_msgs::msg::WaypointArrayStamped>::SharedPtr global_path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr local_validity_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_validity_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  rclcpp::Publisher<hyu_msgs::msg::WaypointArrayStamped>::SharedPtr selected_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_path_viz_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr selected_validity_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr handoff_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}
