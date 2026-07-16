#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "eufs_msgs/msg/waypoint_array_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pure_pursuit_controller/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace pure_pursuit_controller
{

class PurePursuitControllerNode final : public rclcpp::Node
{
public:
  PurePursuitControllerNode()
  : Node("pure_pursuit_controller_node")
  {
    config_.wheelbase_m = declare_parameter("wheelbase_m", config_.wheelbase_m);
    config_.lookahead_m = declare_parameter("lookahead_m", config_.lookahead_m);
    config_.max_steering_rad =
      declare_parameter("max_steering_rad", config_.max_steering_rad);
    config_.command_rate_hz = declare_parameter("command_rate_hz", config_.command_rate_hz);
    config_.input_timeout_sec =
      declare_parameter("input_timeout_sec", config_.input_timeout_sec);
    config_.max_speed_mps = declare_parameter("max_speed_mps", config_.max_speed_mps);
    config_.longitudinal_kp =
      declare_parameter("longitudinal_kp", config_.longitudinal_kp);
    config_.min_acceleration_mps2 =
      declare_parameter("min_acceleration_mps2", config_.min_acceleration_mps2);
    config_.max_acceleration_mps2 =
      declare_parameter("max_acceleration_mps2", config_.max_acceleration_mps2);
    config_.brake_acceleration_mps2 =
      declare_parameter("brake_acceleration_mps2", config_.brake_acceleration_mps2);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    command_publisher_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", qos);
    lookahead_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "/control/lookahead_marker", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    path_subscription_ = create_subscription<eufs_msgs::msg::WaypointArrayStamped>(
      "/path_waypoints", qos,
      std::bind(&PurePursuitControllerNode::onPath, this, std::placeholders::_1));
    validity_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/planning/selected_path_valid", qos,
      std::bind(&PurePursuitControllerNode::onValidity, this, std::placeholders::_1));
    stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/planning/stop_request", qos,
      std::bind(&PurePursuitControllerNode::onStop, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/ego_odom", qos,
      std::bind(&PurePursuitControllerNode::onOdom, this, std::placeholders::_1));
    // rclcpp::create_timer (not create_wall_timer) so the command rate and the
    // input staleness ages below both follow /clock when use_sim_time is set.
    timer_ = rclcpp::create_timer(
      this, get_clock(), commandPeriod(config_),
      std::bind(&PurePursuitControllerNode::onTimer, this));
  }

private:
  static double ageSeconds(const rclcpp::Time & now, const rclcpp::Time & received)
  {
    return (now - received).seconds();
  }

  void onPath(const eufs_msgs::msg::WaypointArrayStamped::SharedPtr message)
  {
    input_.path_received = true;
    input_.path_frame_valid = message->header.frame_id == "map";
    input_.path.clear();
    input_.path.reserve(message->waypoints.size());
    for (const auto & waypoint : message->waypoints) {
      input_.path.push_back(PathPoint{
        waypoint.position.x, waypoint.position.y, waypoint.vx_mps, waypoint.speed});
    }
    path_receive_time_ = get_clock()->now();
  }

  void onValidity(const std_msgs::msg::Bool::SharedPtr message)
  {
    input_.validity_received = true;
    input_.selected_path_valid = message->data;
    validity_receive_time_ = get_clock()->now();
  }

  void onStop(const std_msgs::msg::Bool::SharedPtr message)
  {
    input_.stop_received = true;
    input_.stop_requested = message->data;
    stop_receive_time_ = get_clock()->now();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    input_.odom_received = true;
    input_.odom_frame_valid = hasExpectedOdometryFrameIds(
      message->header.frame_id, message->child_frame_id);
    const auto & position = message->pose.pose.position;
    const auto & orientation = message->pose.pose.orientation;
    const auto yaw = yawFromQuaternion(
      orientation.x, orientation.y, orientation.z, orientation.w);
    if (yaw.has_value()) {
      input_.ego = EgoState{
        position.x, position.y, yaw.value(), message->twist.twist.linear.x};
    } else {
      input_.ego.reset();
    }
    odom_receive_time_ = get_clock()->now();
  }

  void onTimer()
  {
    const auto now = get_clock()->now();
    if (input_.path_received) {
      input_.path_age_sec = ageSeconds(now, path_receive_time_);
    }
    if (input_.validity_received) {
      input_.validity_age_sec = ageSeconds(now, validity_receive_time_);
    }
    if (input_.stop_received) {
      input_.stop_age_sec = ageSeconds(now, stop_receive_time_);
    }
    if (input_.odom_received) {
      input_.odom_age_sec = ageSeconds(now, odom_receive_time_);
    }

    const auto control = computeControl(input_, config_);
    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = now;
    command.header.frame_id = "map";
    command.drive.speed = control.command.speed_mps;
    command.drive.acceleration = control.command.acceleration_mps2;
    command.drive.steering_angle = control.command.steering_angle_rad;
    command_publisher_->publish(command);
    publishLookaheadMarker(now, control.target);
  }

  void publishLookaheadMarker(
    const rclcpp::Time & stamp, const std::optional<TargetPoint> & target)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "map";
    marker.ns = "lookahead";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    // DELETE while braking so RViz never shows a target the controller is not tracking.
    marker.action = target.has_value() ?
      visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    if (target.has_value()) {
      marker.pose.position.x = target->point.x_m;
      marker.pose.position.y = target->point.y_m;
      marker.pose.position.z = 0.15;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.4;
      marker.scale.y = 0.4;
      marker.scale.z = 0.4;
      marker.color.r = 1.0F;
      marker.color.g = 0.1F;
      marker.color.b = 1.0F;
      marker.color.a = 0.9F;
    }
    lookahead_marker_publisher_->publish(marker);
  }

  ControllerConfig config_;
  ControllerInput input_;
  // Assigned from get_clock()->now() before use; ages are only computed once the
  // matching *_received flag is set, so the default clock type is never mixed in.
  rclcpp::Time path_receive_time_;
  rclcpp::Time validity_receive_time_;
  rclcpp::Time stop_receive_time_;
  rclcpp::Time odom_receive_time_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_publisher_;
  rclcpp::Subscription<eufs_msgs::msg::WaypointArrayStamped>::SharedPtr path_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr validity_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pure_pursuit_controller::PurePursuitControllerNode>());
  rclcpp::shutdown();
  return 0;
}
