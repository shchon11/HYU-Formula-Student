#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "eufs_msgs/msg/waypoint_array_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pure_pursuit_controller/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

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

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    command_publisher_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", qos);
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
    timer_ = create_wall_timer(
      commandPeriod(config_), std::bind(&PurePursuitControllerNode::onTimer, this));
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  static double ageSeconds(
    const SteadyClock::time_point & now, const SteadyClock::time_point & received)
  {
    return std::chrono::duration<double>(now - received).count();
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
    path_receive_time_ = SteadyClock::now();
  }

  void onValidity(const std_msgs::msg::Bool::SharedPtr message)
  {
    input_.validity_received = true;
    input_.selected_path_valid = message->data;
    validity_receive_time_ = SteadyClock::now();
  }

  void onStop(const std_msgs::msg::Bool::SharedPtr message)
  {
    input_.stop_received = true;
    input_.stop_requested = message->data;
    stop_receive_time_ = SteadyClock::now();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    input_.odom_received = true;
    input_.odom_frame_valid = message->header.frame_id == "map";
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
    odom_receive_time_ = SteadyClock::now();
  }

  void onTimer()
  {
    const auto now = SteadyClock::now();
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

    const auto control = computeCommand(input_, config_);
    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = get_clock()->now();
    command.header.frame_id = "map";
    command.drive.speed = control.speed_mps;
    command.drive.acceleration = control.acceleration_mps2;
    command.drive.steering_angle = control.steering_angle_rad;
    command_publisher_->publish(command);
  }

  ControllerConfig config_;
  ControllerInput input_;
  SteadyClock::time_point path_receive_time_{};
  SteadyClock::time_point validity_receive_time_{};
  SteadyClock::time_point stop_receive_time_{};
  SteadyClock::time_point odom_receive_time_{};
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_publisher_;
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
