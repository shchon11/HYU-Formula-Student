#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "hyu_msgs/msg/waypoint_array_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "hyu_pure_pursuit/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace hyu_pure_pursuit
{
namespace
{

SteeringMode parseSteeringMode(const std::string & value)
{
  return value == "map" ? SteeringMode::MAP : SteeringMode::GEOMETRIC;
}

MapSpeedSource parseMapSpeedSource(const std::string & value)
{
  return value == "measured" ? MapSpeedSource::MEASURED : MapSpeedSource::PLANNED;
}

TireModel parseTireModel(const std::string & value)
{
  return value == "linear" ? TireModel::LINEAR : TireModel::PACEJKA;
}

}  // namespace

class PurePursuitControllerNode final : public rclcpp::Node
{
public:
  PurePursuitControllerNode()
  : Node("hyu_pure_pursuit_node")
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
    validity_hold_sec_ = declare_parameter("validity_hold_sec", 0.6);

    // Lateral steering law and (MAP-only) adaptive lookahead / model+tyre table.
    const std::string steering_mode = declare_parameter<std::string>("steering_mode", "geometric");
    config_.steering_mode = parseSteeringMode(steering_mode);
    config_.map_lookahead_slope_s =
      declare_parameter("map_lookahead_slope_s", config_.map_lookahead_slope_s);
    config_.map_lookahead_intercept_m =
      declare_parameter("map_lookahead_intercept_m", config_.map_lookahead_intercept_m);
    config_.map_lookahead_min_m =
      declare_parameter("map_lookahead_min_m", config_.map_lookahead_min_m);
    config_.map_lookahead_max_m =
      declare_parameter("map_lookahead_max_m", config_.map_lookahead_max_m);
    config_.map_speed_source =
      parseMapSpeedSource(declare_parameter<std::string>("map_speed_source", "planned"));

    if (config_.steering_mode == SteeringMode::MAP) {
      lut_ = buildLookupTable();
      if (lut_.valid()) {
        RCLCPP_INFO(
          get_logger(), "MAP steering: built %zu x %zu steering lookup table.",
          lut_.steers().size(), lut_.velocities().size());
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "MAP steering selected but the lookup table is invalid; the controller will brake.");
      }
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    command_publisher_ =
      create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/vehicle/cmd", qos);
    lookahead_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "/control/lookahead_marker", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    path_subscription_ = create_subscription<hyu_msgs::msg::WaypointArrayStamped>(
      "/planning/path", qos,
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

  void onPath(const hyu_msgs::msg::WaypointArrayStamped::SharedPtr message)
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
    raw_selected_path_valid_ = message->data;
    validity_receive_time_ = get_clock()->now();
    if (message->data) {
      last_validity_true_time_ = validity_receive_time_;
      had_valid_path_ = true;
    }
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
    // Validity hold: the local path validity flickers false for 1-3 planner
    // cycles at the lap-1 map frontier (measured 11% of moving samples at the
    // corners, 36 of 37 hard-brake pulses coincided with such a blip). Hard
    // braking mid-corner on every blip shifts load exactly when the tyres are
    // loaded laterally and threw the car wide. Keep tracking the last valid
    // path through a short blip; a SUSTAINED invalid (longer than the hold)
    // still brakes, and a stop request is untouched (handled upstream of this
    // flag in computeControl).
    input_.selected_path_valid = raw_selected_path_valid_ ||
      (had_valid_path_ &&
      ageSeconds(now, last_validity_true_time_) <= validity_hold_sec_);
    // Holding the validity FLAG is not enough: onPath overwrites input_.path on
    // every message, so an invalid planner cycle (empty or heading-jumped
    // waypoints, seen when a mid-corner heading skew breaks the cone traversal)
    // destroys the very geometry the hold exists to preserve. Pure Pursuit then
    // has no path and steers to zero -- the measured lap-1 drop (steer 0.52 ->
    // 0 the instant validity flickered) that let an already-loaded corner spin
    // the car. Snapshot the last genuinely-valid path and restore it for the
    // duration of the hold so the controller finishes the maneuver on the last
    // good line; a sustained invalid past the hold still falls through to the
    // brake below.
    if (raw_selected_path_valid_ && !input_.path.empty()) {
      last_valid_path_ = input_.path;
    } else if (input_.selected_path_valid && !last_valid_path_.empty()) {
      // Restore the last good line AND the input flags computeControl gates on:
      // the invalid planner cycle that tripped the hold also cleared the frame
      // flag (empty/framid-less path) and, if the selector stops publishing,
      // ages the path out -- either alone still forces the brake the hold is
      // meant to prevent. The snapshot is a real map-frame path, so within the
      // bounded hold treat it as a fresh, framed path and keep tracking it.
      input_.path = last_valid_path_;
      input_.path_frame_valid = true;
      input_.path_age_sec = 0.0;
    }
    if (input_.stop_received) {
      input_.stop_age_sec = ageSeconds(now, stop_receive_time_);
    }
    if (input_.odom_received) {
      input_.odom_age_sec = ageSeconds(now, odom_receive_time_);
    }

    const auto control = computeControl(input_, config_, &lut_);
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

  // Declares the single-track model + tyre + grid parameters and integrates the
  // steady-state steering table. Defaults describe the EUFS `eufs` car
  // (robots/eufs/configDry.yaml) with the Pacejka shape factor C given as its
  // magnitude (the plant config ships C = -1.38 under an inverted slip sign; the
  // table stores |a_lat|, so only the magnitude matters).
  SteeringLookup buildLookupTable()
  {
    VehicleModel model;
    model.mass_kg = declare_parameter("vehicle_mass_kg", model.mass_kg);
    model.yaw_inertia_kg_m2 = declare_parameter("yaw_inertia_kg_m2", model.yaw_inertia_kg_m2);
    model.cg_to_front_m = declare_parameter("cg_to_front_m", model.cg_to_front_m);
    model.cg_to_rear_m = declare_parameter("cg_to_rear_m", model.cg_to_rear_m);
    model.front_slip_lever_arm_m =
      declare_parameter("front_slip_lever_arm_m", model.front_slip_lever_arm_m);
    model.cg_height_m = declare_parameter("cg_height_m", model.cg_height_m);
    model.gravity_mps2 = declare_parameter("gravity_mps2", model.gravity_mps2);
    model.aero_downforce_coeff =
      declare_parameter("aero_downforce_coeff", model.aero_downforce_coeff);
    model.tire_model = parseTireModel(declare_parameter<std::string>("tire_model", "pacejka"));
    model.tire_mu = declare_parameter("tire_mu", model.tire_mu);
    model.cornering_stiffness_front =
      declare_parameter("cornering_stiffness_front", model.cornering_stiffness_front);
    model.cornering_stiffness_rear =
      declare_parameter("cornering_stiffness_rear", model.cornering_stiffness_rear);
    model.pacejka_mu = declare_parameter("pacejka_mu", model.pacejka_mu);
    model.pacejka_b_front = declare_parameter("pacejka_b_front", 12.56);
    model.pacejka_c_front = declare_parameter("pacejka_c_front", 1.38);
    model.pacejka_d_front = declare_parameter("pacejka_d_front", 1.60);
    model.pacejka_e_front = declare_parameter("pacejka_e_front", -0.58);
    model.pacejka_b_rear = declare_parameter("pacejka_b_rear", model.pacejka_b_front);
    model.pacejka_c_rear = declare_parameter("pacejka_c_rear", model.pacejka_c_front);
    model.pacejka_d_rear = declare_parameter("pacejka_d_rear", model.pacejka_d_front);
    model.pacejka_e_rear = declare_parameter("pacejka_e_rear", model.pacejka_e_front);

    LutGrid grid;
    grid.steer_fine_end_rad =
      declare_parameter("lut_steer_fine_end_rad", grid.steer_fine_end_rad);
    grid.steer_max_rad = declare_parameter("lut_steer_max_rad", config_.max_steering_rad);
    grid.n_steer_fine = static_cast<std::size_t>(
      declare_parameter<int>("lut_n_steer_fine", static_cast<int>(grid.n_steer_fine)));
    grid.n_steer_coarse = static_cast<std::size_t>(
      declare_parameter<int>("lut_n_steer_coarse", static_cast<int>(grid.n_steer_coarse)));
    grid.vel_min_mps = declare_parameter("lut_vel_min_mps", grid.vel_min_mps);
    grid.vel_max_mps =
      declare_parameter("lut_vel_max_mps", std::max(config_.max_speed_mps, 5.0));
    grid.n_vel =
      static_cast<std::size_t>(declare_parameter<int>("lut_n_vel", static_cast<int>(grid.n_vel)));
    grid.sim_dt_s = declare_parameter("lut_sim_dt_s", grid.sim_dt_s);
    grid.sim_duration_s = declare_parameter("lut_sim_duration_s", grid.sim_duration_s);

    return buildSteeringLookup(model, grid);
  }

  ControllerConfig config_;
  SteeringLookup lut_;
  ControllerInput input_;
  // Assigned from get_clock()->now() before use; ages are only computed once the
  // matching *_received flag is set, so the default clock type is never mixed in.
  rclcpp::Time path_receive_time_;
  rclcpp::Time validity_receive_time_;
  rclcpp::Time last_validity_true_time_{0, 0, RCL_ROS_TIME};
  bool raw_selected_path_valid_{false};
  bool had_valid_path_{false};
  double validity_hold_sec_{0.6};
  // Last path received while the raw validity was true; restored while the hold
  // above bridges an invalid planner cycle so Pure Pursuit keeps a real line to
  // track instead of the empty/heading-jumped path onPath would otherwise load.
  std::vector<PathPoint> last_valid_path_;
  rclcpp::Time stop_receive_time_;
  rclcpp::Time odom_receive_time_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_publisher_;
  rclcpp::Subscription<hyu_msgs::msg::WaypointArrayStamped>::SharedPtr path_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr validity_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hyu_pure_pursuit::PurePursuitControllerNode>());
  rclcpp::shutdown();
  return 0;
}
