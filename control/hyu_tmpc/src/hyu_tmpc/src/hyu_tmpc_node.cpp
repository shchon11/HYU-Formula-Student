#include "hyu_tmpc_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>

using namespace std::chrono_literals;

namespace
{
constexpr std::size_t kTrajectoryPoints = 50;
constexpr std::size_t kPredictionPoints = 41;
// ~0.5 s at the 100 Hz loop: RTI iterations granted to a freshly reinitialized
// MPC before its output is trusted again.
constexpr int kPostResetSettleCycles = 50;

TUMStateEstimationState ToStateEstimationState(uint16_t value)
{
  switch (value) {
    case SE_NORMAL:
      return SE_NORMAL;
    case SE_BYPASS:
      return SE_BYPASS;
    case SE_ODOM:
      return SE_ODOM;
    case SE_FAIL:
      return SE_FAIL;
    case SE_OFF:
    default:
      return SE_OFF;
  }
}

TUMHealthStatus ToHealthStatus(uint16_t value)
{
  if (value == OK || value == WARNING || value == ERROR) {
    return value;
  }
  return ERROR;
}
}  // namespace

HyuFormulaControlNode::HyuFormulaControlNode()
: Node("hyu_tmpc")
{
  this->declare_parameter("hyu_tmpc/vehicle_state_topic", cfg_.vehicle_state_topic);
  this->declare_parameter(
    "hyu_tmpc/performance_trajectory_topic", cfg_.performance_trajectory_topic);
  this->declare_parameter(
    "hyu_tmpc/emergency_trajectory_topic", cfg_.emergency_trajectory_topic);
  this->declare_parameter("hyu_tmpc/output_topic", cfg_.output_topic);
  this->declare_parameter("hyu_tmpc/loop_rate_hz", cfg_.loop_rate_hz);
  this->declare_parameter("hyu_tmpc/state_timeout_sec", cfg_.state_timeout_sec);
  this->declare_parameter(
    "hyu_tmpc/performance_trajectory_timeout_sec",
    cfg_.performance_trajectory_timeout_sec);
  this->declare_parameter(
    "hyu_tmpc/emergency_trajectory_timeout_sec", cfg_.emergency_trajectory_timeout_sec);
  this->declare_parameter("hyu_tmpc/enable_emergency", cfg_.enable_emergency);
  this->declare_parameter("hyu_tmpc/publish_on_timeout", cfg_.publish_on_timeout);
  this->declare_parameter(
    "hyu_tmpc/steering_angle_min_rad", cfg_.steering_angle_min_rad);
  this->declare_parameter(
    "hyu_tmpc/steering_angle_max_rad", cfg_.steering_angle_max_rad);
  this->declare_parameter("hyu_tmpc/drive_force_min_n", cfg_.drive_force_min_n);
  this->declare_parameter("hyu_tmpc/drive_force_max_n", cfg_.drive_force_max_n);
  this->declare_parameter(
    "hyu_tmpc/applied_command_topic", cfg_.applied_command_topic);
  this->declare_parameter(
    "hyu_tmpc/applied_command_timeout_sec", cfg_.applied_command_timeout_sec);
  this->declare_parameter(
    "hyu_tmpc/applied_command_mass_kg", cfg_.applied_command_mass_kg);
  this->declare_parameter(
    "hyu_tmpc/divergence_reset_factor", cfg_.divergence_reset_factor);

  ProcessParams();
  UpdateActuatorLimitations();

  path_matching_model_ = mvdc_path_matching();
  mvdc_path_matching_initialize(path_matching_model_);
  mpc_model_ = mvdc_mpc();
  mvdc_mpc_initialize(mpc_model_);

  vehicle_state_sub_ = this->create_subscription<TumVehicleState>(
    cfg_.vehicle_state_topic, rclcpp::QoS(10),
    std::bind(&HyuFormulaControlNode::CallbackVehicleState, this, std::placeholders::_1));
  performance_trajectory_sub_ = this->create_subscription<TumTrajectory>(
    cfg_.performance_trajectory_topic, rclcpp::QoS(10),
    std::bind(&HyuFormulaControlNode::CallbackPerformanceTrajectory, this, std::placeholders::_1));
  emergency_trajectory_sub_ = this->create_subscription<TumTrajectory>(
    cfg_.emergency_trajectory_topic, rclcpp::QoS(10),
    std::bind(&HyuFormulaControlNode::CallbackEmergencyTrajectory, this, std::placeholders::_1));
  applied_command_sub_ = this->create_subscription<AppliedCommand>(
    cfg_.applied_command_topic, rclcpp::QoS(10),
    std::bind(&HyuFormulaControlNode::CallbackAppliedCommand, this, std::placeholders::_1));
  output_pub_ = this->create_publisher<TumMpcOutput>(cfg_.output_topic, rclcpp::QoS(10));

  const auto period_ms = static_cast<int64_t>(1000.0 / std::max(cfg_.loop_rate_hz, 1.0));
  // Node-clock timer: the generated models integrate with a fixed tS = 0.01 s
  // per step, so under use_sim_time the loop must tick in sim time or any
  // RTF != 1 distorts every discrete filter and the MPC prediction.
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(period_ms),
    std::bind(&HyuFormulaControlNode::Run, this));

  RCLCPP_INFO(this->get_logger(), "HYU Formula control started");
  RCLCPP_INFO(this->get_logger(), "vehicle_state_topic: %s", cfg_.vehicle_state_topic.c_str());
  RCLCPP_INFO(
    this->get_logger(), "performance_trajectory_topic: %s",
    cfg_.performance_trajectory_topic.c_str());
  RCLCPP_INFO(
    this->get_logger(), "emergency_trajectory_topic: %s",
    cfg_.emergency_trajectory_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "output_topic: %s", cfg_.output_topic.c_str());
}

HyuFormulaControlNode::~HyuFormulaControlNode()
{
  if (mpc_model_ != nullptr) {
    mvdc_mpc_terminate(mpc_model_);
    mpc_model_ = nullptr;
  }
  if (path_matching_model_ != nullptr) {
    mvdc_path_matching_terminate(path_matching_model_);
    path_matching_model_ = nullptr;
  }
}

void HyuFormulaControlNode::ProcessParams()
{
  this->get_parameter("hyu_tmpc/vehicle_state_topic", cfg_.vehicle_state_topic);
  this->get_parameter(
    "hyu_tmpc/performance_trajectory_topic", cfg_.performance_trajectory_topic);
  this->get_parameter(
    "hyu_tmpc/emergency_trajectory_topic", cfg_.emergency_trajectory_topic);
  this->get_parameter("hyu_tmpc/output_topic", cfg_.output_topic);
  this->get_parameter("hyu_tmpc/loop_rate_hz", cfg_.loop_rate_hz);
  this->get_parameter("hyu_tmpc/state_timeout_sec", cfg_.state_timeout_sec);
  this->get_parameter(
    "hyu_tmpc/performance_trajectory_timeout_sec",
    cfg_.performance_trajectory_timeout_sec);
  this->get_parameter(
    "hyu_tmpc/emergency_trajectory_timeout_sec", cfg_.emergency_trajectory_timeout_sec);
  this->get_parameter("hyu_tmpc/enable_emergency", cfg_.enable_emergency);
  this->get_parameter("hyu_tmpc/publish_on_timeout", cfg_.publish_on_timeout);
  this->get_parameter(
    "hyu_tmpc/steering_angle_min_rad", cfg_.steering_angle_min_rad);
  this->get_parameter(
    "hyu_tmpc/steering_angle_max_rad", cfg_.steering_angle_max_rad);
  this->get_parameter("hyu_tmpc/drive_force_min_n", cfg_.drive_force_min_n);
  this->get_parameter("hyu_tmpc/drive_force_max_n", cfg_.drive_force_max_n);
  this->get_parameter(
    "hyu_tmpc/applied_command_topic", cfg_.applied_command_topic);
  this->get_parameter(
    "hyu_tmpc/applied_command_timeout_sec", cfg_.applied_command_timeout_sec);
  this->get_parameter(
    "hyu_tmpc/applied_command_mass_kg", cfg_.applied_command_mass_kg);
  this->get_parameter(
    "hyu_tmpc/divergence_reset_factor", cfg_.divergence_reset_factor);
}

void HyuFormulaControlNode::CallbackVehicleState(const TumVehicleState::SharedPtr msg)
{
  const auto converted = ConvertVehicleState(*msg);
  const auto stamp = this->now();
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_vehicle_state_ = converted;
  last_vehicle_state_time_ = stamp;
  has_vehicle_state_ = true;
}

void HyuFormulaControlNode::CallbackPerformanceTrajectory(const TumTrajectory::SharedPtr msg)
{
  const auto converted = ConvertTrajectory(*msg);
  const auto stamp = this->now();
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_performance_trajectory_ = converted;
  last_performance_trajectory_time_ = stamp;
  has_performance_trajectory_ = true;
}

void HyuFormulaControlNode::CallbackEmergencyTrajectory(const TumTrajectory::SharedPtr msg)
{
  const auto converted = ConvertTrajectory(*msg);
  const auto stamp = this->now();
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_emergency_trajectory_ = converted;
  last_emergency_trajectory_time_ = stamp;
  has_emergency_trajectory_ = true;
}

void HyuFormulaControlNode::CallbackAppliedCommand(const AppliedCommand::SharedPtr msg)
{
  if (!std::isfinite(msg->drive.steering_angle) || !std::isfinite(msg->drive.acceleration)) {
    return;
  }
  VehicleControl applied{};
  applied.RequestSteeringAngle_rad = msg->drive.steering_angle;
  applied.RequestLongForce_N = msg->drive.acceleration * cfg_.applied_command_mass_kg;
  const auto stamp = this->now();
  std::lock_guard<std::mutex> lock(data_mutex_);
  applied_vehicle_control_ = applied;
  last_applied_command_time_ = stamp;
  has_applied_command_ = true;
}

VehicleDynamicState HyuFormulaControlNode::ConvertVehicleState(const TumVehicleState & msg) const
{
  VehicleDynamicState state{};
  state.SE_Status = ToHealthStatus(msg.se_status);
  state.SE_State = ToStateEstimationState(msg.se_state);
  state.Pos.x_m = msg.x_m;
  state.Pos.y_m = msg.y_m;
  state.Pos.psi_rad = msg.psi_rad;
  state.dPsi_radps = msg.dpsi_radps;
  state.vx_mps = msg.vx_mps;
  state.vy_mps = msg.vy_mps;
  state.v_mps = msg.v_mps;
  state.beta_rad = msg.beta_rad;
  state.ax_mps2 = msg.ax_mps2;
  state.ay_mps2 = msg.ay_mps2;
  state.valid_IMU_b = msg.valid_imu;
  state.ax_vel_mps2 = msg.ax_vel_mps2;
  state.DeltaWheel_rad = msg.delta_wheel_rad;
  return state;
}

Trajectory HyuFormulaControlNode::ConvertTrajectory(const TumTrajectory & msg) const
{
  Trajectory trajectory{};
  trajectory.LapCnt = msg.lap_cnt;
  trajectory.TrajCnt = msg.traj_cnt;

  for (std::size_t i = 0; i < kTrajectoryPoints; ++i) {
    trajectory.s_loc_m[i] = msg.s_loc_m[i];
    trajectory.s_glob_m[i] = msg.s_glob_m[i];
    trajectory.x_m[i] = msg.x_m[i];
    trajectory.y_m[i] = msg.y_m[i];
    trajectory.psi_rad[i] = msg.psi_rad[i];
    trajectory.kappa_radpm[i] = msg.kappa_radpm[i];
    trajectory.v_mps[i] = msg.v_mps[i];
    trajectory.ax_mps2[i] = msg.ax_mps2[i];
    trajectory.banking_rad[i] = msg.banking_rad[i];
    trajectory.ax_lim_mps2[i] = msg.ax_lim_mps2[i];
    trajectory.ay_lim_mps2[i] = msg.ay_lim_mps2[i];
    trajectory.tube_l_m[i] = msg.tube_l_m[i];
    trajectory.tube_r_m[i] = msg.tube_r_m[i];
  }

  return trajectory;
}

TrajectoryPlanning HyuFormulaControlNode::BuildTrajectoryPlanning(
  const Trajectory & performance_trajectory,
  const Trajectory & emergency_trajectory,
  bool state_ok,
  bool performance_trajectory_ok,
  bool emergency_trajectory_ok) const
{
  TrajectoryPlanning planning{};
  planning.Strategy_Status = cfg_.enable_emergency ? S_DRIVING_EMERGENCY : S_DRIVING_RACE;
  planning.Strategy_CommsStatus = state_ok ? OK : ERROR;
  planning.Trajectories_CommsStatus =
    (performance_trajectory_ok && emergency_trajectory_ok) ? OK : ERROR;
  planning.PerformanceTrajectory = performance_trajectory;
  planning.EmergencyTrajectory = emergency_trajectory;
  return planning;
}

void HyuFormulaControlNode::UpdateActuatorLimitations()
{
  actuator_limitations_.SteeringAngleMin_rad = cfg_.steering_angle_min_rad;
  actuator_limitations_.SteeringAngleMax_rad = cfg_.steering_angle_max_rad;
  actuator_limitations_.DriveForceMin_N = cfg_.drive_force_min_n;
  actuator_limitations_.DriveForceMax_N = cfg_.drive_force_max_n;
}

bool HyuFormulaControlNode::IsPathMatchingValid(TUMPathMatchingState status) const
{
  // PM_VALID_VERYHIGHDEVIATION is deliberately NOT drivable: it means the car
  // sits far outside the reference tube (seen at the GLOBAL handoff, where the
  // ego is still on the lap-1 centerline while the reference is the raceline).
  // The tube MPC solved from there diverges -- observed -2.89 rad steering on a
  // 0.52 rad actuator -- and publishing it parked the car at full lock.
  // Withholding the output makes the bridge report invalid, so the selector
  // keeps Pure Pursuit driving the raceline until the deviation shrinks and
  // TMPC can take over from a feasible state.
  return status == PM_VALID || status == PM_VALID_HIGHDEVIATION;
}

void HyuFormulaControlNode::Run()
{
  ProcessParams();
  UpdateActuatorLimitations();

  if (path_matching_model_ == nullptr || mpc_model_ == nullptr) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Generated model instance is null");
    return;
  }

  VehicleDynamicState vehicle_state{};
  Trajectory performance_trajectory{};
  Trajectory emergency_trajectory{};
  VehicleControl applied_vehicle_control{};
  rclcpp::Time last_vehicle_state_time;
  rclcpp::Time last_performance_trajectory_time;
  rclcpp::Time last_emergency_trajectory_time;
  rclcpp::Time last_applied_command_time;
  bool has_vehicle_state = false;
  bool has_performance_trajectory = false;
  bool has_emergency_trajectory = false;
  bool has_applied_command = false;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    vehicle_state = latest_vehicle_state_;
    performance_trajectory = latest_performance_trajectory_;
    emergency_trajectory = latest_emergency_trajectory_;
    applied_vehicle_control = applied_vehicle_control_;
    last_vehicle_state_time = last_vehicle_state_time_;
    last_performance_trajectory_time = last_performance_trajectory_time_;
    last_emergency_trajectory_time = last_emergency_trajectory_time_;
    last_applied_command_time = last_applied_command_time_;
    has_vehicle_state = has_vehicle_state_;
    has_performance_trajectory = has_performance_trajectory_;
    has_emergency_trajectory = has_emergency_trajectory_;
    has_applied_command = has_applied_command_;
  }

  const auto now = this->now();
  const bool state_ok = has_vehicle_state &&
    ((now - last_vehicle_state_time).seconds() <= cfg_.state_timeout_sec);
  const bool performance_trajectory_ok = has_performance_trajectory &&
    ((now - last_performance_trajectory_time).seconds() <=
    cfg_.performance_trajectory_timeout_sec);
  const bool emergency_trajectory_ok = has_emergency_trajectory &&
    ((now - last_emergency_trajectory_time).seconds() <= cfg_.emergency_trajectory_timeout_sec);

  if ((!state_ok || !performance_trajectory_ok || !emergency_trajectory_ok) &&
    !cfg_.publish_on_timeout)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "TMPC input timeout or missing input: state=%d performance=%d emergency=%d",
      state_ok, performance_trajectory_ok, emergency_trajectory_ok);
    return;
  }

  const auto planning = BuildTrajectoryPlanning(
    performance_trajectory, emergency_trajectory, state_ok, performance_trajectory_ok,
    emergency_trajectory_ok);

  path_matching_model_->inputs->VehicleDynamicState_g = vehicle_state;
  path_matching_model_->inputs->TrajectoryPlanning_m = planning;
  path_matching_model_->inputs->EnablePathMatching = state_ok && performance_trajectory_ok;
  path_matching_model_->inputs->EnableEmergency = cfg_.enable_emergency;
  mvdc_path_matching_step(path_matching_model_);

  const bool path_matching_valid = IsPathMatchingValid(path_matching_model_->outputs->Status);
  const bool enable_driving_controller =
    state_ok && performance_trajectory_ok && emergency_trajectory_ok && path_matching_valid;

  // Feed back what the plant actually received (selector-owned /vehicle/cmd), not this
  // node's own last request: during Pure Pursuit phases the MPC's requests are
  // never applied, and pretending they were poisons its internal learning --
  // the first post-takeover commands then saturate in the wrong direction.
  const bool applied_fresh = has_applied_command &&
    ((now - last_applied_command_time).seconds() <= cfg_.applied_command_timeout_sec);
  mpc_model_->inputs->EnableEmergency = cfg_.enable_emergency;
  mpc_model_->inputs->TargetTrajectory = path_matching_model_->outputs->ActualTargetTrajectory;
  mpc_model_->inputs->VehicleDynamicState_p = vehicle_state;
  mpc_model_->inputs->EnableDrivingController = enable_driving_controller;
  mpc_model_->inputs->ActualTrajectoryPoint = path_matching_model_->outputs->ActualTrajectoryPoint;
  mpc_model_->inputs->ActualVehicleControl =
    applied_fresh ? applied_vehicle_control : actual_vehicle_control_;
  mpc_model_->inputs->ActuatorLimitations_e = actuator_limitations_;
  mvdc_mpc_step(mpc_model_);

  // Diverged-solution recovery: one RTI iteration per cycle cannot heal a
  // diverged iterate, and the poisoned solution resurfaces at the next
  // takeover as a saturated wrong-direction command. Reinitialize (initialize
  // only -- terminate would free the model instance and permanently kill the
  // chain) and hold publishing for a settle window so the fresh iterate
  // re-converges before the bridge sees it. The bound sits ABOVE the model's
  // own internal steering saturation (~2x the road-wheel limit, observed
  // riding 1.04 rad on a 0.52 rad actuator): saturation is drivable after the
  // bridge clamp, only a truly runaway solution (observed -2.89 rad) resets.
  const double divergence_bound_rad = cfg_.divergence_reset_factor *
    std::max(std::abs(cfg_.steering_angle_min_rad), std::abs(cfg_.steering_angle_max_rad));
  const double raw_steering_rad = mpc_model_->outputs->RequestSteeringAngle_rad;
  if (cfg_.divergence_reset_factor > 0.0 &&
    (!std::isfinite(raw_steering_rad) || std::abs(raw_steering_rad) > divergence_bound_rad))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "TMPC solution diverged (steering %.2f rad, bound %.2f); reinitializing the MPC model",
      raw_steering_rad, divergence_bound_rad);
    mvdc_mpc_initialize(mpc_model_);
    actual_vehicle_control_ = VehicleControl{};
    settle_cycles_remaining_ = kPostResetSettleCycles;
    return;
  }
  if (settle_cycles_remaining_ > 0) {
    --settle_cycles_remaining_;
    return;
  }

  if (!enable_driving_controller && !cfg_.publish_on_timeout) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Path matching invalid or driving disabled: status=%d", path_matching_model_->outputs->Status);
    return;
  }

  actual_vehicle_control_.RequestSteeringAngle_rad = mpc_model_->outputs->RequestSteeringAngle_rad;
  actual_vehicle_control_.RequestLongForce_N = mpc_model_->outputs->RequestLongForce_N;

  output_pub_->publish(BuildOutputMessage());
}

HyuFormulaControlNode::TumMpcOutput HyuFormulaControlNode::BuildOutputMessage() const
{
  TumMpcOutput msg;
  msg.request_steering_angle_rad = mpc_model_->outputs->RequestSteeringAngle_rad;
  msg.request_long_force_n = mpc_model_->outputs->RequestLongForce_N;
  msg.tube_mpc_status = static_cast<uint16_t>(mpc_model_->outputs->TubeMPCStatus);
  msg.long_acc_target_mps2 =
    mpc_model_->outputs->mvdc_tmpc_fast_debug_d.LongAcc_Target_mps2;
  msg.lat_acc_target_mps2 = mpc_model_->outputs->mvdc_tmpc_fast_debug_d.LatAcc_Target_mps2;

  for (std::size_t i = 0; i < kPredictionPoints; ++i) {
    msg.ax_pred_mps2[i] = mpc_model_->outputs->mvdc_tube_mpc_debug_c.ax_pred_mps2[i];
    msg.ay_pred_mps2[i] = mpc_model_->outputs->mvdc_tube_mpc_debug_c.ay_pred_mps2[i];
  }

  return msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HyuFormulaControlNode>());
  rclcpp::shutdown();
  return 0;
}
