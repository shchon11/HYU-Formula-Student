#ifndef HYU_FORMULAR_CONTROL_NODE_HPP_
#define HYU_FORMULAR_CONTROL_NODE_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "hyu_tmpc_msgs/msg/tum_mpc_output.hpp"
#include "hyu_tmpc_msgs/msg/tum_trajectory.hpp"
#include "hyu_tmpc_msgs/msg/tum_vehicle_state.hpp"
#include "hyu_formular_control_config.hpp"

#include "mvdc_path_matching.h"
#include "mvdc_mpc.h"

class HyuFormulaControlNode : public rclcpp::Node
{
public:
  HyuFormulaControlNode();
  ~HyuFormulaControlNode() override;

private:
  using TumTrajectory = hyu_tmpc_msgs::msg::TumTrajectory;
  using TumVehicleState = hyu_tmpc_msgs::msg::TumVehicleState;
  using TumMpcOutput = hyu_tmpc_msgs::msg::TumMpcOutput;
  using AppliedCommand = ackermann_msgs::msg::AckermannDriveStamped;

  void ProcessParams();
  void Run();

  void CallbackVehicleState(const TumVehicleState::SharedPtr msg);
  void CallbackPerformanceTrajectory(const TumTrajectory::SharedPtr msg);
  void CallbackEmergencyTrajectory(const TumTrajectory::SharedPtr msg);
  void CallbackAppliedCommand(const AppliedCommand::SharedPtr msg);

  VehicleDynamicState ConvertVehicleState(const TumVehicleState & msg) const;
  Trajectory ConvertTrajectory(const TumTrajectory & msg) const;
  TrajectoryPlanning BuildTrajectoryPlanning(
    const Trajectory & performance_trajectory,
    const Trajectory & emergency_trajectory,
    bool state_ok,
    bool performance_trajectory_ok,
    bool emergency_trajectory_ok) const;
  void UpdateActuatorLimitations();
  bool IsPathMatchingValid(TUMPathMatchingState status) const;
  TumMpcOutput BuildOutputMessage() const;

  HyuFormulaControlConfig cfg_;

  rclcpp::Subscription<TumVehicleState>::SharedPtr vehicle_state_sub_;
  rclcpp::Subscription<TumTrajectory>::SharedPtr performance_trajectory_sub_;
  rclcpp::Subscription<TumTrajectory>::SharedPtr emergency_trajectory_sub_;
  rclcpp::Subscription<AppliedCommand>::SharedPtr applied_command_sub_;
  rclcpp::Publisher<TumMpcOutput>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex data_mutex_;
  VehicleDynamicState latest_vehicle_state_{};
  Trajectory latest_performance_trajectory_{};
  Trajectory latest_emergency_trajectory_{};
  rclcpp::Time last_vehicle_state_time_{};
  rclcpp::Time last_performance_trajectory_time_{};
  rclcpp::Time last_emergency_trajectory_time_{};
  bool has_vehicle_state_{false};
  bool has_performance_trajectory_{false};
  bool has_emergency_trajectory_{false};

  // What the plant actually received (the selector-owned /cmd), converted back
  // to the MPC's request units. Falls back to the last own request until the
  // first applied command arrives.
  VehicleControl applied_vehicle_control_{};
  rclcpp::Time last_applied_command_time_{};
  bool has_applied_command_{false};

  VehicleControl actual_vehicle_control_{};
  ActuatorLimitations actuator_limitations_{};
  // Cycles left in the publish-hold after an MPC reinitialization.
  int settle_cycles_remaining_{0};

  RT_MODEL_mvdc_path_matching_T * path_matching_model_{nullptr};
  RT_MODEL_mvdc_mpc_T * mpc_model_{nullptr};
};

#endif  // HYU_FORMULAR_CONTROL_NODE_HPP_
