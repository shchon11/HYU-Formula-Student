#ifndef HYU_FORMULAR_CONTROL_NODE_HPP_
#define HYU_FORMULAR_CONTROL_NODE_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "hyu_formular_control_msgs/msg/tum_mpc_output.hpp"
#include "hyu_formular_control_msgs/msg/tum_trajectory.hpp"
#include "hyu_formular_control_msgs/msg/tum_vehicle_state.hpp"
#include "hyu_formular_control_config.hpp"

#include "mvdc_path_matching.h"
#include "mvdc_mpc.h"

class HyuFormulaControlNode : public rclcpp::Node
{
public:
  HyuFormulaControlNode();
  ~HyuFormulaControlNode() override;

private:
  using TumTrajectory = hyu_formular_control_msgs::msg::TumTrajectory;
  using TumVehicleState = hyu_formular_control_msgs::msg::TumVehicleState;
  using TumMpcOutput = hyu_formular_control_msgs::msg::TumMpcOutput;

  void ProcessParams();
  void Run();

  void CallbackVehicleState(const TumVehicleState::SharedPtr msg);
  void CallbackPerformanceTrajectory(const TumTrajectory::SharedPtr msg);
  void CallbackEmergencyTrajectory(const TumTrajectory::SharedPtr msg);

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

  VehicleControl actual_vehicle_control_{};
  ActuatorLimitations actuator_limitations_{};

  RT_MODEL_mvdc_path_matching_T * path_matching_model_{nullptr};
  RT_MODEL_mvdc_mpc_T * mpc_model_{nullptr};
};

#endif  // HYU_FORMULAR_CONTROL_NODE_HPP_
