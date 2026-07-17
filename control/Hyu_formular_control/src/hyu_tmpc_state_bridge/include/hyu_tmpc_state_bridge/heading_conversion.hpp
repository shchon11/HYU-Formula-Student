#ifndef TUM_VEHICLE_STATE_BRIDGE__HEADING_CONVERSION_HPP_
#define TUM_VEHICLE_STATE_BRIDGE__HEADING_CONVERSION_HPP_

namespace hyu_tmpc_state_bridge
{

/// Normalize an angle to the half-open interval [-pi, pi).
double NormalizeAngle(double angle_rad);

/// Convert ROS ENU yaw (zero=east, positive CCW) to Formula heading
/// (zero=north, positive CCW).
double RosYawToFormulaHeading(double ros_yaw_rad);

}  // namespace hyu_tmpc_state_bridge

#endif  // TUM_VEHICLE_STATE_BRIDGE__HEADING_CONVERSION_HPP_
