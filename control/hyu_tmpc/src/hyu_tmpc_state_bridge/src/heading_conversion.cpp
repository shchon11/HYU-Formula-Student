#include "hyu_tmpc_state_bridge/heading_conversion.hpp"

#include <cmath>

namespace hyu_tmpc_state_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
}  // namespace

double NormalizeAngle(double angle_rad)
{
  if (!std::isfinite(angle_rad)) {
    return angle_rad;
  }

  double wrapped_rad = std::fmod(angle_rad + kPi, kTwoPi);
  if (wrapped_rad < 0.0) {
    wrapped_rad += kTwoPi;
  }
  return wrapped_rad - kPi;
}

double RosYawToFormulaHeading(double ros_yaw_rad)
{
  return NormalizeAngle(ros_yaw_rad - 0.5 * kPi);
}

}  // namespace hyu_tmpc_state_bridge
