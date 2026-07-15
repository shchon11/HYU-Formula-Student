#ifndef EUFS_MODELS_INCLUDE_EUFS_MODELS_DYNAMIC_BICYCLE_HPP_
#define EUFS_MODELS_INCLUDE_EUFS_MODELS_DYNAMIC_BICYCLE_HPP_

#include <string>
#include "eufs_models/vehicle_model.hpp"

namespace eufs {
namespace models {

class DynamicBicycle : public VehicleModel {
 public:
  explicit DynamicBicycle(const std::string &yaml_file);

  void updateState(State &state, Input &input, const double dt);

  /// @brief Fraction of the vertical load the front axle carries right now.
  ///
  /// Static `w_front` unless the car's config opts into load transfer, in which
  /// case accelerating shifts load rearward and braking forward, by the moment
  /// the longitudinal force makes about the contact patches through the CoG
  /// height. Public because it is a physical property of the car's state, and
  /// because the alternative -- inferring it from tyre forces -- is how you end
  /// up unable to tell a load-transfer bug from a tyre-model bug.
  double getDynamicWeightFront(const State &x, const Input &u);

 private:
  State _f(const State &x, const Input &u, const double Fx, const double FyF, const double FyR);
  State _fKinCorrection(const State &x_in, const State &x_state, const Input &u, const double Fx,
                        const double dt);
  double _getFx(const State &x, const Input &u);
  double _getNormalForce(const State &x);
  double _getFdown(const State &x);
  double _getFdrag(const State &x);
  double _getFy(const double Fz, const double w_front, bool front, double slip_angle);
  double _getDownForceFront(const double Fz, const double w_front);
  double _getDownForceRear(const double Fz, const double w_front);
  double _getDynamicWeightFront(const double Fx) const;
};

}  // namespace models
}  // namespace eufs

#endif  // EUFS_MODELS_INCLUDE_EUFS_MODELS_DYNAMIC_BICYCLE_HPP_
