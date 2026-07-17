#ifndef PURE_PURSUIT_CONTROLLER__STEERING_LOOKUP_HPP_
#define PURE_PURSUIT_CONTROLLER__STEERING_LOOKUP_HPP_

#include <cstddef>
#include <optional>
#include <vector>

namespace pure_pursuit_controller
{

// Tyre force flavour used when the single-track model is integrated to build the
// steering lookup table. LINEAR is F_y = mu * F_z * C_S * alpha (cornering-
// stiffness coefficient); PACEJKA is the Magic-Formula lateral force used by the
// EUFS DynamicBicycle plant, F_y = mu * F_z * D * sin(C * atan(B*(1-E)*alpha +
// E*atan(B*alpha))). The two forms share the same alpha, so both reproduce the
// steady-state cornering behaviour the MAP controller inverts.
enum class TireModel
{
  LINEAR,
  PACEJKA
};

// Single-track (bicycle) vehicle plus tyre parameters. Distances are measured
// from the centre of mass. The defaults describe the EUFS `eufs` car
// (robots/eufs/configDry.yaml); the golden test overrides them with the
// reference F1TENTH SIM_linear parameters.
struct VehicleModel
{
  double mass_kg{225.0};
  double yaw_inertia_kg_m2{31.27};
  double cg_to_front_m{0.869};   // l_f
  double cg_to_rear_m{0.711};    // l_r
  double cg_height_m{0.30};      // h_cg (load transfer; unused while a_x == 0)
  double gravity_mps2{9.81};
  // Lever arm in the FRONT slip angle alpha_f = -atan((v_y + arm*r)/v_x) + steer.
  // 0 -> use cg_to_front_m (the physically-correct single-track model, matching
  // the CommonRoad reference the golden test pins). The EUFS DynamicBicycle plant
  // (vehicle_model.cpp getSlipAngle) instead uses l_r for BOTH axles, so set this
  // to cg_to_rear_m to make the table invert that plant exactly. The rear slip
  // always uses cg_to_rear_m, which already matches the plant.
  // Deliberately NOT modelled from the plant: the track-width slip correction
  // (denominator v_x - 0.5*axle_width*r) and the max(1, v_x) low-speed floor --
  // both add low-speed fragility for a second-order steady-state effect, and the
  // L1 guidance absorbs the residual bias.
  double front_slip_lever_arm_m{0.0};
  // Aero downforce coefficient: F_z gains c_down * v^2, split front/rear by the
  // static axle fraction. 0.0 reproduces the load-free MAP reference exactly.
  double aero_downforce_coeff{0.0};

  TireModel tire_model{TireModel::PACEJKA};

  // Linear tyre.
  double tire_mu{1.0};
  double cornering_stiffness_front{0.0};   // C_Sf
  double cornering_stiffness_rear{0.0};    // C_Sr

  // Pacejka tyre. mu is kept separate from D so the reference SIM/NUC models
  // (mu * D) and the EUFS plant (mu folded into D, so mu == 1) both fit. Use the
  // magnitude of C: the table stores |a_lat|, so a sign-flipped convention (the
  // EUFS config ships C = -1.38) cannot invert the steering command.
  double pacejka_mu{1.0};
  double pacejka_b_front{0.0};
  double pacejka_c_front{0.0};
  double pacejka_d_front{0.0};
  double pacejka_e_front{0.0};
  double pacejka_b_rear{0.0};
  double pacejka_c_rear{0.0};
  double pacejka_d_rear{0.0};
  double pacejka_e_rear{0.0};
};

// Grid and integration settings for the table build. The defaults match the
// reference `simulate_model.py` (fine steer sweep [0, 0.1], coarse [0.1, 0.4],
// velocities [0.5, 7.0], 2 s of 0.01 s RK4 steps), which is what the golden test
// pins. steer_max_rad is normally raised to the vehicle's steering limit.
struct LutGrid
{
  double steer_fine_end_rad{0.1};
  double steer_max_rad{0.4};
  std::size_t n_steer_fine{30};
  std::size_t n_steer_coarse{30};
  double vel_min_mps{0.5};
  double vel_max_mps{7.0};
  std::size_t n_vel{65};
  double sim_dt_s{0.01};
  double sim_duration_s{2.0};
  // RK4 micro-steps per sim_dt_s recording interval. Explicit RK4 is stiffness-
  // limited, and full-scale tyres (stiff, low-speed) diverge at dt = 0.01 s; the
  // sub-step keeps the effective step (dt / sim_substeps) stable while the
  // convergence check still samples at the reference's 0.01 s cadence.
  std::size_t sim_substeps{10};
};

// Steady-state map steer -> |a_lat| over a velocity grid, and its inverse lookup
// |a_lat|, v -> steer. Built once (the integration is milliseconds) and queried
// every control cycle. An entry is NaN once the tyre saturates (the yaw rate
// fails to settle): higher steer at that velocity is unreachable and clipped.
class SteeringLookup
{
public:
  SteeringLookup() = default;
  SteeringLookup(
    std::vector<double> steers_rad, std::vector<double> velocities_mps,
    std::vector<double> lateral_accel_mps2);

  // A table is usable only with at least one steer row, one velocity column, and
  // one finite (non-saturated) cell.
  bool valid() const;

  // Steering angle (rad) that produces the requested lateral acceleration at the
  // given speed, sign taken from lat_accel. std::nullopt if the table is unusable
  // or the inputs are not finite. The magnitude is clipped to the reachable range
  // at the nearest tabulated velocity (matches the reference saturation clamp).
  std::optional<double> lookup(double lat_accel_mps2, double speed_mps) const;

  const std::vector<double> & steers() const { return steers_; }
  const std::vector<double> & velocities() const { return velocities_; }
  // |a_lat| at (steer row, velocity column); NaN once the tyre has saturated.
  double lateralAccel(std::size_t steer_index, std::size_t velocity_index) const;

private:
  std::vector<double> steers_;        // ascending, rows
  std::vector<double> velocities_;    // ascending, columns
  std::vector<double> lateral_accel_; // row-major [steer][velocity], NaN = saturated
};

// Integrate the single-track model to steady state across the grid and assemble
// the lookup. Returns an invalid table if the model or grid is degenerate.
SteeringLookup buildSteeringLookup(const VehicleModel & model, const LutGrid & grid);

}  // namespace pure_pursuit_controller

#endif  // PURE_PURSUIT_CONTROLLER__STEERING_LOOKUP_HPP_
