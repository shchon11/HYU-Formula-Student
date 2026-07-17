#include "hyu_pure_pursuit/steering_lookup.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace hyu_pure_pursuit
{
namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// numpy-style linspace: n == 0 -> empty, n == 1 -> {start}, else start..stop
// inclusive. Used for the steer and velocity axes so the grid matches the
// reference simulate_model.py exactly.
std::vector<double> linspace(double start, double stop, std::size_t n)
{
  std::vector<double> out;
  out.reserve(n);
  if (n == 0U) {
    return out;
  }
  if (n == 1U) {
    out.push_back(start);
    return out;
  }
  const double step = (stop - start) / static_cast<double>(n - 1U);
  for (std::size_t i = 0U; i < n; ++i) {
    out.push_back(start + step * static_cast<double>(i));
  }
  return out;
}

// Lateral tyre force at one axle. LINEAR is the cornering-stiffness-coefficient
// law; PACEJKA is the same Magic Formula the EUFS DynamicBicycle plant uses
// (dynamic_bicycle.cpp _getFy), so the controller's internal model matches the
// simulated tyres.
double tireForce(const VehicleModel & model, bool front, double f_z, double alpha)
{
  if (model.tire_model == TireModel::LINEAR) {
    const double c_s =
      front ? model.cornering_stiffness_front : model.cornering_stiffness_rear;
    return model.tire_mu * f_z * c_s * alpha;
  }
  const double b = front ? model.pacejka_b_front : model.pacejka_b_rear;
  const double c = front ? model.pacejka_c_front : model.pacejka_c_rear;
  const double d = front ? model.pacejka_d_front : model.pacejka_d_rear;
  const double e = front ? model.pacejka_e_front : model.pacejka_e_rear;
  const double mu_y =
    d * std::sin(c * std::atan(b * (1.0 - e) * alpha + e * std::atan(b * alpha)));
  return model.pacejka_mu * f_z * mu_y;
}

struct LateralRate
{
  double v_y;
  double r;
};

// Right-hand side of the reduced single-track lateral dynamics (v_y, yaw rate)
// at constant longitudinal speed v_x with zero longitudinal acceleration -- the
// condition under which the steering table is built. The full model's yaw and
// x/y position states do not feed back into (v_y, r), and v_x is held constant
// because a_x == 0, so integrating these two states is exact.
LateralRate lateralDerivative(
  const VehicleModel & model, double v_x, double steer, double v_y, double r)
{
  const double l_f = model.cg_to_front_m;
  const double l_r = model.cg_to_rear_m;
  const double wheelbase = l_f + l_r;
  // Front slip lever arm: cg_to_front_m unless overridden to match the plant's
  // getSlipAngle (which uses l_r for the front). Rear always uses l_r.
  const double front_arm =
    model.front_slip_lever_arm_m > 0.0 ? model.front_slip_lever_arm_m : l_f;

  const double alpha_f = -std::atan((v_y + r * front_arm) / v_x) + steer;
  const double alpha_r = -std::atan((v_y - r * l_r) / v_x);

  // Static axle load split by the lever arms (l_r / L on the front), plus aero
  // downforce. With a_x == 0 there is no longitudinal load transfer.
  const double f_z = model.mass_kg * model.gravity_mps2 +
    model.aero_downforce_coeff * v_x * v_x;
  const double f_zf = f_z * l_r / wheelbase;
  const double f_zr = f_z * l_f / wheelbase;

  const double f_yf = tireForce(model, true, f_zf, alpha_f);
  const double f_yr = tireForce(model, false, f_zr, alpha_r);

  const double dv_y = (f_yr + f_yf) / model.mass_kg - v_x * r;
  const double dr = (l_f * f_yf - l_r * f_yr) / model.yaw_inertia_kg_m2;
  return LateralRate{dv_y, dr};
}

// Integrate to steady state and return |a_lat| = |r_ss| * v_x, or NaN if the yaw
// rate has not settled (tyre saturated). The magnitude makes the table robust to
// the tyre model's internal sign convention (the EUFS config ships C = -1.38); a
// left/right-symmetric vehicle only needs the |steer| -> |a_lat| relationship.
double steadyLateralAccel(
  const VehicleModel & model, const LutGrid & grid, double steer, double v_x)
{
  const double dt = grid.sim_dt_s;
  const auto n_steps =
    static_cast<std::int64_t>(std::llround(grid.sim_duration_s / dt));
  const std::int64_t substeps = std::max<std::int64_t>(1, static_cast<std::int64_t>(grid.sim_substeps));
  const double micro_dt = dt / static_cast<double>(substeps);
  if (n_steps < 10 || dt <= 0.0 || v_x <= 0.0) {
    return kNaN;
  }

  double v_y = 0.0;
  double r = 0.0;
  double r_early = 0.0;  // yaw rate 9 recording steps before the last (ref sol[-10])

  const auto d = [&](double vy, double rr) {
    return lateralDerivative(model, v_x, steer, vy, rr);
  };

  // One recording interval (dt) is integrated with `substeps` RK4 micro-steps,
  // so the convergence samples below stay on the reference's 0.01 s grid while
  // the integration itself is fine enough to be stable for stiff tyres.
  for (std::int64_t k = 0; k < n_steps; ++k) {
    for (std::int64_t s = 0; s < substeps; ++s) {
      const LateralRate k1 = d(v_y, r);
      const LateralRate k2 = d(v_y + 0.5 * micro_dt * k1.v_y, r + 0.5 * micro_dt * k1.r);
      const LateralRate k3 = d(v_y + 0.5 * micro_dt * k2.v_y, r + 0.5 * micro_dt * k2.r);
      const LateralRate k4 = d(v_y + micro_dt * k3.v_y, r + micro_dt * k3.r);
      v_y += micro_dt / 6.0 * (k1.v_y + 2.0 * k2.v_y + 2.0 * k3.v_y + k4.v_y);
      r += micro_dt / 6.0 * (k1.r + 2.0 * k2.r + 2.0 * k3.r + k4.r);
    }
    if (k == n_steps - 10) {
      r_early = r;
    }
  }
  if (!std::isfinite(r) || std::abs(r - r_early) > 0.05) {
    return kNaN;
  }
  return std::abs(r * v_x);
}

std::size_t nearestIndex(const std::vector<double> & axis, double value)
{
  std::size_t best = 0U;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0U; i < axis.size(); ++i) {
    const double distance = std::abs(axis[i] - value);
    if (distance < best_distance) {
      best_distance = distance;
      best = i;
    }
  }
  return best;
}

}  // namespace

SteeringLookup::SteeringLookup(
  std::vector<double> steers_rad, std::vector<double> velocities_mps,
  std::vector<double> lateral_accel_mps2)
: steers_(std::move(steers_rad)),
  velocities_(std::move(velocities_mps)),
  lateral_accel_(std::move(lateral_accel_mps2))
{
}

bool SteeringLookup::valid() const
{
  if (steers_.empty() || velocities_.empty() ||
    lateral_accel_.size() != steers_.size() * velocities_.size())
  {
    return false;
  }
  for (const double value : lateral_accel_) {
    if (std::isfinite(value)) {
      return true;
    }
  }
  return false;
}

double SteeringLookup::lateralAccel(std::size_t steer_index, std::size_t velocity_index) const
{
  return lateral_accel_[steer_index * velocities_.size() + velocity_index];
}

std::optional<double> SteeringLookup::lookup(double lat_accel_mps2, double speed_mps) const
{
  if (!valid() || !std::isfinite(lat_accel_mps2) || !std::isfinite(speed_mps)) {
    return std::nullopt;
  }

  const double sign = (lat_accel_mps2 >= 0.0) ? 1.0 : -1.0;
  const double target = std::abs(lat_accel_mps2);
  const std::size_t n_vel = velocities_.size();
  const std::size_t vi = nearestIndex(velocities_, speed_mps);

  // Walk the velocity column while it stays finite. |a_lat| rises with steer up
  // to the tyre saturation boundary, beyond which the reference marks cells NaN.
  std::size_t last_valid = 0U;
  bool any_valid = false;
  for (std::size_t si = 0U; si < steers_.size(); ++si) {
    if (std::isnan(lateral_accel_[si * n_vel + vi])) {
      break;
    }
    last_valid = si;
    any_valid = true;
  }
  if (!any_valid) {
    return std::nullopt;
  }

  const double a_low = lateral_accel_[0U * n_vel + vi];
  if (target <= a_low) {
    return sign * steers_[0U];
  }
  const double a_high = lateral_accel_[last_valid * n_vel + vi];
  if (target >= a_high) {
    // Clip to the maximum reachable steering at this speed (tyre saturation).
    return sign * steers_[last_valid];
  }

  for (std::size_t si = 0U; si < last_valid; ++si) {
    const double lo = lateral_accel_[si * n_vel + vi];
    const double hi = lateral_accel_[(si + 1U) * n_vel + vi];
    if (target >= lo && target <= hi) {
      const double fraction = (hi > lo) ? (target - lo) / (hi - lo) : 0.0;
      return sign * (steers_[si] + fraction * (steers_[si + 1U] - steers_[si]));
    }
  }
  return sign * steers_[last_valid];
}

SteeringLookup buildSteeringLookup(const VehicleModel & model, const LutGrid & grid)
{
  std::vector<double> steers = linspace(0.0, grid.steer_fine_end_rad, grid.n_steer_fine);
  const std::vector<double> coarse =
    linspace(grid.steer_fine_end_rad, grid.steer_max_rad, grid.n_steer_coarse);
  // Keep the duplicated point at steer_fine_end (matches the reference concat).
  steers.insert(steers.end(), coarse.begin(), coarse.end());
  const std::vector<double> vels = linspace(grid.vel_min_mps, grid.vel_max_mps, grid.n_vel);

  if (steers.empty() || vels.empty()) {
    return SteeringLookup{};
  }

  std::vector<double> table(steers.size() * vels.size(), kNaN);
  for (std::size_t si = 0U; si < steers.size(); ++si) {
    for (std::size_t vi = 0U; vi < vels.size(); ++vi) {
      const double a_lat = steadyLateralAccel(model, grid, steers[si], vels[vi]);
      if (std::isnan(a_lat)) {
        // Tyre saturated: this and every higher velocity for this steer stay NaN.
        break;
      }
      table[si * vels.size() + vi] = a_lat;
    }
  }

  return SteeringLookup{std::move(steers), std::move(vels), std::move(table)};
}

}  // namespace hyu_pure_pursuit
