#include <cmath>
#include <cstddef>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "pure_pursuit_controller/steering_lookup.hpp"

namespace pure_pursuit_controller
{
namespace
{

// The reference F1TENTH SIM_linear parameters (steering_lookup/.../SIM/SIM_linear.txt).
VehicleModel simLinearModel()
{
  VehicleModel model;
  model.mass_kg = 3.47;
  model.yaw_inertia_kg_m2 = 0.04712;
  model.cg_to_front_m = 0.15875;
  model.cg_to_rear_m = 0.17145;
  model.cg_height_m = 0.02;
  model.gravity_mps2 = 9.81;
  model.aero_downforce_coeff = 0.0;
  model.tire_model = TireModel::LINEAR;
  model.tire_mu = 1.0;
  model.cornering_stiffness_front = 4.718;
  model.cornering_stiffness_rear = 5.4562;
  return model;
}

// The grid baked into the reference simulate_model.py that produced the shipped
// SIM_linear_lookup_table.csv.
LutGrid referenceGrid()
{
  LutGrid grid;
  grid.steer_fine_end_rad = 0.1;
  grid.steer_max_rad = 0.4;
  grid.n_steer_fine = 30;
  grid.n_steer_coarse = 30;
  grid.vel_min_mps = 0.5;
  grid.vel_max_mps = 7.0;
  grid.n_vel = 65;
  grid.sim_dt_s = 0.01;
  grid.sim_duration_s = 2.0;
  return grid;
}

// Full-scale grid for the EUFS car: steering to its ±0.52 rad limit and speeds
// into the racing range, so lookups land in the interpolation band.
LutGrid eufsGrid()
{
  LutGrid grid;
  grid.steer_max_rad = 0.52;
  grid.vel_min_mps = 0.5;
  grid.vel_max_mps = 20.0;
  grid.n_vel = 40;
  return grid;
}

// The EUFS `eufs` car (robots/eufs/configDry.yaml), Pacejka C given as magnitude.
VehicleModel eufsModel()
{
  VehicleModel model;
  model.mass_kg = 225.0;
  model.yaw_inertia_kg_m2 = 31.27;
  model.cg_to_front_m = 0.869;
  model.cg_to_rear_m = 0.711;
  model.front_slip_lever_arm_m = 0.711;  // plant getSlipAngle uses l_r for the front
  model.cg_height_m = 0.30;
  model.gravity_mps2 = 9.81;
  model.aero_downforce_coeff = 1.9;
  model.tire_model = TireModel::PACEJKA;
  model.pacejka_mu = 1.0;
  model.pacejka_b_front = model.pacejka_b_rear = 12.56;
  model.pacejka_c_front = model.pacejka_c_rear = 1.38;
  model.pacejka_d_front = model.pacejka_d_rear = 1.60;
  model.pacejka_e_front = model.pacejka_e_rear = -0.58;
  return model;
}

std::vector<std::vector<double>> loadCsv(const std::string & path)
{
  std::vector<std::vector<double>> rows;
  std::ifstream file(path);
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    std::vector<double> row;
    std::stringstream stream(line);
    std::string cell;
    while (std::getline(stream, cell, ',')) {
      row.push_back(std::stod(cell));  // parses "nan" to NaN
    }
    rows.push_back(row);
  }
  return rows;
}

// The single-track integration reproduces the shipped reference table (built with
// scipy odeint) cell-for-cell, including the tyre-saturation NaN boundary.
TEST(SteeringLookup, MatchesReferenceSimLinearTable)
{
  const auto reference = loadCsv(SIM_LINEAR_TABLE_PATH);
  ASSERT_EQ(reference.size(), 61U);      // 1 header row + 60 steers
  ASSERT_EQ(reference.front().size(), 66U);  // 1 header col + 65 velocities

  const SteeringLookup lut = buildSteeringLookup(simLinearModel(), referenceGrid());
  ASSERT_TRUE(lut.valid());
  ASSERT_EQ(lut.steers().size(), 60U);
  ASSERT_EQ(lut.velocities().size(), 65U);

  for (std::size_t si = 0U; si < 60U; ++si) {
    EXPECT_NEAR(lut.steers()[si], reference[si + 1U][0], 1.0e-12);
  }
  for (std::size_t vi = 0U; vi < 65U; ++vi) {
    EXPECT_NEAR(lut.velocities()[vi], reference[0][vi + 1U], 1.0e-12);
  }

  // Interior cells agree to ~1e-9; the last-valid-steer cell of each column sits
  // on the saturation edge, where 2 s of RK4 and scipy's odeint freeze slightly
  // different transients (up to ~0.07 % relative). Accept abs < 1e-4 OR rel < 2e-3.
  std::size_t nan_mismatches = 0U;
  std::size_t value_mismatches = 0U;
  std::size_t tight_cells = 0U;
  for (std::size_t si = 0U; si < 60U; ++si) {
    for (std::size_t vi = 0U; vi < 65U; ++vi) {
      const double ref = reference[si + 1U][vi + 1U];
      const double mine = lut.lateralAccel(si, vi);
      if (std::isnan(ref) != std::isnan(mine)) {
        ++nan_mismatches;
        continue;
      }
      if (std::isnan(ref)) {
        continue;
      }
      const double abs_diff = std::abs(ref - mine);
      const double rel_diff = abs_diff / std::max(std::abs(ref), 1.0e-9);
      if (abs_diff >= 1.0e-4 && rel_diff >= 2.0e-3) {
        ++value_mismatches;
      }
      if (abs_diff < 1.0e-6) {
        ++tight_cells;
      }
    }
  }
  EXPECT_EQ(nan_mismatches, 0U);
  EXPECT_EQ(value_mismatches, 0U);
  // The overwhelming majority of the (non-boundary) table is essentially exact.
  EXPECT_GT(tight_cells, 3600U);
}

TEST(SteeringLookup, LookupIsSignSymmetricAndMonotonic)
{
  const SteeringLookup lut = buildSteeringLookup(eufsModel(), eufsGrid());
  ASSERT_TRUE(lut.valid());

  const double speed = 5.0;
  const auto zero = lut.lookup(0.0, speed);
  ASSERT_TRUE(zero.has_value());
  EXPECT_NEAR(zero.value(), 0.0, 1.0e-9);

  double previous = -1.0;
  for (const double a_lat : {0.5, 1.0, 2.0, 4.0, 6.0}) {
    const auto left = lut.lookup(a_lat, speed);
    const auto right = lut.lookup(-a_lat, speed);
    ASSERT_TRUE(left.has_value());
    ASSERT_TRUE(right.has_value());
    EXPECT_GT(left.value(), 0.0);
    EXPECT_NEAR(left.value(), -right.value(), 1.0e-12);  // symmetric vehicle
    EXPECT_GT(left.value(), previous);                   // more a_lat -> more steer
    previous = left.value();
  }
}

TEST(SteeringLookup, ClampsBeyondSaturationAndRejectsBadInputs)
{
  const LutGrid grid = referenceGrid();
  const SteeringLookup lut = buildSteeringLookup(eufsModel(), grid);
  ASSERT_TRUE(lut.valid());

  // A lateral demand no tyre can meet saturates to the largest tabulated steer,
  // never beyond it.
  const auto saturated = lut.lookup(1.0e3, 5.0);
  ASSERT_TRUE(saturated.has_value());
  EXPECT_GT(saturated.value(), 0.0);
  EXPECT_LE(saturated.value(), grid.steer_max_rad + 1.0e-9);

  EXPECT_FALSE(lut.lookup(std::nan(""), 5.0).has_value());
  EXPECT_FALSE(lut.lookup(1.0, std::nan("")).has_value());

  const SteeringLookup empty;
  EXPECT_FALSE(empty.valid());
  EXPECT_FALSE(empty.lookup(1.0, 5.0).has_value());
}

TEST(SteeringLookup, EufsModelProducesRacingGripEnvelope)
{
  const SteeringLookup lut = buildSteeringLookup(eufsModel(), eufsGrid());
  ASSERT_TRUE(lut.valid());

  // Somewhere in the table a full-scale FS car on grippy tyres should reach well
  // over 1 g of lateral acceleration before the tyre saturates.
  double peak = 0.0;
  for (std::size_t si = 0U; si < lut.steers().size(); ++si) {
    for (std::size_t vi = 0U; vi < lut.velocities().size(); ++vi) {
      const double value = lut.lateralAccel(si, vi);
      if (std::isfinite(value)) {
        peak = std::max(peak, value);
      }
    }
  }
  EXPECT_GT(peak, 9.0);
}

}  // namespace
}  // namespace pure_pursuit_controller
