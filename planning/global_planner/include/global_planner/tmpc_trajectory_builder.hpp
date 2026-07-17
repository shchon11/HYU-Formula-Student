#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "global_planner/path_snapshot.hpp"
#include "hyu_formular_control_msgs/msg/tum_trajectory.hpp"

namespace global_planner::tmpc
{

constexpr std::size_t kTrajectoryPointCount = 50U;

enum class HeadingConvention
{
  // Standard ROS ENU yaw: zero is +x/East and positive is counter-clockwise.
  kRosEastZero,
  // Formula trajectory convention: zero is North. The sign is unchanged.
  kFormulaNorthZero,
};

double normalizeAngle(double angle_rad);
double convertHeading(double heading_rad, HeadingConvention convention);

struct GgvLimits
{
  double ax_mps2{0.0};
  double ay_mps2{0.0};
};

class GgvTable
{
public:
  GgvTable() = default;

  // Accepted CSV layouts are:
  //   v_mps, ax_max_mps2, ay_max_mps2
  //   v_mps, ax_max_mps2, ay_max_mps2, ax_decel_max_mps2
  // For the four-column layout the symmetric longitudinal limit is
  // min(ax_max_mps2, ax_decel_max_mps2).
  static std::optional<GgvTable> loadCsv(
    const std::string & path, std::string * error = nullptr);

  static std::optional<GgvTable> fromVectors(
    const std::vector<double> & speed_mps,
    const std::vector<double> & ax_lim_mps2,
    const std::vector<double> & ay_lim_mps2,
    std::string * error = nullptr);

  static std::optional<GgvTable> fromVectors(
    const std::vector<double> & speed_mps,
    const std::vector<double> & ax_max_mps2,
    const std::vector<double> & ay_lim_mps2,
    const std::vector<double> & ax_decel_max_mps2,
    std::string * error = nullptr);

  [[nodiscard]] bool valid(std::string * error = nullptr) const;
  [[nodiscard]] std::size_t size() const noexcept {return speed_mps_.size();}

  // Speeds outside the table are endpoint-clamped. If supplied, clamped is
  // set true when endpoint clamping was required.
  [[nodiscard]] GgvLimits limitsAt(double speed_mps, bool * clamped = nullptr) const;

private:
  std::vector<double> speed_mps_;
  std::vector<double> ax_lim_mps2_;
  std::vector<double> ay_lim_mps2_;
};

struct BuilderConfig
{
  double foresight_time_sec{3.0};
  double min_horizon_m{10.0};
  double max_horizon_m{30.0};
  double start_behind_m{0.5};

  double banking_rad{0.0};
  double tube_l_m{0.1};
  double tube_r_m{0.1};

  // Cap on the performance-trajectory reference speed (<= 0 disables). The
  // raceline speeds are what the plant can hold under the MAP steering law;
  // the tube MPC additionally fights the plant's 0.2 s command dead time, and
  // past ~7 m/s on tight tracks its corrections land too late (observed:
  // steering saturation -> diverged QP at the first corner after takeover).
  // Keep the TMPC operating envelope inside what it can track until the dead
  // time is fully compensated.
  double performance_speed_cap_mps{0.0};

  double emergency_decel_scale{0.95};
  double emergency_final_speed_mps{0.5};

  double drag_coefficient{1.4};
  double air_density_kgpm3{1.22};
  double vehicle_mass_kg{1300.0};

  double minimum_adjacent_distance_m{1.0e-4};
  double acceleration_utilization_limit{1.025};
  double acceleration_consistency_tolerance_mps2{1.0e-6};
  HeadingConvention heading_convention{HeadingConvention::kRosEastZero};
};

struct BuildInput
{
  const wpnt_publisher::PathSnapshot * path{nullptr};
  double current_s_m{0.0};
  double current_speed_mps{0.0};
  std::uint32_t lap_cnt{0U};
  std::uint32_t traj_cnt{0U};
};

struct BuildResult
{
  bool success{false};
  hyu_formular_control_msgs::msg::TumTrajectory performance;
  hyu_formular_control_msgs::msg::TumTrajectory emergency;
  double horizon_m{0.0};
  bool used_max_horizon{false};
  // True when at least one final performance/emergency sample used an endpoint
  // GGV value because its speed was outside the configured speed axis.
  bool ggv_endpoint_clamped{false};
  std::string error;
};

class TmpcTrajectoryBuilder
{
public:
  TmpcTrajectoryBuilder(BuilderConfig config, GgvTable ggv_table);

  [[nodiscard]] const BuilderConfig & config() const noexcept {return config_;}
  [[nodiscard]] bool valid(std::string * error = nullptr) const;
  [[nodiscard]] BuildResult build(const BuildInput & input) const;

  // Public so the publisher and unit tests can apply exactly the same checks
  // as the final build path. This includes the Formula path-matching L1
  // acceleration-utilization equation.
  [[nodiscard]] bool validateTrajectory(
    const hyu_formular_control_msgs::msg::TumTrajectory & trajectory,
    bool require_emergency_stop,
    std::string * error = nullptr) const;

private:
  BuilderConfig config_;
  GgvTable ggv_table_;
};

}  // namespace global_planner::tmpc
