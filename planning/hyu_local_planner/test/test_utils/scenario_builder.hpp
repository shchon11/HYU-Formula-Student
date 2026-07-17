#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "test_utils/synthetic_scenario.hpp"

namespace ksae_local::test
{

struct ScenarioBuilderConfig
{
  double track_length_m{30.0};
  double cone_spacing_m{3.0};
  double track_width_m{4.0};
  double observation_range_m{12.0};
  std::size_t frame_count{8U};
  double vehicle_step_m{1.5};
  double frame_period_s{0.1};
  double initial_timestamp_s{100.0};
  ConeColorProbability left_color{1.0, 0.0, 0.0, 0.0, 0.0};
  ConeColorProbability right_color{0.0, 1.0, 0.0, 0.0, 0.0};
  Covariance2D observation_covariance_sensor{0.04, 0.0, 0.0, 0.04};
  std::uint32_t seed{kDefaultScenarioSeed};
};

/// Builds ideal boundaries in odom and sensor-frame observations at each vehicle pose.
class ScenarioBuilder
{
public:
  ScenarioBuilder() = default;
  explicit ScenarioBuilder(ScenarioBuilderConfig config);

  ScenarioBuilder & setTrackLength(double track_length_m);
  ScenarioBuilder & setConeSpacing(double cone_spacing_m);
  ScenarioBuilder & setTrackWidth(double track_width_m);
  ScenarioBuilder & setObservationRange(double observation_range_m);
  ScenarioBuilder & setFrameCount(std::size_t frame_count);
  ScenarioBuilder & setVehicleStep(double vehicle_step_m);
  ScenarioBuilder & setFrameTiming(double initial_timestamp_s, double frame_period_s);
  ScenarioBuilder & setColorProbabilities(
    const ConeColorProbability & left, const ConeColorProbability & right);
  ScenarioBuilder & setCovariance(const Covariance2D & covariance_sensor);
  ScenarioBuilder & setSeed(std::uint32_t seed) noexcept;

  SyntheticScenario buildStraight(const std::string & name = "straight") const;
  SyntheticScenario buildLeftCurve(
    double centerline_radius_m, const std::string & name = "left_curve") const;
  SyntheticScenario buildRightCurve(
    double centerline_radius_m, const std::string & name = "right_curve") const;

  const ScenarioBuilderConfig & config() const noexcept;

private:
  SyntheticScenario buildConstantCurvature(
    const std::string & name, double signed_curvature_inv_m) const;
  void validate(double signed_curvature_inv_m) const;

  ScenarioBuilderConfig config_;
};

}  // namespace ksae_local::test
