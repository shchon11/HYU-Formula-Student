#include "test_utils/scenario_builder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ksae_local/geometry.hpp"

namespace ksae_local::test
{
namespace
{

constexpr double kCurvatureEpsilon = 1.0e-12;

bool validColor(const ConeColorProbability & color) noexcept
{
  const bool finite = isFinite(color.blue) && isFinite(color.yellow) &&
    isFinite(color.orange) && isFinite(color.big_orange) && isFinite(color.unknown);
  const bool nonnegative = color.blue >= 0.0 && color.yellow >= 0.0 &&
    color.orange >= 0.0 && color.big_orange >= 0.0 && color.unknown >= 0.0;
  const double total = color.blue + color.yellow + color.orange +
    color.big_orange + color.unknown;
  return finite && nonnegative && total > 0.0;
}

Pose2D centerlinePoseAtArcLength(const double arc_length_m, const double curvature) noexcept
{
  if (std::abs(curvature) <= kCurvatureEpsilon) {
    return {arc_length_m, 0.0, 0.0};
  }
  const double heading = curvature * arc_length_m;
  return {
    std::sin(heading) / curvature,
    (1.0 - std::cos(heading)) / curvature,
    heading};
}

Point2D offsetFromPose(const Pose2D & pose, const double lateral_offset_m) noexcept
{
  const Point2D left_normal{-std::sin(pose.yaw), std::cos(pose.yaw)};
  return {
    pose.x + lateral_offset_m * left_normal.x,
    pose.y + lateral_offset_m * left_normal.y};
}

void appendVisibleObservation(
  ScenarioFrame & frame, const Point2D & cone_odom, const double observation_range_m,
  const Covariance2D & covariance, const ConeColorProbability & color)
{
  const Point2D sensor_origin_odom{frame.sensor_pose_odom.x, frame.sensor_pose_odom.y};
  if (distance(sensor_origin_odom, cone_odom) > observation_range_m) {
    return;
  }
  frame.observations_sensor.push_back(ConeObservation{
    inverseTransformPoint(cone_odom, frame.sensor_pose_odom),
    covariance,
    color,
    frame.timestamp_s});
}

}  // namespace

ScenarioBuilder::ScenarioBuilder(ScenarioBuilderConfig config)
: config_(std::move(config))
{
}

ScenarioBuilder & ScenarioBuilder::setTrackLength(const double track_length_m)
{
  config_.track_length_m = track_length_m;
  return *this;
}

ScenarioBuilder & ScenarioBuilder::setConeSpacing(const double cone_spacing_m)
{
  config_.cone_spacing_m = cone_spacing_m;
  return *this;
}

ScenarioBuilder & ScenarioBuilder::setTrackWidth(const double track_width_m)
{
  config_.track_width_m = track_width_m;
  return *this;
}

ScenarioBuilder & ScenarioBuilder::setObservationRange(const double observation_range_m)
{
  config_.observation_range_m = observation_range_m;
  return *this;
}

ScenarioBuilder & ScenarioBuilder::setFrameCount(const std::size_t frame_count)
{
  config_.frame_count = frame_count;
  return *this;
}

ScenarioBuilder & ScenarioBuilder::setVehicleStep(const double vehicle_step_m)
{
  config_.vehicle_step_m = vehicle_step_m;
  return *this;
}

ScenarioBuilder & ScenarioBuilder::setFrameTiming(
  const double initial_timestamp_s, const double frame_period_s)
{
  config_.initial_timestamp_s = initial_timestamp_s;
  config_.frame_period_s = frame_period_s;
  return *this;
}

ScenarioBuilder & ScenarioBuilder::setColorProbabilities(
  const ConeColorProbability & left, const ConeColorProbability & right)
{
  config_.left_color = left;
  config_.right_color = right;
  return *this;
}

ScenarioBuilder & ScenarioBuilder::setCovariance(const Covariance2D & covariance_sensor)
{
  config_.observation_covariance_sensor = covariance_sensor;
  return *this;
}

ScenarioBuilder & ScenarioBuilder::setSeed(const std::uint32_t seed) noexcept
{
  config_.seed = seed;
  return *this;
}

SyntheticScenario ScenarioBuilder::buildStraight(const std::string & name) const
{
  return buildConstantCurvature(name, 0.0);
}

SyntheticScenario ScenarioBuilder::buildLeftCurve(
  const double centerline_radius_m, const std::string & name) const
{
  if (!isFinite(centerline_radius_m) || centerline_radius_m <= 0.0) {
    throw std::invalid_argument("left-curve radius must be finite and positive");
  }
  return buildConstantCurvature(name, 1.0 / centerline_radius_m);
}

SyntheticScenario ScenarioBuilder::buildRightCurve(
  const double centerline_radius_m, const std::string & name) const
{
  if (!isFinite(centerline_radius_m) || centerline_radius_m <= 0.0) {
    throw std::invalid_argument("right-curve radius must be finite and positive");
  }
  return buildConstantCurvature(name, -1.0 / centerline_radius_m);
}

const ScenarioBuilderConfig & ScenarioBuilder::config() const noexcept
{
  return config_;
}

SyntheticScenario ScenarioBuilder::buildConstantCurvature(
  const std::string & name, const double signed_curvature_inv_m) const
{
  validate(signed_curvature_inv_m);

  const std::size_t cone_count =
    static_cast<std::size_t>(std::floor(config_.track_length_m / config_.cone_spacing_m)) + 1U;
  std::vector<Point2D> centerline;
  std::vector<Point2D> left_boundary;
  std::vector<Point2D> right_boundary;
  centerline.reserve(cone_count);
  left_boundary.reserve(cone_count);
  right_boundary.reserve(cone_count);

  const double half_width = 0.5 * config_.track_width_m;
  for (std::size_t index = 0U; index < cone_count; ++index) {
    const double arc_length = static_cast<double>(index) * config_.cone_spacing_m;
    const Pose2D center_pose = centerlinePoseAtArcLength(arc_length, signed_curvature_inv_m);
    centerline.push_back({center_pose.x, center_pose.y});
    left_boundary.push_back(offsetFromPose(center_pose, half_width));
    right_boundary.push_back(offsetFromPose(center_pose, -half_width));
  }

  SyntheticScenario scenario;
  scenario.name = name;
  scenario.ground_truth_left_boundary_odom = left_boundary;
  scenario.ground_truth_right_boundary_odom = right_boundary;
  scenario.ground_truth_centerline_odom = centerline;
  scenario.nominal_left_color = config_.left_color;
  scenario.nominal_right_color = config_.right_color;
  scenario.nominal_observation_covariance_sensor = config_.observation_covariance_sensor;
  scenario.seed = config_.seed;
  scenario.frames.reserve(config_.frame_count);

  for (std::size_t frame_index = 0U; frame_index < config_.frame_count; ++frame_index) {
    const double vehicle_arc_length = std::min(
      static_cast<double>(frame_index) * config_.vehicle_step_m,
      config_.track_length_m);
    ScenarioFrame frame;
    frame.sensor_pose_odom =
      centerlinePoseAtArcLength(vehicle_arc_length, signed_curvature_inv_m);
    frame.timestamp_s = config_.initial_timestamp_s +
      static_cast<double>(frame_index) * config_.frame_period_s;

    for (const Point2D & point : left_boundary) {
      appendVisibleObservation(
        frame, point, config_.observation_range_m,
        config_.observation_covariance_sensor, config_.left_color);
    }
    for (const Point2D & point : right_boundary) {
      appendVisibleObservation(
        frame, point, config_.observation_range_m,
        config_.observation_covariance_sensor, config_.right_color);
    }
    scenario.frames.push_back(std::move(frame));
  }

  scenario.expected.ground_truth_left_cone_count = left_boundary.size();
  scenario.expected.ground_truth_right_cone_count = right_boundary.size();
  scenario.expected.total_unique_physical_cone_count =
    left_boundary.size() + right_boundary.size();
  scenario.expected.temporal_accumulation_possible = scenario.frames.size() > 1U;
  scenario.expected.spatial_information_expected_to_increase = scenario.frames.size() > 1U;
  scenario.expected.expected_mode_hint = PlannerMode::BOTH_BOUNDARIES;
  scenario.expected.notes.push_back(
    "Ideal observations contain only range filtering; no planning output is prescribed.");
  refreshObservationInvariants(scenario);
  return scenario;
}

void ScenarioBuilder::validate(const double signed_curvature_inv_m) const
{
  if (!isFinite(config_.track_length_m) || config_.track_length_m <= 0.0 ||
    !isFinite(config_.cone_spacing_m) || config_.cone_spacing_m <= 0.0 ||
    !isFinite(config_.track_width_m) || config_.track_width_m <= 0.0 ||
    !isFinite(config_.observation_range_m) || config_.observation_range_m <= 0.0 ||
    config_.frame_count == 0U ||
    !isFinite(config_.vehicle_step_m) || config_.vehicle_step_m < 0.0 ||
    !isFinite(config_.frame_period_s) || config_.frame_period_s <= 0.0 ||
    !isFinite(config_.initial_timestamp_s) ||
    !isFinite(signed_curvature_inv_m) ||
    !validColor(config_.left_color) || !validColor(config_.right_color) ||
    !isFinite(config_.observation_covariance_sensor))
  {
    throw std::invalid_argument("ScenarioBuilder contains an invalid finite/range setting");
  }

  if (std::abs(signed_curvature_inv_m) > kCurvatureEpsilon) {
    const double radius = 1.0 / std::abs(signed_curvature_inv_m);
    if (radius <= 0.5 * config_.track_width_m) {
      throw std::invalid_argument("curve radius must exceed half of the track width");
    }
  }
}

}  // namespace ksae_local::test
