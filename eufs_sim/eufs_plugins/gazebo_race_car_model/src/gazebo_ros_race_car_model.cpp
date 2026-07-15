/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Main Include
#include "gazebo_race_car_model/gazebo_ros_race_car.hpp"

// STD Include
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <mutex>  // NOLINT(build/c++11)
#include <string>
#include <thread>  // NOLINT(build/c++11)

// ROS Include
#include <ament_index_cpp/get_package_share_directory.hpp>


namespace gazebo_plugins {
namespace eufs_plugins {

RaceCarModelPlugin::RaceCarModelPlugin()
: _wheel_joint_positions{0.0, 0.0, 0.0, 0.0},
  _wheel_joint_velocities{0.0, 0.0, 0.0, 0.0}
{}

RaceCarModelPlugin::~RaceCarModelPlugin() { _update_connection.reset(); }

void RaceCarModelPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) {
  _rosnode = gazebo_ros::Node::Get(sdf);

  RCLCPP_DEBUG(_rosnode->get_logger(), "Loading RaceCarModelPlugin");

  _model = model;
  _world = _model->GetWorld();

  _tf_br = std::make_unique<tf2_ros::TransformBroadcaster>(_rosnode);
  _state_machine = std::make_unique<StateMachine>(_rosnode);

  // Initialize parameters
  initParams(sdf);

  // Initialize vehicle model
  initVehicleModel(sdf);

  // Initialize handles to Gazebo vehicle components
  initModel(sdf);

  // Initialize noise object
  initNoise(sdf);

  // Initialize continuous road-roughness vibration
  initRoadNoise(sdf);

  // Initialize acceleration-driven load transfer (roll/pitch)
  initLoadTransfer(sdf);

  // Initialize the procedural bump field and insert its geometry into the world
  initTerrain(sdf);

  // ROS Publishers
  _pub_ground_truth_car_state =
      _rosnode->create_publisher<eufs_msgs::msg::CarState>(_ground_truth_car_state_topic, 1);
  _pub_localisation_car_state =
      _rosnode->create_publisher<eufs_msgs::msg::CarState>(_localisation_car_state_topic, 1);
  _pub_wheel_speeds =
      _rosnode->create_publisher<eufs_msgs::msg::WheelSpeedsStamped>(_wheel_speeds_topic_name, 1);
  _pub_ground_truth_wheel_speeds = _rosnode->create_publisher<eufs_msgs::msg::WheelSpeedsStamped>(
      _ground_truth_wheel_speeds_topic_name, 1);
  _pub_joint_states =
      _rosnode->create_publisher<sensor_msgs::msg::JointState>(_joint_states_topic_name, 10);
  _pub_odom = _rosnode->create_publisher<nav_msgs::msg::Odometry>(_odom_topic_name, 1);

  // ROS Services
  _reset_vehicle_pos_srv = _rosnode->create_service<std_srvs::srv::Trigger>(
      "/ros_can/reset_vehicle_pos", std::bind(&RaceCarModelPlugin::resetVehiclePosition, this,
                                              std::placeholders::_1, std::placeholders::_2));
  _command_mode_srv = _rosnode->create_service<std_srvs::srv::Trigger>(
      "/race_car_model/command_mode", std::bind(&RaceCarModelPlugin::returnCommandMode, this,
                                                std::placeholders::_1, std::placeholders::_2));

  // ROS Subscriptions
  _sub_cmd = _rosnode->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/cmd", 1, std::bind(&RaceCarModelPlugin::onCmd, this, std::placeholders::_1));

  // Connect to Gazebo
  _update_connection =
      gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RaceCarModelPlugin::update, this));
  _last_sim_time = _world->SimTime();

  _max_steering_rate =
      (_vehicle->getParam().input_ranges.delta.max - _vehicle->getParam().input_ranges.delta.min) /
      _steering_lock_time;

  // Set offset
  setPositionFromWorld();

  RCLCPP_INFO(_rosnode->get_logger(), "RaceCarModelPlugin Loaded");
}

void RaceCarModelPlugin::initParams(const sdf::ElementPtr &sdf) {
  if (!sdf->HasElement("update_rate")) {
    _update_rate = 1000.0;
  } else {
    _update_rate = sdf->GetElement("update_rate")->Get<double>();
  }

  if (!sdf->HasElement("publish_rate")) {
    _publish_rate = 200.0;
  } else {
    _publish_rate = sdf->GetElement("publish_rate")->Get<double>();
  }

  if (!sdf->HasElement("referenceFrame")) {
    RCLCPP_DEBUG(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <referenceFrame>, defaults to map");
    _reference_frame = "map";
  } else {
    _reference_frame = sdf->GetElement("referenceFrame")->Get<std::string>();
  }

  if (!sdf->HasElement("robotFrame")) {
    RCLCPP_DEBUG(
        _rosnode->get_logger(),
        "gazebo_ros_race_car_model plugin missing <robotFrame>, defaults to base_footprint");
    _robot_frame = "base_footprint";
  } else {
    _robot_frame = sdf->GetElement("robotFrame")->Get<std::string>();
  }

  if (!sdf->HasElement("publishTransform")) {
    RCLCPP_DEBUG(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <publishTransform>, defaults to false");
    _publish_tf = false;
  } else {
    _publish_tf = sdf->GetElement("publishTransform")->Get<bool>();
  }

  // Drift odometry parameters for the localisation car state.
  _drift_odometry =
      sdf->HasElement("driftOdometry") && sdf->GetElement("driftOdometry")->Get<bool>();
  _drift_v_bias =
      sdf->HasElement("driftVelocityBias") ? sdf->GetElement("driftVelocityBias")->Get<double>()
                                           : 0.02;
  _drift_w_bias =
      sdf->HasElement("driftYawRateBias") ? sdf->GetElement("driftYawRateBias")->Get<double>()
                                          : 0.01;
  _drift_sigma_v =
      sdf->HasElement("driftVelocityNoise") ? sdf->GetElement("driftVelocityNoise")->Get<double>()
                                            : 0.05;
  _drift_sigma_w =
      sdf->HasElement("driftYawRateNoise") ? sdf->GetElement("driftYawRateNoise")->Get<double>()
                                           : 0.01;
  _drift_rng.seed(
      sdf->HasElement("driftSeed") ? sdf->GetElement("driftSeed")->Get<unsigned int>() : 42u);
  _drift_initialized = false;
  if (_drift_odometry) {
    RCLCPP_INFO(_rosnode->get_logger(),
                "Localisation car state uses drifting odometry "
                "(v_bias=%.3f w_bias=%.3f sigma_v=%.3f sigma_w=%.3f)",
                _drift_v_bias, _drift_w_bias, _drift_sigma_v, _drift_sigma_w);
  }

  if (!sdf->HasElement("wheelSpeedsTopicName")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <wheelSpeedsTopicName>, cannot proceed");
    return;
  } else {
    _wheel_speeds_topic_name = sdf->GetElement("wheelSpeedsTopicName")->Get<std::string>();
  }

  if (!sdf->HasElement("groundTruthWheelSpeedsTopicName")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <groundTruthWheelSpeedsTopicName>, "
                 "cannot proceed");
    return;
  } else {
    _ground_truth_wheel_speeds_topic_name =
        sdf->GetElement("groundTruthWheelSpeedsTopicName")->Get<std::string>();
  }

  if (!sdf->HasElement("jointStatesTopicName")) {
    _joint_states_topic_name = "/eufs/joint_states";
  } else {
    _joint_states_topic_name = sdf->GetElement("jointStatesTopicName")->Get<std::string>();
  }

  if (!sdf->HasElement("groundTruthCarStateTopic")) {
    RCLCPP_FATAL(
        _rosnode->get_logger(),
        "gazebo_ros_race_car_model plugin missing <groundTruthCarStateTopic>, cannot proceed");
    return;
  } else {
    _ground_truth_car_state_topic = sdf->GetElement("groundTruthCarStateTopic")->Get<std::string>();
  }

  if (!sdf->HasElement("localisationCarStateTopic")) {
    RCLCPP_FATAL(
        _rosnode->get_logger(),
        "gazebo_ros_race_car_model plugin missing <localisationCarStateTopic>, cannot proceed");
    return;
  } else {
    _localisation_car_state_topic =
        sdf->GetElement("localisationCarStateTopic")->Get<std::string>();
  }

  if (!sdf->HasElement("odometryTopicName")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <odometryTopicName>, cannot proceed");
    return;
  } else {
    _odom_topic_name = sdf->GetElement("odometryTopicName")->Get<std::string>();
  }

  if (!sdf->HasElement("commandMode")) {
    RCLCPP_DEBUG(
        _rosnode->get_logger(),
        "gazebo_ros_race_car_model plugin missing <commandMode>, defaults to acceleration");
    _command_mode = acceleration;
  } else {
    auto temp = sdf->GetElement("commandMode")->Get<std::string>();
    if (temp.compare("acceleration") == 0) {
      _command_mode = acceleration;
    } else if (temp.compare("velocity") == 0) {
      _command_mode = velocity;
    } else {
      RCLCPP_WARN(_rosnode->get_logger(),
                  "commandMode parameter string is invalid, defaults to acceleration");
      _command_mode = acceleration;
    }
  }

  if (!sdf->HasElement("controlDelay")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <controlDelay>, cannot proceed");
    return;
  } else {
    _control_delay = sdf->GetElement("controlDelay")->Get<double>();
  }

  if (!sdf->HasElement("steeringLockTime")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <steeringLockTime>, cannot proceed");
    return;
  } else {
    _steering_lock_time = sdf->GetElement("steeringLockTime")->Get<double>();
  }

  if (!sdf->HasElement("pubGroundTruth")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <pubGroundTruth>, cannot proceed");
    return;
  } else {
    _pub_ground_truth = sdf->GetElement("pubGroundTruth")->Get<bool>();
  }
}

void RaceCarModelPlugin::initVehicleModel(const sdf::ElementPtr &sdf) {
  // Get the vehicle model from the sdf
  std::string vehicle_model_ = "";
  if (!sdf->HasElement("vehicle_model")) {
    vehicle_model_ = "DynamicBicycle";
  } else {
    vehicle_model_ = sdf->GetElement("vehicle_model")->Get<std::string>();
  }

  std::string yaml_name = "";
  if (!sdf->HasElement("yaml_config")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <yaml_config>, cannot proceed");
    return;
  } else {
    yaml_name = sdf->GetElement("yaml_config")->Get<std::string>();
  }

  RCLCPP_DEBUG(_rosnode->get_logger(), "RaceCarModelPlugin finished loading params");

  if (vehicle_model_ == "PointMass") {
    _vehicle = std::unique_ptr<eufs::models::VehicleModel>(new eufs::models::PointMass(yaml_name));
  } else if (vehicle_model_ == "DynamicBicycle") {
    _vehicle =
        std::unique_ptr<eufs::models::VehicleModel>(new eufs::models::DynamicBicycle(yaml_name));
  } else {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin invalid vehicle model, cannot proceed");
    return;
  }
}

void RaceCarModelPlugin::initModel(const sdf::ElementPtr &sdf) {
  // Steering joints
  _left_steering_joint_name = sdf->Get<std::string>("front_left_wheel_steering");
  std::string leftSteeringJointName = _model->GetName() + "::" + _left_steering_joint_name;
  _left_steering_joint = _model->GetJoint(leftSteeringJointName);
  _right_steering_joint_name = sdf->Get<std::string>("front_right_wheel_steering");
  std::string rightSteeringJointName = _model->GetName() + "::" + _right_steering_joint_name;
  _right_steering_joint = _model->GetJoint(rightSteeringJointName);

  _front_left_wheel_joint_name = sdf->Get<std::string>("front_left_wheel");
  _front_right_wheel_joint_name = sdf->Get<std::string>("front_right_wheel");
  _rear_left_wheel_joint_name = sdf->Get<std::string>("rear_left_wheel");
  _rear_right_wheel_joint_name = sdf->Get<std::string>("rear_right_wheel");
}

void RaceCarModelPlugin::initNoise(const sdf::ElementPtr &sdf) {
  std::string yaml_name = "";
  if (!sdf->HasElement("noise_config")) {
    RCLCPP_FATAL(_rosnode->get_logger(),
                 "gazebo_ros_race_car_model plugin missing <noise_config>, cannot proceed");
    return;
  } else {
    yaml_name = sdf->GetElement("noise_config")->Get<std::string>();
  }

  // Create noise object
  _noise = std::make_unique<eufs::models::Noise>(yaml_name);
}

void RaceCarModelPlugin::initRoadNoise(const sdf::ElementPtr &sdf) {
  auto get_double = [&](const char *name, double def) {
    return sdf->HasElement(name) ? sdf->GetElement(name)->Get<double>() : def;
  };

  _road_noise_enabled = sdf->HasElement("roadNoise") && sdf->GetElement("roadNoise")->Get<bool>();
  _road_sigma_z = get_double("roadNoiseZ", 0.010);          // 1 cm vertical bounce
  _road_sigma_roll = get_double("roadNoiseRoll", 0.006);    // ~0.34 deg
  _road_sigma_pitch = get_double("roadNoisePitch", 0.006);  // ~0.34 deg
  _road_tau = std::max(get_double("roadNoiseCorrelation", 0.05), 1.0e-3);
  _road_speed_ref = std::max(get_double("roadNoiseSpeedRef", 5.0), 1.0e-3);
  _road_max_gain = std::max(get_double("roadNoiseMaxGain", 2.0), 0.0);
  _road_max_z_rate = std::max(get_double("roadNoiseMaxZRate", 0.5), 0.0);
  _road_max_ang_rate = std::max(get_double("roadNoiseMaxAngRate", 1.0), 0.0);
  _road_rng.seed(sdf->HasElement("roadNoiseSeed") ? sdf->GetElement("roadNoiseSeed")->Get<unsigned int>()
                                                  : 7u);
  _road_z = 0.0;
  _road_roll = 0.0;
  _road_pitch = 0.0;

  if (_road_noise_enabled) {
    RCLCPP_INFO(_rosnode->get_logger(),
                "Road-roughness vibration enabled (sigma_z=%.3f m, roll=%.4f, pitch=%.4f rad, "
                "tau=%.3f s, v_ref=%.2f m/s)",
                _road_sigma_z, _road_sigma_roll, _road_sigma_pitch, _road_tau, _road_speed_ref);
  }
}

void RaceCarModelPlugin::initLoadTransfer(const sdf::ElementPtr &sdf) {
  auto get_double = [&](const char *name, double def) {
    return sdf->HasElement(name) ? sdf->GetElement(name)->Get<double>() : def;
  };

  _load_transfer_enabled =
      sdf->HasElement("loadTransfer") && sdf->GetElement("loadTransfer")->Get<bool>();
  // Clamps only. How far the body leans and how it gets there come from the
  // car's own suspension parameters, not from the SDF.
  _lt_max_roll = std::max(get_double("loadTransferMaxRoll", 0.10), 0.0);
  _lt_max_pitch = std::max(get_double("loadTransferMaxPitch", 0.10), 0.0);
  _lt_roll = 0.0;
  _lt_pitch = 0.0;
  _lt_roll_rate = 0.0;
  _lt_pitch_rate = 0.0;

  if (_load_transfer_enabled) {
    const auto &param = _vehicle->getParam();
    const auto &suspension = param.suspension;
    const double mass = param.inertia.m;
    // Report the lean at 1 g, because that is the number anyone can sanity-check
    // against the real car -- unlike a stiffness in N*m/rad.
    const double roll_at_1g = mass * param.inertia.g * suspension.h_cg / suspension.k_roll;
    RCLCPP_INFO(_rosnode->get_logger(),
                "Acceleration load transfer enabled (h_cg=%.3f m, k_roll=%.0f k_pitch=%.0f N*m/rad,"
                " %.2f Hz zeta=%.2f) -> %.2f deg of roll at 1 g; tyre coupling %s",
                suspension.h_cg, suspension.k_roll, suspension.k_pitch, suspension.natural_freq_hz,
                suspension.damping_ratio, roll_at_1g * 180.0 / M_PI,
                suspension.load_transfer_to_tires ? "ON" : "off");
  }
}

void RaceCarModelPlugin::initTerrain(const sdf::ElementPtr &sdf) {
  auto get_double = [&](const char *name, double def) {
    return sdf->HasElement(name) ? sdf->GetElement(name)->Get<double>() : def;
  };

  TerrainField::Config config;
  config.enabled = sdf->HasElement("terrain") && sdf->GetElement("terrain")->Get<bool>();
  config.seed = sdf->HasElement("terrainSeed") ? sdf->GetElement("terrainSeed")->Get<unsigned int>()
                                               : 7u;
  config.density = get_double("terrainDensity", 0.02);
  config.half_extent_x = get_double("terrainHalfExtentX", 60.0);
  config.half_extent_y = get_double("terrainHalfExtentY", 60.0);
  config.height_mean = get_double("terrainHeightMean", 0.020);
  config.height_sigma = get_double("terrainHeightSigma", 0.010);
  config.height_min = get_double("terrainHeightMin", 0.004);
  config.radius_min = get_double("terrainRadiusMin", 0.25);
  config.radius_max = get_double("terrainRadiusMax", 0.60);
  config.track_margin = get_double("terrainTrackMargin", 3.5);

  _terrain_enabled = config.enabled;
  if (!_terrain_enabled) {
    return;
  }

  // Grow the field around the track's cones, which is the only ground the car
  // can reach. The track model is loaded before the car, so its links are
  // already here; this is the same enumeration the ground-truth cone plugin
  // does.
  if (gazebo::physics::ModelPtr track = _world->ModelByName("track")) {
    for (const gazebo::physics::LinkPtr &link : track->GetLinks()) {
      const ignition::math::Vector3d position = link->WorldPose().Pos();
      config.track_points.emplace_back(position.X(), position.Y());
    }
  }
  if (config.track_points.empty()) {
    RCLCPP_WARN(_rosnode->get_logger(),
                "Terrain: no 'track' model found, so bumps will be spread over the whole "
                "+-%.0f x %.0f m area. Lower terrainDensity to keep the count sane.",
                config.half_extent_x, config.half_extent_y);
  }

  _terrain.generate(config);

  // Each bump is a separate trimesh collision, and Gazebo has been seen to die
  // outright somewhere above a thousand of them. Refuse to build a world that
  // kills the simulator: a run that dies on startup teaches nothing.
  const auto max_bumps = static_cast<std::size_t>(get_double("terrainMaxBumps", 600.0));
  if (_terrain.bumps().size() > max_bumps) {
    _terrain_enabled = false;
    RCLCPP_ERROR(_rosnode->get_logger(),
                 "Terrain: %zu bumps exceeds terrainMaxBumps=%zu and would risk killing "
                 "gzserver; disabling. Lower terrainDensity or terrainTrackMargin.",
                 _terrain.bumps().size(), max_bumps);
    return;
  }
  if (_terrain.empty()) {
    _terrain_enabled = false;
    RCLCPP_WARN(_rosnode->get_logger(),
                "Terrain enabled but the configuration generated no bumps; disabling.");
    return;
  }

  // Resolve the unit dome by absolute path rather than model://, so the field
  // does not depend on GAZEBO_MODEL_PATH being set up.
  const std::string mesh_uri =
      "file://" + ament_index_cpp::get_package_share_directory("eufs_plugins") +
      "/meshes/bump_dome.stl";

  const std::string model_name = "eufs_terrain";
  _world->InsertModelString(_terrain.sdf(model_name, mesh_uri));

  RCLCPP_INFO(_rosnode->get_logger(),
              "Terrain bump field enabled: %zu bumps (seed=%u, density=%.3f /m^2, "
              "height %.3f+-%.3f m, radius %.2f-%.2f m) within %.1f m of %zu track cones",
              _terrain.bumps().size(), config.seed, config.density, config.height_mean,
              config.height_sigma, config.radius_min, config.radius_max, config.track_margin,
              config.track_points.size());
}

void RaceCarModelPlugin::sampleTerrain(double x, double y, double yaw, double *z, double *roll,
                                       double *pitch) const {
  *z = 0.0;
  *roll = 0.0;
  *pitch = 0.0;
  if (!_terrain_enabled) {
    return;
  }

  const auto &param = _vehicle->getParam();
  const double half_track = 0.5 * param.kinematic.axle_width;
  const double l_f = param.kinematic.l_F;
  const double l_r = param.kinematic.l_R;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  // Contact points in body frame, front/rear x left/right.
  const std::array<double, 4> body_x = {l_f, l_f, -l_r, -l_r};
  const std::array<double, 4> body_y = {half_track, -half_track, half_track, -half_track};
  std::array<double, 4> h{};
  for (std::size_t i = 0; i < h.size(); ++i) {
    h[i] = _terrain.height(x + body_x[i] * cos_yaw - body_y[i] * sin_yaw,
                           y + body_x[i] * sin_yaw + body_y[i] * cos_yaw);
  }

  // Least-squares plane z = a + b*x_body + c*y_body through the four contact
  // points. The wheel layout is symmetric in y and two-valued in x, so the
  // normal equations collapse to these differences.
  const double front = 0.5 * (h[0] + h[1]);
  const double rear = 0.5 * (h[2] + h[3]);
  const double left = 0.5 * (h[0] + h[2]);
  const double right = 0.5 * (h[1] + h[3]);
  const double wheelbase = l_f + l_r;

  const double b = wheelbase > 1.0e-6 ? (front - rear) / wheelbase : 0.0;
  const double c = half_track > 1.0e-6 ? (left - right) / (2.0 * half_track) : 0.0;
  const double mean_h = 0.25 * (h[0] + h[1] + h[2] + h[3]);
  // The contact-point centroid sits at x_body = (l_F - l_R)/2, which is the
  // body origin only on a car with equal overhangs; carry the term so it is not
  // silently wrong for one that is not.
  *z = mean_h - b * 0.5 * (l_f - l_r);
  // REP-103: +pitch is nose down, so a front-high plane pitches negative.
  // +roll rotates +y up, so a left-high plane rolls positive.
  *pitch = -std::atan(b);
  *roll = std::atan(c);
}

void RaceCarModelPlugin::setPositionFromWorld() {
  _offset = _model->WorldPose();

  RCLCPP_DEBUG(_rosnode->get_logger(), "Got starting offset %f %f %f", _offset.Pos()[0],
               _offset.Pos()[1], _offset.Pos()[2]);

  _state.x = 0.0;
  _state.y = 0.0;
  _state.z = 0.0;
  _state.yaw = 0.0;
  _state.v_x = 0.0;
  _state.v_y = 0.0;
  _state.v_z = 0.0;
  _state.r_x = 0.0;
  _state.r_y = 0.0;
  _state.r_z = 0.0;
  _state.a_x = 0.0;
  _state.a_y = 0.0;
  _state.a_z = 0.0;
}

bool RaceCarModelPlugin::resetVehiclePosition(
    std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  _state.x = 0.0;
  _state.y = 0.0;
  _state.z = 0.0;
  _state.yaw = 0.0;
  _state.v_x = 0.0;
  _state.v_y = 0.0;
  _state.v_z = 0.0;
  _state.r_x = 0.0;
  _state.r_y = 0.0;
  _state.r_z = 0.0;
  _state.a_x = 0.0;
  _state.a_y = 0.0;
  _state.a_z = 0.0;

  const ignition::math::Vector3d vel(0.0, 0.0, 0.0);
  const ignition::math::Vector3d angular(0.0, 0.0, 0.0);

  // Clear the road-roughness and load-transfer accumulators so the vehicle
  // resets flat. The terrain accumulators hold the bump the car was sitting on;
  // clearing them stops the jump back to the start line reading as a rate spike
  // on the IMU. The field itself is unchanged -- it is the ground, not a state.
  _road_z = 0.0;
  _road_roll = 0.0;
  _road_pitch = 0.0;
  _lt_roll = 0.0;
  _lt_pitch = 0.0;
  _terrain_z = 0.0;
  _terrain_v_z = 0.0;
  _terrain_roll = 0.0;
  _terrain_pitch = 0.0;

  _model->SetWorldPose(_offset);
  _model->SetAngularVel(angular);
  _model->SetLinearVel(vel);
  _wheel_joint_positions = {0.0, 0.0, 0.0, 0.0};
  _wheel_joint_velocities = {0.0, 0.0, 0.0, 0.0};

  return response->success;
}

void RaceCarModelPlugin::returnCommandMode(
    std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  std::string command_mode_str;
  if (_command_mode == acceleration) {
    command_mode_str = "acceleration";
  } else {
    command_mode_str = "velocity";
  }

  response->success = true;
  response->message = command_mode_str;
}

void RaceCarModelPlugin::setModelState(double dt) {
  double yaw = _state.yaw + _offset.Rot().Yaw();

  double x =
      _offset.Pos().X() + _state.x * cos(_offset.Rot().Yaw()) - _state.y * sin(_offset.Rot().Yaw());
  double y =
      _offset.Pos().Y() + _state.x * sin(_offset.Rot().Yaw()) + _state.y * cos(_offset.Rot().Yaw());
  // Body attitude perturbations added to the otherwise-flat pose: the terrain
  // under the wheels, an acceleration-driven load transfer, and continuous road
  // roughness. The matching body rates are fed to Gazebo so the rigidly-attached
  // IMU also senses the motion.
  //
  // Physical contributions (terrain, load transfer) and the roughness noise are
  // accumulated separately, because only the noise gets rate-clamped. Clamping
  // the total would silently cap real motion: crossing a 3 cm bump of 0.4 m
  // radius at 20 m/s is a genuine 2.4 m/s of vertical speed, five times the
  // clamp that exists to bound the noise.
  double roll = 0.0, pitch = 0.0;
  double roll_rate = 0.0, pitch_rate = 0.0, z_rate = 0.0;
  double road_roll_rate = 0.0, road_pitch_rate = 0.0, road_z_rate = 0.0;
  const double road_z_prev = _road_z;

  // Terrain: the plane through the four wheel contact points. This is the same
  // surface the LiDAR ray-traces, so a bump in the point cloud is a bump the
  // car climbs -- and vice versa. Sampled at the pose we are about to apply.
  double terrain_z = 0.0;
  double terrain_z_rate = 0.0;
  double terrain_a_z = 0.0;
  if (_terrain_enabled) {
    double terrain_roll = 0.0, terrain_pitch = 0.0;
    sampleTerrain(x, y, yaw, &terrain_z, &terrain_roll, &terrain_pitch);

    if (dt > 0.0) {
      terrain_z_rate = (terrain_z - _terrain_z) / dt;
      terrain_a_z = (terrain_z_rate - _terrain_v_z) / dt;
      z_rate += terrain_z_rate;
      roll_rate += (terrain_roll - _terrain_roll) / dt;
      pitch_rate += (terrain_pitch - _terrain_pitch) / dt;
    }
    _terrain_z = terrain_z;
    _terrain_v_z = terrain_z_rate;
    _terrain_roll = terrain_roll;
    _terrain_pitch = terrain_pitch;

    roll += terrain_roll;
    pitch += terrain_pitch;
  }

  // Load transfer: longitudinal accel pitches the body (accel = nose up,
  // braking = dive), lateral accel rolls it outward in a corner. Uses the
  // body-frame specific forces (with the convective yaw-rate terms) so a steady
  // corner still leans the car.
  //
  // The lean is the inertial moment about the CoG divided by the suspension's
  // resistance to it, and the body takes a spring-mass path to get there. This
  // replaces a hand-tuned rad-per-m/s^2 gain behind a first-order lag, which
  // was wrong in two ways: the gain was a number with no car behind it (0.010
  // leaned this car 5.6 deg at 1 g, roughly triple what its stiffness allows),
  // and a lag cannot overshoot, so a step of throttle settled onto the stops
  // like a damper rather than bouncing the way a car on springs does.
  if (_load_transfer_enabled && dt > 0.0) {
    const auto &param = _vehicle->getParam();
    const auto &suspension = param.suspension;
    const double mass = param.inertia.m;
    const double a_long = _state.a_x - _state.v_y * _state.r_z;
    const double a_lat = _state.a_y + _state.v_x * _state.r_z;

    const double roll_target =
        std::clamp(mass * a_lat * suspension.h_cg / suspension.k_roll, -_lt_max_roll, _lt_max_roll);
    const double pitch_target = std::clamp(-mass * a_long * suspension.h_cg / suspension.k_pitch,
                                           -_lt_max_pitch, _lt_max_pitch);

    const double omega = 2.0 * M_PI * suspension.natural_freq_hz;
    const double zeta = suspension.damping_ratio;
    _lt_roll_rate += (omega * omega * (roll_target - _lt_roll) - 2.0 * zeta * omega * _lt_roll_rate) * dt;
    _lt_pitch_rate +=
        (omega * omega * (pitch_target - _lt_pitch) - 2.0 * zeta * omega * _lt_pitch_rate) * dt;
    _lt_roll += _lt_roll_rate * dt;
    _lt_pitch += _lt_pitch_rate * dt;

    roll += _lt_roll;
    pitch += _lt_pitch;
    roll_rate += _lt_roll_rate;
    pitch_rate += _lt_pitch_rate;
  }

  // Continuous road roughness: speed-scaled AR(1) colored noise on z/roll/pitch.
  // This is texture finer than the terrain field resolves, not terrain: it is
  // re-rolled every step, so it is the same patch of road only by accident. Any
  // feature the LiDAR must also see belongs in the terrain field instead.
  if (_road_noise_enabled && dt > 0.0) {
    const double speed = std::hypot(_state.v_x, _state.v_y);
    const double gain = std::min(speed / _road_speed_ref, _road_max_gain);
    const double a = std::exp(-dt / _road_tau);
    const double b = std::sqrt(std::max(1.0 - a * a, 0.0));
    const double prev_roll = _road_roll, prev_pitch = _road_pitch;

    _road_z = a * _road_z + b * _road_sigma_z * gain * _road_normal(_road_rng);
    _road_roll = a * _road_roll + b * _road_sigma_roll * gain * _road_normal(_road_rng);
    _road_pitch = a * _road_pitch + b * _road_sigma_pitch * gain * _road_normal(_road_rng);

    roll += _road_roll;
    pitch += _road_pitch;
    road_z_rate = (_road_z - road_z_prev) / dt;
    road_roll_rate = (_road_roll - prev_roll) / dt;
    road_pitch_rate = (_road_pitch - prev_pitch) / dt;
  }

  // Clamp the roughness rates only. These bound a noise process whose step-to-
  // step difference is unbounded by construction; the terrain and load-transfer
  // rates above are real motion and are left alone.
  z_rate += std::clamp(road_z_rate, -_road_max_z_rate, _road_max_z_rate);
  roll_rate += std::clamp(road_roll_rate, -_road_max_ang_rate, _road_max_ang_rate);
  pitch_rate += std::clamp(road_pitch_rate, -_road_max_ang_rate, _road_max_ang_rate);

  double z;
  if (_terrain_enabled) {
    // The terrain owns the height outright; roughness rides on top of it.
    z = terrain_z + _road_z;
  } else {
    // _state.z came back from the world with last step's roughness already in
    // it, so remove that before adding this step's, or the offset random-walks.
    z = _state.z - road_z_prev + _road_z;
  }

  // Report the pose and motion actually applied. The planar vehicle model never
  // writes any of these, so this is their only source. Doing it here, after the
  // model has run and before publishCarState, is what keeps CarState honest
  // about the attitude the IMU is simultaneously feeling.
  //
  // a_z comes from the terrain alone. Differencing the total would differentiate
  // the roughness noise twice, and a white process differentiated twice is not
  // an acceleration -- it is 1/dt^2 times a random number, which at 1 kHz pinned
  // a_z to +-1000 m/s^2. The terrain profile is smooth, so its second difference
  // is the real thing: a few g crossing a bump at speed, which is what a car does.
  _state.a_z = terrain_a_z;
  _state.z = z;
  _state.v_z = z_rate;
  _state.roll = roll;
  _state.pitch = pitch;
  _state.r_x = roll_rate;
  _state.r_y = pitch_rate;

  double vx = _state.v_x * cos(yaw) - _state.v_y * sin(yaw);
  double vy = _state.v_x * sin(yaw) + _state.v_y * cos(yaw);

  const ignition::math::Pose3d pose(x, y, z, roll, pitch, yaw);
  const ignition::math::Vector3d vel(vx, vy, z_rate);
  const ignition::math::Vector3d angular(roll_rate, pitch_rate, _state.r_z);

  _model->SetWorldPose(pose);
  _model->SetAngularVel(angular);
  _model->SetLinearVel(vel);
}

eufs_msgs::msg::CarState RaceCarModelPlugin::stateToCarStateMsg(const eufs::models::State &state) {
  // Publish Car Info
  eufs_msgs::msg::CarState car_state;

  car_state.header.stamp.sec = _last_sim_time.sec;
  car_state.header.stamp.nanosec = _last_sim_time.nsec;
  car_state.header.frame_id = _reference_frame;
  car_state.child_frame_id = _robot_frame;

  car_state.pose.pose.position.x = state.x;
  car_state.pose.pose.position.y = state.y;
  car_state.pose.pose.position.z = state.z;

  // Report the attitude the body is actually in. This used to be hardcoded flat,
  // which silently zeroed the load transfer and terrain lean the body really
  // carries: the IMU is rigidly attached and feels the lean, but every consumer
  // of this topic was told the car was level. sim_ellipse_d reads roll/pitch
  // straight from here to synthesise the INS attitude output, so a flat lie here
  // became a flat lie on /sbg/ekf_euler.
  std::vector<double> orientation = {state.yaw, state.pitch, state.roll};

  orientation = ToQuaternion(orientation);

  car_state.pose.pose.orientation.x = orientation[0];
  car_state.pose.pose.orientation.y = orientation[1];
  car_state.pose.pose.orientation.z = orientation[2];
  car_state.pose.pose.orientation.w = orientation[3];

  car_state.twist.twist.linear.x = state.v_x;
  car_state.twist.twist.linear.y = state.v_y;
  car_state.twist.twist.linear.z = state.v_z;

  car_state.twist.twist.angular.x = state.r_x;
  car_state.twist.twist.angular.y = state.r_y;
  car_state.twist.twist.angular.z = state.r_z;

  car_state.linear_acceleration.x = state.a_x;
  car_state.linear_acceleration.y = state.a_y;
  car_state.linear_acceleration.z = state.a_z;

  car_state.slip_angle = _vehicle->getSlipAngle(_state, _act_input, true);

  car_state.state_of_charge = 999;

  return car_state;
}

eufs::models::State RaceCarModelPlugin::integrateDriftedState() {
  if (!_drift_initialized) {
    _drift_x = _state.x;
    _drift_y = _state.y;
    _drift_yaw = _state.yaw;
    _drift_last_time = _last_sim_time;
    _drift_initialized = true;
  } else {
    const double dt = (_last_sim_time - _drift_last_time).Double();
    _drift_last_time = _last_sim_time;
    if (dt > 0.0 && dt < 0.5) {
      // Corrupt the body-frame twist with bias + white noise, then integrate.
      const double v = _state.v_x * (1.0 + _drift_v_bias) + _drift_normal(_drift_rng) * _drift_sigma_v;
      const double vy = _state.v_y;
      const double w = _state.r_z * (1.0 + _drift_w_bias) + _drift_normal(_drift_rng) * _drift_sigma_w;
      _drift_x += (v * std::cos(_drift_yaw) - vy * std::sin(_drift_yaw)) * dt;
      _drift_y += (v * std::sin(_drift_yaw) + vy * std::cos(_drift_yaw)) * dt;
      _drift_yaw = std::atan2(std::sin(_drift_yaw + w * dt), std::cos(_drift_yaw + w * dt));
    }
  }

  eufs::models::State drifted = _state;
  drifted.x = _drift_x;
  drifted.y = _drift_y;
  drifted.yaw = _drift_yaw;
  return drifted;
}

void RaceCarModelPlugin::publishCarState() {
  eufs_msgs::msg::CarState car_state = stateToCarStateMsg(_state);

  // Publish the ground truth car state if it has subscribers and is allowed to publish
  if (_pub_ground_truth_car_state->get_subscription_count() > 0 && _pub_ground_truth) {
    _pub_ground_truth_car_state->publish(car_state);
  }

  // Localisation car state: drifting odometry if enabled, otherwise iid noise.
  eufs::models::State state_noisy =
      _drift_odometry ? integrateDriftedState() : _noise->applyNoise(_state);
  eufs_msgs::msg::CarState car_state_noisy = stateToCarStateMsg(state_noisy);

  // Fill in covariance matrix
  const eufs::models::NoiseParam &noise_param = _noise->getNoiseParam();
  car_state_noisy.pose.covariance[0] = pow(noise_param.position[0], 2);
  car_state_noisy.pose.covariance[7] = pow(noise_param.position[1], 2);
  car_state_noisy.pose.covariance[14] = pow(noise_param.position[2], 2);

  car_state_noisy.pose.covariance[21] = pow(noise_param.orientation[0], 2);
  car_state_noisy.pose.covariance[28] = pow(noise_param.orientation[1], 2);
  car_state_noisy.pose.covariance[35] = pow(noise_param.orientation[2], 2);

  car_state_noisy.twist.covariance[0] = pow(noise_param.linear_velocity[0], 2);
  car_state_noisy.twist.covariance[7] = pow(noise_param.linear_velocity[1], 2);
  car_state_noisy.twist.covariance[14] = pow(noise_param.linear_velocity[2], 2);

  car_state_noisy.twist.covariance[21] = pow(noise_param.angular_velocity[0], 2);
  car_state_noisy.twist.covariance[28] = pow(noise_param.angular_velocity[1], 2);
  car_state_noisy.twist.covariance[35] = pow(noise_param.angular_velocity[2], 2);

  car_state_noisy.linear_acceleration_covariance[0] = pow(noise_param.linear_acceleration[0], 2);
  car_state_noisy.linear_acceleration_covariance[4] = pow(noise_param.linear_acceleration[1], 2);
  car_state_noisy.linear_acceleration_covariance[8] = pow(noise_param.linear_acceleration[2], 2);

  if (_drift_odometry) {
    // Per-step drift noise as the reported pose uncertainty (the accumulated
    // error itself is unbounded and not represented here).
    car_state_noisy.pose.covariance[0] = _drift_sigma_v * _drift_sigma_v;
    car_state_noisy.pose.covariance[7] = _drift_sigma_v * _drift_sigma_v;
    car_state_noisy.pose.covariance[35] = _drift_sigma_w * _drift_sigma_w;
  }

  // Publish with noise
  if (_pub_localisation_car_state->get_subscription_count() > 0) {
    _pub_localisation_car_state->publish(car_state_noisy);
  }
}

void RaceCarModelPlugin::publishWheelSpeeds() {
  eufs_msgs::msg::WheelSpeedsStamped wheel_speeds_stamped;
  eufs_msgs::msg::WheelSpeeds wheel_speeds;

  wheel_speeds_stamped.header.stamp.sec = _last_sim_time.sec;
  wheel_speeds_stamped.header.stamp.nanosec = _last_sim_time.nsec;
  wheel_speeds_stamped.header.frame_id = _robot_frame;

  wheel_speeds = _vehicle->getWheelSpeeds(_state, _act_input);
  wheel_speeds_stamped.speeds = wheel_speeds;

  // Publish the ground truth wheel speeds if it has subscribers and is allowed to publish
  if (_pub_ground_truth_wheel_speeds->get_subscription_count() > 0 && _pub_ground_truth) {
    _pub_ground_truth_wheel_speeds->publish(wheel_speeds_stamped);
  }

  wheel_speeds = _noise->applyNoiseToWheelSpeeds(wheel_speeds);
  wheel_speeds_stamped.speeds = wheel_speeds;

  // Publish with Noise
  if (_pub_wheel_speeds->get_subscription_count() > 0) {
    _pub_wheel_speeds->publish(wheel_speeds_stamped);
  }
}

void RaceCarModelPlugin::updateWheelJointPositions(double dt) {
  const auto &param = _vehicle->getParam();
  const double half_track = 0.5 * param.kinematic.axle_width;
  const double wheel_radius = param.tire.radius;
  const double steering = _act_input.delta;

  const std::array<double, 4> wheel_x = {
    param.kinematic.l_F,
    param.kinematic.l_F,
    -param.kinematic.l_R,
    -param.kinematic.l_R};
  const std::array<double, 4> wheel_y = {
    half_track,
    -half_track,
    half_track,
    -half_track};
  const std::array<double, 4> wheel_heading = {steering, steering, 0.0, 0.0};

  for (std::size_t i = 0; i < _wheel_joint_velocities.size(); ++i) {
    const double wheel_vx = _state.v_x - _state.r_z * wheel_y[i];
    const double wheel_vy = _state.v_y + _state.r_z * wheel_x[i];
    const double rolling_speed =
        wheel_vx * std::cos(wheel_heading[i]) + wheel_vy * std::sin(wheel_heading[i]);
    _wheel_joint_velocities[i] = rolling_speed / wheel_radius;
    _wheel_joint_positions[i] += _wheel_joint_velocities[i] * dt;
  }
}

void RaceCarModelPlugin::publishJointStates() {
  sensor_msgs::msg::JointState joint_states;

  joint_states.header.stamp.sec = _last_sim_time.sec;
  joint_states.header.stamp.nanosec = _last_sim_time.nsec;

  joint_states.name = {
    _left_steering_joint_name,
    _front_left_wheel_joint_name,
    _right_steering_joint_name,
    _front_right_wheel_joint_name,
    _rear_left_wheel_joint_name,
    _rear_right_wheel_joint_name};

  joint_states.position = {
    _act_input.delta,
    _wheel_joint_positions[0],
    _act_input.delta,
    _wheel_joint_positions[1],
    _wheel_joint_positions[2],
    _wheel_joint_positions[3]};

  joint_states.velocity = {
    0.0,
    _wheel_joint_velocities[0],
    0.0,
    _wheel_joint_velocities[1],
    _wheel_joint_velocities[2],
    _wheel_joint_velocities[3]};

  _pub_joint_states->publish(joint_states);
}

void RaceCarModelPlugin::publishOdom() {
  nav_msgs::msg::Odometry odom;

  odom.header.stamp.sec = _last_sim_time.sec;
  odom.header.stamp.nanosec = _last_sim_time.nsec;

  odom.header.frame_id = _reference_frame;
  odom.child_frame_id = _robot_frame;

  eufs::models::State state_noisy = _noise->applyNoise(_state);

  odom.pose.pose.position.x = state_noisy.x;
  odom.pose.pose.position.y = state_noisy.y;
  odom.pose.pose.position.z = state_noisy.z;

  std::vector<double> orientation = {state_noisy.yaw, 0.0, 0.0};
  orientation = ToQuaternion(orientation);
  odom.pose.pose.orientation.x = orientation[0];
  odom.pose.pose.orientation.y = orientation[1];
  odom.pose.pose.orientation.z = orientation[2];
  odom.pose.pose.orientation.w = orientation[3];

  odom.twist.twist.linear.x = state_noisy.v_x;
  odom.twist.twist.linear.y = state_noisy.v_y;
  odom.twist.twist.linear.z = state_noisy.v_z;

  odom.twist.twist.angular.x = state_noisy.r_x;
  odom.twist.twist.angular.y = state_noisy.r_y;
  odom.twist.twist.angular.z = state_noisy.r_z;

  // fill in covariance matrix
  const eufs::models::NoiseParam &noise_param = _noise->getNoiseParam();
  odom.pose.covariance[0] = pow(noise_param.position[0], 2);
  odom.pose.covariance[7] = pow(noise_param.position[1], 2);
  odom.pose.covariance[14] = pow(noise_param.position[2], 2);

  odom.pose.covariance[21] = pow(noise_param.orientation[0], 2);
  odom.pose.covariance[28] = pow(noise_param.orientation[1], 2);
  odom.pose.covariance[35] = pow(noise_param.orientation[2], 2);

  odom.twist.covariance[0] = pow(noise_param.linear_velocity[0], 2);
  odom.twist.covariance[7] = pow(noise_param.linear_velocity[1], 2);
  odom.twist.covariance[14] = pow(noise_param.linear_velocity[2], 2);

  odom.twist.covariance[21] = pow(noise_param.angular_velocity[0], 2);
  odom.twist.covariance[28] = pow(noise_param.angular_velocity[1], 2);
  odom.twist.covariance[35] = pow(noise_param.angular_velocity[2], 2);

  // Publish the ground truth odom if it has subscribers and is allowed to publish
  if (_pub_odom->get_subscription_count() > 0 && _pub_ground_truth) {
    _pub_odom->publish(odom);
  }
}

void RaceCarModelPlugin::publishTf() {
  eufs::models::State state_noisy = _noise->applyNoise(_state);

  // Position
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(state_noisy.x, state_noisy.y, 0.0));

  // Orientation
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, state_noisy.yaw);
  transform.setRotation(q);

  // Send TF
  geometry_msgs::msg::TransformStamped transform_stamped;

  transform_stamped.header.stamp.sec = _last_sim_time.sec;
  transform_stamped.header.stamp.nanosec = _last_sim_time.nsec;
  transform_stamped.header.frame_id = _reference_frame;
  transform_stamped.child_frame_id = _robot_frame;
  tf2::convert(transform, transform_stamped.transform);

  _tf_br->sendTransform(transform_stamped);
}

void RaceCarModelPlugin::Reset() {
  _last_sim_time = 0;
  // Re-seed drift integration from the reset pose on the next publish.
  _drift_initialized = false;
}

void RaceCarModelPlugin::update() {
  gazebo::common::Time curTime = _world->SimTime();
  double dt = (curTime - _last_sim_time).Double();
  if (dt < (1 / _update_rate)) {
    return;
  }

  _last_sim_time = curTime;
  updateState(dt);
}

void RaceCarModelPlugin::updateState(const double dt) {
  if (!_command_Q.empty()) {
    gazebo::common::Time cmd_time = _cmd_time_Q.front();
    if ((_last_sim_time - cmd_time).Double() >= _control_delay) {
      std::shared_ptr<ackermann_msgs::msg::AckermannDriveStamped> cmd = _command_Q.front();
      _des_input.acc = cmd->drive.acceleration;
      _des_input.vel = cmd->drive.speed;
      _des_input.delta = cmd->drive.steering_angle;

      _command_Q.pop();
      _cmd_time_Q.pop();
    }
  }

  if (_command_mode == velocity) {
    double current_speed = std::sqrt(std::pow(_state.v_x, 2) + std::pow(_state.v_y, 2));
    _des_input.acc = (_des_input.vel - current_speed) / dt;
  }

  // If last command was more than 1s ago, then slow down car
  _act_input.acc = (_last_sim_time - _last_cmd_time) < 1.0 ? _des_input.acc : -1.0;
  // Make sure steering rate is within limits
  _act_input.delta +=
      (_des_input.delta - _act_input.delta >= 0 ? 1 : -1) *
      std::min(_max_steering_rate * dt, std::abs(_des_input.delta - _act_input.delta));

  // Update z value from simulation
  // This allows the state to have the most up to date value of z. Without this
  // the vehicle in simulation has problems interacting with the ground plane.
  // This may cause problems if the vehicle models start to take into account z
  // but because this simulation isn't for flying cars we should be ok (at least for now).
  //
  // With a terrain field the height is not the ground plane's to settle: the
  // car is teleported every step, so letting contact decide z would make the
  // ride over a bump depend on penetration and impulse rather than on the bump.
  // setModelState takes z from the terrain instead, and owns it outright.
  if (!_terrain_enabled) {
    _state.z = _model->WorldPose().Pos().Z();
  }

  _vehicle->updateState(_state, _act_input, dt);
  updateWheelJointPositions(dt);

  _left_steering_joint->SetPosition(0, _act_input.delta);
  _right_steering_joint->SetPosition(0, _act_input.delta);
  setModelState(dt);

  double time_since_last_published = (_last_sim_time - _time_last_published).Double();
  if (time_since_last_published < (1 / _publish_rate)) {
    return;
  }
  _time_last_published = _last_sim_time;

  // Publish Everything
  publishCarState();
  publishWheelSpeeds();
  publishJointStates();
  publishOdom();

  if (_publish_tf) {
    publishTf();
  }

  _state_machine->spinOnce(_last_sim_time);
}

void RaceCarModelPlugin::onCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
  // Override commands if we're not in canDrive state
  if (!_state_machine->canDrive()) {
    msg->drive.steering_angle = 0;
    msg->drive.acceleration = -100;
    msg->drive.speed = 0;
  }
  _command_Q.push(msg);
  _cmd_time_Q.push(_world->SimTime());
  _last_cmd_time = _world->SimTime();
}

std::vector<double> RaceCarModelPlugin::ToQuaternion(std::vector<double> &euler) {
  // Abbreviations for the various angular functions
  double cy = cos(euler[0] * 0.5);
  double sy = sin(euler[0] * 0.5);
  double cp = cos(euler[1] * 0.5);
  double sp = sin(euler[1] * 0.5);
  double cr = cos(euler[2] * 0.5);
  double sr = sin(euler[2] * 0.5);

  // resize, not reserve: reserve leaves size() at 0, so the writes below and
  // every read in the caller were out of bounds. It happened to work because
  // the capacity was allocated, but the vector returned still claimed to be
  // empty.
  std::vector<double> q;
  q.resize(4);
  q[0] = cy * cp * sr - sy * sp * cr;  // x
  q[1] = sy * cp * sr + cy * sp * cr;  // y
  q[2] = sy * cp * cr - cy * sp * sr;  // z
  q[3] = cy * cp * cr + sy * sp * sr;  // w

  return q;
}

GZ_REGISTER_MODEL_PLUGIN(RaceCarModelPlugin)

}  // namespace eufs_plugins
}  // namespace gazebo_plugins
