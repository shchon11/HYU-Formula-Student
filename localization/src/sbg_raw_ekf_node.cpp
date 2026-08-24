// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "hyu_localization/sbg_raw_ekf_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace hyu_localization
{

namespace
{
constexpr double kHugeSigma = 1.0e3;  // graph_slam's "pose invalid" marker (>= odom_invalid_sigma)

const char * fixName(int type)
{
  switch (type) {
    case 0: return "NO_SOL";
    case 1: return "UNKNOWN";
    case 2: return "SINGLE";
    case 3: return "PSRDIFF";
    case 4: return "SBAS";
    case 5: return "OMNISTAR";
    case 6: return "RTK_FLOAT";
    case 7: return "RTK_FIXED";
    case 8: return "PPP_FLOAT";
    case 9: return "PPP_FIXED";
    case 10: return "FIXED";
    default: return "-";
  }
}

std::string fmt(const char * f, double v)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), f, v);
  return buf;
}
}  // namespace

SbgRawEkfNode::SbgRawEkfNode()
: rclcpp::Node("sbg_raw_ekf")
{
  // --- topics / frames -----------------------------------------------------
  car_state_topic_ = declare_parameter<std::string>("car_state_topic", "/localization/ins_odom");
  gnss_odom_topic_ = declare_parameter<std::string>("gnss_odom_topic", "/localization/gnss_odom");
  health_topic_ = declare_parameter<std::string>("health_topic", "/sbg_bridge/status");
  overlay_topic_ = declare_parameter<std::string>(
    "overlay_topic",
    "/localization/debug/gnss_overlay");
  publish_overlay_ = declare_parameter<bool>("publish_overlay", true);
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/sbg/imu_data");
  gps_pos_topic_ = declare_parameter<std::string>("gps_pos_topic", "/sbg/gps_pos");
  gps_vel_topic_ = declare_parameter<std::string>("gps_vel_topic", "/sbg/gps_vel");
  gps_hdt_topic_ = declare_parameter<std::string>("gps_hdt_topic", "/sbg/gps_hdt");
  world_frame_ = declare_parameter<std::string>("world_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  // Must match the driver's output.use_enu (false -> "ned", the car default).
  std::string convention = declare_parameter<std::string>("frame_convention", "ned");
  std::transform(convention.begin(), convention.end(), convention.begin(), ::tolower);
  if (convention != "ned" && convention != "enu") {
    throw std::invalid_argument("frame_convention must be 'ned' or 'enu'");
  }
  enu_wire_ = convention == "enu";
  // Datum: NaN = first valid gps_pos.
  const double datum_lat = declare_parameter<double>("datum_latitude", std::nan(""));
  const double datum_lon = declare_parameter<double>("datum_longitude", std::nan(""));
  if (std::isfinite(datum_lat) && std::isfinite(datum_lon)) {
    proj_.setOrigin(datum_lat, datum_lon);
  }

  // --- reference point ----------------------------------------------------------
  // The receiver reports the PRIMARY GNSS ANTENNA: gps_pos/gps_vel are its
  // position/velocity, so the EKF state is the antenna's. SLAM attaches
  // base_footprint-frame cone observations to this pose, so the published
  // pose must be base_footprint's: p_base = p_antenna - R(yaw) r, with r the
  // antenna's position in base_footprint (x forward, y left). A wrong r only
  // shows when the heading changes -- every cone re-observed from a new
  // heading lands R(yaw) r away from before, 2|r| after a U-turn -- which is
  // exactly how the map broke while turning on the 2026-08-01 bags (and
  // not at a standstill or on straights). Estimated from those bags by
  // cone re-observation consistency: (+1.25, 0.00) m, re-observation
  // distance 3.0 -> 0.9 cm. Measure it properly after any remount (antenna
  // phase centre to the ground point below the ZED stereo centre).
  antenna_offset_x_ = declare_parameter<double>("antenna_offset_x", 1.25);
  antenna_offset_y_ = declare_parameter<double>("antenna_offset_y", 0.0);
  allow_gnss_denied_init_ = declare_parameter<bool>("allow_gnss_denied_init", false);
  reacq_min_gap_s_ = declare_parameter<double>("reacquisition_min_gap_s", 3.0);
  reacq_jump_m_ = declare_parameter<double>("reacquisition_jump_m", 0.5);
  reacq_jump_yaw_rad_ = deg2rad(declare_parameter<double>("reacquisition_jump_yaw_deg", 10.0));
  wheel_scale_online_ = declare_parameter<bool>("wheel_scale_online", true);
  wheel_scale_tau_s_ = declare_parameter<double>("wheel_scale_tau_s", 45.0);
  wheel_scale_min_speed_ = declare_parameter<double>("wheel_scale_min_speed", 2.0);
  wheel_scale_max_yaw_rate_ = declare_parameter<double>("wheel_scale_max_yaw_rate", 0.25);
  wheel_scale_bound_ = declare_parameter<double>("wheel_scale_bound", 0.05);
  wheel_scale_max_accel_ = declare_parameter<double>("wheel_scale_max_accel", 0.3);
  wheel_scale_min_samples_ = static_cast<std::uint64_t>(declare_parameter<int>("wheel_scale_min_samples", 20));
  gnss_denied_init_sec_ = declare_parameter<double>("gnss_denied_init_sec", 2.0);
  gnss_denied_init_pos_sig_ = declare_parameter<double>("gnss_denied_init_pos_sig", 100.0);

  // --- output trust ----------------------------------------------------------
  // Relative-odometry sigma floors by mode for the CarState covariance. The
  // EKF's own position sigma is max'ed in, so coasting widens honestly.
  // 2026-08-24: OK floor 0.05 -> 0.02. The 5 cm "reality tax" for unmodelled
  // bias cost graph_slam 4-5x its pose accuracy (relative odometry chain
  // freedom ~ sqrt(N) * floor: KASE lap p95 1.2 m vs EKF 0.2 m). The biggest
  // unmodelled bias (wheel scale, 2 % -> 0.2 m) is now estimated online and
  // GNSS re-acquisition snaps are flagged with kHugeSigma, so the honest floor
  // is the EKF's own ~1-2 cm plus a small margin.
  odom_sigma_ok_ = declare_parameter<double>("odom_sigma_ok", 0.02);
  odom_sigma_degraded_ = declare_parameter<double>("odom_sigma_degraded", 0.20);
  // IMU gap beyond which the next message is published with a huge sigma.
  blind_gap_sec_ = declare_parameter<double>("blind_gap_sec", 0.5);
  imu_timeout_ = declare_parameter<double>("imu_timeout", 1.0);
  status_period_ = declare_parameter<double>("status_period", 0.2);

  // --- filter parameters (my_ekf.py DEFAULT_PARAMS) ---------------------------
  RawGnssEkfParams p;
  p.hdt_offset = deg2rad(declare_parameter<double>("hdt_offset_deg", 180.0));
  p.sig_gyro = deg2rad(declare_parameter<double>("sig_gyro_dps", rad2deg(p.sig_gyro)));
  p.sig_acc = declare_parameter<double>("sig_acc", p.sig_acc);
  p.sig_bg_rw = deg2rad(declare_parameter<double>("sig_bg_rw_dps", rad2deg(p.sig_bg_rw)));
  p.sig_ba_rw = declare_parameter<double>("sig_ba_rw", p.sig_ba_rw);
  p.zupt_window = static_cast<int>(declare_parameter<int>("zupt_window", p.zupt_window));
  p.zupt_gyro_std =
    deg2rad(declare_parameter<double>("zupt_gyro_std_dps", rad2deg(p.zupt_gyro_std)));
  p.zupt_gyro_mean =
    deg2rad(declare_parameter<double>("zupt_gyro_mean_dps", rad2deg(p.zupt_gyro_mean)));
  p.zupt_acc_std = declare_parameter<double>("zupt_acc_std", p.zupt_acc_std);
  p.zupt_gps_speed = declare_parameter<double>("zupt_gps_speed", p.zupt_gps_speed);
  p.zupt_sig_v = declare_parameter<double>("zupt_sig_v", p.zupt_sig_v);
  p.zaru_sig = deg2rad(declare_parameter<double>("zaru_sig_dps", rad2deg(p.zaru_sig)));
  p.pos_floor = declare_parameter<double>("pos_floor", p.pos_floor);
  p.pos_timing = declare_parameter<double>("pos_timing_sigma", p.pos_timing);
  p.vel_floor = declare_parameter<double>("vel_floor", p.vel_floor);
  p.gate_pos = declare_parameter<double>("gate_pos", p.gate_pos);
  p.gate_vel = declare_parameter<double>("gate_vel", p.gate_vel);
  p.gate_hdt = declare_parameter<double>("gate_hdt", p.gate_hdt);
  p.gate_crs = declare_parameter<double>("gate_crs", p.gate_crs);
  p.max_rej_pos = static_cast<int>(declare_parameter<int>("max_rej_pos", p.max_rej_pos));
  p.max_rej_vel = static_cast<int>(declare_parameter<int>("max_rej_vel", p.max_rej_vel));
  p.max_rej_hdt = static_cast<int>(declare_parameter<int>("max_rej_hdt", p.max_rej_hdt));
  p.hdt_max_acc = deg2rad(declare_parameter<double>("hdt_max_acc_deg", rad2deg(p.hdt_max_acc)));
  p.hdt_min_sig = deg2rad(declare_parameter<double>("hdt_min_sig_deg", rad2deg(p.hdt_min_sig)));
  p.crs_hdt_gap = declare_parameter<double>("crs_hdt_gap", p.crs_hdt_gap);
  p.crs_min_speed = declare_parameter<double>("crs_min_speed", p.crs_min_speed);
  p.max_dt = declare_parameter<double>("max_dt", p.max_dt);
  p.mode_pos_age = declare_parameter<double>("mode_pos_age", p.mode_pos_age);
  p.mode_hdt_age = declare_parameter<double>("mode_hdt_age", p.mode_hdt_age);
  p.mode_yaw_sig = deg2rad(declare_parameter<double>("mode_yaw_sig_deg", rad2deg(p.mode_yaw_sig)));
  // --- wheel speeds (optional velocity aiding) ---------------------------------
  // /vehicle/wheel_speeds from drive_udp_bridge (ECU encoder counts -> m/s).
  // Fused only while samples keep arriving; with none the filter is exactly
  // the IMU+GNSS one. The bridge stops publishing when the ECU feed drops, so
  // "no fresh sample" is the whole hand-over logic.
  use_wheel_speeds_ = declare_parameter<bool>("use_wheel_speeds", true);
  wheel_speeds_topic_ = declare_parameter<std::string>("wheel_speeds_topic", "/vehicle/wheel_speeds");
  wheel_source_ = declare_parameter<std::string>("wheel_source", "rear");  // rear | all
  wheel_scale_ = declare_parameter<double>("wheel_scale", 1.0);
  wheel_scale_est_ = wheel_scale_;
  wheel_scale_acc_ = wheel_scale_;
  wheel_timeout_ = declare_parameter<double>("wheel_timeout", 0.3);
  p.sig_wheel = declare_parameter<double>("wheel_sigma", p.sig_wheel);
  p.wheel_sig_per_acc = declare_parameter<double>("wheel_sigma_per_acc", p.wheel_sig_per_acc);
  p.gate_wheel = declare_parameter<double>("gate_wheel", p.gate_wheel);
  p.max_rej_wheel = static_cast<int>(declare_parameter<int>("max_rej_wheel", p.max_rej_wheel));
  p.zupt_wheel_speed = declare_parameter<double>("zupt_wheel_speed", p.zupt_wheel_speed);
  p.zupt_wheel_age = declare_parameter<double>("zupt_wheel_age", p.zupt_wheel_age);
  // The state velocity is the antenna's; the rear axle reports the centreline
  // speed. They differ by w * (antenna lateral offset): ROS y-left -> NED y-right.
  p.wheel_lever_y_right = -antenna_offset_y_;
  if (wheel_source_ != "rear" && wheel_source_ != "all") {
    throw std::invalid_argument("wheel_source must be 'rear' or 'all'");
  }
  // Late-measurement buffer: receiver epochs arrive ~90 ms (p50) / 113 ms
  // (p90) after the IMU frame of the same device time.
  history_sec_ = declare_parameter<double>("oosm_history_sec", 1.0);
  ekf_params_ = p;
  ekf_ = std::make_unique<RawGnssEkf>(ekf_params_, history_sec_);

  // --- I/O --------------------------------------------------------------------
  car_state_pub_ = create_publisher<hyu_msgs::msg::CarState>(car_state_topic_, 10);
  gnss_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(gnss_odom_topic_, 10);
  health_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(health_topic_, 10);
#ifdef HYU_HAVE_RVIZ_OVERLAY
  if (publish_overlay_) {
    // Latched so RViz shows the latest state immediately on (re)connect.
    overlay_pub_ = create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
      overlay_topic_, rclcpp::QoS(1).transient_local());
  }
#endif
  const auto qos = rclcpp::SensorDataQoS().keep_last(10);
  imu_sub_ = create_subscription<sbg_driver::msg::SbgImuData>(
    imu_topic_, qos, std::bind(&SbgRawEkfNode::onImu, this, std::placeholders::_1));
  gps_pos_sub_ = create_subscription<sbg_driver::msg::SbgGpsPos>(
    gps_pos_topic_, qos, std::bind(&SbgRawEkfNode::onGpsPos, this, std::placeholders::_1));
  gps_vel_sub_ = create_subscription<sbg_driver::msg::SbgGpsVel>(
    gps_vel_topic_, qos, std::bind(&SbgRawEkfNode::onGpsVel, this, std::placeholders::_1));
  gps_hdt_sub_ = create_subscription<sbg_driver::msg::SbgGpsHdt>(
    gps_hdt_topic_, qos, std::bind(&SbgRawEkfNode::onGpsHdt, this, std::placeholders::_1));
  if (use_wheel_speeds_) {
    wheel_sub_ = create_subscription<hyu_msgs::msg::WheelSpeedsStamped>(
      wheel_speeds_topic_, rclcpp::QoS(10),
      std::bind(&SbgRawEkfNode::onWheelSpeeds, this, std::placeholders::_1));
  }
  status_timer_ = create_wall_timer(
    std::chrono::duration<double>(std::max(0.05, status_period_)),
    std::bind(&SbgRawEkfNode::onStatusTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "SBG raw EKF: %s + %s + %s + %s -> %s (odom) + %s (raw fix) [convention=%s, "
    "hdt_offset=%.1f deg, datum=%s, antenna at (%+.2f, %+.2f) m in %s, no ekf_nav/ekf_euler]",
    imu_topic_.c_str(), gps_pos_topic_.c_str(), gps_vel_topic_.c_str(), gps_hdt_topic_.c_str(),
    car_state_topic_.c_str(), gnss_odom_topic_.c_str(), enu_wire_ ? "enu" : "ned",
    rad2deg(p.hdt_offset), proj_.valid() ? "fixed" : "first fix",
    antenna_offset_x_, antenna_offset_y_, base_frame_.c_str());
  if (use_wheel_speeds_) {
    RCLCPP_INFO(
      get_logger(),
      "wheel speeds: %s (%s wheels, scale %.4f, sigma %.3f + %.3f*|a_x| m/s, fused while "
      "samples are < %.2f s old; none -> IMU+GNSS only)",
      wheel_speeds_topic_.c_str(), wheel_source_.c_str(), wheel_scale_, p.sig_wheel,
      p.wheel_sig_per_acc, wheel_timeout_);
  } else {
    RCLCPP_INFO(get_logger(), "wheel speeds: off (use_wheel_speeds=false)");
  }
}

// ---------------------------------------------------------------------------

builtin_interfaces::msg::Time SbgRawEkfNode::rosStamp(
  double t_dev, const builtin_interfaces::msg::Time & fallback) const
{
  if (!off_init_) {
    return fallback;
  }
  const double t = t_dev + ros_minus_dev_;
  if (t < 0.0) {
    return fallback;
  }
  return rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
}

bool SbgRawEkfNode::imuStale() const
{
  if (!have_last_imu_) {
    return true;
  }
  return (now().seconds() - last_imu_ros_t_) > imu_timeout_;
}

void SbgRawEkfNode::restart(const char * why)
{
  RCLCPP_WARN(get_logger(), "%s: restarting the filter from scratch", why);
  ekf_ = std::make_unique<RawGnssEkf>(ekf_params_, history_sec_);
  dev_clock_ = sbg_raw::DeviceClock();
  off_init_ = false;
  have_last_imu_ = false;
  blind_gap_ = false;
  announced_init_ = false;
  last_pub_t_ = -std::numeric_limits<double>::infinity();
  last_fix_valid_ = false;
  last_wheel_ros_t_ = -std::numeric_limits<double>::infinity();
  last_wheel_mps_ = 0.0;
}

void SbgRawEkfNode::onImu(const sbg_driver::msg::SbgImuData::SharedPtr msg)
{
  const double hdr_t = rclcpp::Time(msg->header.stamp).seconds();
  // Time running backwards by more than a second is a replay loop / sim
  // reset, not jitter: every buffer (device clock, OOSM events, publish
  // guard) would otherwise wait for the old stamps to come round again.
  if (have_last_imu_ && hdr_t < last_imu_ros_t_ - 1.0) {
    restart("imu stamp jumped backwards (bag loop or sim reset?)");
  }
  const double t = dev_clock_.toSeconds(msg->time_stamp);
  // ROS <-> device clock offset: low-pass over the IMU stream (wire jitter
  // is a few ms); re-anchor on a jump (driver restart / bag loop).
  if (!off_init_) {
    ros_minus_dev_ = hdr_t - t;
    off_init_ = true;
  } else {
    const double r = hdr_t - t - ros_minus_dev_;
    if (std::fabs(r) > 0.5) {
      ros_minus_dev_ = hdr_t - t;
    } else {
      ros_minus_dev_ += 0.02 * r;
    }
  }
  if (have_last_imu_ && (t - last_imu_dev_t_) > blind_gap_sec_) {
    RCLCPP_WARN(
      get_logger(), "imu_data gap of %.2f s: next odometry message is published with a huge sigma",
      t - last_imu_dev_t_);
    blind_gap_ = true;
  }
  have_last_imu_ = true;
  last_imu_dev_t_ = t;
  last_imu_ros_t_ = hdr_t;
  if (!have_first_imu_) {
    have_first_imu_ = true;
    first_imu_ros_t_ = hdr_t;
  }

  ekf_->imu(sbg_raw::imuSample(*msg, t, enu_wire_));
  if (!ekf_->core().initialized()) {
    // GNSS-denied cold start: after waiting gnss_denied_init_sec for a fix,
    // start at the datum origin and dead-reckon on IMU(+wheel). Needs a datum
    // (proj_ valid, i.e. datum_latitude/longitude set) so the frame stays
    // consistent with the GNSS that later returns.
    if (allow_gnss_denied_init_ && proj_.valid() &&
      (hdr_t - first_imu_ros_t_) > gnss_denied_init_sec_)
    {
      ekf_->initAtOrigin(t, gnss_denied_init_pos_sig_);
      if (!denied_init_announced_) {
        denied_init_announced_ = true;
        RCLCPP_WARN(
          get_logger(),
          "GNSS-denied init: no valid gps_pos after %.1f s -- starting at the datum "
          "origin, dead-reckoning on IMU%s; GNSS corrects the position when it returns.",
          gnss_denied_init_sec_, use_wheel_speeds_ ? "+wheel" : "");
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "waiting for the first valid gps_pos (last fix type %s)", fixName(last_fix_type_));
      return;
    }
  }
  if (!announced_init_) {
    announced_init_ = true;
    RCLCPP_INFO(
      get_logger(), "EKF running: datum (%.7f, %.7f), first fix %s, yaw %.1f deg (sigma %.1f deg)",
      proj_.originLatDeg(), proj_.originLonDeg(), fixName(last_fix_type_),
      rad2deg(wrapAngle(M_PI / 2.0 - ekf_->core().yaw())), rad2deg(ekf_->core().yawSigma()));
  }
  publishCarState(msg->header.stamp);
}

void SbgRawEkfNode::onGpsPos(const sbg_driver::msg::SbgGpsPos::SharedPtr msg)
{
  last_fix_type_ = msg->status.type;
  last_fix_sats_ = msg->num_sv_used;
  last_fix_valid_ = sbg_raw::gpsPosValid(*msg);
  if (!last_fix_valid_) {
    return;
  }
  if (!proj_.valid()) {
    proj_.setOrigin(msg->latitude, msg->longitude);
    RCLCPP_INFO(
      get_logger(), "datum from first valid fix: (%.7f, %.7f), type %s",
      msg->latitude, msg->longitude, fixName(msg->status.type));
  }
  const double t = dev_clock_.toSeconds(msg->time_stamp);
  last_fix_dev_t_ = t;
  const GpsPosMeas m = sbg_raw::gpsPosMeas(*msg, t, proj_, enu_wire_);
  // Re-acquisition: a big correction after a position gap is a frame snap,
  // not motion -- flag the next odometry sample as invalid for SLAM.
  {
    const RawGnssEkfCore & c = ekf_->core();
    // Only a RE-acquisition counts: the filter must have had a position before
    // (lastPosTime set) and still claim to know where it is (sigma below the
    // SLAM "pose invalid" level) -- a first fix after a GNSS-denied start is
    // initialisation, already published with a huge sigma, not a jump.
    const double prev_sig = std::max(c.posSigmaN(), c.posSigmaE());
    if (c.initialized() && c.lastPosTime() > 0.0 && prev_sig < 10.0 &&
      (c.time() - c.lastPosTime()) > reacq_min_gap_s_)
    {
      const double d = std::hypot(m.N - c.x()[0], m.E - c.x()[1]);
      if (d > reacq_jump_m_) {
        pose_jump_ = true;
        RCLCPP_WARN(
          get_logger(),
          "GNSS re-acquired after %.1f s with a %.2f m position jump: flagging the next odometry "
          "sample invalid (huge sigma) so SLAM re-anchors instead of trusting the snap",
          c.time() - c.lastPosTime(), d);
      }
    }
  }
  ekf_->gpsPos(m);
  publishGnssOdom(msg->header.stamp, m);
}

void SbgRawEkfNode::onGpsVel(const sbg_driver::msg::SbgGpsVel::SharedPtr msg)
{
  if (!sbg_raw::gpsVelValid(*msg)) {
    return;
  }
  const double t = dev_clock_.toSeconds(msg->time_stamp);
  const GpsVelMeas v = sbg_raw::gpsVelMeas(*msg, t, enu_wire_);
  ekf_->gpsVel(v);
  // Online wheel scale (effective rolling radius): |Doppler speed| / raw wheel
  // speed, independent of the filter state. Only while RTK-grade (float or
  // fixed), fast and straight, with a fresh wheel sample; EMA with
  // wheel_scale_tau_s, capped at +-wheel_scale_bound. Frozen otherwise, so a
  // coast dead-reckons on the last learned scale.
  if (wheel_scale_online_ && use_wheel_speeds_ && last_fix_type_ >= 6 && off_init_) {
    const RawGnssEkfCore & c = ekf_->core();
    const double gps_speed = std::hypot(v.vN, v.vE);
    const double wheel_age = now().seconds() - last_wheel_ros_t_;
    const double yaw_rate = std::fabs(c.gyroZ() - c.x()[5]);
    // The Doppler epoch is ~100 ms older than the wheel sample; while
    // accelerating that lag alone is a percent-level ratio error, so only
    // near-constant-speed samples count (|a_x| small).
    const double ax = std::fabs(c.accX() - c.x()[6]);
    if (gps_speed > wheel_scale_min_speed_ && wheel_age < 0.2 && last_wheel_raw_mps_ > 0.5 &&
      yaw_rate < wheel_scale_max_yaw_rate_ && ax < wheel_scale_max_accel_)
    {
      // Doppler speed is the ANTENNA's; in a turn it exceeds the rear-axle
      // centre speed by the lever term (w * antenna_x) -- compensate it.
      const double lever_v = yaw_rate * antenna_offset_x_;
      const double wheel_at_antenna = std::sqrt(
        last_wheel_raw_mps_ * last_wheel_raw_mps_ + lever_v * lever_v);
      const double ratio = gps_speed / wheel_at_antenna;
      // A single sample outside the physically plausible radius band is a
      // timing/slip artefact, not a scale -- discard, never clamp into the estimate.
      if (std::isfinite(ratio) && std::fabs(ratio - 1.0) < wheel_scale_bound_ + 0.03) {
        const double dt = 0.2;   // receiver epoch period
        // Running mean for the first samples, capped so no single sample moves
        // the estimate more than 10 %; then an EMA with wheel_scale_tau_s.
        const double alpha = std::min(
          0.1, std::max(dt / std::max(1.0, wheel_scale_tau_s_),
                        1.0 / static_cast<double>(wheel_scale_samples_ + 1)));
        ++wheel_scale_samples_;
        wheel_scale_acc_ += alpha * (ratio - wheel_scale_acc_);
        wheel_scale_acc_ = std::clamp(wheel_scale_acc_, 1.0 - wheel_scale_bound_, 1.0 + wheel_scale_bound_);
        // Apply only once the estimate is based on enough samples.
        const double before = wheel_scale_est_;
        if (wheel_scale_samples_ >= wheel_scale_min_samples_) {
          wheel_scale_est_ = wheel_scale_acc_;
        }
        if (std::fabs(wheel_scale_est_ - before) > 0.002 || wheel_scale_samples_ == wheel_scale_min_samples_) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "wheel scale estimate %.4f (applied %.4f, n=%lu, gps %.2f / wheel %.2f m/s)",
            wheel_scale_acc_, wheel_scale_est_, static_cast<unsigned long>(wheel_scale_samples_),
            gps_speed, last_wheel_raw_mps_);
        }
      }
    }
  }
}

void SbgRawEkfNode::onGpsHdt(const sbg_driver::msg::SbgGpsHdt::SharedPtr msg)
{
  if (!sbg_raw::gpsHdtValid(*msg)) {
    return;
  }
  const double t = dev_clock_.toSeconds(msg->time_stamp);
  const GpsHdtMeas h = sbg_raw::gpsHdtMeas(*msg, t, enu_wire_);
  {
    const RawGnssEkfCore & c = ekf_->core();
    // Same rule for heading: only after a previous HDT and with an established
    // yaw (sigma < ~30 deg); the first heading after an unknown-yaw init is a
    // legitimate correction, not a jump.
    if (c.initialized() && c.lastHdtTime() > 0.0 && c.yawSigma() < 0.5 &&
      (c.time() - c.lastHdtTime()) > reacq_min_gap_s_)
    {
      const double psi_meas = h.heading - c.params().hdt_offset;
      const double dpsi = std::fabs(wrapAngle(psi_meas - c.yaw()));
      if (dpsi > reacq_jump_yaw_rad_) {
        pose_jump_ = true;
        RCLCPP_WARN(
          get_logger(),
          "heading re-acquired after %.1f s with a %.1f deg jump: flagging the next odometry "
          "sample invalid for SLAM", c.time() - c.lastHdtTime(), rad2deg(dpsi));
      }
    }
  }
  ekf_->gpsHdt(h);
}

void SbgRawEkfNode::onWheelSpeeds(const hyu_msgs::msg::WheelSpeedsStamped::SharedPtr msg)
{
  const auto & w = msg->speeds;
  double v;
  if (wheel_source_ == "all") {
    v = 0.25 * (w.lf_speed + w.rf_speed + w.lb_speed + w.rb_speed);
  } else {
    v = 0.5 * (w.lb_speed + w.rb_speed);  // unsteered rear axle
  }
  last_wheel_raw_mps_ = v;            // before the scale, for the online estimate
  v *= wheel_scale_est_;
  const double t_ros = rclcpp::Time(msg->header.stamp).seconds();
  if (!std::isfinite(v) || !std::isfinite(t_ros)) {
    ++wheel_dropped_;
    return;
  }
  // Health bookkeeping first, so a stale stream is still reported as seen.
  last_wheel_ros_t_ = t_ros;
  last_wheel_mps_ = v;
  // The filter runs on the device clock; the ROS<->device offset is anchored
  // on the IMU stream. No IMU yet -> nothing to fuse into anyway.
  if (!off_init_) {
    ++wheel_dropped_;
    return;
  }
  const double age = now().seconds() - t_ros;
  if (age > wheel_timeout_) {
    ++wheel_dropped_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "wheel speed sample %.2f s old (> wheel_timeout %.2f s): not fused", age, wheel_timeout_);
    return;
  }
  WheelMeas m;
  m.t = t_ros - ros_minus_dev_;
  m.v_fwd = v;
  ekf_->wheel(m);
}

// ---------------------------------------------------------------------------

void SbgRawEkfNode::publishCarState(const builtin_interfaces::msg::Time & stamp)
{
  const double ts = rclcpp::Time(stamp).seconds();
  // graph_slam clears its interpolation buffer on a backward stamp.
  if (ts <= last_pub_t_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "non-monotonic imu stamp (%.3f <= %.3f), not publishing",
      ts, last_pub_t_);
    return;
  }
  const RawGnssEkfCore & c = ekf_->core();
  const RawGnssEkfCore::Vec & x = c.x();
  const double psi = x[4];
  const double yaw_enu = wrapAngle(M_PI / 2.0 - psi);
  const int mode = c.mode();

  hyu_msgs::msg::CarState s;
  s.header.stamp = stamp;
  s.header.frame_id = world_frame_;
  s.child_frame_id = base_frame_;
  // Antenna -> base_footprint: p_base = p_antenna - R(yaw_enu) r (ENU).
  const double cy = std::cos(yaw_enu), sy = std::sin(yaw_enu);
  s.pose.pose.position.x = x[1] - (cy * antenna_offset_x_ - sy * antenna_offset_y_);  // East
  s.pose.pose.position.y = x[0] - (sy * antenna_offset_x_ + cy * antenna_offset_y_);  // North
  s.pose.pose.orientation.z = std::sin(0.5 * yaw_enu);
  s.pose.pose.orientation.w = std::cos(0.5 * yaw_enu);
  double sig_t, sig_y;
  if (blind_gap_ || pose_jump_) {
    sig_t = sig_y = kHugeSigma;
    blind_gap_ = false;
    pose_jump_ = false;
  } else {
    const double tier = mode == RawGnssEkfCore::OK ? odom_sigma_ok_ : odom_sigma_degraded_;
    sig_t = std::max(tier, std::max(c.posSigmaN(), c.posSigmaE()));
    sig_y = c.yawSigma();
  }
  s.pose.covariance[0] = sig_t * sig_t;
  s.pose.covariance[7] = sig_t * sig_t;
  s.pose.covariance[35] = sig_y * sig_y;
  // Body twist (ROS body: x forward, y left, z up). Downstream speed loops
  // read this through ego_odom -- an empty twist reads as v = 0.
  if (c.stationary()) {
    s.twist.twist.linear.x = 0.0;
    s.twist.twist.linear.y = 0.0;
    s.twist.twist.angular.z = 0.0;
  } else {
    const double cp = std::cos(psi), sp = std::sin(psi);
    const double v_fwd = x[2] * cp + x[3] * sp;
    const double v_right = -x[2] * sp + x[3] * cp;
    const double wz = -(c.gyroZ() - x[5]);  // NED CW -> ENU CCW, bias removed
    // Antenna velocity -> base_footprint velocity: v_base = v_ant - w x r.
    s.twist.twist.linear.x = v_fwd + wz * antenna_offset_y_;
    s.twist.twist.linear.y = -v_right - wz * antenna_offset_x_;
    s.twist.twist.angular.z = wz;
  }
  car_state_pub_->publish(s);
  last_pub_t_ = ts;
}

void SbgRawEkfNode::publishGnssOdom(
  const builtin_interfaces::msg::Time & fallback_stamp, const GpsPosMeas & m)
{
  nav_msgs::msg::Odometry o;
  o.header.stamp = rosStamp(m.t, fallback_stamp);
  o.header.frame_id = world_frame_;
  o.child_frame_id = base_frame_;
  o.pose.pose.position.x = m.E;
  o.pose.pose.position.y = m.N;
  const RawGnssEkfCore & c = ekf_->core();
  const double yaw_enu = c.initialized() ? wrapAngle(M_PI / 2.0 - c.yaw()) : 0.0;
  o.pose.pose.orientation.z = std::sin(0.5 * yaw_enu);
  o.pose.pose.orientation.w = std::cos(0.5 * yaw_enu);
  // The driver reports 0.0 accuracy for "not computed".
  const double se = m.accE > 0.0 ? m.accE : kHugeSigma;
  const double sn = m.accN > 0.0 ? m.accN : kHugeSigma;
  const double sy = c.initialized() ? c.yawSigma() : kHugeSigma;
  o.pose.covariance[0] = se * se;
  o.pose.covariance[7] = sn * sn;
  o.pose.covariance[35] = sy * sy;
  gnss_odom_pub_->publish(o);
}

SbgRawEkfNode::StateStyle SbgRawEkfNode::stateStyle() const
{
  const RawGnssEkfCore & c = ekf_->core();
  if (!c.initialized()) {
    return {0.55f, 0.55f, 0.55f, "WAITING FOR FIX"};
  }
  if (imuStale()) {
    return {0.9f, 0.1f, 0.1f, "NO IMU (output stopped)"};
  }
  switch (c.mode()) {
    case RawGnssEkfCore::OK:
      if (c.stationary()) {
        return {0.2f, 0.6f, 0.95f, "STATIONARY (ZUPT)"};
      }
      return {0.1f, 0.8f, 0.2f, "RAW EKF OK"};
    case RawGnssEkfCore::NO_HDT:
      return {0.9f, 0.8f, 0.1f, "NO HDT (gyro/course)"};
    default:
      return {0.95f, 0.5f, 0.1f, "COAST (no GNSS pos)"};
  }
}

void SbgRawEkfNode::onStatusTimer()
{
  const RawGnssEkfCore & c = ekf_->core();
  const bool started = c.initialized();
  const int mode = c.mode();
  const double pos_age = started ? c.time() - c.lastPosTime() : std::nan("");
  const double hdt_age = started ? c.time() - c.lastHdtTime() : std::nan("");
  const StateStyle style = stateStyle();

  // --- diagnostics ------------------------------------------------------------
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "sbg_raw_ekf";
  std::string motion;
  if (!started) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "waiting for first valid gps_pos";
    motion = "none";
  } else if (imuStale()) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message = "fault: no imu_data, odometry output stopped";
    motion = "fault";
  } else if (mode == RawGnssEkfCore::OK) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = c.stationary() ? "stationary: ZUPT" : "raw GNSS EKF tracking";
    motion = c.stationary() ? "zupt" : "raw_ekf";
  } else if (mode == RawGnssEkfCore::NO_HDT) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "degraded: heading unobserved (gyro/course only)";
    motion = "raw_ekf_no_hdt";
  } else {
    st.level = pos_age > 5.0 ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "coasting: no GNSS position for " + fmt("%.1f", pos_age) + " s";
    motion = "coast";
  }
  auto kv = [&st](const std::string & k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue e;
      e.key = k;
      e.value = v;
      st.values.push_back(e);
    };
  kv("mode", std::to_string(mode));
  kv("motion_source", motion);
  kv("fix_type", fixName(last_fix_type_));
  kv("fix_valid", last_fix_valid_ ? "True" : "False");
  kv("sats", std::to_string(last_fix_sats_));
  kv("pos_age", started ? fmt("%.2f", pos_age) : "-");
  kv("hdt_age", started ? fmt("%.2f", hdt_age) : "-");
  kv("sigma_pos", started ? fmt("%.3f", std::max(c.posSigmaN(), c.posSigmaE())) : "-");
  kv("sigma_yaw_deg", started ? fmt("%.2f", rad2deg(c.yawSigma())) : "-");
  kv("stationary", c.stationary() ? "True" : "False");
  kv("hdt_offset_deg", fmt("%.1f", rad2deg(c.params().hdt_offset)));
  kv("antenna_offset_xy", fmt("%.2f", antenna_offset_x_) + "/" + fmt("%.2f", antenna_offset_y_));
  kv("bias_gz_dps", started ? fmt("%.3f", rad2deg(c.x()[5])) : "-");
  // Wheel aiding: off / none yet / stale (bridge silent -> IMU+GNSS only) / fresh.
  {
    std::string wheel;
    double wheel_age = std::nan("");
    if (!use_wheel_speeds_) {
      wheel = "off";
    } else if (!std::isfinite(last_wheel_ros_t_)) {
      wheel = "none";
    } else {
      wheel_age = now().seconds() - last_wheel_ros_t_;
      wheel = wheel_age > wheel_timeout_ ? "stale" : "fresh";
    }
    kv("wheel", wheel);
    kv("wheel_scale", fmt("%.4f", wheel_scale_est_));
    kv("wheel_scale_n", std::to_string(wheel_scale_samples_));
    kv("wheel_age", std::isfinite(wheel_age) ? fmt("%.2f", wheel_age) : "-");
    kv("wheel_mps", std::isfinite(last_wheel_ros_t_) ? fmt("%.2f", last_wheel_mps_) : "-");
    kv(
      "accepted_rejected_wheel", std::to_string(c.accepted()[RawGnssEkfCore::WHEEL]) + "/" +
      std::to_string(c.rejected()[RawGnssEkfCore::WHEEL]));
    kv("wheel_dropped", std::to_string(wheel_dropped_));
  }
  kv(
    "accepted_pos_vel_hdt_crs", std::to_string(c.accepted()[RawGnssEkfCore::POS]) + "/" +
    std::to_string(c.accepted()[RawGnssEkfCore::VEL]) + "/" +
    std::to_string(c.accepted()[RawGnssEkfCore::HDT]) + "/" +
    std::to_string(c.accepted()[RawGnssEkfCore::CRS]));
  kv(
    "rejected_pos_vel_hdt_crs", std::to_string(c.rejected()[RawGnssEkfCore::POS]) + "/" +
    std::to_string(c.rejected()[RawGnssEkfCore::VEL]) + "/" +
    std::to_string(c.rejected()[RawGnssEkfCore::HDT]) + "/" +
    std::to_string(c.rejected()[RawGnssEkfCore::CRS]));
  kv("oosm_replays", std::to_string(ekf_->stats().replays));
  kv("oosm_dropped_stale", std::to_string(ekf_->stats().dropped_stale));
  kv("oosm_max_lag", fmt("%.3f", ekf_->stats().max_lag));
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = now();
  arr.status.push_back(st);
  health_pub_->publish(arr);

  // --- HUD --------------------------------------------------------------------
#ifdef HYU_HAVE_RVIZ_OVERLAY
  if (overlay_pub_) {
    rviz_2d_overlay_msgs::msg::OverlayText ov;
    ov.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    // Sits just below the stack HUD board (hyu_planning_bringup stack_hud.py:
    // x12, y56, height ~195). Keep positions in sync when moving either.
    ov.width = 220;
    ov.height = 96;
    ov.horizontal_distance = 12;
    ov.vertical_distance = 256;
    ov.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    ov.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::TOP;
    ov.bg_color.r = 0.0f;
    ov.bg_color.g = 0.0f;
    ov.bg_color.b = 0.0f;
    ov.bg_color.a = 0.55f;
    ov.fg_color.r = style.r;
    ov.fg_color.g = style.g;
    ov.fg_color.b = style.b;
    ov.fg_color.a = 1.0f;
    ov.line_width = 2;
    ov.text_size = 13.0;
    ov.font = "DejaVu Sans Mono";
    std::ostringstream text;
    text << "GNSS  " << style.word << "\n";
    text << "fix   " << fixName(last_fix_type_);
    if (last_fix_sats_ >= 0 && last_fix_sats_ != 255) {
      text << " " << last_fix_sats_ << "sv";
    }
    text << "\n";
    if (started) {
      text << "sigma " << fmt("%.3f", std::max(c.posSigmaN(), c.posSigmaE())) << " m  yaw "
           << fmt("%.2f", rad2deg(c.yawSigma())) << " deg\n";
      if (hdt_age <= c.params().mode_hdt_age) {
        text << "hdt   age " << fmt("%.1f", hdt_age) << " s  off "
             << fmt("%.0f", rad2deg(c.params().hdt_offset)) << " deg";
      } else {
        text << "hdt   none " << fmt("%.0f", hdt_age) << " s (gyro/course)";
      }
    } else {
      text << "sigma -\nhdt   -";
    }
    ov.text = text.str();
    overlay_pub_->publish(ov);
  }
#endif
}

}  // namespace hyu_localization
