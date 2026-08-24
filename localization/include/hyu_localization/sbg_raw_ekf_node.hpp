// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// SBG Ellipse-D raw GNSS/IMU -> odometry node (replaces sbg_odometry_bridge).
//
// Reads ONLY the receiver/IMU outputs -- /sbg/imu_data, /sbg/gps_pos,
// /sbg/gps_vel, /sbg/gps_hdt -- and runs RawGnssEkf (the port of the offline
// my_ekf.py reference) on them. The device's own EKF (ekf_nav / ekf_euler) is
// not used at all: on 2026-08-01 it re-initialised mid-run three times while
// the raw receiver kept an RTK fix, and the offline raw-EKF scored better than
// anything built on the device solution.
//
// Publishes (same contract as the retired bridge, see docs/topic_contract.md):
//   car_state_topic  hyu_msgs/CarState   ENU pose + body twist of
//                    base_footprint (the antenna solution moved by
//                    antenna_offset_x/y), 1 per IMU sample (25 Hz). pose.covariance[0/7] = sigma_t^2 with
//                    sigma_t = max(tier, EKF position sigma); [35] = yaw var.
//                    First message after an IMU gap > blind_gap_sec carries
//                    sigma 1e3 (graph_slam's "pose invalid" marker).
//   gnss_odom_topic  nav_msgs/Odometry   raw fix in ENU with reported sigma.
//   health_topic     diagnostic_msgs/DiagnosticArray  mode / ages / counters.
//   overlay_topic    rviz_2d_overlay_msgs/OverlayText HUD (if built with it).
//
// Frames: the EKF works in NED (psi 0 = North, CW); output is ROS ENU
// (x East, y North, yaw 0 = East, CCW). frame_convention must match the
// driver's output.use_enu (false -> "ned", the vehicle default).

#ifndef HYU_LOCALIZATION__SBG_RAW_EKF_NODE_HPP_
#define HYU_LOCALIZATION__SBG_RAW_EKF_NODE_HPP_

#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hyu_msgs/msg/car_state.hpp"
#include "hyu_msgs/msg/wheel_speeds_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sbg_driver/msg/sbg_gps_hdt.hpp"
#include "sbg_driver/msg/sbg_gps_pos.hpp"
#include "sbg_driver/msg/sbg_gps_vel.hpp"
#include "sbg_driver/msg/sbg_imu_data.hpp"
#ifdef HYU_HAVE_RVIZ_OVERLAY
#include "rviz_2d_overlay_msgs/msg/overlay_text.hpp"
#endif

#include "hyu_localization/local_projection.hpp"
#include "hyu_localization/raw_gnss_ekf.hpp"
#include "hyu_localization/sbg_raw_adapters.hpp"

namespace hyu_localization
{

class SbgRawEkfNode : public rclcpp::Node
{
public:
  SbgRawEkfNode();

private:
  void onImu(const sbg_driver::msg::SbgImuData::SharedPtr msg);
  void onGpsPos(const sbg_driver::msg::SbgGpsPos::SharedPtr msg);
  void onGpsVel(const sbg_driver::msg::SbgGpsVel::SharedPtr msg);
  void onGpsHdt(const sbg_driver::msg::SbgGpsHdt::SharedPtr msg);
  void onWheelSpeeds(const hyu_msgs::msg::WheelSpeedsStamped::SharedPtr msg);
  void publishCarState(const builtin_interfaces::msg::Time & stamp);
  void publishGnssOdom(const builtin_interfaces::msg::Time & fallback_stamp, const GpsPosMeas & m);
  void onStatusTimer();
  /// ROS stamp for a device time (through the IMU-anchored offset), or the
  /// message's own header stamp before the offset is known.
  builtin_interfaces::msg::Time rosStamp(
    double t_dev, const builtin_interfaces::msg::Time & fallback) const;
  bool imuStale() const;
  /// Drop every buffer and start over (replay loop / sim reset).
  void restart(const char * why);
  struct StateStyle
  {
    float r, g, b;
    std::string word;
  };
  StateStyle stateStyle() const;

  // Parameters.
  std::string car_state_topic_, gnss_odom_topic_, health_topic_, overlay_topic_;
  std::string imu_topic_, gps_pos_topic_, gps_vel_topic_, gps_hdt_topic_;
  std::string world_frame_, base_frame_;
  bool enu_wire_ = false;
  bool publish_overlay_ = true;
  double antenna_offset_x_ = 1.25;  // primary antenna in base_footprint [m], x fwd
  double antenna_offset_y_ = 0.0;   // ... y left
  // Wheel speeds (drive_udp_bridge, m/s) -- optional velocity aiding.
  std::string wheel_speeds_topic_;
  bool use_wheel_speeds_ = true;
  std::string wheel_source_ = "rear";   // rear | all
  double wheel_scale_ = 1.0;            // effective-radius correction (RTK parity)
  double wheel_timeout_ = 0.3;          // [s] older samples are not fused
  double odom_sigma_ok_ = 0.05;
  double odom_sigma_degraded_ = 0.20;
  double blind_gap_sec_ = 0.5;
  double imu_timeout_ = 1.0;
  double status_period_ = 0.2;

  // State.
  LocalProjection proj_;
  RawGnssEkfParams ekf_params_;
  double history_sec_ = 1.0;
  std::unique_ptr<RawGnssEkf> ekf_;
  sbg_raw::DeviceClock dev_clock_;
  bool off_init_ = false;
  double ros_minus_dev_ = 0.0;
  bool have_last_imu_ = false;
  double last_imu_dev_t_ = 0.0;
  double last_imu_ros_t_ = 0.0;
  bool blind_gap_ = false;
  // Re-acquisition jump (권고 2): the first odometry sample after a large
  // GNSS/HDT correction that follows a gap is published with kHugeSigma so
  // graph_slam treats the snap as "pose unknown" (a free edge, re-anchored by
  // cone observations) instead of a trusted 5 cm motion.
  bool pose_jump_ = false;
  double reacq_min_gap_s_ = 3.0;       // a gap shorter than this is a gating streak, not an outage
  double reacq_jump_m_ = 0.5;          // position innovation that flags a jump
  double reacq_jump_yaw_rad_ = 0.1745; // heading innovation (10 deg) that flags a jump
  // Online wheel-scale estimate (권고 1): |gps_vel| / raw wheel speed, EMA while
  // RTK-grade, fast, straight. Applied to every wheel sample; frozen in coast.
  bool wheel_scale_online_ = true;
  double wheel_scale_tau_s_ = 45.0;
  double wheel_scale_min_speed_ = 2.0;
  double wheel_scale_max_yaw_rate_ = 0.25;   // [rad/s] lever-compensated, so mild turns count
  double wheel_scale_bound_ = 0.05;          // |scale - 1| cap
  double wheel_scale_max_accel_ = 0.3;       // [m/s^2] Doppler lag makes accelerating samples biased
  std::uint64_t wheel_scale_min_samples_ = 20;  // apply only after this many samples
  double wheel_scale_acc_ = 1.0;             // running estimate (applied once n >= min_samples)
  double wheel_scale_est_ = 1.0;             // starts at wheel_scale_
  double last_wheel_raw_mps_ = 0.0;          // before scale, for the ratio
  std::uint64_t wheel_scale_samples_ = 0;
  double last_pub_t_ = -std::numeric_limits<double>::infinity();
  bool announced_init_ = false;
  // GNSS-denied cold start.
  bool allow_gnss_denied_init_ = false;
  double gnss_denied_init_sec_ = 2.0;
  double gnss_denied_init_pos_sig_ = 100.0;
  bool have_first_imu_ = false;
  double first_imu_ros_t_ = 0.0;
  bool denied_init_announced_ = false;
  // Latest raw fix for the HUD / health.
  int last_fix_type_ = -1;
  int last_fix_sats_ = -1;
  bool last_fix_valid_ = false;
  double last_fix_dev_t_ = -1e9;
  // Latest wheel sample for the health report.
  double last_wheel_ros_t_ = -std::numeric_limits<double>::infinity();
  double last_wheel_mps_ = 0.0;
  std::uint64_t wheel_dropped_ = 0;  // stale / non-finite / before the clock offset

  rclcpp::Publisher<hyu_msgs::msg::CarState>::SharedPtr car_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gnss_odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
#ifdef HYU_HAVE_RVIZ_OVERLAY
  rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr overlay_pub_;
#endif
  rclcpp::Subscription<sbg_driver::msg::SbgImuData>::SharedPtr imu_sub_;
  rclcpp::Subscription<sbg_driver::msg::SbgGpsPos>::SharedPtr gps_pos_sub_;
  rclcpp::Subscription<sbg_driver::msg::SbgGpsVel>::SharedPtr gps_vel_sub_;
  rclcpp::Subscription<sbg_driver::msg::SbgGpsHdt>::SharedPtr gps_hdt_sub_;
  rclcpp::Subscription<hyu_msgs::msg::WheelSpeedsStamped>::SharedPtr wheel_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace hyu_localization

#endif  // HYU_LOCALIZATION__SBG_RAW_EKF_NODE_HPP_
