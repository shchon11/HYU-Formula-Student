// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// 2D odometry EKF built from the Ellipse-D's RAW receiver/IMU outputs only
// (no ekf_nav / ekf_euler). Port of the offline reference my_ekf.py that was
// scored on the 2026-08-01 bags; the maths here is kept 1:1 with it so the
// offline results carry over (see scripts/ docs and the bag eval tool).
//
//   state x = [pN, pE, vN, vE, psi, b_gz, b_ax, b_ay]
//   NED: pN/pE metres from the datum, psi 0 = North, clockwise positive.
//   propagate: gyro.z -> psi, body accel x (forward) / y (right) rotated by
//              psi -> vN/vE -> pN/pE. b_ax/b_ay absorb mount tilt / gravity
//              leak, b_gz the gyro bias.
//   correct:   gps_pos (N,E)       R = acc^2 + floor^2 + (v*timing)^2
//              gps_vel (vN,vE)     R = acc^2 + floor^2
//              gps_hdt psi = heading + hdt_offset (antenna order), R = acc^2
//              gps_vel.course      only without HDT for crs_hdt_gap s, v>1 m/s
//              wheel v_fwd         rear-axle mean speed from the ECU encoders
//                                  (drive_udp_bridge, m/s): body-x speed of the
//                                  antenna = vN cos psi + vE sin psi (+ w*lever
//                                  if the antenna sits off the centreline),
//                                  R = sig_wheel^2 + (wheel_sig_per_acc*|a_x|)^2
//                                  so traction/braking slip is trusted less.
//                                  OPTIONAL: with no wheel events the filter is
//                                  exactly the IMU+GNSS one above.
//   ZUPT/ZARU: IMU window still (gyro std/mean, accel std) + GPS speed ~0 ->
//              v=(0,0) and gyro.z = b_gz observations every IMU sample.
//              A fresh wheel speed above zupt_wheel_speed vetoes it.
//   gating:    Mahalanobis chi^2; after max_rej consecutive rejects the next
//              sample is taken with R*9 (soft re-acquire).
//   mode:      200 OK / 201 NO_HDT (heading unobserved) / 202 COAST (no pos).
//
// RawGnssEkfCore is the in-order filter (events must arrive in time order,
// like the offline script's sorted stream). RawGnssEkf wraps it with a short
// event buffer so late measurements -- the receiver epochs reach ROS ~90 ms
// after the IMU frame of the same device time -- are applied at their true
// time by rewinding and replaying, without delaying the output.

#ifndef HYU_LOCALIZATION__RAW_GNSS_EKF_HPP_
#define HYU_LOCALIZATION__RAW_GNSS_EKF_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>

namespace hyu_localization
{

inline double deg2rad(double d) {return d * M_PI / 180.0;}
inline double rad2deg(double r) {return r * 180.0 / M_PI;}
/// Wrap to [-pi, pi).
inline double wrapAngle(double a)
{
  return std::fmod(std::fmod(a + M_PI, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI) - M_PI;
}

struct RawGnssEkfParams
{
  // Continuous-time process noise: white accel / gyro [m/s^2/sqrt(Hz)],
  // [rad/s/sqrt(Hz)] and bias random walks [/sqrt(s)].
  double sig_gyro = deg2rad(0.3);
  double sig_acc = 0.3;
  double sig_bg_rw = deg2rad(0.02);
  double sig_ba_rw = 0.05;
  // ZUPT detector on a window of IMU samples (0.5 s @ 25 Hz).
  int zupt_window = 12;
  double zupt_gyro_std = deg2rad(0.2);   // rms of per-axis gyro std
  double zupt_gyro_mean = deg2rad(0.5);  // |mean gyro| (slow turn is not still)
  double zupt_acc_std = 0.06;            // std of horizontal |accel|
  double zupt_gps_speed = 0.1;           // GPS speed must be < max(this, 2 sigma_v)
  double zupt_vel_age = 3.0;             // ... when a gps_vel is this recent;
                                         // else IMU thresholds are halved
  double zupt_sig_v = 0.02;              // v = 0 observation sigma [m/s]
  double zaru_sig = deg2rad(0.1);        // gyro.z = b_gz observation sigma
  // Measurement noise floors.
  double pos_floor = 0.02;     // [m]
  double pos_timing = 0.005;   // [s] x speed -> extra pos sigma
  double vel_floor = 0.03;     // [m/s]
  // Mahalanobis gates and consecutive-reject counts before soft re-acquire.
  double gate_pos = 20.0;
  double gate_vel = 20.0;
  double gate_hdt = 12.0;
  double gate_crs = 12.0;
  int max_rej_pos = 10;
  int max_rej_vel = 10;
  int max_rej_hdt = 25;
  double soft_reacquire_r_scale = 9.0;
  // Dual-antenna heading -> vehicle heading [rad] (antenna order; 180 deg
  // with the rear-secondary installation) and acceptance limits.
  double hdt_offset = M_PI;
  double hdt_max_acc = deg2rad(10.0);
  double hdt_min_sig = deg2rad(0.2);
  double hdt_init_min_sig = deg2rad(2.0);
  // Course-over-ground fallback (no HDT for crs_hdt_gap s, speed > min).
  double crs_hdt_gap = 3.0;
  double crs_min_speed = 1.0;
  double crs_max_innov = deg2rad(120.0);
  double crs_p_unlock = deg2rad(60.0);  // ... or if yaw sigma is beyond this
  double crs_sig_min = deg2rad(5.0);
  double crs_sig_per_speed = deg2rad(10.0);  // sigma = max(min, this / v)
  // Prediction step clamp [s] (a long gap advances at most this much motion).
  double max_dt = 0.2;
  // Mode thresholds.
  double mode_pos_age = 1.0;
  double mode_hdt_age = 2.0;
  double mode_yaw_sig = deg2rad(3.0);
  // Initial yaw sigma without an HDT at init.
  double init_psi_sig_unknown = M_PI;
  // Wheel-speed observation (rear-axle mean, m/s) -- optional input.
  double sig_wheel = 0.08;            // base 1-sigma [m/s] (encoder quantisation + tyre)
  double wheel_sig_per_acc = 0.02;    // [s]: extra sigma = this * |a_x| (traction/braking slip)
  double gate_wheel = 12.0;           // Mahalanobis gate (1 dof)
  int max_rej_wheel = 10;             // consecutive rejects before soft re-acquire
  double wheel_lever_y_right = 0.0;   // antenna lateral offset from the centreline, NED body (+right) [m]
  // A fresh (< zupt_wheel_age old) wheel speed above this vetoes the ZUPT.
  double zupt_wheel_speed = 0.05;     // [m/s]
  double zupt_wheel_age = 0.3;        // [s]
};

/// IMU sample in the NED body frame (x forward, y right, z down).
struct ImuSample
{
  double t = 0.0;   // device time [s]
  double gx = 0.0, gy = 0.0, gz = 0.0;  // [rad/s]
  double ax = 0.0, ay = 0.0;            // [m/s^2]
};
/// Doppler velocity (N, E) with 1-sigma and NED course (0 = North, CW) [rad].
struct GpsVelMeas
{
  double t = 0.0;
  double vN = 0.0, vE = 0.0;
  double accN = 0.0, accE = 0.0;
  double course = 0.0;
};
/// Dual-antenna heading as reported (NED, before the offset) [rad] + 1-sigma.
struct GpsHdtMeas
{
  double t = 0.0;
  double heading = 0.0;
  double acc = 0.0;
};
/// Receiver position projected to N/E metres from the datum + 1-sigma.
struct GpsPosMeas
{
  double t = 0.0;
  double N = 0.0, E = 0.0;
  double accN = 0.0, accE = 0.0;
};
/// Wheel-encoder speed: longitudinal speed of the vehicle centreline at the
/// (unsteered) rear axle, m/s, signed (reverse < 0), already tyre-scaled.
struct WheelMeas
{
  double t = 0.0;   // device time [s] (the node maps the ROS stamp)
  double v_fwd = 0.0;
};

class RawGnssEkfCore
{
public:
  static constexpr int NX = 8;
  using Vec = Eigen::Matrix<double, NX, 1>;
  using Mat = Eigen::Matrix<double, NX, NX>;
  enum Kind {POS = 0, VEL, HDT, CRS, ZUPT, WHEEL, NKIND};
  enum Mode {NOT_STARTED = 0, OK = 200, NO_HDT = 201, COAST = 202};

  explicit RawGnssEkfCore(const RawGnssEkfParams & p = RawGnssEkfParams());

  // Events, in time order. Validity (solution status etc.) is the caller's.
  void imu(const ImuSample & s);
  void gpsVel(const GpsVelMeas & m);
  void gpsHdt(const GpsHdtMeas & m);
  void gpsPos(const GpsPosMeas & m);
  void wheel(const WheelMeas & m);

  const RawGnssEkfParams & params() const {return p_;}
  bool initialized() const {return inited_;}
  double time() const {return t_;}
  const Vec & x() const {return x_;}
  const Mat & P() const {return P_;}
  double yaw() const {return x_[4];}
  double yawSigma() const {return std::sqrt(std::max(0.0, P_(4, 4)));}
  double posSigmaN() const {return std::sqrt(std::max(0.0, P_(0, 0)));}
  double posSigmaE() const {return std::sqrt(std::max(0.0, P_(1, 1)));}
  /// ZUPT decision of the latest IMU sample.
  bool stationary() const {return stationary_;}
  double lastImuTime() const {return last_imu_t_;}
  double lastPosTime() const {return last_pos_t_;}
  double lastHdtTime() const {return last_hdt_t_;}
  double lastGpsSpeed() const {return last_gps_speed_;}
  /// Latest wheel sample (device time / m/s); -1e9 before any.
  double lastWheelTime() const {return last_wheel_t_;}
  double lastWheelSpeed() const {return last_wheel_speed_;}
  /// Latest gyro.z input (NED body, CW positive) [rad/s].
  double gyroZ() const {return gyro_z_;}
  /// Latest body-x specific force input [m/s^2] (bias not removed).
  double accX() const {return acc_x_;}
  int mode() const;
  const std::array<int, NKIND> & accepted() const {return n_acc_;}
  const std::array<int, NKIND> & rejected() const {return n_rej_;}
  // GNSS-denied cold start: init at the datum origin with a large position
  // sigma (pending HDT heading if any), no position fix recorded.
  void initAtOrigin(double t, double pos_sig = 100.0);

private:
  void init(double t, double pN, double pE, double vN, double vE, double psi, double psi_sig,
            double pos_sig = 1.0);

  void predict(double t_to);
  template<int M>
  bool update(
    const Eigen::Matrix<double, M, 1> & z, const Eigen::Matrix<double, M, NX> & H,
    Eigen::Matrix<double, M, M> R, double gate, Kind kind, bool angular, int max_rej);
  void pushWindow(const ImuSample & s);
  bool windowStill(double t) const;

  RawGnssEkfParams p_;
  Vec x_ = Vec::Zero();
  Mat P_ = Mat::Identity();
  double t_ = 0.0;
  bool inited_ = false;
  // Inputs held for the next prediction (set AFTER predicting to the sample).
  double gyro_z_ = 0.0, acc_x_ = 0.0, acc_y_ = 0.0;
  double last_imu_t_ = -1e9;
  double last_pos_t_ = -1e9, last_hdt_t_ = -1e9;
  double last_gps_speed_ = 0.0, last_vel_t_ = -1e9, last_vel_acc_ = 1.0;
  double last_wheel_t_ = -1e9, last_wheel_speed_ = 0.0;
  bool has_pending_vel_ = false, has_pending_psi_ = false;
  double pending_vN_ = 0.0, pending_vE_ = 0.0, pending_psi_ = 0.0, pending_psi_sig_ = 0.0;
  bool stationary_ = false;
  std::array<int, NKIND> rej_run_{}, n_acc_{}, n_rej_{};
  // ZUPT window ring: gx, gy, gz, |a_xy| per sample.
  static constexpr int kWindowCap = 64;
  std::array<std::array<double, 4>, kWindowCap> win_{};
  int win_head_ = 0, win_count_ = 0;
};

/// Out-of-sequence tolerant front end around RawGnssEkfCore: events are kept
/// in a time-ordered buffer of `history_sec`; a late event is inserted at its
/// place and the filter is rewound to the snapshot before it and replayed.
/// Events older than the buffer are dropped (counted).
class RawGnssEkf
{
public:
  explicit RawGnssEkf(const RawGnssEkfParams & p = RawGnssEkfParams(), double history_sec = 1.0);

  void imu(const ImuSample & s);
  void gpsVel(const GpsVelMeas & m);
  void gpsHdt(const GpsHdtMeas & m);
  void gpsPos(const GpsPosMeas & m);
  void wheel(const WheelMeas & m);
  void initAtOrigin(double t, double pos_sig = 100.0);

  const RawGnssEkfCore & core() const {return core_;}
  struct Stats
  {
    std::uint64_t replays = 0;          // late events that triggered a rewind
    std::uint64_t replayed_events = 0;  // events re-applied in those rewinds
    std::uint64_t dropped_stale = 0;    // events older than the buffer
    double max_lag = 0.0;               // largest (latest_t - event_t) replayed
  };
  const Stats & stats() const {return stats_;}
  double historySec() const {return history_sec_;}

private:
  enum Type {T_IMU = 0, T_POS = 1, T_VEL = 2, T_HDT = 3, T_WHEEL = 4};  // = sort priority
  struct Event
  {
    double t;
    int type;
    ImuSample imu;
    GpsPosMeas pos;
    GpsVelMeas vel;
    GpsHdtMeas hdt;
    WheelMeas wheel;
    RawGnssEkfCore after;  // filter state once this event was applied
  };
  static bool before(double ta, int pa, double tb, int pb)
  {
    return ta < tb || (ta == tb && pa < pb);
  }
  void apply(const Event & e);
  void push(Event && e);

  RawGnssEkfCore core_;
  RawGnssEkfCore checkpoint_;  // state before events_.front()
  std::deque<Event> events_;
  double history_sec_;
  bool have_trimmed_ = false;
  double trimmed_t_ = -1e9;
  int trimmed_type_ = 0;
  Stats stats_;
};

}  // namespace hyu_localization

#endif  // HYU_LOCALIZATION__RAW_GNSS_EKF_HPP_
