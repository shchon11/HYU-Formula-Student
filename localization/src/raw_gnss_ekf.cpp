// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "hyu_localization/raw_gnss_ekf.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace hyu_localization
{

namespace
{
constexpr double kNoGate = 1.0e9;  // ZUPT/ZARU are never gated
}  // namespace

RawGnssEkfCore::RawGnssEkfCore(const RawGnssEkfParams & p)
: p_(p)
{
  p_.zupt_window = std::clamp(p_.zupt_window, 2, kWindowCap);
  rej_run_.fill(0);
  n_acc_.fill(0);
  n_rej_.fill(0);
}

void RawGnssEkfCore::init(
  double t, double pN, double pE, double vN, double vE, double psi, double psi_sig)
{
  x_ << pN, pE, vN, vE, psi, 0.0, 0.0, 0.0;
  P_.setZero();
  P_(0, 0) = 1.0;
  P_(1, 1) = 1.0;
  P_(2, 2) = 0.5 * 0.5;
  P_(3, 3) = 0.5 * 0.5;
  P_(4, 4) = psi_sig * psi_sig;
  P_(5, 5) = deg2rad(0.5) * deg2rad(0.5);
  P_(6, 6) = 1.5 * 1.5;
  P_(7, 7) = 1.5 * 1.5;
  t_ = t;
  inited_ = true;
}

void RawGnssEkfCore::predict(double t_to)
{
  double dt = t_to - t_;
  if (dt <= 0.0) {
    return;
  }
  dt = std::min(dt, p_.max_dt);
  const double pN = x_[0], pE = x_[1], vN = x_[2], vE = x_[3], psi = x_[4];
  const double bg = x_[5], bax = x_[6], bay = x_[7];
  const double w = gyro_z_ - bg;
  const double ax = acc_x_ - bax, ay = acc_y_ - bay;
  const double c = std::cos(psi), s = std::sin(psi);
  const double aN = c * ax - s * ay;
  const double aE = s * ax + c * ay;
  // state
  x_[0] = pN + vN * dt + 0.5 * aN * dt * dt;
  x_[1] = pE + vE * dt + 0.5 * aE * dt * dt;
  x_[2] = vN + aN * dt;
  x_[3] = vE + aE * dt;
  x_[4] = wrapAngle(psi + w * dt);
  // Jacobian
  Mat F = Mat::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;
  const double daN_dpsi = -aE, daE_dpsi = aN;
  F(0, 4) = 0.5 * dt * dt * daN_dpsi;
  F(1, 4) = 0.5 * dt * dt * daE_dpsi;
  F(2, 4) = dt * daN_dpsi;
  F(3, 4) = dt * daE_dpsi;
  // d a / d b_a: aN = c(ax-bax) - s(ay-bay) -> -c, +s ; aE = s(ax-bax) + c(ay-bay) -> -s, -c
  F(2, 6) = -c * dt;
  F(2, 7) = s * dt;
  F(3, 6) = -s * dt;
  F(3, 7) = -c * dt;
  F(0, 6) = -c * 0.5 * dt * dt;
  F(0, 7) = s * 0.5 * dt * dt;
  F(1, 6) = -s * 0.5 * dt * dt;
  F(1, 7) = -c * 0.5 * dt * dt;
  F(4, 5) = -dt;
  const double qa = p_.sig_acc * p_.sig_acc;     // PSD [m^2/s^3]
  const double qg = p_.sig_gyro * p_.sig_gyro;   // PSD [rad^2/s]
  Mat Q = Mat::Zero();
  Q(0, 0) = Q(1, 1) = qa * dt * dt * dt / 3.0;
  Q(0, 2) = Q(2, 0) = Q(1, 3) = Q(3, 1) = qa * dt * dt / 2.0;
  Q(2, 2) = Q(3, 3) = qa * dt;
  Q(4, 4) = qg * dt;
  Q(5, 5) = p_.sig_bg_rw * p_.sig_bg_rw * dt;
  Q(6, 6) = Q(7, 7) = p_.sig_ba_rw * p_.sig_ba_rw * dt;
  P_ = F * P_ * F.transpose() + Q;
  t_ = t_to;
}

template<int M>
bool RawGnssEkfCore::update(
  const Eigen::Matrix<double, M, 1> & z, const Eigen::Matrix<double, M, NX> & H,
  Eigen::Matrix<double, M, M> R, double gate, Kind kind, bool angular, int max_rej)
{
  Eigen::Matrix<double, M, 1> nu = z - H * x_;
  if (angular) {
    nu[0] = wrapAngle(nu[0]);
  }
  Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
  Eigen::Matrix<double, M, M> Si = S.inverse();
  if (!Si.allFinite()) {
    return false;
  }
  const double d2 = nu.dot(Si * nu);
  if (d2 > gate) {
    rej_run_[kind] += 1;
    if (rej_run_[kind] < max_rej) {
      n_rej_[kind] += 1;
      return false;
    }
    // soft re-acquire: take this one with an inflated R
    R *= p_.soft_reacquire_r_scale;
    S = H * P_ * H.transpose() + R;
    Si = S.inverse();
    if (!Si.allFinite()) {
      return false;
    }
  }
  rej_run_[kind] = 0;
  const Eigen::Matrix<double, NX, M> K = P_ * H.transpose() * Si;
  x_ = x_ + K * nu;
  x_[4] = wrapAngle(x_[4]);
  const Mat I_KH = Mat::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();  // Joseph form
  n_acc_[kind] += 1;
  return true;
}

void RawGnssEkfCore::pushWindow(const ImuSample & s)
{
  const int W = p_.zupt_window;
  win_[win_head_] = {s.gx, s.gy, s.gz, std::hypot(s.ax, s.ay)};
  win_head_ = (win_head_ + 1) % W;
  win_count_ = std::min(win_count_ + 1, W);
}

bool RawGnssEkfCore::windowStill(double t) const
{
  const int W = p_.zupt_window;
  if (win_count_ < W) {
    return false;
  }
  // gstd = sqrt(mean over axes of the per-axis population variance),
  // gmean = |mean gyro|, astd = population std of |a_xy|  (as my_ekf.py).
  double mg[3] = {0.0, 0.0, 0.0}, ma = 0.0;
  for (int i = 0; i < W; ++i) {
    mg[0] += win_[i][0];
    mg[1] += win_[i][1];
    mg[2] += win_[i][2];
    ma += win_[i][3];
  }
  for (double & v : mg) {
    v /= W;
  }
  ma /= W;
  double vg = 0.0, va = 0.0;
  for (int i = 0; i < W; ++i) {
    for (int k = 0; k < 3; ++k) {
      const double d = win_[i][k] - mg[k];
      vg += d * d;
    }
    const double da = win_[i][3] - ma;
    va += da * da;
  }
  const double gstd = std::sqrt(vg / (3.0 * W));
  const double astd = std::sqrt(va / W);
  const double gmean = std::sqrt(mg[0] * mg[0] + mg[1] * mg[1] + mg[2] * mg[2]);
  bool still = gstd < p_.zupt_gyro_std && astd < p_.zupt_acc_std && gmean < p_.zupt_gyro_mean;
  if (!still) {
    return false;
  }
  if ((t - last_vel_t_) < p_.zupt_vel_age) {
    // A recent GPS velocity must agree (RTK ~0.1 m/s, SINGLE ~2 sigma).
    if (last_gps_speed_ > std::max(p_.zupt_gps_speed, 2.0 * last_vel_acc_)) {
      still = false;
    }
  } else {
    // GNSS outage: IMU alone has to be twice as convincing.
    still = gstd < 0.5 * p_.zupt_gyro_std && astd < 0.5 * p_.zupt_acc_std &&
      gmean < 0.5 * p_.zupt_gyro_mean;
  }
  return still;
}

void RawGnssEkfCore::imu(const ImuSample & s)
{
  if (inited_) {
    predict(s.t);
  }
  // Inputs for the NEXT interval (the reference sets them after predicting).
  gyro_z_ = s.gz;
  acc_x_ = s.ax;
  acc_y_ = s.ay;
  last_imu_t_ = s.t;
  pushWindow(s);
  stationary_ = windowStill(s.t);
  if (inited_ && stationary_) {
    Eigen::Matrix<double, 2, NX> Hv = Eigen::Matrix<double, 2, NX>::Zero();
    Hv(0, 2) = 1.0;
    Hv(1, 3) = 1.0;
    Eigen::Matrix<double, 2, 2> Rv = Eigen::Matrix<double, 2, 2>::Zero();
    Rv(0, 0) = Rv(1, 1) = p_.zupt_sig_v * p_.zupt_sig_v;
    update<2>(Eigen::Vector2d::Zero(), Hv, Rv, kNoGate, ZUPT, false, 1);
    Eigen::Matrix<double, 1, NX> Hz = Eigen::Matrix<double, 1, NX>::Zero();
    Hz(0, 5) = 1.0;  // ZARU: gyro.z = b_gz
    Eigen::Matrix<double, 1, 1> Rz;
    Rz(0, 0) = p_.zaru_sig * p_.zaru_sig;
    Eigen::Matrix<double, 1, 1> zz;
    zz(0, 0) = s.gz;
    update<1>(zz, Hz, Rz, kNoGate, ZUPT, false, 1);
  }
}

void RawGnssEkfCore::gpsVel(const GpsVelMeas & m)
{
  last_gps_speed_ = std::hypot(m.vN, m.vE);
  last_vel_t_ = m.t;
  last_vel_acc_ = std::max(m.accN, m.accE);
  const double sN = std::sqrt(m.accN * m.accN + p_.vel_floor * p_.vel_floor);
  const double sE = std::sqrt(m.accE * m.accE + p_.vel_floor * p_.vel_floor);
  if (!inited_) {
    pending_vN_ = m.vN;
    pending_vE_ = m.vE;
    has_pending_vel_ = true;
    return;
  }
  predict(m.t);
  Eigen::Matrix<double, 2, NX> Hv = Eigen::Matrix<double, 2, NX>::Zero();
  Hv(0, 2) = 1.0;
  Hv(1, 3) = 1.0;
  Eigen::Matrix<double, 2, 2> Rv = Eigen::Matrix<double, 2, 2>::Zero();
  Rv(0, 0) = sN * sN;
  Rv(1, 1) = sE * sE;
  update<2>(Eigen::Vector2d(m.vN, m.vE), Hv, Rv, p_.gate_vel, VEL, false, p_.max_rej_vel);
  // Course fallback: only while the HDT has been absent and the car moves.
  if ((m.t - last_hdt_t_) > p_.crs_hdt_gap && last_gps_speed_ > p_.crs_min_speed) {
    const double crs = m.course;
    if (std::fabs(wrapAngle(crs - x_[4])) < p_.crs_max_innov ||
      P_(4, 4) > p_.crs_p_unlock * p_.crs_p_unlock)
    {
      const double sig = std::max(p_.crs_sig_min, p_.crs_sig_per_speed / last_gps_speed_);
      Eigen::Matrix<double, 1, NX> Hh = Eigen::Matrix<double, 1, NX>::Zero();
      Hh(0, 4) = 1.0;
      Eigen::Matrix<double, 1, 1> R;
      R(0, 0) = sig * sig;
      Eigen::Matrix<double, 1, 1> z;
      z(0, 0) = crs;
      update<1>(z, Hh, R, p_.gate_crs, CRS, true, std::numeric_limits<int>::max());
    }
  }
}

void RawGnssEkfCore::gpsHdt(const GpsHdtMeas & m)
{
  if (!(m.acc > 0.0 && m.acc < p_.hdt_max_acc)) {
    return;
  }
  const double psi = wrapAngle(m.heading + p_.hdt_offset);
  const double sig = std::max(m.acc, p_.hdt_min_sig);
  if (!inited_) {
    pending_psi_ = psi;
    pending_psi_sig_ = sig;
    has_pending_psi_ = true;
    return;
  }
  predict(m.t);
  Eigen::Matrix<double, 1, NX> Hh = Eigen::Matrix<double, 1, NX>::Zero();
  Hh(0, 4) = 1.0;
  Eigen::Matrix<double, 1, 1> R;
  R(0, 0) = sig * sig;
  Eigen::Matrix<double, 1, 1> z;
  z(0, 0) = psi;
  if (update<1>(z, Hh, R, p_.gate_hdt, HDT, true, p_.max_rej_hdt)) {
    last_hdt_t_ = m.t;
  }
}

void RawGnssEkfCore::gpsPos(const GpsPosMeas & m)
{
  const double spd_now = inited_ ? std::hypot(x_[2], x_[3]) : 0.0;
  const double extra = p_.pos_floor * p_.pos_floor + (spd_now * p_.pos_timing) *
    (spd_now * p_.pos_timing);
  const double sN = std::sqrt(m.accN * m.accN + extra);
  const double sE = std::sqrt(m.accE * m.accE + extra);
  if (!inited_) {
    const double vN = has_pending_vel_ ? pending_vN_ : 0.0;
    const double vE = has_pending_vel_ ? pending_vE_ : 0.0;
    if (has_pending_psi_) {
      init(m.t, m.N, m.E, vN, vE, pending_psi_, std::max(pending_psi_sig_, p_.hdt_init_min_sig));
    } else {
      init(m.t, m.N, m.E, vN, vE, 0.0, p_.init_psi_sig_unknown);
    }
    P_(0, 0) = sN * sN;
    P_(1, 1) = sE * sE;
    last_pos_t_ = m.t;
    if (has_pending_psi_) {
      last_hdt_t_ = m.t;
    }
    return;
  }
  predict(m.t);
  Eigen::Matrix<double, 2, NX> Hp = Eigen::Matrix<double, 2, NX>::Zero();
  Hp(0, 0) = 1.0;
  Hp(1, 1) = 1.0;
  Eigen::Matrix<double, 2, 2> Rp = Eigen::Matrix<double, 2, 2>::Zero();
  Rp(0, 0) = sN * sN;
  Rp(1, 1) = sE * sE;
  if (update<2>(Eigen::Vector2d(m.N, m.E), Hp, Rp, p_.gate_pos, POS, false, p_.max_rej_pos)) {
    last_pos_t_ = m.t;
  }
}

int RawGnssEkfCore::mode() const
{
  if (!inited_) {
    return NOT_STARTED;
  }
  const double pos_age = t_ - last_pos_t_;
  const double hdt_age = t_ - last_hdt_t_;
  if (pos_age > p_.mode_pos_age) {
    return COAST;
  }
  if (hdt_age > p_.mode_hdt_age || yawSigma() > p_.mode_yaw_sig) {
    return NO_HDT;
  }
  return OK;
}

// ---------------------------------------------------------------------------

RawGnssEkf::RawGnssEkf(const RawGnssEkfParams & p, double history_sec)
: core_(p), checkpoint_(p), history_sec_(std::max(0.05, history_sec))
{
}

void RawGnssEkf::apply(const Event & e)
{
  switch (e.type) {
    case T_IMU: core_.imu(e.imu); break;
    case T_POS: core_.gpsPos(e.pos); break;
    case T_VEL: core_.gpsVel(e.vel); break;
    case T_HDT: core_.gpsHdt(e.hdt); break;
    default: break;
  }
}

void RawGnssEkf::push(Event && e)
{
  if (have_trimmed_ && before(e.t, e.type, trimmed_t_, trimmed_type_)) {
    stats_.dropped_stale += 1;
    return;
  }
  std::size_t i = events_.size();
  while (i > 0 && before(e.t, e.type, events_[i - 1].t, events_[i - 1].type)) {
    --i;
  }
  if (i == events_.size()) {
    apply(e);
    e.after = core_;
    events_.push_back(std::move(e));
  } else {
    stats_.replays += 1;
    stats_.max_lag = std::max(stats_.max_lag, events_.back().t - e.t);
    core_ = (i == 0) ? checkpoint_ : events_[i - 1].after;
    events_.insert(events_.begin() + static_cast<std::ptrdiff_t>(i), std::move(e));
    for (std::size_t j = i; j < events_.size(); ++j) {
      apply(events_[j]);
      events_[j].after = core_;
      if (j > i) {
        stats_.replayed_events += 1;
      }
    }
  }
  const double latest = events_.back().t;
  while (events_.size() > 1 && events_.front().t < latest - history_sec_) {
    checkpoint_ = events_.front().after;
    trimmed_t_ = events_.front().t;
    trimmed_type_ = events_.front().type;
    have_trimmed_ = true;
    events_.pop_front();
  }
}

void RawGnssEkf::imu(const ImuSample & s)
{
  Event e{s.t, T_IMU, s, {}, {}, {}, core_};
  push(std::move(e));
}
void RawGnssEkf::gpsVel(const GpsVelMeas & m)
{
  Event e{m.t, T_VEL, {}, {}, m, {}, core_};
  push(std::move(e));
}
void RawGnssEkf::gpsHdt(const GpsHdtMeas & m)
{
  Event e{m.t, T_HDT, {}, {}, {}, m, core_};
  push(std::move(e));
}
void RawGnssEkf::gpsPos(const GpsPosMeas & m)
{
  Event e{m.t, T_POS, {}, m, {}, {}, core_};
  push(std::move(e));
}

}  // namespace hyu_localization
