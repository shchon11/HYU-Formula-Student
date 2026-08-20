// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// sbg_driver raw messages -> RawGnssEkf measurements. One place for the
// validity rules and the wire-convention handling (driver output.use_enu),
// shared by the live node and the bag evaluation tool.
//
// Validity (as the offline reference my_ekf.py):
//   gps_pos  status.status == SOL_COMPUTED && status.type >= SINGLE && lat/lon != 0
//   gps_vel  status.vel_status == SOL_COMPUTED
//   gps_hdt  (status & 0x3F) == SOL_COMPUTED   (bit 6 = baseline flag)
//            and 0 < true_heading_acc < hdt_max_acc (checked in the filter)

#ifndef HYU_LOCALIZATION__SBG_RAW_ADAPTERS_HPP_
#define HYU_LOCALIZATION__SBG_RAW_ADAPTERS_HPP_

#include <cmath>

#include "hyu_localization/local_projection.hpp"
#include "hyu_localization/raw_gnss_ekf.hpp"
#include "sbg_driver/msg/sbg_gps_hdt.hpp"
#include "sbg_driver/msg/sbg_gps_pos.hpp"
#include "sbg_driver/msg/sbg_gps_vel.hpp"
#include "sbg_driver/msg/sbg_imu_data.hpp"

namespace hyu_localization
{
namespace sbg_raw
{

constexpr unsigned kSolComputed = 0;
constexpr unsigned kPosTypeSingle = 2;
constexpr unsigned kHdtStatusMask = 0x3F;

/// Body-frame IMU sample in NED body (x fwd, y right, z down). With the
/// driver in ENU mode (x fwd, y left, z up) the y/z axes are flipped back.
inline ImuSample imuSample(const sbg_driver::msg::SbgImuData & m, double t, bool enu_wire)
{
  ImuSample s;
  s.t = t;
  s.gx = m.gyro.x;
  s.gy = enu_wire ? -m.gyro.y : m.gyro.y;
  s.gz = enu_wire ? -m.gyro.z : m.gyro.z;
  s.ax = m.accel.x;
  s.ay = enu_wire ? -m.accel.y : m.accel.y;
  return s;
}

inline bool gpsPosValid(const sbg_driver::msg::SbgGpsPos & m)
{
  return m.status.status == kSolComputed && m.status.type >= kPosTypeSingle &&
         !(m.latitude == 0.0 && m.longitude == 0.0);
}
inline bool gpsVelValid(const sbg_driver::msg::SbgGpsVel & m)
{
  return m.status.vel_status == kSolComputed;
}
inline bool gpsHdtValid(const sbg_driver::msg::SbgGpsHdt & m)
{
  return (m.status & kHdtStatusMask) == kSolComputed;
}

inline GpsPosMeas gpsPosMeas(
  const sbg_driver::msg::SbgGpsPos & m, double t, const LocalProjection & proj, bool enu_wire)
{
  GpsPosMeas g;
  g.t = t;
  proj.toNE(m.latitude, m.longitude, g.N, g.E);
  g.accN = enu_wire ? m.position_accuracy.y : m.position_accuracy.x;
  g.accE = enu_wire ? m.position_accuracy.x : m.position_accuracy.y;
  return g;
}

inline GpsVelMeas gpsVelMeas(const sbg_driver::msg::SbgGpsVel & m, double t, bool enu_wire)
{
  GpsVelMeas v;
  v.t = t;
  if (enu_wire) {
    v.vN = m.velocity.y;
    v.vE = m.velocity.x;
    v.accN = m.velocity_accuracy.y;
    v.accE = m.velocity_accuracy.x;
    v.course = deg2rad(90.0 - m.course);
  } else {
    v.vN = m.velocity.x;
    v.vE = m.velocity.y;
    v.accN = m.velocity_accuracy.x;
    v.accE = m.velocity_accuracy.y;
    v.course = deg2rad(m.course);
  }
  return v;
}

inline GpsHdtMeas gpsHdtMeas(const sbg_driver::msg::SbgGpsHdt & m, double t, bool enu_wire)
{
  GpsHdtMeas h;
  h.t = t;
  h.heading = deg2rad(enu_wire ? 90.0 - m.true_heading : m.true_heading);
  h.acc = deg2rad(m.true_heading_acc);
  return h;
}

/// Device clock (uint32 microseconds, wraps every 71.6 min) -> continuous
/// seconds. The reference advances to the latest stamp seen; any stamp is
/// resolved relative to it with a signed 32-bit difference, so the receiver
/// epochs (tens of ms behind the IMU frame) and a wrap are both handled.
class DeviceClock
{
public:
  double toSeconds(std::uint32_t ts)
  {
    if (!init_) {
      init_ = true;
      ref_ts_ = ts;
      ref_t_ = static_cast<double>(ts) * 1e-6;
      return ref_t_;
    }
    std::int64_t d = static_cast<std::int64_t>(ts) - static_cast<std::int64_t>(ref_ts_);
    if (d >= (1LL << 31)) {
      d -= (1LL << 32);
    } else if (d < -(1LL << 31)) {
      d += (1LL << 32);
    }
    const double t = ref_t_ + static_cast<double>(d) * 1e-6;
    if (t > ref_t_) {
      ref_ts_ = ts;
      ref_t_ = t;
    }
    return t;
  }
  bool initialized() const {return init_;}

private:
  bool init_ = false;
  std::uint32_t ref_ts_ = 0;
  double ref_t_ = 0.0;
};

}  // namespace sbg_raw
}  // namespace hyu_localization

#endif  // HYU_LOCALIZATION__SBG_RAW_ADAPTERS_HPP_
