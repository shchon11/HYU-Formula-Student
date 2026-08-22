// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// RawGnssEkf: tracking on synthetic truth, ZUPT at a standstill, mode
// transitions, and -- the property the live node depends on -- that feeding
// the receiver epochs LATE through the out-of-sequence wrapper gives the
// same filter as the time-sorted stream.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "hyu_localization/local_projection.hpp"
#include "hyu_localization/raw_gnss_ekf.hpp"
#include "hyu_localization/sbg_raw_adapters.hpp"

using hyu_localization::deg2rad;
using hyu_localization::GpsHdtMeas;
using hyu_localization::GpsPosMeas;
using hyu_localization::GpsVelMeas;
using hyu_localization::ImuSample;
using hyu_localization::LocalProjection;
using hyu_localization::rad2deg;
using hyu_localization::RawGnssEkf;
using hyu_localization::RawGnssEkfCore;
using hyu_localization::RawGnssEkfParams;
using hyu_localization::wrapAngle;

namespace
{

// Deterministic pseudo-noise (LCG -> approx. normal via 12 uniforms).
class Noise
{
public:
  explicit Noise(std::uint64_t seed)
  : s_(seed) {}
  double uniform()
  {
    s_ = s_ * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<double>(s_ >> 11) / 9007199254740992.0;
  }
  double gauss(double sigma)
  {
    double a = 0.0;
    for (int i = 0; i < 12; ++i) {a += uniform();}
    return (a - 6.0) * sigma;
  }

private:
  std::uint64_t s_;
};

struct Meas
{
  double t;
  int type;  // 0 imu, 1 pos, 2 vel, 3 hdt
  ImuSample imu;
  GpsPosMeas pos;
  GpsVelMeas vel;
  GpsHdtMeas hdt;
};

struct Truth
{
  double t, pN, pE, vN, vE, psi, w;
};

struct Scenario
{
  std::vector<Meas> events;   // time-sorted (imu before epochs at equal t)
  std::vector<Truth> truth;   // one per IMU sample
};

/// Circle of radius R at speed v (CW, psi increasing), gyro bias bg, mount
/// accel leak (bax, bay), noisy GNSS at 5 Hz. `pos_gap`/`hdt_gap` suppress
/// those epochs in [a, b).
Scenario makeCircle(
  double T, double R, double v, double bg, double bax, double bay, bool moving = true,
  double pos_gap_a = -1, double pos_gap_b = -1, double hdt_gap_a = -1, double hdt_gap_b = -1)
{
  Scenario sc;
  Noise nz(12345);
  const double dt = 0.04;
  const double w = moving ? v / R : 0.0;
  const double psi0 = deg2rad(30.0);
  const double hdt_offset = M_PI;
  const int n = static_cast<int>(T / dt);
  int epoch = 0;
  for (int k = 0; k <= n; ++k) {
    const double t = k * dt;
    Truth tr;
    tr.t = t;
    tr.w = w;
    if (moving) {
      tr.psi = wrapAngle(psi0 + w * t);
      tr.pN = (v / w) * std::sin(tr.psi);
      tr.pE = -(v / w) * std::cos(tr.psi);
      tr.vN = v * std::cos(tr.psi);
      tr.vE = v * std::sin(tr.psi);
    } else {
      tr.psi = psi0;
      tr.pN = 3.0;
      tr.pE = -2.0;
      tr.vN = tr.vE = 0.0;
    }
    sc.truth.push_back(tr);
    Meas m;
    m.t = t;
    m.type = 0;
    m.imu.t = t;
    m.imu.gx = nz.gauss(0.001);
    m.imu.gy = nz.gauss(0.001);
    m.imu.gz = w + bg + nz.gauss(0.001);
    m.imu.ax = 0.0 + bax + nz.gauss(0.02);
    m.imu.ay = v * w + bay + nz.gauss(0.02);
    sc.events.push_back(m);
    if (k % 5 == 0) {  // 5 Hz epochs at the IMU instant
      ++epoch;
      if (!(t >= pos_gap_a && t < pos_gap_b)) {
        Meas p;
        p.t = t;
        p.type = 1;
        p.pos.t = t;
        p.pos.N = tr.pN + nz.gauss(0.01);
        p.pos.E = tr.pE + nz.gauss(0.01);
        p.pos.accN = p.pos.accE = 0.01;
        sc.events.push_back(p);
      }
      Meas vm;
      vm.t = t;
      vm.type = 2;
      vm.vel.t = t;
      vm.vel.vN = tr.vN + nz.gauss(0.03);
      vm.vel.vE = tr.vE + nz.gauss(0.03);
      vm.vel.accN = vm.vel.accE = 0.03;
      vm.vel.course = std::atan2(tr.vE, tr.vN);
      sc.events.push_back(vm);
      if (!(t >= hdt_gap_a && t < hdt_gap_b)) {
        Meas h;
        h.t = t;
        h.type = 3;
        h.hdt.t = t;
        h.hdt.heading = wrapAngle(tr.psi - hdt_offset + nz.gauss(deg2rad(0.3)));
        h.hdt.acc = deg2rad(0.3);
        sc.events.push_back(h);
      }
    }
  }
  return sc;
}

void feed(RawGnssEkfCore & f, const Meas & m)
{
  switch (m.type) {
    case 0: f.imu(m.imu); break;
    case 1: f.gpsPos(m.pos); break;
    case 2: f.gpsVel(m.vel); break;
    default: f.gpsHdt(m.hdt); break;
  }
}
void feed(RawGnssEkf & f, const Meas & m)
{
  switch (m.type) {
    case 0: f.imu(m.imu); break;
    case 1: f.gpsPos(m.pos); break;
    case 2: f.gpsVel(m.vel); break;
    default: f.gpsHdt(m.hdt); break;
  }
}

}  // namespace

TEST(RawGnssEkf, StandstillConvergesAndZupts)
{
  const Scenario sc = makeCircle(30.0, 20.0, 0.0, 0.005, 0.02, -0.01, false);
  RawGnssEkfCore f;
  for (const Meas & m : sc.events) {
    feed(f, m);
  }
  ASSERT_TRUE(f.initialized());
  EXPECT_NEAR(f.x()[0], 3.0, 0.05);
  EXPECT_NEAR(f.x()[1], -2.0, 0.05);
  EXPECT_NEAR(rad2deg(wrapAngle(f.x()[4] - deg2rad(30.0))), 0.0, 0.5);
  EXPECT_TRUE(f.stationary());
  EXPECT_EQ(f.mode(), RawGnssEkfCore::OK);
  EXPECT_GT(f.accepted()[RawGnssEkfCore::ZUPT], 100);
  EXPECT_NEAR(f.x()[5], 0.005, 0.002);  // ZARU pins the gyro bias
  EXPECT_LT(std::hypot(f.x()[2], f.x()[3]), 0.02);
}

TEST(RawGnssEkf, CircleTracksTruth)
{
  const Scenario sc = makeCircle(60.0, 20.0, 5.0, 0.01, 0.15, -0.10);
  RawGnssEkfCore f;
  std::size_t ti = 0;
  double se_pos = 0.0, se_yaw = 0.0;
  int n = 0;
  for (const Meas & m : sc.events) {
    feed(f, m);
    if (m.type == 0) {
      const Truth & tr = sc.truth[ti++];
      if (f.initialized() && tr.t > 30.0) {
        se_pos += std::pow(f.x()[0] - tr.pN, 2) + std::pow(f.x()[1] - tr.pE, 2);
        se_yaw += std::pow(wrapAngle(f.x()[4] - tr.psi), 2);
        ++n;
      }
    }
  }
  ASSERT_GT(n, 100);
  EXPECT_LT(std::sqrt(se_pos / n), 0.10);
  EXPECT_LT(rad2deg(std::sqrt(se_yaw / n)), 1.0);
  EXPECT_NEAR(f.x()[5], 0.01, 0.004);
  EXPECT_FALSE(f.stationary());
  EXPECT_EQ(f.mode(), RawGnssEkfCore::OK);
  // A few epochs may be gated while the yaw/bias estimates settle after
  // init (yaw sigma starts at 2 deg, accel leak unknown); never many.
  EXPECT_LT(f.rejected()[RawGnssEkfCore::POS], f.accepted()[RawGnssEkfCore::POS] / 10);
}

TEST(RawGnssEkf, LateEpochsReplayToTheSortedResult)
{
  const Scenario sc = makeCircle(40.0, 20.0, 5.0, 0.01, 0.15, -0.10);
  RawGnssEkfCore sorted;
  RawGnssEkf live(RawGnssEkfParams(), 1.0);
  // Live order: epochs reach the node ~90 ms after the IMU frame of the same
  // device time, i.e. after two more IMU samples.
  const double lag = 0.09;
  std::vector<Meas> pending;
  for (const Meas & m : sc.events) {
    feed(sorted, m);
    if (m.type == 0) {
      // deliver epochs whose arrival time has come
      std::vector<Meas> keep;
      for (const Meas & p : pending) {
        if (p.t + lag <= m.t) {feed(live, p);} else {keep.push_back(p);}
      }
      pending.swap(keep);
      feed(live, m);
    } else {
      pending.push_back(m);
    }
  }
  for (const Meas & p : pending) {
    feed(live, p);
  }
  ASSERT_TRUE(sorted.initialized());
  ASSERT_TRUE(live.core().initialized());
  EXPECT_GT(live.stats().replays, 100u);
  EXPECT_EQ(live.stats().dropped_stale, 0u);
  for (int i = 0; i < RawGnssEkfCore::NX; ++i) {
    EXPECT_NEAR(live.core().x()[i], sorted.x()[i], 1e-9) << "state " << i;
  }
  EXPECT_LT((live.core().P() - sorted.P()).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_EQ(live.core().accepted(), sorted.accepted());
  EXPECT_EQ(live.core().rejected(), sorted.rejected());
}

TEST(RawGnssEkf, StaleEventsAreDroppedNotApplied)
{
  const Scenario sc = makeCircle(10.0, 20.0, 5.0, 0.0, 0.0, 0.0);
  RawGnssEkf live(RawGnssEkfParams(), 0.5);
  for (const Meas & m : sc.events) {
    feed(live, m);
  }
  GpsPosMeas old;
  old.t = 2.0;  // 8 s in the past: far beyond the 0.5 s buffer
  old.N = 1000.0;
  old.E = 1000.0;
  old.accN = old.accE = 0.01;
  const auto before = live.core().x();
  live.gpsPos(old);
  EXPECT_EQ(live.stats().dropped_stale, 1u);
  EXPECT_EQ(live.core().x()[0], before[0]);
}

TEST(RawGnssEkf, ModeFollowsPositionAndHeadingAges)
{
  // pos gap 20-25 s -> COAST; hdt gap 35-40 s -> NO_HDT; recovers after both.
  const Scenario sc = makeCircle(50.0, 20.0, 5.0, 0.01, 0.0, 0.0, true, 20.0, 25.0, 35.0, 40.0);
  RawGnssEkfCore f;
  int mode_at_23 = -1, mode_at_30 = -1, mode_at_38 = -1, mode_at_48 = -1;
  for (const Meas & m : sc.events) {
    feed(f, m);
    if (m.type != 0) {continue;}
    if (std::fabs(m.t - 23.0) < 1e-6) {mode_at_23 = f.mode();}
    if (std::fabs(m.t - 30.0) < 1e-6) {mode_at_30 = f.mode();}
    if (std::fabs(m.t - 38.0) < 1e-6) {mode_at_38 = f.mode();}
    if (std::fabs(m.t - 48.0) < 1e-6) {mode_at_48 = f.mode();}
  }
  EXPECT_EQ(mode_at_23, RawGnssEkfCore::COAST);
  EXPECT_EQ(mode_at_30, RawGnssEkfCore::OK);
  EXPECT_EQ(mode_at_38, RawGnssEkfCore::NO_HDT);
  EXPECT_EQ(mode_at_48, RawGnssEkfCore::OK);
  // The coast was short: still tracking within a metre at its end.
  EXPECT_TRUE(f.initialized());
}

TEST(DeviceClock, WrapsAndLateEpochs)
{
  hyu_localization::sbg_raw::DeviceClock clk;
  const std::uint32_t near_wrap = 0xFFFFFFFFu - 500000u;  // 0.5 s before wrap
  const double t0 = clk.toSeconds(near_wrap);
  const double t1 = clk.toSeconds(near_wrap + 400000u);   // +0.4 s
  EXPECT_NEAR(t1 - t0, 0.4, 1e-9);
  const double t2 = clk.toSeconds(static_cast<std::uint32_t>(near_wrap + 900000u));  // wrapped
  EXPECT_NEAR(t2 - t0, 0.9, 1e-9);
  // an epoch stamped 90 ms before the latest IMU resolves to the past
  const double te = clk.toSeconds(static_cast<std::uint32_t>(near_wrap + 810000u));
  EXPECT_NEAR(te - t0, 0.81, 1e-9);
  // and does not move the reference backwards
  const double t3 = clk.toSeconds(static_cast<std::uint32_t>(near_wrap + 940000u));
  EXPECT_NEAR(t3 - t0, 0.94, 1e-9);
}

TEST(LocalProjection, RoundTripAndScale)
{
  LocalProjection wgs(false), sph(true);
  wgs.setOrigin(37.2971, 126.8351);
  sph.setOrigin(37.2971, 126.8351);
  double N, E, lat, lon;
  wgs.toNE(37.2980, 126.8360, N, E);
  wgs.toLatLon(N, E, lat, lon);
  EXPECT_NEAR(lat, 37.2980, 1e-9);
  EXPECT_NEAR(lon, 126.8360, 1e-9);
  double Ns, Es;
  sph.toNE(37.2980, 126.8360, Ns, Es);
  // spherical R=6378137 overstates N by ~0.3 % at 37 deg vs the meridional radius
  EXPECT_NEAR(Ns / N, 1.003, 0.002);
  EXPECT_NEAR(Es / E, 0.9987, 0.002);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// ---------------------------------------------------------------------------
// Wheel-speed aiding (optional input from the ECU encoder bridge).

namespace
{

using hyu_localization::WheelMeas;

/// Feed a scenario into a core, optionally injecting a wheel sample right
/// after every IMU sample (the bridge publishes at the ECU rate, ~IMU rate)
/// and optionally dropping the gps_vel epochs from t >= vel_gap_from.
void runWithWheels(
  RawGnssEkfCore & f, const Scenario & sc, bool wheels, double wheel_bias,
  double vel_gap_from, std::vector<double> * speed_err = nullptr, double err_from = 0.0)
{
  std::size_t ti = 0;
  for (const Meas & m : sc.events) {
    if (m.type == 2 && m.t >= vel_gap_from) {
      continue;
    }
    feed(f, m);
    if (m.type == 0) {
      const Truth & tr = sc.truth[ti++];
      if (wheels) {
        WheelMeas w;
        w.t = m.t;
        w.v_fwd = std::hypot(tr.vN, tr.vE) + wheel_bias;
        f.wheel(w);
      }
      if (speed_err && f.initialized() && tr.t >= err_from) {
        const double v_est = std::hypot(f.x()[2], f.x()[3]);
        speed_err->push_back(v_est - std::hypot(tr.vN, tr.vE));
      }
    }
  }
}

double rms(const std::vector<double> & v)
{
  double s = 0.0;
  for (double e : v) {s += e * e;}
  return v.empty() ? 0.0 : std::sqrt(s / static_cast<double>(v.size()));
}

}  // namespace

TEST(RawGnssEkfWheel, NoWheelEventsLeaveTheFilterBitIdentical)
{
  // The whole hand-over logic is "a wheel sample came or it did not": with
  // none, the filter must be the IMU+GNSS one, state for state.
  const Scenario sc = makeCircle(30.0, 20.0, 5.0, 0.01, 0.15, -0.10);
  RawGnssEkfCore plain, wheel_capable;
  for (const Meas & m : sc.events) {
    feed(plain, m);
    feed(wheel_capable, m);
  }
  ASSERT_TRUE(plain.initialized());
  for (int i = 0; i < RawGnssEkfCore::NX; ++i) {
    EXPECT_EQ(plain.x()[i], wheel_capable.x()[i]) << "state " << i;
  }
  EXPECT_EQ(plain.accepted(), wheel_capable.accepted());
  EXPECT_EQ(wheel_capable.accepted()[RawGnssEkfCore::WHEEL], 0);
}

TEST(RawGnssEkfWheel, WheelsHoldTheSpeedThroughAGpsVelOutage)
{
  // gps_vel stops at 20 s (pos + hdt keep coming): without wheels the speed
  // rides on IMU integration between position fixes, with wheels it is
  // observed at the IMU rate. Wheels must cut the speed error clearly and
  // be accepted, not gated.
  const Scenario sc = makeCircle(60.0, 20.0, 5.0, 0.01, 0.15, -0.10);
  RawGnssEkfCore without, with;
  std::vector<double> e_without, e_with;
  runWithWheels(without, sc, false, 0.0, 20.0, &e_without, 25.0);
  runWithWheels(with, sc, true, 0.0, 20.0, &e_with, 25.0);
  ASSERT_GT(e_with.size(), 100u);
  EXPECT_LT(rms(e_with), 0.05);
  EXPECT_LT(rms(e_with), 0.5 * rms(e_without) + 1e-9);
  EXPECT_GT(with.accepted()[RawGnssEkfCore::WHEEL], 800);
  EXPECT_LT(with.rejected()[RawGnssEkfCore::WHEEL], with.accepted()[RawGnssEkfCore::WHEEL] / 20);
  EXPECT_EQ(with.mode(), RawGnssEkfCore::OK);
}

TEST(RawGnssEkfWheel, FreshMovingWheelsVetoTheZupt)
{
  // IMU says still (standstill scenario), GPS speed 0 -- but the wheels read
  // 3 m/s: the filter must NOT declare a ZUPT (wheels only veto).
  const Scenario sc = makeCircle(20.0, 20.0, 0.0, 0.005, 0.02, -0.01, false);
  RawGnssEkfCore f;
  int zupt_before_wheels = -1;
  for (const Meas & m : sc.events) {
    feed(f, m);
    if (m.type == 0 && m.t >= 10.0) {
      if (zupt_before_wheels < 0) {
        zupt_before_wheels = f.accepted()[RawGnssEkfCore::ZUPT];
      }
      WheelMeas w;
      w.t = m.t;
      w.v_fwd = 3.0;
      f.wheel(w);
    }
  }
  ASSERT_TRUE(f.initialized());
  EXPECT_GT(zupt_before_wheels, 50);  // it was zupting before the wheels spoke
  EXPECT_FALSE(f.stationary());
  // Every IMU sample after 10 s was vetoed: the ZUPT count froze (the one
  // sample at exactly 10.0 s was judged before its wheel arrived).
  EXPECT_LE(f.accepted()[RawGnssEkfCore::ZUPT] - zupt_before_wheels, 1);
}

TEST(RawGnssEkfWheel, StaleWheelsDoNotVetoTheZupt)
{
  // A wheel sample older than zupt_wheel_age is no evidence either way.
  const Scenario sc = makeCircle(20.0, 20.0, 0.0, 0.005, 0.02, -0.01, false);
  RawGnssEkfCore f;
  for (const Meas & m : sc.events) {
    feed(f, m);
    if (m.type == 0 && std::fabs(m.t - 10.0) < 1e-6) {
      WheelMeas w;
      w.t = m.t;
      w.v_fwd = 3.0;  // one stray sample, then silence
      f.wheel(w);
    }
  }
  EXPECT_TRUE(f.stationary());
}

TEST(RawGnssEkfWheel, WildSampleIsGatedAndSoftReacquired)
{
  const Scenario sc = makeCircle(30.0, 20.0, 5.0, 0.01, 0.15, -0.10);
  RawGnssEkfCore f;
  runWithWheels(f, sc, true, 0.0, 1e9);
  ASSERT_TRUE(f.initialized());
  const int acc0 = f.accepted()[RawGnssEkfCore::WHEEL];
  const int rej0 = f.rejected()[RawGnssEkfCore::WHEEL];
  const double v0 = std::hypot(f.x()[2], f.x()[3]);
  WheelMeas wild;
  wild.t = f.time() + 0.04;
  wild.v_fwd = 40.0;  // 35 m/s off: far outside the gate
  f.wheel(wild);
  EXPECT_EQ(f.rejected()[RawGnssEkfCore::WHEEL], rej0 + 1);
  EXPECT_EQ(f.accepted()[RawGnssEkfCore::WHEEL], acc0);
  EXPECT_NEAR(std::hypot(f.x()[2], f.x()[3]), v0, 0.05);
  // After max_rej_wheel consecutive rejects the next one is taken with R*9
  // (soft re-acquire), exactly like the GNSS channels.
  for (int i = 0; i < f.params().max_rej_wheel; ++i) {
    wild.t += 0.04;
    f.wheel(wild);
  }
  EXPECT_EQ(f.accepted()[RawGnssEkfCore::WHEEL], acc0 + 1);
}

TEST(RawGnssEkfWheel, LateWheelEventsReplayToTheSortedResult)
{
  const Scenario sc = makeCircle(30.0, 20.0, 5.0, 0.01, 0.15, -0.10);
  RawGnssEkfCore sorted;
  RawGnssEkf live(RawGnssEkfParams(), 1.0);
  // Live: wheel samples reach the node 60 ms after the IMU frame of the same
  // time (ECU -> UDP -> bridge), i.e. after one or two more IMU samples.
  const double lag = 0.06;
  std::vector<WheelMeas> pending;
  std::size_t ti = 0;
  // The wrapper orders equal-time events IMU < POS < VEL < HDT < WHEEL, so
  // the sorted reference applies a wheel sample after the epochs of its time.
  WheelMeas sorted_pending;
  bool has_sorted_pending = false;
  for (const Meas & m : sc.events) {
    if (has_sorted_pending && m.t > sorted_pending.t) {
      sorted.wheel(sorted_pending);
      has_sorted_pending = false;
    }
    feed(sorted, m);
    if (m.type == 0) {
      const Truth & tr = sc.truth[ti++];
      WheelMeas w;
      w.t = m.t;
      w.v_fwd = std::hypot(tr.vN, tr.vE);
      sorted_pending = w;
      has_sorted_pending = true;
      std::vector<WheelMeas> keep;
      for (const WheelMeas & p : pending) {
        if (p.t + lag <= m.t) {live.wheel(p);} else {keep.push_back(p);}
      }
      pending.swap(keep);
      feed(live, m);
      pending.push_back(w);
    } else {
      feed(live, m);
    }
  }
  for (const WheelMeas & p : pending) {
    live.wheel(p);
  }
  if (has_sorted_pending) {
    sorted.wheel(sorted_pending);
  }
  ASSERT_TRUE(live.core().initialized());
  EXPECT_GT(live.stats().replays, 100u);
  for (int i = 0; i < RawGnssEkfCore::NX; ++i) {
    EXPECT_NEAR(live.core().x()[i], sorted.x()[i], 1e-9) << "state " << i;
  }
  EXPECT_EQ(live.core().accepted(), sorted.accepted());
}
