// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// Offline replay of a recorded bag's /sbg/imu_data + gps_pos/gps_vel/gps_hdt
// through RawGnssEkf, scored against the raw fixes. The C++ twin of the
// offline reference my_ekf.py (same CSV columns, same summary numbers), so a
// run of both on the same bag shows whether the port is faithful.
//
//   ros2 run hyu_localization sbg_raw_ekf_bag_eval <bag_dir> [--out x.csv]
//        [--order device|receipt] [--hdt-offset 180] [--sphere] [--enu]
//        [--history 1.0]
//
//   --order device   (default) events sorted by device time, fed to the
//                    in-order core exactly as the reference script does
//   --order receipt  bag receipt order through the out-of-sequence wrapper,
//                    i.e. what the live node sees (epochs ~90 ms late)
//   --sphere         spherical N/E projection (reference script) instead of
//                    WGS84 radii (the node)
//
// Time axis: device time_stamp (us, unwrapped) shifted by the median
// (bag_time - device_time) over the IMU stream, as the reference does.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "hyu_localization/local_projection.hpp"
#include "hyu_localization/raw_gnss_ekf.hpp"
#include "hyu_localization/sbg_raw_adapters.hpp"

using hyu_localization::GpsHdtMeas;
using hyu_localization::GpsPosMeas;
using hyu_localization::GpsVelMeas;
using hyu_localization::ImuSample;
using hyu_localization::LocalProjection;
using hyu_localization::RawGnssEkf;
using hyu_localization::RawGnssEkfCore;
using hyu_localization::RawGnssEkfParams;
using hyu_localization::wrapAngle;
namespace sbg_raw = hyu_localization::sbg_raw;

namespace
{

struct Row
{
  double t, pN, pE, vN, vE, yaw, sN, sE, svN, svE, syaw, bgz, bax, bay;
  int mode;
  double pos_age, hdt_age;
  int zupt;
};

template<typename T>
struct Stamped
{
  double bag_t;
  double dev_t;
  T msg;
};

/// uint32 us device stamps -> unwrapped seconds (per topic, as my_ekf.py).
template<typename T>
void unwrapDeviceTimes(std::vector<Stamped<T>> & v)
{
  double wraps = 0.0;
  double prev = 0.0;
  for (std::size_t i = 0; i < v.size(); ++i) {
    double t = static_cast<double>(v[i].msg.time_stamp) * 1e-6;
    if (i > 0 && t - prev < -1000.0) {
      wraps += 4294.967296;
    }
    prev = t;
    v[i].dev_t = t + wraps;
  }
}

template<typename T>
T deserialize(const rosbag2_storage::SerializedBagMessage & m)
{
  rclcpp::SerializedMessage ser(*m.serialized_data);
  rclcpp::Serialization<T> s;
  T out;
  s.deserialize_message(&ser, &out);
  return out;
}

struct Event
{
  double t;      // filter time
  double bag_t;  // receipt time
  int prio;      // 0 imu, 1 pos, 2 vel, 3 hdt
  std::size_t idx;
};

std::size_t nearest(const std::vector<Row> & rows, double t)
{
  auto it = std::lower_bound(
    rows.begin(), rows.end(), t, [](const Row & r, double tt) {return r.t < tt;});
  if (it == rows.begin()) {return 0;}
  if (it == rows.end()) {return rows.size() - 1;}
  const std::size_t j = static_cast<std::size_t>(it - rows.begin());
  return std::fabs(rows[j - 1].t - t) < std::fabs(rows[j].t - t) ? j - 1 : j;
}

std::string fmtLag(double v)
{
  char b[32];
  std::snprintf(b, sizeof(b), "%.3f", v);
  return b;
}

void usage()
{
  std::cerr << "usage: sbg_raw_ekf_bag_eval <bag_dir> [--out file.csv] [--order device|receipt]\n"
            << "       [--hdt-offset DEG] [--sphere] [--enu] [--history SEC] [--quiet]\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    usage();
    return 2;
  }
  std::string bag = argv[1];
  std::string out_csv;
  std::string order = "device";
  double hdt_offset_deg = 180.0;
  bool sphere = false, enu = false, quiet = false;
  double history = 1.0;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc) {usage(); std::exit(2);}
        return argv[++i];
      };
    if (a == "--out") {out_csv = next();} else if (a == "--order") {order = next();} else if (a ==
      "--hdt-offset")
    {
      hdt_offset_deg = std::stod(next());
    } else if (a == "--sphere") {sphere = true;} else if (a == "--enu") {enu = true;} else if (a ==
      "--history")
    {
      history = std::stod(next());
    } else if (a == "--quiet") {quiet = true;} else {usage(); return 2;}
  }
  if (order != "device" && order != "receipt") {
    usage();
    return 2;
  }

  // --- read ---------------------------------------------------------------------
  const std::string kImu = "/sbg/imu_data", kPos = "/sbg/gps_pos", kVel = "/sbg/gps_vel",
    kHdt = "/sbg/gps_hdt";
  std::vector<Stamped<sbg_driver::msg::SbgImuData>> imus;
  std::vector<Stamped<sbg_driver::msg::SbgGpsPos>> poss;
  std::vector<Stamped<sbg_driver::msg::SbgGpsVel>> vels;
  std::vector<Stamped<sbg_driver::msg::SbgGpsHdt>> hdts;
  {
    rosbag2_storage::StorageOptions so;
    so.uri = bag;
    so.storage_id = "sqlite3";
    rosbag2_cpp::ConverterOptions co;
    co.input_serialization_format = "cdr";
    co.output_serialization_format = "cdr";
    rosbag2_cpp::Reader reader;
    reader.open(so, co);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {kImu, kPos, kVel, kHdt};
    reader.set_filter(filter);
    while (reader.has_next()) {
      auto m = reader.read_next();
      const double bt = static_cast<double>(m->time_stamp) * 1e-9;
      if (m->topic_name == kImu) {
        imus.push_back({bt, 0.0, deserialize<sbg_driver::msg::SbgImuData>(*m)});
      } else if (m->topic_name == kPos) {
        poss.push_back({bt, 0.0, deserialize<sbg_driver::msg::SbgGpsPos>(*m)});
      } else if (m->topic_name == kVel) {
        vels.push_back({bt, 0.0, deserialize<sbg_driver::msg::SbgGpsVel>(*m)});
      } else if (m->topic_name == kHdt) {
        hdts.push_back({bt, 0.0, deserialize<sbg_driver::msg::SbgGpsHdt>(*m)});
      }
    }
  }
  if (imus.empty()) {
    std::cerr << "no /sbg/imu_data in " << bag << "\n";
    return 1;
  }
  unwrapDeviceTimes(imus);
  unwrapDeviceTimes(poss);
  unwrapDeviceTimes(vels);
  unwrapDeviceTimes(hdts);
  // bag-equivalent time axis: median(bag_t - dev_t) over the IMU
  std::vector<double> offs;
  offs.reserve(imus.size());
  for (const auto & s : imus) {
    offs.push_back(s.bag_t - s.dev_t);
  }
  std::nth_element(offs.begin(), offs.begin() + offs.size() / 2, offs.end());
  const double off = offs[offs.size() / 2];
  if (!quiet) {
    std::printf(
      "loaded: imu=%zu gps_pos=%zu gps_vel=%zu gps_hdt=%zu  (dev->bag offset %.3f s)\n",
      imus.size(), poss.size(), vels.size(), hdts.size(), off);
  }

  // --- datum: first valid gps_pos ------------------------------------------------
  LocalProjection proj(sphere);
  for (const auto & p : poss) {
    if (sbg_raw::gpsPosValid(p.msg)) {
      proj.setOrigin(p.msg.latitude, p.msg.longitude);
      break;
    }
  }
  if (!proj.valid()) {
    std::cerr << "no usable gps_pos\n";
    return 1;
  }

  // --- events ---------------------------------------------------------------------
  std::vector<Event> ev;
  ev.reserve(imus.size() + poss.size() + vels.size() + hdts.size());
  for (std::size_t i = 0; i < imus.size(); ++i) {
    ev.push_back({imus[i].dev_t + off, imus[i].bag_t, 0, i});
  }
  for (std::size_t i = 0; i < poss.size(); ++i) {
    ev.push_back({poss[i].dev_t + off, poss[i].bag_t, 1, i});
  }
  for (std::size_t i = 0; i < vels.size(); ++i) {
    ev.push_back({vels[i].dev_t + off, vels[i].bag_t, 2, i});
  }
  for (std::size_t i = 0; i < hdts.size(); ++i) {
    ev.push_back({hdts[i].dev_t + off, hdts[i].bag_t, 3, i});
  }
  if (order == "device") {
    std::stable_sort(
      ev.begin(), ev.end(), [](const Event & a, const Event & b) {
        return a.t < b.t || (a.t == b.t && a.prio < b.prio);
      });
  } else {
    std::stable_sort(
      ev.begin(), ev.end(), [](const Event & a, const Event & b) {return a.bag_t < b.bag_t;});
  }

  RawGnssEkfParams params;
  params.hdt_offset = hyu_localization::deg2rad(hdt_offset_deg);
  RawGnssEkf wrapper(params, history);  // receipt order
  RawGnssEkfCore core(params);          // device order
  const bool use_wrapper = order == "receipt";
  auto state = [&]() -> const RawGnssEkfCore & {return use_wrapper ? wrapper.core() : core;};

  std::vector<Row> rows;
  rows.reserve(imus.size());
  for (const Event & e : ev) {
    switch (e.prio) {
      case 0: {
          const ImuSample s = sbg_raw::imuSample(imus[e.idx].msg, e.t, enu);
          if (use_wrapper) {wrapper.imu(s);} else {core.imu(s);}
          const RawGnssEkfCore & c = state();
          if (c.initialized()) {
            const auto & x = c.x();
            const auto & P = c.P();
            Row r;
            r.t = e.t;
            r.pN = x[0]; r.pE = x[1]; r.vN = x[2]; r.vE = x[3]; r.yaw = x[4];
            r.sN = std::sqrt(P(0, 0)); r.sE = std::sqrt(P(1, 1));
            r.svN = std::sqrt(P(2, 2)); r.svE = std::sqrt(P(3, 3)); r.syaw = std::sqrt(P(4, 4));
            r.bgz = x[5]; r.bax = x[6]; r.bay = x[7];
            r.mode = c.mode();
            r.pos_age = c.time() - c.lastPosTime();
            r.hdt_age = c.time() - c.lastHdtTime();
            r.zupt = c.stationary() ? 1 : 0;
            rows.push_back(r);
          }
          break;
        }
      case 1: {
          const auto & m = poss[e.idx].msg;
          if (!sbg_raw::gpsPosValid(m)) {break;}
          const GpsPosMeas g = sbg_raw::gpsPosMeas(m, e.t, proj, enu);
          if (use_wrapper) {wrapper.gpsPos(g);} else {core.gpsPos(g);}
          break;
        }
      case 2: {
          const auto & m = vels[e.idx].msg;
          if (!sbg_raw::gpsVelValid(m)) {break;}
          const GpsVelMeas g = sbg_raw::gpsVelMeas(m, e.t, enu);
          if (use_wrapper) {wrapper.gpsVel(g);} else {core.gpsVel(g);}
          break;
        }
      case 3: {
          const auto & m = hdts[e.idx].msg;
          if (!sbg_raw::gpsHdtValid(m)) {break;}
          const GpsHdtMeas g = sbg_raw::gpsHdtMeas(m, e.t, enu);
          if (use_wrapper) {wrapper.gpsHdt(g);} else {core.gpsHdt(g);}
          break;
        }
      default: break;
    }
  }
  if (rows.empty()) {
    std::cerr << "EKF never initialised\n";
    return 1;
  }

  // --- CSV ----------------------------------------------------------------------
  if (!out_csv.empty()) {
    std::ofstream f(out_csv);
    f << "t,pN,pE,vN,vE,yaw_deg,sig_pN,sig_pE,sig_vN,sig_vE,sig_yaw_deg,b_gz_dps,b_ax,b_ay,mode,"
      "pos_age,hdt_age,zupt\n";
    char buf[512];
    for (const Row & r : rows) {
      double yaw_deg = std::fmod(hyu_localization::rad2deg(r.yaw) + 360.0, 360.0);
      std::snprintf(
        buf, sizeof(buf),
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.3f,%.3f,%d\n",
        r.t, r.pN, r.pE, r.vN, r.vE, yaw_deg, r.sN, r.sE, r.svN, r.svE,
        hyu_localization::rad2deg(r.syaw), hyu_localization::rad2deg(r.bgz), r.bax, r.bay, r.mode,
        r.pos_age, r.hdt_age, r.zupt);
      f << buf;
    }
  }

  // --- summary (as my_ekf.py compare_plot) -------------------------------------
  const RawGnssEkfCore & c = state();
  std::vector<double> dm_all, dm_rtk;
  for (const auto & p : poss) {
    if (!sbg_raw::gpsPosValid(p.msg)) {continue;}
    const double t = p.dev_t + off;
    if (t < rows.front().t - 0.1) {continue;}
    double N, E;
    proj.toNE(p.msg.latitude, p.msg.longitude, N, E);
    const Row & r = rows[nearest(rows, t)];
    const double d = std::hypot(r.pN - N, r.pE - E);
    dm_all.push_back(d);
    if (p.msg.status.type == 7) {dm_rtk.push_back(d);}
  }
  std::vector<double> dh;
  for (const auto & h : hdts) {
    if (!sbg_raw::gpsHdtValid(h.msg)) {continue;}
    const double t = h.dev_t + off;
    if (t < rows.front().t - 0.1) {continue;}
    const Row & r = rows[nearest(rows, t)];
    const double hd = hyu_localization::deg2rad(h.msg.true_heading) + params.hdt_offset;
    dh.push_back(hyu_localization::rad2deg(wrapAngle(r.yaw - hd)));
  }
  auto rms = [](const std::vector<double> & v) {
      double s = 0.0;
      for (double x : v) {
        s += x * x;
      }
      return v.empty() ? std::nan("") : std::sqrt(s / v.size());
    };
  auto pct = [](std::vector<double> v, double q) {
      if (v.empty()) {return std::nan("");}
      std::sort(v.begin(), v.end());
      const double k = q * (v.size() - 1);
      const std::size_t lo = static_cast<std::size_t>(k);
      const std::size_t hi = std::min(lo + 1, v.size() - 1);
      return v[lo] + (v[hi] - v[lo]) * (k - lo);
    };
  auto meanstd = [](const std::vector<double> & v, double & m, double & s) {
      m = s = std::nan("");
      if (v.empty()) {return;}
      m = 0.0;
      for (double x : v) {
        m += x;
      }
      m /= v.size();
      s = 0.0;
      for (double x : v) {
        s += (x - m) * (x - m);
      }
      s = std::sqrt(s / v.size());
    };
  double fr_ok = 0, fr_nohdt = 0, fr_coast = 0, fr_zupt = 0;
  for (const Row & r : rows) {
    fr_ok += r.mode == 200;
    fr_nohdt += r.mode == 201;
    fr_coast += r.mode == 202;
    fr_zupt += r.zupt;
  }
  fr_ok /= rows.size(); fr_nohdt /= rows.size(); fr_coast /= rows.size(); fr_zupt /= rows.size();
  double hm, hs;
  meanstd(dh, hm, hs);
  const auto & acc = c.accepted();
  const auto & rej = c.rejected();
  std::printf(
    "EKF: %zu states  accepted pos %d vel %d hdt %d crs %d zupt %d  rejected pos %d vel %d hdt %d "
    "crs %d  final bias: gz %+.3f deg/s  ax %+.3f ay %+.3f m/s^2\n",
    rows.size(), acc[0], acc[1], acc[2], acc[3], acc[4] / 2, rej[0], rej[1], rej[2], rej[3],
    hyu_localization::rad2deg(c.x()[5]), c.x()[6], c.x()[7]);
  std::printf(
    "pos_rms_rtk=%.3f pos_p95=%.3f pos_max=%.3f yaw_vs_hdt_std=%.3f yaw_vs_hdt_mean=%.3f "
    "frac_ok=%.3f frac_nohdt=%.3f frac_coast=%.3f frac_zupt=%.3f\n",
    rms(dm_rtk), pct(dm_all, 0.95), dm_all.empty() ? std::nan("") : *std::max_element(
      dm_all.begin(), dm_all.end()), hs, hm, fr_ok, fr_nohdt, fr_coast, fr_zupt);
  if (use_wrapper) {
    const auto & st = wrapper.stats();
    std::cout << "oosm: replays=" << st.replays << " replayed_events=" << st.replayed_events
              << " dropped_stale=" << st.dropped_stale << " max_lag=" << fmtLag(st.max_lag)
              << " s\n";
  }
  return 0;
}
