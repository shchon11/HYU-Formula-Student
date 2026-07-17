#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
r"""
Sensor parity report: measure a sensor stream's error structure, sim or real.

Every noise number in the sensor sim is either a datasheet value or a labelled
engineering estimate. This tool is how each estimate gets pinned: it computes
the same statistics from live topics whether the source is the simulator or a
replayed real-car bag (``ros2 bag play``), writes them to JSON, and diffs two
such JSONs. Sim run vs bench run, same script, same numbers side by side.

What it measures (all timing from message stamps, i.e. sim-time aware):

  /imu/data            rate; per-axis gyro/accel mean (bias), sample sigma,
                       noise density sigma/sqrt(rate), and an overlapping
                       Allan deviation curve with an ARW estimate from its
                       white-noise region (sigma_A * sqrt(tau) there). If the
                       curve is still falling at the longest tau, bias
                       instability is reported as unresolved, not invented.
  /sbg/ekf_nav         E/N scatter about the mean (a STATIC bench run makes
                       scatter = error), empirical Gauss-Markov correlation
                       time (autocorr 1/e), mean reported accuracy for
                       comparison against the realized scatter.
  /ros_can/wheel_speeds rear-wheel rate, sigma about the median, and the
                       detected quantization step (min positive gap between
                       sorted unique values) -> pins wheelSpeedQuantumRPM
                       (AMK N_act LSB / gear ratio) and wheelSpeedNoise.
  /velodyne_points     points per frame and a range-binned histogram of
                       non-ground returns -> the density curve the dropout
                       model must reproduce. (Per-cone curves need ground
                       truth or clustering; deliberately out of scope here.)

Usage:
  ros2 run eufs_sensors sensor_parity_report --ros-args -p duration:=120.0 \\
      -p output:=/tmp/sim.json
  # on the bench / against a real bag being replayed:
  ros2 run eufs_sensors sensor_parity_report --ros-args -p duration:=300.0 \\
      -p output:=/tmp/real.json
  # then:
  ros2 run eufs_sensors sensor_parity_report --compare /tmp/sim.json /tmp/real.json

A section missing from a run (topic never seen) is reported as absent and
skipped in the comparison — silence is labelled, never averaged over.
"""

import json
import math
import sys
import time

import numpy as np


def allan_deviation(samples, rate, max_points=24):
    """
    Overlapping Allan deviation over a log-spaced tau grid.

    Returns (taus, sigmas). Needs a regularly-sampled series; stamp jitter
    in the source shows up as curve wobble, not a wrong trend.
    """
    x = np.asarray(samples, dtype=np.float64)
    n = x.size
    if n < 64:
        return [], []
    theta = np.cumsum(x) / rate
    max_m = n // 4
    ms = np.unique(
        np.round(np.logspace(0, math.log10(max_m), max_points)).astype(int)
    )
    taus, sigmas = [], []
    for m in ms:
        if m < 1 or 2 * m >= n:
            continue
        d = theta[2 * m:] - 2.0 * theta[m:-m] + theta[: -2 * m]
        tau = m / rate
        sigmas.append(math.sqrt(float(np.mean(d * d)) / (2.0 * tau * tau)))
        taus.append(tau)
    return taus, sigmas


def arw_from_allan(taus, sigmas):
    """
    Angular/velocity random walk from the white region of an Allan curve.

    For white noise sigma_A(tau) = density/sqrt(tau); estimate the density as
    the median of sigma_A*sqrt(tau) over the short-tau half of the curve.
    """
    if not taus:
        return None
    half = max(1, len(taus) // 2)
    vals = [s * math.sqrt(t) for t, s in zip(taus[:half], sigmas[:half])]
    return float(np.median(vals))


def autocorr_1e_time(samples, dt):
    """Lag time where the autocorrelation first drops below 1/e."""
    x = np.asarray(samples, dtype=np.float64)
    x = x - x.mean()
    var = float(np.mean(x * x))
    if var <= 0.0 or x.size < 16:
        return None
    target = math.exp(-1.0)
    for lag in range(1, x.size - 1):
        c = float(np.mean(x[:-lag] * x[lag:])) / var
        if c < target:
            return lag * dt
    return x.size * dt


def _series_rate(stamps):
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return None
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def _imu_section(stamps, gyro, accel):
    rate = _series_rate(stamps)
    if rate is None:
        return None
    out = {"rate_hz": rate, "gyro": {}, "accel": {}}
    for name, cols in (("gyro", gyro), ("accel", accel)):
        arr = np.asarray(cols, dtype=np.float64)
        for i, axis in enumerate("xyz"):
            series = arr[:, i]
            taus, sigmas = allan_deviation(series, rate)
            arw = arw_from_allan(taus, sigmas)
            resolved = bool(
                sigmas and sigmas.index(min(sigmas)) < len(sigmas) - 1
            )
            out[name][axis] = {
                "mean": float(series.mean()),
                "sigma": float(series.std()),
                "noise_density": float(series.std() / math.sqrt(rate)),
                "allan_tau_s": [round(t, 4) for t in taus],
                "allan_sigma": sigmas,
                "arw_density": arw,
                "bias_instability_resolved": resolved,
            }
    return out


def _gnss_section(stamps, east, north, acc):
    rate = _series_rate(stamps)
    if rate is None:
        return None
    e = np.asarray(east) - np.mean(east)
    n = np.asarray(north) - np.mean(north)
    dt = 1.0 / rate
    return {
        "rate_hz": rate,
        "sigma_e": float(e.std()),
        "sigma_n": float(n.std()),
        "gm_tau_e_s": autocorr_1e_time(e, dt),
        "gm_tau_n_s": autocorr_1e_time(n, dt),
        "mean_reported_acc": float(np.mean(acc)),
    }


def _wheels_section(stamps, speeds):
    rate = _series_rate(stamps)
    if rate is None:
        return None
    arr = np.asarray(speeds, dtype=np.float64)
    uniq = np.unique(np.round(arr, 6))
    gaps = np.diff(uniq)
    gaps = gaps[gaps > 1e-6]
    return {
        "rate_hz": rate,
        "sigma_rpm": float(np.std(arr - np.median(arr))),
        "quantum_rpm": float(gaps.min()) if gaps.size else None,
        "distinct_values": int(uniq.size),
    }


def _lidar_section(frames):
    if not frames:
        return None
    bins = np.arange(0.0, 22.0, 2.0)
    hist = np.zeros(bins.size - 1)
    for rng, z in frames:
        above_ground = z > (np.min(z) + 0.15)
        h, _ = np.histogram(rng[above_ground], bins=bins)
        hist += h
    return {
        "frames": len(frames),
        "mean_points_per_frame": float(np.mean([r.size for r, _ in frames])),
        "range_bins_m": bins.tolist(),
        "nonground_points_per_frame_by_bin": (hist / len(frames)).tolist(),
    }


def _fmt(value):
    if value is None:
        return "absent"
    if isinstance(value, float):
        return f"{value:.4g}"
    return str(value)


def compare(path_a, path_b):
    """Print two reports side by side with ratios where both exist."""
    with open(path_a) as f:
        a = json.load(f)
    with open(path_b) as f:
        b = json.load(f)

    def walk(prefix, va, vb):
        if isinstance(va, dict) and isinstance(vb, dict):
            for k in sorted(set(va) | set(vb)):
                walk(f"{prefix}.{k}" if prefix else k, va.get(k), vb.get(k))
            return
        if isinstance(va, list) or isinstance(vb, list):
            return  # curves: plot externally, not diffable line-by-line
        if isinstance(va, (int, float)) and isinstance(vb, (int, float)) and vb:
            ratio = f"  x{va / vb:.2f}" if vb else ""
            print(f"{prefix:55s} {_fmt(va):>12s} {_fmt(vb):>12s}{ratio}")
        elif va is not None or vb is not None:
            print(f"{prefix:55s} {_fmt(va):>12s} {_fmt(vb):>12s}")

    print(f"{'metric':55s} {'A':>12s} {'B':>12s}  A/B")
    print(f"A = {path_a}\nB = {path_b}")
    walk("", a, b)
    return 0


def record():
    """Collect from live topics and write the JSON report."""
    import rclpy
    from rclpy.node import Node as RclpyNode
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, PointCloud2
    from sensor_msgs_py import point_cloud2
    from eufs_msgs.msg import WheelSpeedsStamped

    try:
        from sbg_driver.msg import SbgEkfNav
    except ImportError:
        SbgEkfNav = None

    rclpy.init()
    node = RclpyNode("sensor_parity_report")
    duration = node.declare_parameter("duration", 120.0).value
    output = node.declare_parameter("output", "sensor_parity_report.json").value
    max_lidar_frames = node.declare_parameter("max_lidar_frames", 100).value

    qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
    data = {
        "imu": {"stamps": [], "gyro": [], "accel": []},
        "gnss": {"stamps": [], "east": [], "north": [], "acc": []},
        "wheels": {"stamps": [], "speeds": []},
        "lidar": [],
    }

    def stamp_of(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_imu(msg):
        data["imu"]["stamps"].append(stamp_of(msg))
        data["imu"]["gyro"].append(
            (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        )
        data["imu"]["accel"].append(
            (msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z)
        )

    def on_nav(msg):
        data["gnss"]["stamps"].append(stamp_of(msg))
        # Small-angle equirectangular about the first fix: fine for scatter.
        if not data["gnss"]["east"]:
            on_nav.lat0, on_nav.lon0 = msg.latitude, msg.longitude
        scale = 111320.0
        data["gnss"]["north"].append((msg.latitude - on_nav.lat0) * scale)
        data["gnss"]["east"].append(
            (msg.longitude - on_nav.lon0) * scale
            * math.cos(math.radians(on_nav.lat0))
        )
        data["gnss"]["acc"].append(
            0.5 * (msg.position_accuracy.x + msg.position_accuracy.y)
        )

    def on_wheels(msg):
        data["wheels"]["stamps"].append(stamp_of(msg))
        data["wheels"]["speeds"].append(msg.speeds.lb_speed)
        data["wheels"]["speeds"].append(msg.speeds.rb_speed)

    def on_cloud(msg):
        if len(data["lidar"]) >= max_lidar_frames:
            return
        arr = np.asarray(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"),
                                     skip_nans=True)
        )
        if arr.size == 0:
            return
        x, y, z = (arr["x"].astype(np.float64), arr["y"].astype(np.float64),
                   arr["z"].astype(np.float64))
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        rng = np.sqrt(x[finite] ** 2 + y[finite] ** 2 + z[finite] ** 2)
        data["lidar"].append((rng, z[finite]))

    node.create_subscription(Imu, "/imu/data", on_imu, qos)
    node.create_subscription(WheelSpeedsStamped, "/ros_can/wheel_speeds",
                             on_wheels, qos)
    node.create_subscription(PointCloud2, "/velodyne_points", on_cloud, qos)
    if SbgEkfNav is not None:
        node.create_subscription(SbgEkfNav, "/sbg/ekf_nav", on_nav, qos)

    node.get_logger().info(
        f"collecting for {duration:.0f} s of message-stamp time -> {output}"
    )

    def span(stamps):
        return stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0

    wall_deadline = time.monotonic() + 3.0 * duration + 30.0
    while time.monotonic() < wall_deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        spans = [span(data[k]["stamps"]) for k in ("imu", "gnss", "wheels")]
        active = [s for s in spans if s > 0.0]
        if active and min(active) >= duration:
            break

    report = {
        "meta": {
            "duration_requested_s": duration,
            "sections_seen": sorted(
                k for k in ("imu", "gnss", "wheels")
                if data[k]["stamps"]
            ) + (["lidar"] if data["lidar"] else []),
        },
        "imu": _imu_section(data["imu"]["stamps"], data["imu"]["gyro"],
                            data["imu"]["accel"]),
        "gnss": _gnss_section(data["gnss"]["stamps"], data["gnss"]["east"],
                              data["gnss"]["north"], data["gnss"]["acc"]),
        "wheels": _wheels_section(data["wheels"]["stamps"],
                                  data["wheels"]["speeds"]),
        "lidar": _lidar_section(data["lidar"]),
    }
    with open(output, "w") as f:
        json.dump(report, f, indent=1)
    for section in ("imu", "gnss", "wheels", "lidar"):
        state = "ok" if report[section] else "ABSENT"
        node.get_logger().info(f"  {section}: {state}")
    node.get_logger().info(f"wrote {output}")

    node.destroy_node()
    rclpy.try_shutdown()
    return 0


def main():
    if "--compare" in sys.argv:
        i = sys.argv.index("--compare")
        return compare(sys.argv[i + 1], sys.argv[i + 2])
    return record()


if __name__ == "__main__":
    sys.exit(main())
