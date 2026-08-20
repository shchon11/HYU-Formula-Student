#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Offline replay of recorded /sbg/* through sbg_odometry_bridge, scored against
the raw RTK fix.

Feeds a rosbag2's SBG messages straight into the bridge callbacks in
recording order (no executor, no clock) and captures the CarState the bridge
would have published, then measures how well the integrated RELATIVE
odometry tracks the receiver's RTK position through every EKF outage
(solution_mode < 3) in the bag. Ground truth is /sbg/gps_pos (RTK_INT, ~1 cm)
with the antenna lever arm removed using the bridge's own heading.

The car has no wheel encoder, so ``--wheels doppler`` synthesises
/vehicle/wheel_speeds from the raw Doppler ground speed (independent of the
EKF, unlike sbg_wheels.py which back-computes from ekf_nav and therefore dies
with the EKF). That lets the wheel rung be exercised through the recorded
outages as well.

    ros2 run hyu_localization bridge_bag_eval.py BAG [--raw on|off]
        [--wheels off|doppler] [--bridge PATH] [--csv out.csv]

Prints per-outage: duration, distance driven (RTK), gap in the published
odometry, position error of the relative odometry at the end of the outage
and its max inside it, heading error vs. HDT, and the sigma tiers used.
"""
import argparse
import bisect
import importlib.util
import math
import os
import sys

import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from hyu_msgs.msg import WheelSpeedsStamped

_RPM_TO_RAD_S = 2.0 * math.pi / 60.0
_A = 6378137.0
_E2 = 6.69437999014e-3


class _Cap:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _load_bridge(path):
    spec = importlib.util.spec_from_file_location("sbg_odometry_bridge_eval", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _read(bag, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    present = [t for t in topics if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=present))
    cls = {t: get_message(types[t]) for t in present}
    while reader.has_next():
        top, data, ts = reader.read_next()
        yield top, deserialize_message(data, cls[top]), ts * 1e-9


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("bag")
    ap.add_argument("--raw", choices=["on", "off", "heading"], default="on",
                    help="raw-GNSS rung: on (gps_pos/gps_vel/gps_hdt/imu), "
                         "heading (gps_hdt/imu only: heading fallback without "
                         "raw position/velocity, to exercise the wheel rung), off")
    ap.add_argument("--wheels", choices=["off", "doppler"], default="off",
                    help="synthesise /vehicle/wheel_speeds from raw Doppler")
    ap.add_argument("--bridge", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "sbg_odometry_bridge.py"))
    ap.add_argument("--csv", default=None, help="write the odometry trace")
    ap.add_argument("--wheel-radius", type=float, default=0.2525)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    rclpy.init()
    module = _load_bridge(args.bridge)
    node = module.SbgOdometryBridge()
    node.raw_gnss_fallback_enable = args.raw != "off"
    feed_pos_vel = args.raw == "on"
    node.car_state_pub = _Cap()
    node.gnss_odom_pub = _Cap()
    node.health_pub = _Cap()
    node.marker_pub = None
    node.overlay_pub = None
    if args.quiet:
        node.get_logger().set_level(40)  # ERROR

    has_raw = hasattr(node, "on_gps_pos")
    topics = ["/sbg/ekf_nav", "/sbg/ekf_euler", "/sbg/gps_pos", "/sbg/gps_vel",
              "/sbg/gps_hdt", "/sbg/imu_data"]

    # Ground truth + traces.
    origin = None
    fixes = []       # (t, e_ant, n_ant, type)
    hdts = []        # (t, yaw_enu_raw)
    nav_modes = []   # (t, mode)
    out = []         # (t, x, y, yaw, sigma, source)
    last_doppler = None

    def project(lat, lon):
        nonlocal origin
        if origin is None:
            la = math.radians(lat)
            d = 1.0 - _E2 * math.sin(la) ** 2
            origin = (la, math.radians(lon),
                      _A * (1.0 - _E2) / d ** 1.5, _A / math.sqrt(d))
        la0, lo0, rm, rn = origin
        return (rn * math.cos(la0) * (math.radians(lon) - lo0),
                rm * (math.radians(lat) - la0))

    n_before = 0
    anchor = [None]  # (device us, ROS s) from imu_data: receiver epochs are
    #                  emitted ~90 ms after the IMU frame with the same device
    #                  stamp, so score them at the instant they describe.

    def epoch_time(msg):
        if anchor[0] is None:
            return _stamp(msg.header)
        d = (int(msg.time_stamp) - anchor[0][0]) & 0xFFFFFFFF
        if d >= 0x80000000:
            d -= 0x100000000
        t = anchor[0][1] + d * 1e-6
        return t if abs(t - _stamp(msg.header)) < 0.5 else _stamp(msg.header)

    for top, msg, t_bag in _read(args.bag, topics):
        if top == "/sbg/ekf_euler":
            node.on_euler(msg)
        elif top == "/sbg/imu_data":
            anchor[0] = (int(msg.time_stamp), _stamp(msg.header))
            if has_raw and node.raw_gnss_fallback_enable:
                node.on_imu(msg)
        elif top == "/sbg/gps_pos":
            e, n = project(msg.latitude, msg.longitude)
            fixes.append((epoch_time(msg), e, n, msg.status.type,
                          msg.status.status))
            if has_raw and feed_pos_vel:
                node.on_gps_pos(msg)
        elif top == "/sbg/gps_vel":
            if has_raw and feed_pos_vel:
                node.on_gps_vel(msg)
            if msg.status.vel_status == 0 and msg.status.vel_type != 0:
                # Encoder = forward speed of the car. The Doppler velocity is
                # the ANTENNA's, which in a turn also carries w x l laterally;
                # the forward component (along the bridge's heading) is the
                # rear-axle speed a wheel sensor would read.
                ve, vn = node._vel_enu(msg.velocity)
                yaw, _ = node._heading_source(_stamp(msg.header))
                if yaw is None:
                    v_fwd = math.hypot(ve, vn)
                else:
                    v_fwd = ve * math.cos(yaw) + vn * math.sin(yaw)
                last_doppler = (_stamp(msg.header), v_fwd)
            if args.wheels == "doppler" and last_doppler is not None:
                ws = WheelSpeedsStamped()
                ws.header = msg.header
                rpm = last_doppler[1] / (args.wheel_radius * _RPM_TO_RAD_S)
                ws.speeds.lb_speed = rpm
                ws.speeds.rb_speed = rpm
                node.on_wheel_speeds(ws)
        elif top == "/sbg/gps_hdt":
            if (msg.status & 0x3F) == 0:
                hdts.append((epoch_time(msg),
                             node._yaw_to_enu(math.radians(msg.true_heading))))
            if has_raw and node.raw_gnss_fallback_enable:
                node.on_gps_hdt(msg)
        elif top == "/sbg/ekf_nav":
            t = _stamp(msg.header)
            nav_modes.append((t, msg.status.solution_mode))
            if args.wheels == "doppler" and last_doppler is not None:
                # Re-stamp the last Doppler speed at the nav rate so the wheel
                # freshness gate (0.3 s) sees a 25 Hz encoder, as a CAN wheel
                # sensor would deliver.
                ws = WheelSpeedsStamped()
                ws.header = msg.header
                rpm = last_doppler[1] / (args.wheel_radius * _RPM_TO_RAD_S)
                ws.speeds.lb_speed = rpm
                ws.speeds.rb_speed = rpm
                node.on_wheel_speeds(ws)
            node.on_nav(msg)
            if len(node.car_state_pub.msgs) > n_before:
                m = node.car_state_pub.msgs[-1]
                n_before = len(node.car_state_pub.msgs)
                yaw = 2.0 * math.atan2(m.pose.pose.orientation.z,
                                       m.pose.pose.orientation.w)
                src = "?"
                if node.health_pub.msgs:
                    for kv in node.health_pub.msgs[-1].status[0].values:
                        if kv.key == "motion_source":
                            src = kv.value
                out.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                            yaw, math.sqrt(m.pose.covariance[0]), src))

    node.destroy_node()
    rclpy.try_shutdown()

    if not out:
        print("bridge never published (no start) -- nothing to score")
        return 1

    # Outage windows from the nav modes.
    outages = []
    cur = None
    for t, mode in nav_modes:
        if mode < 3 and cur is None:
            cur = t
        elif mode >= 3 and cur is not None:
            outages.append((cur, t))
            cur = None
    if cur is not None:
        outages.append((cur, nav_modes[-1][0]))

    out_t = [o[0] for o in out]
    fix_t = [f[0] for f in fixes]
    hdt_t = [h[0] for h in hdts]

    def odom_at(t):
        i = bisect.bisect_left(out_t, t)
        i = min(max(i, 0), len(out) - 1)
        if i > 0 and abs(out[i - 1][0] - t) < abs(out[i][0] - t):
            i -= 1
        return out[i]

    def truth_at(t):
        """RTK antenna position nearest t (None if not RTK)."""
        i = bisect.bisect_left(fix_t, t)
        i = min(max(i, 0), len(fixes) - 1)
        if i > 0 and abs(fixes[i - 1][0] - t) < abs(fixes[i][0] - t):
            i -= 1
        f = fixes[i]
        if abs(f[0] - t) > 0.25 or f[3] < 6 or f[4] != 0:
            return None
        return f

    def hdt_at(t):
        i = bisect.bisect_left(hdt_t, t)
        i = min(max(i, 0), len(hdts) - 1)
        if abs(hdts[i][0] - t) > 0.25:
            return None
        return hdts[i][1]

    lx = getattr(node, "antenna_lever_arm_x", 0.0)

    def rel_error(t0, t1):
        """Relative-odometry error over [t0,t1] vs. RTK (lever arm removed)."""
        o0, o1 = odom_at(t0), odom_at(t1)
        f0, f1 = truth_at(t0), truth_at(t1)
        if f0 is None or f1 is None:
            return None
        # Truth base point = antenna - R(yaw)*l, using the bridge's yaw.
        b0 = (f0[1] - lx * math.cos(o0[3]), f0[2] - lx * math.sin(o0[3]))
        b1 = (f1[1] - lx * math.cos(o1[3]), f1[2] - lx * math.sin(o1[3]))
        de = (o1[1] - o0[1]) - (b1[0] - b0[0])
        dn = (o1[2] - o0[2]) - (b1[1] - b0[1])
        return math.hypot(de, dn)

    print(f"bag: {args.bag}")
    print(f"raw rung: {args.raw}   wheels: {args.wheels}   published: {len(out)} "
          f"CarState over {out[-1][0] - out[0][0]:.0f} s")
    srcs = {}
    for o in out:
        srcs[o[5]] = srcs.get(o[5], 0) + 1
    print("motion sources:", srcs)
    if not outages:
        print("no EKF outage (mode < 3) in this bag")
    total_gap = 0.0
    for a, b in outages:
        # Distance driven per RTK during the outage.
        dist = 0.0
        prev = None
        for f in fixes:
            if a <= f[0] <= b and f[3] >= 6:
                if prev is not None:
                    dist += math.hypot(f[1] - prev[1], f[2] - prev[2])
                prev = f
        # Published gap inside the outage.
        inside = [o for o in out if a - 0.05 <= o[0] <= b + 0.05]
        if inside:
            gaps = [inside[i + 1][0] - inside[i][0] for i in range(len(inside) - 1)]
            max_gap = max(gaps) if gaps else 0.0
            first_gap = inside[0][0] - a
            tail_gap = b - inside[-1][0]
            gap = max(max_gap, first_gap, tail_gap)
        else:
            gap = b - a
        total_gap += gap
        # Errors: at end of outage relative to its start, and max inside.
        e_end = rel_error(a, b)
        # Error trace inside (every 1 s).
        errs = []
        tt = a
        while tt <= b:
            e = rel_error(a, tt)
            if e is not None:
                errs.append(e)
            tt += 1.0
        # After re-entry (2 s later) -- did the pose jump?
        e_after = rel_error(a, min(b + 2.0, out[-1][0]))
        # Heading vs HDT at the end of the outage (mod the learned offset).
        o1 = odom_at(b)
        h1 = hdt_at(b)
        off = getattr(node, "_hdt_offset", None)
        yaw_err = None
        if h1 is not None and off is not None:
            yaw_err = math.degrees(_wrap(o1[3] - (h1 + off)))
        sig = sorted({round(o[4], 2) for o in inside}) if inside else []
        used = {}
        for o in inside:
            used[o[5]] = used.get(o[5], 0) + 1
        fmt = lambda v: "n/a" if v is None else f"{v:.2f} m"  # noqa: E731
        print(f"\noutage {a - out[0][0]:.1f}-{b - out[0][0]:.1f} s ({b - a:.1f} s), "
              f"RTK distance driven {dist:.1f} m")
        print(f"   published gap max {gap:.2f} s   sources {used}   sigmas {sig}")
        print(f"   relative-odometry error: end-of-outage {fmt(e_end)}, "
              f"max inside {fmt(max(errs) if errs else None)}, "
              f"2 s after re-entry {fmt(e_after)}"
              + (f", heading vs HDT {yaw_err:+.1f} deg" if yaw_err is not None else ""))
    if args.csv:
        with open(args.csv, "w") as fh:
            fh.write("t,x,y,yaw,sigma,source\n")
            for o in out:
                fh.write(f"{o[0]:.3f},{o[1]:.3f},{o[2]:.3f},{o[3]:.4f},{o[4]:.3f},{o[5]}\n")
        print(f"trace -> {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
