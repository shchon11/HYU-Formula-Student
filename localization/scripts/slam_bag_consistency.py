#!/usr/bin/env python3
"""Offline SLAM consistency report from a bag (2026-08-22, written while chasing a lap-2 pose offset).

Reads /localization/ego_odom, /localization/ins_odom, /perception/cones, /localization/cone_map
(and /localization/status if present) and prints, per 10 s:
  * ego_odom - ins_odom (how much SLAM moved the pose away from the INS),
  * map-vs-live offset: live cones placed with the SLAM pose vs nearest map landmark
    (robust median displacement + the per-frame rotation that best aligns them about the ego),
  * a MAP-FREE cross-lap check: cones placed with the INS pose only; lap-2 cones vs lap-1 cones,
    swept over a yaw bias (base x-axis vs INS heading), the antenna lever arm and a cone-stamp
    latency. If the INS+perception are self-consistent here (~5-7 cm on the 0801 RTK bags) any
    pose-vs-map offset comes from the SLAM backend, not the inputs.

  ros2 run hyu_localization slam_bag_consistency.py <bag_dir> [--lap1 20 70] [--lap2 250 410]
"""
import argparse, bisect, math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from hyu_msgs.msg import CarState, ConeArrayWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import String
try:
    from scipy.spatial import cKDTree
except Exception:  # pragma: no cover
    cKDTree = None


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=[
        "/localization/ego_odom", "/localization/ins_odom", "/perception/cones",
        "/localization/cone_map", "/localization/status"]))
    ego, ins, cones, maps, status = [], [], [], [], []
    first = None
    while r.has_next():
        t, d, ts = r.read_next()
        first = ts if first is None else first
        tt = (ts - first) / 1e9
        if t == "/localization/ego_odom":
            m = deserialize_message(d, Odometry)
            ego.append((tt, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x,
                        m.pose.pose.position.y, yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x))
        elif t == "/localization/ins_odom":
            m = deserialize_message(d, CarState)
            ins.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x,
                        m.pose.pose.position.y, yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x))
        elif t == "/perception/cones":
            m = deserialize_message(d, ConeArrayWithCovariance)
            cones.append((tt, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                          [(c.point.x, c.point.y) for f in (m.blue_cones, m.yellow_cones, m.unknown_color_cones) for c in f]))
        elif t == "/localization/cone_map":
            m = deserialize_message(d, ConeArrayWithCovariance)
            maps.append((tt, [(c.point.x, c.point.y) for f in
                              (m.blue_cones, m.yellow_cones, m.unknown_color_cones, m.orange_cones, m.big_orange_cones) for c in f]))
        else:
            status.append((round(tt, 1), deserialize_message(d, String).data))
    return ego, ins, cones, maps, status


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--lap1", nargs=2, type=float, default=[20, 70])
    ap.add_argument("--lap2", nargs=2, type=float, default=[250, 410])
    ap.add_argument("--lever", type=float, default=1.25, help="antenna_offset_x the INS already applied")
    a = ap.parse_args()
    ego, ins, cones, maps, status = read(a.bag)
    print(f"ego {len(ego)} ins {len(ins)} cones {len(cones)} maps {len(maps)} status {status}")
    istamps = [i[0] for i in ins]
    estamps = [e[1] for e in ego]

    print("\n[1] ego_odom - ins_odom every 20 s:  t  ego(x,y,yaw)  d(x,y,yaw deg)  v")
    last = -1e9
    for e in ego:
        if e[0] - last >= 20:
            last = e[0]
            k = min(max(bisect.bisect_left(istamps, e[1]), 0), len(ins) - 1)
            i = ins[k]
            print("  %6.1f (%6.2f,%6.2f,%6.1f)  (%+5.2f,%+5.2f,%+5.1f) v=%.2f" % (
                e[0], e[2], e[3], math.degrees(e[4]), e[2] - i[1], e[3] - i[2], math.degrees(wrap(e[4] - i[3])), e[5]))

    print("\n[2] map-vs-live (SLAM pose): per 10 s median displacement map-live, rotation about ego, median NN")
    mi, rows = 0, []
    for (tt, st, pts) in cones:
        while mi + 1 < len(maps) and maps[mi + 1][0] <= tt:
            mi += 1
        mp = maps[mi][1] if maps else []
        if len(mp) < 5 or len(pts) < 3:
            continue
        k = min(max(bisect.bisect_left(estamps, st), 0), len(ego) - 1)
        if abs(ego[k][1] - st) > 0.3:
            continue
        _, _, ex, ey, eyaw, v = ego[k]
        c, s = math.cos(eyaw), math.sin(eyaw)
        dxs, dys, nn, P, Q = [], [], [], [], []
        for (x, y) in pts:
            if not 1.5 <= math.hypot(x, y) <= 10:
                continue
            mx, my = ex + c * x - s * y, ey + s * x + c * y
            px, py = min(mp, key=lambda p: math.hypot(mx - p[0], my - p[1]))
            dd = math.hypot(mx - px, my - py)
            nn.append(dd)
            if dd <= 1.0:
                dxs.append(px - mx); dys.append(py - my); P.append((mx - ex, my - ey)); Q.append((px - ex, py - ey))
        if len(dxs) < 3:
            continue
        sxx = sum(p[0] * q[0] for p, q in zip(P, Q)); sxy = sum(p[0] * q[1] for p, q in zip(P, Q))
        syx = sum(p[1] * q[0] for p, q in zip(P, Q)); syy = sum(p[1] * q[1] for p, q in zip(P, Q))
        rows.append((tt, np.median(dxs), np.median(dys), math.degrees(math.atan2(sxy - syx, sxx + syy)), np.median(nn), v))
    bucket = {}
    for row in rows:
        bucket.setdefault(int(row[0] // 10), []).append(row)
    for k in sorted(bucket):
        v = bucket[k]
        mv = [r for r in v if abs(r[5]) >= 0.5]
        rot = np.median([r[3] for r in mv]) if mv else float("nan")  # rotation only means something while moving
        print("  %4d-%4ds  d=(%+.2f,%+.2f) rot=%+.2f deg  nn=%.2f  (%d frames, %d moving)" % (
            k * 10, k * 10 + 10, np.median([r[1] for r in v]), np.median([r[2] for r in v]),
            rot, np.median([r[4] for r in v]), len(v), len(mv)))

    if cKDTree is None:
        print("\n[3] scipy missing; skipping the map-free cross-lap sweep")
        return
    print("\n[3] MAP-FREE cross-lap consistency (INS pose only): lap2 cones vs lap1 cones, median NN distance [m]")
    IS = np.array(istamps); IX = np.array([i[1] for i in ins]); IY = np.array([i[2] for i in ins])
    IYAW = np.unwrap([i[3] for i in ins]); IV = np.array([i[4] for i in ins])

    def place(beta_deg, d, dt, lo, hi):
        out = []
        for (tt, st, pts) in cones:
            if not lo <= tt < hi:
                continue
            x = np.interp(st + dt, IS, IX); y = np.interp(st + dt, IS, IY)
            yw = np.interp(st + dt, IS, IYAW); v = np.interp(st + dt, IS, IV)
            if abs(v) < 0.5:
                continue
            P = np.array([(px, py) for px, py in pts if 1.5 <= math.hypot(px, py) <= 10.0])
            if len(P) == 0:
                continue
            bx = x + math.cos(yw) * (a.lever - d); by = y + math.sin(yw) * (a.lever - d)
            th = yw + math.radians(beta_deg); c, s = math.cos(th), math.sin(th)
            out.append(np.column_stack([bx + c * P[:, 0] - s * P[:, 1], by + s * P[:, 0] + c * P[:, 1]]))
        return np.vstack(out) if out else np.zeros((0, 2))

    def metric(beta, d, dt):
        A1 = place(beta, d, dt, *a.lap1); A2 = place(beta, d, dt, *a.lap2)
        if len(A1) < 10 or len(A2) < 10:
            return float("nan")
        dist, _ = cKDTree(A1).query(A2)
        return float(np.median(np.minimum(dist, 2.0)))
    print("  yaw bias beta (deg): " + "  ".join("%+.1f:%.3f" % (b, metric(b, a.lever, 0.0)) for b in np.arange(-3, 3.01, 1.0)))
    print("  lever arm d (m):     " + "  ".join("%.2f:%.3f" % (d, metric(0.0, d, 0.0)) for d in (0.75, 1.0, a.lever, 1.5, 1.75)))
    print("  stamp offset dt (s): " + "  ".join("%+.2f:%.3f" % (dt, metric(0.0, a.lever, dt)) for dt in (-0.2, -0.1, 0.0, 0.1, 0.2)))


if __name__ == "__main__":
    main()
