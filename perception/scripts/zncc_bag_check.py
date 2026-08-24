#!/usr/bin/env python3
"""Replay the ZNCC stereo tier on a real bag against the LiDAR, offline.

What it answers (the 2026-08-23 diagnosis, re-runnable after any change):
  * is the rectified pair biased?  -> disparity offset ZNCC - LiDAR by range and
    by image column, and the yaw that explains it (fx*yaw*(1+x^2))
  * what does the node publish?     -> depth error vs LiDAR with the configured
    yaw correction (or --yaw-deg), with the online estimator run on the same
    boxes, and with no correction at all

Needs a bag with /perception/bounding_boxes, the compressed left/right images,
the left camera_info, /sensors/lidar/points and /tf_static (base_footprint ->
zed_left_camera_optical_frame and -> rslidar). The LiDAR reference is the
densest depth bin of the points projecting into each box (static TFs, no
deskew -- fine at the 0801 bags' <3 m/s).

    ros2 run hyu_perception zncc_bag_check.py bag/0801_outdoor/rosbag2_2026_08_01-17_22_34 \
        [--frames 400] [--yaw-deg 0.28] [--csv out.csv]
"""
import argparse
import csv
import math
import sys

import cv2
import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit("source the ROS 2 workspace first (rosbag2_py, rclpy)")

from hyu_perception.fusion_core import (
    StereoYawEstimator,
    analytic_monocular_depth,
    yaw_disparity_bias_px,
    zncc_disparity,
)

CONE_H = {"blue": 0.45, "yellow": 0.45, "orange": 0.45, "big_orange": 0.5255}
TOPICS = dict(
    left="/sensors/zed/left/color/rect/image/compressed",
    right="/sensors/zed/right/color/rect/image/compressed",
    info="/sensors/zed/left/color/rect/camera_info",
    bbox="/perception/bounding_boxes",
    cloud="/sensors/lidar/points",
    tf="/tf_static",
)


def _quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _T(tr):
    T = np.eye(4)
    t, q = tr.transform.translation, tr.transform.rotation
    T[:3, :3] = _quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def _cloud_xyz(msg):
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = np.stack([buf[:, o:o + 4].copy().view(np.float32).ravel() for o in (0, 4, 8)], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def _lidar_depth(pix, depth, box, shrink=0.15):
    x0, y0, x1, y1 = box
    w, h = x1 - x0, y1 - y0
    m = ((pix[:, 0] >= x0 + shrink * w) & (pix[:, 0] <= x1 - shrink * w)
         & (pix[:, 1] >= y0 + shrink * h) & (pix[:, 1] <= y1 - shrink * h)
         & (depth > 0.5) & (depth < 40))
    d = depth[m]
    if d.size < 3:
        return None, int(d.size)
    hist, edges = np.histogram(d, np.arange(0, 40.5, 0.3))
    k = int(np.argmax(hist))
    sel = d[(d >= edges[k] - 0.3) & (d <= edges[k + 1] + 0.3)]
    return float(np.median(sel)), int(d.size)


def _gray(msg):
    img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
    return img[:, :, :3].astype(np.float64) @ np.array([1 / 3, 1 / 3, 1 / 3])


def _ns(stamp):
    return stamp.sec * 10 ** 9 + stamp.nanosec


def _colour(raw):
    t = str(raw).lower()
    if "big" in t and "orange" in t:
        return "big_orange"
    for c in ("blue", "yellow", "orange"):
        if c in t:
            return c
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--frames", type=int, default=400, help="bbox frames to process")
    ap.add_argument("--yaw-deg", type=float, default=0.28, help="stereo_right_yaw_deg under test")
    ap.add_argument("--baseline", type=float, default=0.12)
    ap.add_argument("--min-score", type=float, default=0.5)
    ap.add_argument("--low", type=float, default=0.70)
    ap.add_argument("--high", type=float, default=1.45)
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS.values())))
    K = T_cam = T_lidar = None
    lefts, rights, clouds, pending, rows = {}, {}, [], [], []
    estimator = StereoYawEstimator(math.radians(args.yaw_deg), min_samples=30)
    frames = 0
    B = args.baseline

    def process(bb):
        nonlocal frames
        s = _ns(bb.image_header.stamp)
        if s not in lefts or s not in rights or K is None or T_cam is None or T_lidar is None:
            return
        if not clouds:
            return
        cs = min(clouds, key=lambda c: abs(c[0] - s))
        if abs(cs[0] - s) > 150_000_000:
            return
        xyz = _cloud_xyz(cs[1])
        Tcl = np.linalg.inv(T_cam) @ T_lidar
        pc = xyz @ Tcl[:3, :3].T + Tcl[:3, 3]
        z = pc[:, 2]
        ok = z > 0.1
        pix = np.full((len(z), 2), np.nan)
        pix[ok, 0] = K[0, 0] * pc[ok, 0] / z[ok] + K[0, 2]
        pix[ok, 1] = K[1, 1] * pc[ok, 1] / z[ok] + K[1, 2]
        lg, rg = _gray(lefts[s]), _gray(rights[s])
        fx, cx, fy = K[0, 0], K[0, 2], K[1, 1]
        for b in bb.bounding_boxes:
            colour = _colour(b.color)
            if colour is None:
                continue
            box = (float(b.xmin), float(b.ymin), float(b.xmax), float(b.ymax))
            h, u = box[3] - box[1], 0.5 * (box[0] + box[2])
            if h < 16:
                continue
            z_l, npts = _lidar_depth(pix, z, box)
            prior_depth = analytic_monocular_depth(h, fy, CONE_H[colour])
            row = dict(stamp=s, color=colour, h=h, u=u, z_lidar=z_l, npts=npts,
                       prior_depth=prior_depth)
            # the node's path: prior from intrinsics, window centred on prior+bias
            if prior_depth is not None and 0.5 <= prior_depth <= 12.0:
                for tag, yaw in (("cfg", math.radians(args.yaw_deg)), ("online", estimator.yaw_rad()), ("none", 0.0)):
                    bias = yaw_disparity_bias_px(u, fx, cx, yaw) or 0.0
                    expected = fx * B / prior_depth + bias
                    m = zncc_disparity(lg, rg, box, expected * args.low, expected * args.high,
                                       min_score=args.min_score)
                    if m is not None and m.disparity_px - bias > 0:
                        row[f"z_{tag}"] = fx * B / (m.disparity_px - bias)
                        row[f"s_{tag}"] = m.score
            # wide, unconstrained peak: the raw offset measurement
            mw = zncc_disparity(lg, rg, box, 1.0, 150.0, min_score=-1.0)
            if mw is not None:
                row["d_wide"], row["s_wide"] = mw.disparity_px, mw.score
            # feed the estimator exactly like the node (LiDAR depth 2..12 m)
            if z_l is not None and 2.0 <= z_l <= 12.0:
                centre = fx * B / z_l + (yaw_disparity_bias_px(u, fx, cx, estimator.yaw_rad()) or 0.0)
                me = zncc_disparity(lg, rg, box, max(0.0, centre - 10.0), centre + 10.0,
                                    min_score=args.min_score)
                if me is not None:
                    estimator.add(me.disparity_px, z_l, u, fx, cx, B)
            row["yaw_online_deg"] = math.degrees(estimator.yaw_rad())
            row["yaw_n"] = estimator.count
            rows.append(row)
        frames += 1

    while reader.has_next() and frames < args.frames:
        topic, data, _t = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == TOPICS["tf"]:
            for tr in msg.transforms:
                if tr.header.frame_id == "base_footprint" and tr.child_frame_id in ("zed_left_camera_optical_frame", "zed_left_camera_frame_optical"):
                    T_cam = _T(tr)
                if tr.header.frame_id == "base_footprint" and tr.child_frame_id == "rslidar":
                    T_lidar = _T(tr)
        elif topic == TOPICS["info"]:
            if K is None:
                K = np.array(msg.p).reshape(3, 4)[:, :3]
        elif topic in (TOPICS["left"], TOPICS["right"]):
            store = lefts if topic == TOPICS["left"] else rights
            store[_ns(msg.header.stamp)] = msg
            while len(store) > 40:
                store.pop(next(iter(store)))
        elif topic == TOPICS["cloud"]:
            clouds.append((_ns(msg.header.stamp), msg))
            del clouds[:-6]
        elif topic == TOPICS["bbox"]:
            pending.append(msg)
        still = []
        newest = max(lefts) if lefts else 0
        for bb in pending:
            if newest - _ns(bb.image_header.stamp) > 300_000_000:
                process(bb)
            else:
                still.append(bb)
        pending = still

    if not rows:
        sys.exit("no boxes processed: check the topics and that the bag has bboxes + images")
    if args.csv:
        keys = sorted({k for r in rows for k in r})
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
    fx, cx = K[0, 0], K[0, 2]
    ref = [r for r in rows if r["z_lidar"] is not None and r.get("d_wide") is not None
           and r.get("s_wide", 0) >= 0.6 and 1.5 <= r["z_lidar"] <= 12.0 and r["npts"] >= 5]
    print(f"frames {frames}  boxes {len(rows)}  LiDAR-referenced {len(ref)}")
    if ref:
        off = np.array([r["d_wide"] - fx * B / r["z_lidar"] for r in ref])
        x = np.array([(r["u"] - cx) / fx for r in ref])
        yaw = np.median(off / (fx * (1 + x * x)))
        print(f"raw pair bias (ZNCC wide peak - LiDAR): median {np.median(off):+.2f} px, "
              f"yaw-only fit {math.degrees(yaw):+.3f} deg (centre {fx*yaw:+.2f} px), "
              f"resid MAD {np.median(np.abs(off - fx*yaw*(1+x*x))):.2f} px")
        pr = np.array([r["prior_depth"] / r["z_lidar"] for r in ref if r["prior_depth"]])
        print(f"prior (D=fy*H/h) / LiDAR: median {np.median(pr):.3f}")
        print(f"online estimator after the run: {rows[-1]['yaw_online_deg']:+.3f} deg (n={rows[-1]['yaw_n']})")
    for tag, label in (("none", "no correction"), ("cfg", f"yaw {args.yaw_deg:+.2f} deg (config)"),
                       ("online", "online estimate (as it evolved)")):
        sel = [r for r in rows if r["z_lidar"] is not None and r.get(f"z_{tag}") is not None
               and 1.5 <= r["z_lidar"] <= 12.0 and r["npts"] >= 5]
        if not sel:
            continue
        e = np.array([(r[f"z_{tag}"] - r["z_lidar"]) / r["z_lidar"] for r in sel])
        print(f"-- {label}: n={len(e)} rel err median {np.median(e)*100:+.1f}%  "
              f"|e|<10%: {(np.abs(e)<0.1).mean()*100:.0f}%  |e|<20%: {(np.abs(e)<0.2).mean()*100:.0f}%")
        zl = np.array([r["z_lidar"] for r in sel])
        for lo, hi in ((1.5, 4), (4, 6), (6, 8), (8, 10), (10, 12)):
            m = (zl >= lo) & (zl < hi)
            if m.sum() < 5:
                continue
            print(f"     {lo:4.1f}-{hi:4.1f} m n={m.sum():4d} med {np.median(e[m])*100:+6.1f}%  "
                  f"p10 {np.percentile(e[m],10)*100:+6.1f}%  p90 {np.percentile(e[m],90)*100:+6.1f}%  "
                  f"|e|<10% {(np.abs(e[m])<0.1).mean()*100:3.0f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
