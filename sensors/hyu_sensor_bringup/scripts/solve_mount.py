#!/usr/bin/env python3
"""Where is the ground, seen from the camera?  ->  base_footprint.

base_footprint is DEFINED as the point on the ground directly below the ZED's
stereo centre, +x = camera forward projected onto the ground, +z = ground
normal.  Nothing about the car is measured with a tape: the only inputs are
the LiDAR<->camera extrinsic (calib.sh) and a few LiDAR frames of the car
standing on flat ground.  This script turns those into the three numbers
sensors.launch.py needs to publish the whole TF chain:

    camera_height_m   left optical centre above the ground plane
    camera_roll_deg   camera body vs. the ground normal, ROS body rpy
    camera_pitch_deg  (yaw is 0 by definition -- base +x IS camera forward)

How: fit the dominant near-horizontal plane in each LiDAR frame (RANSAC +
SVD, normal held within --max-tilt of the LiDAR's +z so a wall or the
ChArUco board can never win), carry it into the camera optical frame with the
extrinsic, and read the camera's height and tilt straight off the plane.
The per-frame spread is printed: with the car parked it should be a few mm.

    ros2 run hyu_sensor_bringup solve_mount.py --pcd captures/<session>
    ros2 run hyu_sensor_bringup solve_mount.py --bag bag/0801_outdoor/rosbag2_...
    ros2 run hyu_sensor_bringup solve_mount.py --pcd <session> --write

--write stores the result as a `ground:` block INSIDE the extrinsic yaml.  It
belongs there and nowhere else: the plane is expressed through THIS extrinsic,
so a different extrinsic needs a different block, and `calib -e <name>` then
switches both together.  calib.sh runs this on the capture session right
after the solve, so a fresh calibration needs no manual step.  Old extrinsics
without the block fall back to config/vehicle_mount.yaml (tape values).

Assumes the ground under the car is flat where it was measured; a slope
becomes a base tilt.  The calibration site is fine, a kerb is not.
"""
import argparse
import glob
import math
import os
import sys

import numpy as np
import yaml

DEFAULT_EXTRINSIC = os.path.expanduser('~/fsk/extrinsics/active')
LIDAR_TOPIC = '/sensors/lidar/points'

# ROS body (x fwd, y left, z up) <- optical (x right, y down, z fwd): the ZED
# URDF's left_camera_frame -> left_camera_frame_optical joint is rpy
# (-pi/2, 0, -pi/2), and this is that rotation as a matrix.
R_BODY_OPT = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])


# ── clouds ────────────────────────────────────────────────────────────────
def read_pcd(path):
    """ASCII PCD (what sync_capture.py writes) -> Nx3 float64."""
    with open(path, 'rb') as f:
        header, fields = [], None
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f'{path}: no DATA line')
            text = line.decode('ascii', 'replace').strip()
            header.append(text)
            if text.startswith('FIELDS'):
                fields = text.split()[1:]
            if text.startswith('DATA'):
                kind = text.split()[1]
                break
        if kind != 'ascii':
            raise ValueError(f'{path}: DATA {kind} not supported (ascii only)')
        cols = [fields.index(c) for c in ('x', 'y', 'z')]
        data = np.loadtxt(f, usecols=cols, dtype=np.float64, ndmin=2)
    return data


def clouds_from_pcd(args_pcd):
    paths = []
    for a in args_pcd:
        if os.path.isdir(a):
            paths.extend(sorted(glob.glob(os.path.join(a, '*.pcd'))))
        else:
            paths.extend(sorted(glob.glob(a)) or [a])
    if not paths:
        sys.exit(f'no .pcd under {args_pcd}')
    return [(os.path.basename(p), read_pcd(p)) for p in paths]


def clouds_from_bag(bag_dir, topic, n_frames):
    """Evenly spaced frames of `topic` across the bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    def cloud_xyz(msg):
        # A structured view over the raw buffer: the RS-16 mixes float32
        # xyz/intensity with a float64 timestamp, which sensor_msgs_py's
        # read_points_numpy refuses.
        by_name = {f.name: f for f in msg.fields}
        dt = np.dtype({'names': ['x', 'y', 'z'],
                       'formats': ['<f4'] * 3,
                       'offsets': [by_name[c].offset for c in 'xyz'],
                       'itemsize': msg.point_step})
        rec = np.frombuffer(bytes(msg.data), dtype=dt,
                            count=msg.width * msg.height)
        return np.stack([rec['x'], rec['y'], rec['z']], axis=1).astype(np.float64)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    total = 0
    for meta in reader.get_metadata().topics_with_message_count:
        if meta.topic_metadata.name == topic:
            total = meta.message_count
    if total == 0:
        sys.exit(f'{bag_dir}: no messages on {topic}')
    want = sorted(set(int(round(i * (total - 1) / max(1, n_frames - 1)))
                      for i in range(min(n_frames, total))))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out, idx = [], 0
    while reader.has_next() and len(out) < len(want):
        _, raw, _ = reader.read_next()
        if idx == want[len(out)]:
            out.append((f'#{idx}', cloud_xyz(deserialize_message(raw, PointCloud2))))
        idx += 1
    return out


# ── plane ─────────────────────────────────────────────────────────────────
def fit_plane_svd(pts):
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    n = vt[-1]
    return np.append(n, -n @ c)


def ground_plane(xyz, r_min, r_max, max_tilt_deg, thresh, iters, seed):
    """Dominant near-horizontal plane, (a, b, c, d) with |n| = 1, n up."""
    xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    pts = xyz[(r >= r_min) & (r <= r_max)]
    if pts.shape[0] < 50:
        return None, 0
    cos_max = math.cos(math.radians(max_tilt_deg))
    rng = np.random.default_rng(seed)
    best, best_n = None, 0
    for _ in range(iters):
        s = pts[rng.choice(pts.shape[0], 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if n[2] < 0:
            n = -n
        if n[2] < cos_max:
            continue
        d = -n @ s[0]
        if d <= 0:          # sensor below the plane: a ceiling, not the ground
            continue
        cnt = int(np.count_nonzero(np.abs(pts @ n + d) <= thresh))
        if cnt > best_n:
            best_n, best = cnt, np.append(n, d)
    if best is None:
        return None, 0
    # Two rounds of re-select + least squares tighten the RANSAC sample.
    plane = best
    for _ in range(2):
        inl = np.abs(pts @ plane[:3] + plane[3]) <= thresh
        cand = fit_plane_svd(pts[inl])
        if cand[2] < 0:
            cand = -cand
        if cand[2] >= cos_max:
            plane = cand
    inl = np.abs(pts @ plane[:3] + plane[3]) <= thresh
    rms = float(np.sqrt(np.mean((pts[inl] @ plane[:3] + plane[3]) ** 2)))
    if plane[3] <= 0:       # refinement wandered above the sensor -- reject
        return None, 0
    return plane, rms       # n up, sensor on the positive side


def camera_from_plane(plane_l, R, t):
    """Ground plane in the LiDAR frame -> camera height, roll, pitch.

    p_cam = R p_lidar + t, so n_cam = R n_lidar and d_cam = d_lidar - n_cam.t.
    The camera's signed distance along the up-normal is then simply d_cam.
    """
    n_c = R @ plane_l[:3]
    d_c = plane_l[3] - n_c @ t
    n_body = R_BODY_OPT @ n_c            # ground normal in the camera body frame
    # For R_base_cam = Ry(pitch) Rx(roll), the base +z seen from the camera is
    # (-sin p, cos p sin r, cos p cos r).
    pitch = -math.asin(max(-1.0, min(1.0, n_body[0])))
    roll = math.atan2(n_body[1], n_body[2])
    return float(d_c), math.degrees(roll), math.degrees(pitch), n_c


# ── main ──────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--pcd', nargs='+', metavar='DIR|FILE',
                     help='capture session dir(s) or .pcd files (LiDAR frame)')
    src.add_argument('--bag', metavar='DIR', help='rosbag2 directory')
    ap.add_argument('--topic', default=LIDAR_TOPIC)
    ap.add_argument('--frames', type=int, default=10,
                    help='frames sampled from a bag (default 10)')
    ap.add_argument('--extrinsic', default=DEFAULT_EXTRINSIC)
    ap.add_argument('--r-min', type=float, default=2.0, help='m, from the LiDAR')
    ap.add_argument('--r-max', type=float, default=12.0)
    ap.add_argument('--max-tilt', type=float, default=20.0,
                    help='deg, plane normal vs. LiDAR +z')
    ap.add_argument('--thresh', type=float, default=0.03, help='m, inlier band')
    ap.add_argument('--iters', type=int, default=300)
    ap.add_argument('--write', action='store_true',
                    help='store the result as `ground:` in the extrinsic yaml')
    a = ap.parse_args()

    ext_path = os.path.realpath(a.extrinsic)
    with open(ext_path) as f:
        ext = yaml.safe_load(f) or {}
    l2c = ext.get('lidar_to_camera') or {}
    if not l2c:
        sys.exit(f'{ext_path}: no lidar_to_camera block')
    R = np.array(l2c['R'], dtype=np.float64)
    t = np.array(l2c['t'], dtype=np.float64)

    if a.pcd:
        clouds = clouds_from_pcd(a.pcd)
        source = os.path.relpath(os.path.dirname(os.path.realpath(
            glob.glob(os.path.join(a.pcd[0], '*.pcd'))[0]
            if os.path.isdir(a.pcd[0]) else a.pcd[0])), os.path.expanduser('~/fsk'))
    else:
        clouds = clouds_from_bag(a.bag, a.topic, a.frames)
        source = os.path.relpath(os.path.realpath(a.bag), os.path.expanduser('~/fsk'))

    print(f'extrinsic: {ext_path}')
    print(f'{"frame":>10} {"pts":>7} {"h_cam[m]":>9} {"roll":>7} {"pitch":>7} '
          f'{"lidar_h":>8} {"rms_mm":>7}')
    rows = []
    for name, xyz in clouds:
        plane, rms = ground_plane(xyz, a.r_min, a.r_max, a.max_tilt, a.thresh,
                                  a.iters, seed=len(rows))
        if plane is None:
            print(f'{name:>10} {xyz.shape[0]:>7}   -- no near-horizontal plane --')
            continue
        h, roll, pitch, _ = camera_from_plane(plane, R, t)
        rows.append((h, roll, pitch, plane[3], rms))
        print(f'{name:>10} {xyz.shape[0]:>7} {h:9.4f} {roll:7.2f} {pitch:7.2f} '
              f'{plane[3]:8.3f} {rms * 1e3:7.1f}')
    if not rows:
        sys.exit('no frame yielded a ground plane -- is the LiDAR seeing the ground?')

    arr = np.array(rows)
    med = np.median(arr, axis=0)
    std = arr.std(axis=0)
    print()
    print(f'camera_height_m : {med[0]:.4f}  (std {std[0] * 1e3:.1f} mm, {len(rows)} frames)')
    print(f'camera_roll_deg : {med[1]:+.2f}  (std {std[1]:.2f})')
    print(f'camera_pitch_deg: {med[2]:+.2f}  (std {std[2]:.2f})')
    print(f'lidar above ground: {med[3]:.3f} m   plane rms {med[4] * 1e3:.1f} mm')
    if std[0] > 0.02 or std[1] > 0.5 or std[2] > 0.5:
        print('WARNING: frames disagree -- was the car moving, or the ground uneven?')

    if not a.write:
        print(f'\n(dry run; add --write to store this in {ext_path})')
        return
    ext.pop('ground', None)
    ground = {
        'camera_height_m': round(float(med[0]), 4),
        'camera_roll_deg': round(float(med[1]), 3),
        'camera_pitch_deg': round(float(med[2]), 3),
        'lidar_height_m': round(float(med[3]), 4),
        'plane_rms_mm': round(float(med[4] * 1e3), 2),
        'height_std_mm': round(float(std[0] * 1e3), 2),
        'n_frames': int(len(rows)),
        'source': source,
        'note': 'base_footprint = ground point below the stereo centre; '
                'written by solve_mount.py, read by sensors.launch.py',
    }
    with open(ext_path, 'w') as f:
        yaml.safe_dump(ext, f, sort_keys=False, default_flow_style=None, width=100)
        yaml.safe_dump({'ground': ground}, f, sort_keys=False,
                       default_flow_style=False, width=100)
    print(f'\nwritten: ground -> {ext_path}')


if __name__ == '__main__':
    main()
