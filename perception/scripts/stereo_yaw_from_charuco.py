#!/usr/bin/env python3
"""Residual stereo yaw of the ZED's rectified pair, from ChArUco captures.

The SDK publishes a rectified left/right pair; if its calibration has drifted,
a point at infinity no longer sits at disparity 0. This reads the L/R PNGs a
calib.sh capture session leaves behind (captures/<session>/NNNN_L.png, _R.png,
the same 8x7 / 0.12 m / DICT_5X5_50 board the LiDAR-camera solve uses) and
reports, per pair and overall:

  * vertical residual vL - vR        (should be ~0: rows aligned)
  * disparity offset (uL-uR) - fx*B/z  (z from the left PnP; the yaw bias)
  * stereoCalibrate with K fixed     -> R, T; R's y-component IS the yaw

The yaw goes into perception.yaml as stereo_right_yaw_deg (sign: positive
when the observed disparity is too large, which is what +fx*yaw*(1+x^2)
describes -- the table's "disparity offset" has that sign; stereoCalibrate's
own convention is the opposite, so read the sign off the offset column).

    ros2 run hyu_perception stereo_yaw_from_charuco.py captures/20260801_145355 \
        [--fx 676.951 --cx 653.366 --cy 407.603] [--baseline 0.12]

Measured 2026-08-23 on S/N 14352: offset +3.7 px (centre) / +5.0 (board at
the side), yaw 0.25-0.29 deg over two sessions, vertical residual -0.08 px.
"""
import argparse
import glob
import math
import os
import sys

import cv2
import cv2.aruco as aruco
import numpy as np


def _dictionary(name):
    enum = getattr(aruco, name)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(enum)
    return aruco.Dictionary_get(enum)


def _board(sx, sy, square, marker, dictionary):
    if hasattr(aruco, "CharucoBoard_create"):
        return aruco.CharucoBoard_create(sx, sy, square, marker, dictionary)
    board = aruco.CharucoBoard((sx, sy), square, marker, dictionary)
    if hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(True)
    return board


def _detect(gray, dictionary, board, K, D):
    corners, ids, _ = aruco.detectMarkers(gray, dictionary)
    if ids is None or len(ids) < 4:
        return None, None
    n, cc, ci = aruco.interpolateCornersCharuco(corners, ids, gray, board,
                                                cameraMatrix=K, distCoeffs=D)
    if cc is None or n < 8:
        return None, None
    return cc.reshape(-1, 2), ci.ravel()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", help="capture directories with NNNN_L.png / NNNN_R.png")
    ap.add_argument("--fx", type=float, default=676.951)
    ap.add_argument("--cx", type=float, default=653.366)
    ap.add_argument("--cy", type=float, default=407.603)
    ap.add_argument("--baseline", type=float, default=0.12)
    ap.add_argument("--squares", type=int, nargs=2, default=(8, 7))
    ap.add_argument("--square", type=float, default=0.12)
    ap.add_argument("--marker", type=float, default=0.09)
    ap.add_argument("--dict", default="DICT_5X5_50")
    args = ap.parse_args()
    K = np.array([[args.fx, 0.0, args.cx], [0.0, args.fx, args.cy], [0.0, 0.0, 1.0]])
    D = np.zeros(5)
    dictionary = _dictionary(args.dict)
    board = _board(args.squares[0], args.squares[1], args.square, args.marker, dictionary)
    obj = board.chessboardCorners.reshape(-1, 3)
    fx, B = args.fx, args.baseline
    for session in args.sessions:
        rows, obj_all, l_all, r_all, size = [], [], [], [], None
        for lp in sorted(glob.glob(os.path.join(session, "*_L.png"))):
            rp = lp.replace("_L.png", "_R.png")
            if not os.path.exists(rp):
                continue
            li, ri = cv2.imread(lp, cv2.IMREAD_GRAYSCALE), cv2.imread(rp, cv2.IMREAD_GRAYSCALE)
            if li is None or ri is None:
                continue
            size = (li.shape[1], li.shape[0])
            cl, il = _detect(li, dictionary, board, K, D)
            cr, ir = _detect(ri, dictionary, board, K, D)
            if cl is None or cr is None:
                continue
            common = np.intersect1d(il, ir)
            if len(common) < 8:
                continue
            pl = np.array([cl[list(il).index(i)] for i in common])
            pr = np.array([cr[list(ir).index(i)] for i in common])
            ok, rvec, tvec = cv2.solvePnP(obj[common], pl, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                continue
            R, _ = cv2.Rodrigues(rvec)
            z = (obj[common] @ R.T + tvec.ravel())[:, 2]
            du, dv = pl[:, 0] - pr[:, 0], pl[:, 1] - pr[:, 1]
            off = du - fx * B / z
            x = (pl[:, 0] - args.cx) / fx
            yaw = np.median(off / (fx * (1.0 + x * x)))
            rows.append((os.path.basename(lp), len(common), float(np.median(z)),
                         float(np.median(pl[:, 0])), float(np.median(dv)),
                         float(np.median(off)), math.degrees(yaw)))
            obj_all.append(obj[common].astype(np.float32))
            l_all.append(pl.astype(np.float32)); r_all.append(pr.astype(np.float32))
        print(f"== {session}: {len(rows)} usable pairs")
        for r in rows:
            print(f"   {r[0]} n={r[1]:2d} z={r[2]:4.2f} m u={r[3]:6.1f}  vL-vR {r[4]:+5.2f} px  "
                  f"disp offset {r[5]:+5.2f} px  -> yaw {r[6]:+.3f} deg")
        if not rows:
            continue
        arr = np.array([[r[4], r[5], r[6]] for r in rows])
        print(f"   ALL: vertical residual med {np.median(arr[:, 0]):+.2f} px, disparity offset med "
              f"{np.median(arr[:, 1]):+.2f} px, yaw med {np.median(arr[:, 2]):+.3f} deg "
              f"(p25 {np.percentile(arr[:, 2], 25):+.3f} p75 {np.percentile(arr[:, 2], 75):+.3f})")
        if len(obj_all) >= 2:
            ret, *_rest = cv2.stereoCalibrate(
                obj_all, l_all, r_all, K, D, K, D, size, flags=cv2.CALIB_FIX_INTRINSIC,
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-7))
            Rst, Tst = _rest[4], _rest[5]
            rv, _ = cv2.Rodrigues(Rst)
            ang = np.degrees(rv.ravel())
            print(f"   stereoCalibrate (K fixed): rms {ret:.3f} px  R x/y/z {ang.round(3)} deg  "
                  f"|T| {np.linalg.norm(Tst):.4f} m (baseline {B})")
        print(f"   -> perception.yaml stereo_right_yaw_deg: {np.median(arr[:, 2]):+.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
