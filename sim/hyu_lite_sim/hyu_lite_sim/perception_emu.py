"""Emulated perception output: what hyu_perception would publish on
/perception/cones (hyu_msgs/ConeArrayWithCovariance, base_footprint) given
the true cones, the clutter and the car pose.

Model (numbers from perception.yaml / the 2026-08 evaluations):
  * LiDAR sees cones from min_range (ROI roi_min_x 1.4) to max_range within
    lidar_fov (the nose RS-16 is cropped to a forward 180 deg wedge).
  * Detection probability is 1 up to near_range and falls linearly to
    p_detect_far at max_range.
  * Colour needs the camera: inside camera_fov and within colour_max_range
    the cone gets its colour with probability p_colour(range) (linear from
    p_colour_near to p_colour_far); otherwise it is published as
    unknown_color. A small colour-confusion rate swaps blue/yellow.
  * Position noise: lateral/longitudinal sigmas plus a speed-proportional
    term (deskew residual), rare gross outliers.
  * Covariance tiers measured from LIVE perception on bag/0801_sensors/
    17_22_34 (2026-08-23, tools scripts/analyze_perception + zncc_bag_check):
    coloured <=8 m cov 0.07, unknown 0.23 (both == perception.yaml), far
    >12 m cov ~1.07. Colour range capped at 8 m because ZNCC depth error
    jumps from ~0%% (4-8 m) to -18..-25%% (8-12 m). Colour probabilities are
    30 Hz-corrected: the bag's YOLO ran 3.1 Hz so its raw 55%% colour rate is
    depressed by stale bboxes (fresh <0.25 s only 74%% of frames); at the real
    30 Hz colour recovers toward ~70-77%%.
  * Clutter returns are LiDAR-only (unknown colour) with their own detection
    probability and jitter; plus transient false positives per frame.
"""
import math
import random
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

from .track import Clutter, Cone, clutter_returns


@dataclass
class PerceptionParams:
    rate_hz: float = 10.0
    latency_s: float = 0.08
    min_range_m: float = 1.5
    max_range_m: float = 20.0
    lidar_fov_deg: float = 180.0
    camera_fov_deg: float = 100.0
    near_range_m: float = 10.0             # RS16 geometry: >=2 beams to ~10 m (robust); 1 beam 11-20 m
    p_detect_far: float = 0.15             # single-beam far regime (flaky per frame)
    colour_min_range_m: float = 2.0     # below this the camera cannot frame a cone -> stays unknown (measured 0-2 m only 4% coloured even at 30 Hz)
    colour_max_range_m: float = 8.0     # 2026-08-23 zncc_bag_check: ZNCC trustworthy <=8 m
    sparse_range_m: float = 12.0        # beyond this LiDAR returns go thin -> sparse tier (measured cov jumps ~0.23 -> ~1.07)
    p_colour_near: float = 0.85            # 30 Hz colour rate for in-range cones, measured on FRESH-bbox frames (2-8 m ~84%, flat)
    p_colour_far: float = 0.83
    colour_confusion: float = 0.01
    sigma_lat_m: float = 0.03
    sigma_lon_m: float = 0.04
    sigma_per_mps: float = 0.015
    outlier_prob: float = 0.004
    outlier_m: float = 0.6
    var_coloured: float = 0.07                     # lidar_variance_x/y (variance, m^2)
    var_unknown: float = 0.23                      # lidar_only_variance_x/y
    var_far_lat: float = 0.90              # measured far (>12 m) cov ~1.07 on 0801_sensors
    var_far_lon: float = 1.20
    false_positives_per_frame: float = 0.25
    clutter_colour_prob: float = 0.0               # chance a clutter return gets a (wrong) colour


def _yaw_cs(yaw: float):
    return math.cos(yaw), math.sin(yaw)


def _poisson(rng: random.Random, lam: float) -> int:
    """Knuth's Poisson sampler on the emulator's own RNG (keeps noise_seed repeatable)."""
    limit = math.exp(-lam)
    k, prod = 0, rng.random()
    while prod > limit:
        k += 1
        prod *= rng.random()
    return k


class PerceptionEmulator:
    def __init__(self, params: PerceptionParams, cones: Sequence[Cone], clutter: Sequence[Clutter],
                 rng: random.Random):
        self.p = params
        self.rng = rng
        self.set_world(cones, clutter)

    def set_world(self, cones: Sequence[Cone], clutter: Sequence[Clutter]) -> None:
        self.cones = list(cones)
        self.cone_xy = np.array([[c.x, c.y] for c in cones], dtype=float).reshape(-1, 2)
        self.cone_tag = [c.tag for c in cones]
        rets = clutter_returns(clutter)
        self.clutter_xy = np.array([[r[0], r[1]] for r in rets], dtype=float).reshape(-1, 2)
        self.clutter_p = np.array([r[2] for r in rets], dtype=float)
        self.clutter_jit = np.array([r[3] for r in rets], dtype=float)

    def _to_base(self, xy: np.ndarray, base_pose):
        bx, by, yaw = base_pose
        c, s = _yaw_cs(yaw)
        d = xy - np.array([bx, by])
        x = c * d[:, 0] + s * d[:, 1]
        y = -s * d[:, 0] + c * d[:, 1]
        return x, y

    def _visible(self, x, y, fov_deg):
        r = np.hypot(x, y)
        ang = np.degrees(np.abs(np.arctan2(y, x)))
        return (r >= self.p.min_range_m) & (r <= self.p.max_range_m) & (ang <= 0.5 * fov_deg), r, ang

    def observe(self, base_pose, speed_mps: float) -> Dict[str, List[Tuple[float, float, List[float]]]]:
        """-> {colour: [(x, y, [xx, xy, yx, yy]), ...]} in base_footprint."""
        p, rng = self.p, self.rng
        out: Dict[str, List] = {k: [] for k in ('blue', 'yellow', 'orange', 'big_orange', 'unknown')}
        sig_extra = p.sigma_per_mps * abs(speed_mps)

        def add(colour, x, y, far):
            # noise in the ray frame
            r = math.hypot(x, y)
            if r < 1e-6:
                return
            ux, uy = x / r, y / r
            e_lon = rng.gauss(0.0, p.sigma_lon_m + sig_extra)
            e_lat = rng.gauss(0.0, p.sigma_lat_m + sig_extra)
            if rng.random() < p.outlier_prob:
                e_lon += rng.choice((-1.0, 1.0)) * p.outlier_m
            xn = x + ux * e_lon - uy * e_lat
            yn = y + uy * e_lon + ux * e_lat
            if r > p.sparse_range_m:
                # sparse LiDAR tier: thin far returns, high anisotropic covariance
                # (measured >12 m cov ~1.07 on 0801_sensors), rotated into base
                c2, s2 = ux * ux, uy * uy
                cs = ux * uy
                vl, vt = p.var_far_lon, p.var_far_lat
                cov = [vl * c2 + vt * s2, (vl - vt) * cs, (vl - vt) * cs, vl * s2 + vt * c2]
            elif colour == 'unknown':
                v = p.var_unknown
                cov = [v, 0.0, 0.0, v]
            else:
                v = p.var_coloured
                cov = [v, 0.0, 0.0, v]
            out[colour].append((xn, yn, cov))

        # --- track cones ---
        if len(self.cone_xy):
            x, y = self._to_base(self.cone_xy, base_pose)
            vis, r, ang = self._visible(x, y, p.lidar_fov_deg)
            for i in np.nonzero(vis)[0]:
                ri = float(r[i])
                if ri <= p.near_range_m:
                    pdet = 1.0
                else:
                    f = (ri - p.near_range_m) / max(1e-6, p.max_range_m - p.near_range_m)
                    pdet = 1.0 + (p.p_detect_far - 1.0) * f
                if rng.random() > pdet:
                    continue
                colour = 'unknown'
                far = ri > p.colour_max_range_m
                # Colour needs the camera: a fresh bbox (~always at 30 Hz) AND the
                # cone framed. Below colour_min_range the camera cannot frame it.
                if ang[i] <= 0.5 * p.camera_fov_deg and p.colour_min_range_m <= ri <= p.colour_max_range_m:
                    span = max(1e-6, p.colour_max_range_m - p.colour_min_range_m)
                    f = (ri - p.colour_min_range_m) / span
                    pcol = p.p_colour_near + (p.p_colour_far - p.p_colour_near) * f
                    if rng.random() < pcol:
                        colour = self.cone_tag[i]
                        if colour in ('blue', 'yellow') and rng.random() < p.colour_confusion:
                            colour = 'yellow' if colour == 'blue' else 'blue'
                add(colour, float(x[i]), float(y[i]), far)
        # --- clutter ---
        if len(self.clutter_xy):
            x, y = self._to_base(self.clutter_xy, base_pose)
            vis, r, ang = self._visible(x, y, p.lidar_fov_deg)
            for i in np.nonzero(vis)[0]:
                ri = float(r[i])
                pdet = float(self.clutter_p[i])
                if ri > p.near_range_m:
                    f = (ri - p.near_range_m) / max(1e-6, p.max_range_m - p.near_range_m)
                    pdet *= 1.0 + (p.p_detect_far - 1.0) * f
                if rng.random() > pdet:
                    continue
                j = float(self.clutter_jit[i])
                colour = 'unknown'
                if p.clutter_colour_prob > 0.0 and rng.random() < p.clutter_colour_prob:
                    colour = rng.choice(('blue', 'yellow'))
                add(colour, float(x[i]) + rng.gauss(0.0, j), float(y[i]) + rng.gauss(0.0, j),
                    ri > p.colour_max_range_m)
        # --- transient false positives ---
        n_fp = _poisson(rng, p.false_positives_per_frame) if p.false_positives_per_frame > 0 else 0
        for _ in range(int(n_fp)):
            r = rng.uniform(p.min_range_m, p.max_range_m)
            a = math.radians(rng.uniform(-0.5 * p.lidar_fov_deg, 0.5 * p.lidar_fov_deg))
            add('unknown', r * math.cos(a), r * math.sin(a), r > p.colour_max_range_m)
        return out

    def truth_visible(self, base_pose) -> Dict[str, List[Tuple[float, float]]]:
        """Noise-free cones inside the LiDAR envelope (for /ground_truth/cones)."""
        out: Dict[str, List] = {k: [] for k in ('blue', 'yellow', 'orange', 'big_orange', 'unknown')}
        if not len(self.cone_xy):
            return out
        x, y = self._to_base(self.cone_xy, base_pose)
        vis, _, _ = self._visible(x, y, self.p.lidar_fov_deg)
        for i in np.nonzero(vis)[0]:
            out[self.cone_tag[i]].append((float(x[i]), float(y[i])))
        return out
