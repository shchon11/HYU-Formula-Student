# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Behaviour checks for the lidar_realism module functions (no ROS spin).

What is pinned here and why:
  * the output wire layout IS the humble velodyne_pointcloud PointXYZIRT
    layout — field parity is the whole point of the bridge;
  * a perfect deskewer (inverse constant-twist warp) recovers the ideal
    cloud exactly — the contract that makes deskew code testable in sim;
  * the dropout model touches nothing inside the perception envelope
    (cones keep >10x snr margin) and does kill far grazing ground.
"""
import importlib.util
import math
import os

import numpy as np
import pytest

_MODULE_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "lidar_realism.py"
)


@pytest.fixture(scope="module")
def mod():
    """Import the node script as a module by path."""
    spec = importlib.util.spec_from_file_location("lidar_realism", _MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_wire_layout_matches_real_driver(mod):
    """Offsets/itemsize must be velodyne_pointcloud's 22-byte PointXYZIRT."""
    dtype = mod._OUT_DTYPE
    assert dtype.names == ("x", "y", "z", "intensity", "ring", "time")
    assert [dtype.fields[n][1] for n in dtype.names] == [0, 4, 8, 12, 16, 18]
    assert dtype.itemsize == 22
    assert dtype.fields["ring"][0] == np.dtype(np.uint16)
    assert dtype.fields["time"][0] == np.dtype(np.float32)
    offsets = [f.offset for f in mod._OUT_FIELDS]
    assert offsets == [0, 4, 8, 12, 16, 18]


def test_sweep_progress_endpoints(mod):
    """Clockwise runs +pi -> -pi, ccw the reverse; both cover [0, 1]."""
    azimuth = np.array([math.pi, 0.0, -math.pi])
    assert np.allclose(mod.sweep_progress(azimuth, "cw"), [0.0, 0.5, 1.0])
    assert np.allclose(mod.sweep_progress(azimuth, "ccw"), [1.0, 0.5, 0.0])


def test_ring_mapping_is_bottom_up(mod):
    """VLP-16 rings: 0 at -15 deg, 2 deg spacing, clipped at the ends."""
    elev = np.radians(np.arange(-15.0, 16.0, 2.0))
    assert (mod.ring_from_elevation(elev) == np.arange(16)).all()
    rounded = mod.ring_from_elevation(np.radians(np.array([-15.4, 14.6, -30.0, 30.0])))
    assert (rounded == np.array([0, 15, 0, 15])).all()


def test_deskew_inverse_recovers_ideal_cloud(mod):
    """R(+w*tau) @ p' + v*tau must reproduce the ideal points exactly."""
    rng = np.random.default_rng(7)
    xyz = rng.uniform(-20, 20, size=(5000, 3))
    azimuth = np.arctan2(xyz[:, 1], xyz[:, 0])
    tau = (mod.sweep_progress(azimuth, "cw") - 1.0) * 0.1  # end-stamp
    vx, vy, omega = 6.0, 0.4, 0.9

    distorted = mod.distort(xyz, tau, vx, vy, omega)

    cos_t, sin_t = np.cos(omega * tau), np.sin(omega * tau)
    recovered = np.empty_like(distorted)
    recovered[:, 0] = cos_t * distorted[:, 0] - sin_t * distorted[:, 1] + vx * tau
    recovered[:, 1] = sin_t * distorted[:, 0] + cos_t * distorted[:, 1] + vy * tau
    recovered[:, 2] = distorted[:, 2]
    assert np.abs(recovered - xyz).max() < 1e-9

    # Effect size is bounded by (|v| + w*r_max)*T; rotation dominates at
    # range, which is exactly the turn-in smear deskewing exists to fix.
    r_max = np.linalg.norm(xyz[:, :2], axis=1).max()
    bound = (math.hypot(vx, vy) + omega * r_max) * 0.1
    displacement = np.linalg.norm(distorted[:, :2] - xyz[:, :2], axis=1)
    assert 0.4 < displacement.max() <= bound
    assert displacement.min() < 0.02


def test_dropout_spares_the_perception_envelope(mod):
    """Cones and near ground keep; far grazing ground and far objects fade."""
    height, ground_max, object_max = 0.6, 28.0, 80.0

    def ground(r):
        return r, -math.asin(height / r)

    cases = [
        (10.0, math.radians(-3.0), True),   # cone at 10 m
        (12.0, math.radians(-5.0), True),   # cone at the sparse cap
        (ground(15.0)[0], ground(15.0)[1], True),
        (ground(22.0)[0], ground(22.0)[1], True),
        (ground(40.0)[0], ground(40.0)[1], False),
        (60.0, math.radians(1.0), True),    # above horizon, inside ceiling
        (100.0, math.radians(1.0), False),  # object ceiling (80 m * 1.14)
    ]
    for r, elev, want_keep in cases:
        for draw in (0.001, 0.999):
            kept = mod.keep_mask(
                np.array([r]), np.array([elev]), height, ground_max,
                object_max, np.array([draw]),
            )[0]
            assert bool(kept) == want_keep, f"r={r} elev={math.degrees(elev):.2f}"


def test_dropout_fades_smoothly_at_the_knee(mod):
    """At max_ground_range the keep rate is ~50%, a band not a wall."""
    height, ground_max = 0.6, 28.0
    r = np.full(20000, ground_max)
    elev = np.full(20000, -math.asin(height / ground_max))
    uniform = np.random.default_rng(3).random(20000)
    kept = mod.keep_mask(r, elev, height, ground_max, 80.0, uniform).mean()
    assert 0.3 < kept < 0.7
