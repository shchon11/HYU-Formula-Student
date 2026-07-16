# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Synthetic-feed behaviour checks for the simulated Ellipse-D INS.

A 200 Hz circular trajectory is fed straight into on_ground_truth (no
executor); publishes are captured through a stub publisher. Pinned here:

  * the REPORTED accuracy is a deterministic function of the mode history —
    bit-identical across seeds — while the realised error differs: the
    oracle (reporting sigma + |realised error|) must never come back;
  * output errors are Gauss-Markov, not white: correlation time ~ the
    configured tau, and the 30 s velocity-error integral is far above the
    white-noise prediction (the bridge integrates velocity into pose);
  * the free-inertial divergence the docstring promises (~0.3 m @ 8 s) is
    what both the realised ensemble and the reported sigma actually do;
  * position_valid flips on BELIEVED accuracy, not on the realised draw.
"""
import importlib.util
import math
import os
import random

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sbg_driver.msg")

from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402
from eufs_msgs.msg import CarState  # noqa: E402

_MODULE_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "sim_ellipse_d.py"
)
_EPOCH = 1000.0
_SPEED = 6.0
_OMEGA = 0.3  # rad/s -> 20 m circle


class _StubPub:
    """Captures publishes instead of sending them anywhere."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        """Record the message."""
        self.msgs.append(msg)


def _make_state(t):
    """Ground-truth CarState on the circle at absolute stamp t."""
    msg = CarState()
    sec = int(t)
    msg.header.stamp = TimeMsg(sec=sec, nanosec=int(round((t - sec) * 1e9)))
    yaw = _OMEGA * (t - _EPOCH)
    radius = _SPEED / _OMEGA
    msg.pose.pose.position.x = radius * math.sin(yaw)
    msg.pose.pose.position.y = radius * (1.0 - math.cos(yaw))
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    msg.twist.twist.linear.x = _SPEED
    return msg


def _reset(node, seed, mode_schedule, corr_schedule):
    """Re-arm one node instance for a fresh scenario run."""
    node.rng = random.Random(seed)
    node.mode = 4
    node.correction = "rtk_fixed"
    node.mode_schedule = list(mode_schedule)
    node.correction_schedule = list(corr_schedule)
    node.mode_schedule_index = node.correction_schedule_index = 0
    node.manual_mode = node.manual_correction = False
    node.pos_err = [0.0, 0.0]
    node.vel_err = [0.0, 0.0]
    node.heading_err = 0.0
    node.gm_err = [0.0, 0.0]
    node.vel_gm = [0.0, 0.0]
    node.heading_gm = 0.0
    node.att_gm = [0.0, 0.0]
    node.alt_gm = 0.0
    node.vel_bias_vec = [0.0, 0.0]
    node.accel_dir = [1.0, 0.0]
    node.drift_dir = [1.0, 0.0]
    node.degraded_since = None
    node.acc_pos_lin = node.acc_pos_var = 0.0
    node.acc_vel_lin = node.acc_vel_var = 0.0
    node.acc_hdg_lin = node.acc_hdg_var = 0.0
    node.first_stamp = node.last_stamp = node.last_pub_stamp = None
    node.latency_mean = node.latency_jitter = 0.0  # scenario tests: no delay
    node._outbox.clear()
    node.nav_pub = _StubPub()
    node.euler_pub = _StubPub()


def _run(node, seed, duration, rate, mode_schedule=(), corr_schedule=()):
    """Feed the trajectory and return decoded rows (t, errors, acc, valid)."""
    _reset(node, seed, mode_schedule, corr_schedule)
    dt = 1.0 / rate
    for i in range(int(round(duration * rate))):
        node.on_ground_truth(_make_state(_EPOCH + i * dt))
    rows = []
    for nav in node.nav_pub.msgs:
        t = nav.header.stamp.sec + nav.header.stamp.nanosec * 1e-9 - _EPOCH
        north = math.radians(nav.latitude - node.datum_lat) * node._meridional_radius
        east = (
            math.radians(nav.longitude - node.datum_lon)
            * node._prime_vertical_radius
            * node._cos_lat0
        )
        yaw = _OMEGA * t
        radius = _SPEED / _OMEGA
        rows.append(
            dict(
                t=t,
                err_e=east - radius * math.sin(yaw),
                err_n=north - radius * (1.0 - math.cos(yaw)),
                # NED: velocity.x = N, velocity.y = E
                verr_e=nav.velocity.y - _SPEED * math.cos(yaw),
                acc=nav.position_accuracy.x,
                valid=nav.status.position_valid,
            )
        )
    return rows


def _at(rows, t):
    """Row closest to trajectory time t."""
    return min(rows, key=lambda r: abs(r["t"] - t))


def _median(values):
    """Middle element (upper) of the sorted values."""
    return sorted(values)[len(values) // 2]


@pytest.fixture(scope="module")
def node():
    """One SimEllipseD instance, reset per scenario by the helpers."""
    rclpy.init()
    spec = importlib.util.spec_from_file_location("sim_ellipse_d", _MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    instance = module.SimEllipseD()
    yield instance
    instance.destroy_node()
    rclpy.try_shutdown()


@pytest.fixture(scope="module")
def scenario(node):
    """Mode dip 40-48 s + single-point window 70-95 s, four seeds."""
    mode_schedule = [(40.0, 2), (48.0, 4)]
    corr_schedule = [(70.0, "single"), (95.0, "rtk_fixed")]
    return {
        seed: _run(node, seed, 120.0, 200.0, mode_schedule, corr_schedule)
        for seed in range(1, 5)
    }


def test_reported_accuracy_is_deterministic_realised_error_is_not(scenario):
    """Reported sigma is seed-independent; realised errors are not."""
    acc_series = [[r["acc"] for r in rows] for rows in scenario.values()]
    assert all(series == acc_series[0] for series in acc_series[1:])
    # Free-inertial magnitude is near-deterministic; the DIRECTION is the
    # per-seed draw, so compare a vector component across seeds.
    err_e = [_at(rows, 47.5)["err_e"] for rows in scenario.values()]
    assert max(err_e) - min(err_e) > 0.05


def test_velocity_error_is_colored_not_white(scenario):
    """1/e autocorrelation ~ tau, sigma ~ datasheet, integral >> white."""
    taus, sigmas, integrals = [], [], []
    for rows in scenario.values():
        seg = [r["verr_e"] for r in rows if 5.0 <= r["t"] <= 35.0]
        mean = sum(seg) / len(seg)
        centered = [v - mean for v in seg]
        var = sum(v * v for v in centered) / len(centered)
        tau = 30.0
        for lag in range(1, len(centered)):
            c = sum(
                centered[i] * centered[i + lag]
                for i in range(len(centered) - lag)
            ) / ((len(centered) - lag) * var)
            if c < math.exp(-1.0):
                tau = lag / 200.0
                break
        taus.append(tau)
        sigmas.append(math.sqrt(var))
        integrals.append(abs(sum(v / 200.0 for v in seg)))
    assert 0.8 <= _median(taus) <= 5.0  # vel_error_tau default 2 s
    assert 0.015 <= _median(sigmas) <= 0.06  # sigma_vel default 0.03
    white_integral = 0.03 * math.sqrt(30.0 / 200.0)
    assert _median(integrals) > 4 * white_integral


def test_free_inertial_reported_tracks_realised_ensemble(scenario):
    """After 8 s of mode 2 both are ~0.3 m and within 5x of each other."""
    realised = [
        math.hypot(_at(rows, 47.9)["err_e"], _at(rows, 47.9)["err_n"])
        for rows in scenario.values()
    ]
    reported = _at(next(iter(scenario.values())), 47.9)["acc"]
    ratio = reported / max(_median(realised), 1e-9)
    assert 0.2 <= ratio <= 5.0


def test_tier_degradation_and_refix_pull_in(scenario):
    """Single-point: sigma jumps, error follows; RTK refix pulls in fast."""
    rows_by_seed = list(scenario.values())
    assert _at(rows_by_seed[0], 71.0)["acc"] > 1.0
    at_94 = [
        math.hypot(_at(rows, 94.0)["err_e"], _at(rows, 94.0)["err_n"])
        for rows in rows_by_seed
    ]
    at_98 = [
        math.hypot(_at(rows, 98.0)["err_e"], _at(rows, 98.0)["err_n"])
        for rows in rows_by_seed
    ]
    assert _median(at_94) > 0.3
    assert _median(at_98) < 0.2


def test_position_valid_flips_on_believed_accuracy(node):
    """Mode 3 keeps mode>=3; only believed sigma can invalidate (~330 s)."""
    rows = _run(node, 3, 420.0, 100.0, mode_schedule=[(5.0, 3)])
    flips = [r["t"] for r in rows if not r["valid"]]
    assert flips and 250.0 < flips[0] < 420.0


def test_transport_latency_delays_arrival_not_stamp(node):
    """Messages arrive ~latency later, FIFO, stamped at measurement time."""
    _reset(node, 11, (), ())
    node.latency_mean, node.latency_jitter = 0.05, 0.005
    dt = 1.0 / 200.0
    arrivals = []
    seen = 0
    for i in range(400):
        t = _EPOCH + i * dt
        node.on_ground_truth(_make_state(t))
        while seen < len(node.nav_pub.msgs):
            nav = node.nav_pub.msgs[seen]
            stamp = nav.header.stamp.sec + nav.header.stamp.nanosec * 1e-9
            arrivals.append((t, stamp))
            seen += 1
    assert len(arrivals) > 300
    delays = [arrival - stamp for arrival, stamp in arrivals]
    assert min(delays) > 0.02 and max(delays) < 0.1
    stamps = [stamp for _, stamp in arrivals]
    assert stamps == sorted(stamps)
