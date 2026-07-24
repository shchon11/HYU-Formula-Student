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
  * position_valid flips on BELIEVED accuracy, not on the realised draw;
  * rot_accel_body reports the centripetal load of a steady corner, which the
    ground-truth linear_acceleration field does NOT carry, in the SBG body
    frame rather than ROS FLU.
"""
import importlib.util
import math
import os
import random

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sbg_driver.msg")

from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402
from hyu_msgs.msg import CarState  # noqa: E402

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
    """
    Ground-truth CarState on the circle at absolute stamp t.

    Steady state, so the body-frame velocity derivatives the vehicle model
    publishes as ``linear_acceleration`` are both ZERO: v_x_dot = r*v_y + Fx/m
    with v_y = 0 and constant speed, v_y_dot = Fy/m - r*v_x with the cornering
    force exactly balancing the transport term. The real lateral load is the
    whole r*v_x = 1.8 m/s^2 and lives nowhere in this message except implicitly
    in twist — which is the entire reason sim_ellipse_d has to reconstruct it.
    """
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
    msg.twist.twist.angular.z = _OMEGA
    msg.linear_acceleration.x = 0.0
    msg.linear_acceleration.y = 0.0
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
    node.accel_body_dir = [1.0, 0.0]
    node.drift_dir = [1.0, 0.0]
    node.degraded_since = None
    node.accel_bias_gm = [0.0, 0.0, 0.0]
    node.gyro_bias_gm = [0.0, 0.0, 0.0]
    node.att_ramp = 0.0
    node._stationary = False
    node._stand_heading = 0.0
    node.acc_pos_lin = node.acc_pos_var = 0.0
    node.acc_vel_lin = node.acc_vel_var = 0.0
    node.acc_hdg_lin = node.acc_hdg_var = 0.0
    node.first_stamp = node.last_stamp = node.last_pub_stamp = None
    node.last_rot_pub_stamp = None
    node.latency_mean = node.latency_jitter = 0.0  # scenario tests: no delay
    node._outbox.clear()
    node.nav_pub = _StubPub()
    node.euler_pub = _StubPub()
    node.rot_accel_pub = _StubPub()


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
    # velocity sigma is now per-tier; the scenario runs rtk_fixed by default ->
    # sigma_vel["rtk_fixed"] = 0.03 (measured RTK bag 2026-07-24, 0.027 m/s).
    assert 0.015 <= _median(sigmas) <= 0.06
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


def test_rot_accel_body_reports_centripetal_load(node):
    """
    Steady corner: lateral load is r*v_x, NOT ground truth's ~0 v_y_dot.

    The regression this exists for is forwarding CarState.linear_acceleration
    straight through, which reports a cornering car as pulling no lateral g —
    silently telling a friction-limit gate that everything is feasible in the
    exact case it is meant to catch.
    """
    _run(node, 7, 20.0, 200.0)
    rows = node.rot_accel_pub.msgs
    assert len(rows) > 100

    # Ground truth carried zero lateral acceleration; the INS must not.
    expected = _OMEGA * _SPEED  # 1.8 m/s^2
    settled = rows[len(rows) // 2:]
    lateral = _median([abs(m.acceleration.y) for m in settled])
    assert abs(lateral - expected) < 0.05

    # SBG body frame is Y-right, so a left turn (omega > 0) pulls the car's
    # acceleration to the LEFT: negative on the reported axis.
    assert all(m.acceleration.y < 0.0 for m in settled)

    # Rate: Z-down, so a CCW (left) turn reports a negative yaw rate.
    yaw_rate = _median([m.rate.z for m in settled])
    assert abs(yaw_rate + _OMEGA) < 0.01

    # Longitudinal is unloaded at constant speed, and the transport term must
    # not leak into it: r*v_y = 0 here, so only the error budget remains.
    assert _median([abs(m.acceleration.x) for m in settled]) < 0.05


def test_rot_accel_body_rate_is_independent_of_nav_rate(node):
    """Each sbgECom log has its own divider; nav at 25 Hz, rot at 100 Hz."""
    _reset(node, 5, (), ())
    node.ekf_rate, node.rot_accel_rate = 25.0, 100.0
    try:
        for i in range(400):
            node.on_ground_truth(_make_state(_EPOCH + i / 200.0))
        # 2 s of ground truth: ~50 nav, ~200 rot_accel.
        assert 45 <= len(node.nav_pub.msgs) <= 55
        assert 190 <= len(node.rot_accel_pub.msgs) <= 210
    finally:
        node.ekf_rate, node.rot_accel_rate = 200.0, 100.0


def test_rot_accel_body_degrades_with_the_dropout(node):
    """Free inertial: gravity leaks through the ramping attitude error."""
    _run(node, 9, 40.0, 200.0, mode_schedule=[(10.0, 2)])
    rows = node.rot_accel_pub.msgs
    stamped = [
        (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - _EPOCH, m) for m in rows
    ]
    nominal = _median(
        [abs(m.acceleration.x) for t, m in stamped if 5.0 <= t <= 9.0]
    )
    degraded = _median(
        [abs(m.acceleration.x) for t, m in stamped if 35.0 <= t <= 39.0]
    )
    # 0.05 deg initial + 7 deg/h over ~28 s -> g*sin(theta) well above nominal.
    assert degraded > 3.0 * max(nominal, 1e-6)


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


def _make_still_state(t, x=0.0, y=0.0, yaw=0.0):
    """Ground truth for a genuinely stopped car: zero twist, fixed pose."""
    msg = CarState()
    sec = int(t)
    msg.header.stamp = TimeMsg(sec=sec, nanosec=int(round((t - sec) * 1e9)))
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    msg.twist.twist.linear.x = 0.0
    msg.twist.twist.angular.z = 0.0
    return msg


def test_standstill_realised_degrades_while_believed_stays_confident(node):
    """At a stop the sim must reproduce the believed!=realised trap: heading
    free-drifts and velocity reads a few cm/s of noise (not a clean zero), while
    solution_mode stays 4, the valid flags stay true, and the reported sigmas do
    not move -- only gps1_hdt_used drops, which is why the heading drifts."""
    _reset(node, seed=7, mode_schedule=(), corr_schedule=())
    rate = 200.0
    dt = 1.0 / rate
    for i in range(int(1.0 * rate)):          # a second of motion, then stop
        node.on_ground_truth(_make_state(_EPOCH + i * dt))
    t1 = _EPOCH + 1.0
    for i in range(int(120.0 * rate)):        # 120 s stopped
        node.on_ground_truth(_make_still_state(t1 + i * dt, yaw=0.0))

    def _t(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    navs = [m for m in node.nav_pub.msgs if _t(m) >= t1 + 1.0]
    euls = [m for m in node.euler_pub.msgs if _t(m) >= t1 + 1.0]
    assert navs and euls

    last = navs[-1]
    # Believed: still fully confident.
    assert last.status.solution_mode == 4
    assert last.status.velocity_valid and last.status.heading_valid
    assert last.velocity_accuracy.x == pytest.approx(
        node.sigma_vel[node.correction], abs=0.05)
    # ...but the dual-antenna heading is no longer fused (the drift's cause).
    assert last.status.gps1_hdt_used is False

    # Realised velocity: small noise, not a clean zero, far below moving tier.
    vmags = [math.hypot(m.velocity.x, m.velocity.y) for m in navs]
    assert 0.005 < _median(vmags) < 0.12

    # Realised heading DRIFTS: NED wire angle.z = normalize(pi/2 - yaw_out);
    # true yaw is 0 so the departure from pi/2 is the accumulated heading error.
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))
    hd = [abs(math.degrees(_wrap(m.angle.z - 0.5 * math.pi))) for m in euls]
    assert max(hd) > 0.7            # drifted well past the 0.2 deg moving wander
    # Reported heading sigma stays at its nominal (believed unchanged).
    assert euls[-1].accuracy.z < math.radians(1.0)


def test_velocity_accuracy_tracks_correction_tier(node):
    """The reported velocity_accuracy is the ONE INS number that reaches graph
    SLAM (it sets the odom EdgeSE2 weight); it must track the correction tier --
    cm-grade at RTK, degrading toward single -- since that is the only channel
    through which the RTK/PSRDIFF split bites the map."""
    want = {"rtk_fixed": 0.03, "rtk_float": 0.06, "psrdiff": 0.12, "single": 0.15}
    for corr, exp in want.items():
        _reset(node, seed=3, mode_schedule=(), corr_schedule=())
        node.correction = corr
        for i in range(int(2.0 * 200)):        # 2 s so the GM state settles
            node.on_ground_truth(_make_state(_EPOCH + i / 200.0))
        va = node.nav_pub.msgs[-1].velocity_accuracy.x
        assert va == pytest.approx(exp, abs=0.02), f"{corr}: {va:.3f} != {exp}"


def test_rtk_auto_visits_multiple_tiers(node):
    """rtk_auto reproduces the field bag's tier dynamics without a schedule:
    a startup ramp (single -> float -> fixed) plus stochastic dropouts, so a run
    exercises RTK_FIXED, a degraded tier, and single-point start."""
    _reset(node, seed=1, mode_schedule=(), corr_schedule=())
    node.rtk_auto = True
    node.rtk_startup_sec = 20.0
    node.rtk_dropout_rate = 0.1        # dense dropouts so the test is robust
    seen = set()
    for i in range(int(60.0 * 200)):
        node.on_ground_truth(_make_state(_EPOCH + i / 200.0))
        seen.add(node.correction)
    assert "rtk_fixed" in seen                       # dominant steady tier
    assert "single" in seen                          # startup ramp
    assert seen & {"rtk_float", "psrdiff"}           # dropouts occurred
