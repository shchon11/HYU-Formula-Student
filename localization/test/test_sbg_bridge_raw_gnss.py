# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Synthetic-feed checks for the SBG bridge's raw-GNSS rung and hold/fault rules.

The scenario pinned here is what the car did on 2026-08-01 (bags
rosbag2_2026_08_01-17_40_39 / 17_47_09 / 17_50_00): the Ellipse EKF
re-initialised mid-run (solution_mode 4 -> 0/1 for ~30 s) while the raw
receiver kept reporting RTK_INT position, Doppler velocity and dual-antenna
heading at 5 Hz. With the original mode-only ladder that was a FAULT: the
motion input went silent and on re-entry the SLAM pose was 50-70 m behind
the car. Contract now:

  * mode <= 1 with raw GNSS + HDT/gyro heading keeps the SAME integrator
    advancing (rtk grade when the fix is RTK, Doppler grade otherwise), with
    the tier reporting itself in the covariance and health;
  * the HDT->vehicle heading offset (antenna order) is learned from the EKF
    while healthy and applied when the EKF heading is gone;
  * without HDT the raw gyro carries the heading, sigma growing with time;
  * a hold is time-limited (held_max_sec) and then faults; the first message
    after any blind gap carries a huge sigma;
  * wheels + gyro-carried heading dead-reckon even at mode 1.
"""
import importlib.util
import math
import os
import sys

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sbg_driver.msg")

from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
from sbg_driver.msg import SbgGpsHdt, SbgGpsPos, SbgGpsVel, SbgImuData  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
from test_sbg_bridge_fallback import (  # noqa: E402
    _LAT0, _LON0, _euler, _nav, _reset, _speeds, _stamp,
)

_MODULE_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "sbg_odometry_bridge.py"
)
_HDT_OFFSET_DEG = 180.0  # rear secondary antenna: baseline points backwards


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    spec = importlib.util.spec_from_file_location(
        "sbg_odometry_bridge_raw", _MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    instance = module.SbgOdometryBridge()
    yield instance
    instance.destroy_node()
    rclpy.try_shutdown()


def _ned_heading_deg(yaw_enu):
    return math.degrees(math.atan2(
        math.sin(0.5 * math.pi - yaw_enu), math.cos(0.5 * math.pi - yaw_enu))) % 360.0


def _hdt(t, yaw_enu, acc_deg=0.4, status=0):
    msg = SbgGpsHdt()
    msg.header.stamp = _stamp(t)
    msg.status = status | 0x40  # SOL_COMPUTED + BASELINE_VALID
    msg.true_heading = (_ned_heading_deg(yaw_enu) - _HDT_OFFSET_DEG) % 360.0
    msg.true_heading_acc = acc_deg
    return msg


def _gps_pos(t, lat, lon, ptype=7, acc=0.01):
    msg = SbgGpsPos()
    msg.header.stamp = _stamp(t)
    msg.status.status = 0
    msg.status.type = ptype
    msg.latitude = lat
    msg.longitude = lon
    msg.position_accuracy.x = acc
    msg.position_accuracy.y = acc
    return msg


def _gps_vel(t, ve, vn, acc=0.03):
    msg = SbgGpsVel()
    msg.header.stamp = _stamp(t)
    msg.status.vel_status = 0
    msg.status.vel_type = 2
    msg.velocity.x = vn  # NED wire order
    msg.velocity.y = ve
    msg.velocity_accuracy.x = acc
    msg.velocity_accuracy.y = acc
    return msg


def _imu(t, yaw_rate_enu):
    msg = SbgImuData()
    msg.header.stamp = _stamp(t)
    msg.gyro.z = -yaw_rate_enu  # NED body: z down, CW positive
    return msg


class _Truth:
    """Constant-speed, constant-yaw-rate ground truth on the WGS84 plane."""

    def __init__(self, node, v, yaw_rate):
        self.v, self.w = v, yaw_rate
        self.x = self.y = 0.0
        self.yaw = 0.0
        self.node = node

    def step(self, dt):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.w * dt),
                              math.cos(self.yaw + self.w * dt))

    def latlon(self):
        n = self.node
        lat = _LAT0 + math.degrees(self.y / n._meridional_radius)
        lon = _LON0 + math.degrees(
            self.x / (n._prime_vertical_radius * math.cos(n._origin_lat)))
        return lat, lon

    def vel_enu(self):
        return self.v * math.cos(self.yaw), self.v * math.sin(self.yaw)


def _tick(node, truth, t, mode, raw=True, hdt=True, imu=True, wheels=False,
          ptype=7, hdt_used=True):
    """One 25 Hz EKF tick at stamp t (+ a 5 Hz raw epoch every 5th tick)."""
    heading_valid = mode >= 2
    node.on_euler(_euler(t, yaw_enu=(truth.yaw if heading_valid else 0.0),
                         valid=heading_valid))
    if imu:
        node.on_imu(_imu(t, truth.w))
    if wheels:
        node.on_wheel_speeds(_speeds(t, truth.v))
    if raw and int(round(t * 25.0)) % 5 == 0:
        lat, lon = truth.latlon()
        # Antenna is lever_arm ahead of the base point.
        lx = node.antenna_lever_arm_x
        ax = truth.x + lx * math.cos(truth.yaw)
        ay = truth.y + lx * math.sin(truth.yaw)
        lat = _LAT0 + math.degrees(ay / node._meridional_radius)
        lon = _LON0 + math.degrees(
            ax / (node._prime_vertical_radius * math.cos(node._origin_lat)))
        ve, vn = truth.vel_enu()
        # Antenna velocity = base + w x l.
        ve += -truth.w * lx * math.sin(truth.yaw)
        vn += truth.w * lx * math.cos(truth.yaw)
        node.on_gps_pos(_gps_pos(t, lat, lon, ptype=ptype))
        node.on_gps_vel(_gps_vel(t, ve, vn))
        if hdt:
            node.on_gps_hdt(_hdt(t, truth.yaw))
    lat, lon = truth.latlon()
    ve, vn = truth.vel_enu()
    nav = _nav(t, mode, ve=ve, vn=vn, lat=lat, lon=lon)
    nav.status.gps1_hdt_used = hdt_used and mode >= 4
    node.on_nav(nav)


def _run(node, truth, t0, duration, mode, rate=25.0, **kw):
    n = int(round(duration * rate))
    for i in range(n):
        t = t0 + i / rate
        _tick(node, truth, t, mode, **kw)
        truth.step(1.0 / rate)
    return t0 + n / rate


def _last_xy(node):
    m = node.car_state_pub.msgs[-1]
    return m.pose.pose.position.x, m.pose.pose.position.y


def _last_yaw(node):
    m = node.car_state_pub.msgs[-1]
    return 2.0 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)


def _prime(node, truth, t0=100.0, secs=2.0, **kw):
    """Healthy mode-4 phase: georeference + learn the HDT offset."""
    _reset(node)
    node._set_origin(_LAT0, _LON0)  # so the truth's lat/lon are defined
    t = _run(node, truth, t0, secs, mode=4, **kw)
    assert node._started and node._georeferenced
    return t


def test_ekf_reset_is_bridged_by_raw_rtk(node):
    """mode 4 -> 1 with RTK gps_pos/gps_vel/HDT: odometry keeps tracking."""
    truth = _Truth(node, v=2.0, yaw_rate=math.radians(30.0))
    t = _prime(node, truth)
    assert node._hdt_offset is not None
    assert math.degrees(node._hdt_offset) == pytest.approx(_HDT_OFFSET_DEG, abs=1.0)
    x_ref, y_ref = _last_xy(node)
    tx_ref, ty_ref = truth.x, truth.y
    n_before = len(node.car_state_pub.msgs)

    t = _run(node, truth, t, 30.0, mode=1)  # 30 s EKF outage, car circling

    assert len(node.car_state_pub.msgs) > n_before + 700  # ~25 Hz kept up
    x, y = _last_xy(node)
    err = math.hypot((x - x_ref) - (truth.x - tx_ref), (y - y_ref) - (truth.y - ty_ref))
    assert err < 0.30, err  # 60 m of circling: rtk-corrected DR stays sub-decimetre
    dyaw = math.atan2(math.sin(_last_yaw(node) - truth.yaw), math.cos(_last_yaw(node) - truth.yaw))
    assert abs(math.degrees(dyaw)) < 1.5
    last = node.car_state_pub.msgs[-1]
    assert math.sqrt(last.pose.covariance[0]) == pytest.approx(node.odom_sigma_raw_rtk)
    assert last.twist.twist.linear.x == pytest.approx(truth.v, abs=0.1)
    assert last.twist.twist.angular.z == pytest.approx(truth.w, abs=1e-6)
    st = node.health_pub.msgs[-1].status[0]
    assert st.level == DiagnosticStatus.WARN
    assert dict((kv.key, kv.value) for kv in st.values)["motion_source"] == "raw_gnss_rtk"

    # Re-entry to mode 4: continuous, no step larger than the physical motion.
    t = _run(node, truth, t, 2.0, mode=4)
    xs = [m.pose.pose.position.x for m in node.car_state_pub.msgs]
    ys = [m.pose.pose.position.y for m in node.car_state_pub.msgs]
    steps = [math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i]) for i in range(len(xs) - 1)]
    assert max(steps) < 2.0 * truth.v / 25.0


def test_single_point_fix_uses_doppler_grade(node):
    """gps_pos type SINGLE: no epoch correction, Doppler DR at the wider sigma."""
    truth = _Truth(node, v=3.0, yaw_rate=math.radians(10.0))
    t = _prime(node, truth)
    x_ref, y_ref = _last_xy(node)
    tx_ref, ty_ref = truth.x, truth.y
    t = _run(node, truth, t, 10.0, mode=1, ptype=2)
    last = node.car_state_pub.msgs[-1]
    assert math.sqrt(last.pose.covariance[0]) == pytest.approx(node.odom_sigma_raw_doppler)
    x, y = _last_xy(node)
    err = math.hypot((x - x_ref) - (truth.x - tx_ref), (y - y_ref) - (truth.y - ty_ref))
    assert err < 0.5, err  # 30 m of Doppler-only DR (5 Hz hold) stays well inside 0.5 m
    st = node.health_pub.msgs[-1].status[0]
    assert dict((kv.key, kv.value) for kv in st.values)["motion_source"] == "raw_gnss_doppler"


def test_gyro_carries_heading_through_hdt_gap(node):
    """No HDT during the outage: the raw gyro carries the heading; sigma grows."""
    truth = _Truth(node, v=2.0, yaw_rate=math.radians(20.0))
    t = _prime(node, truth)
    t = _run(node, truth, t, 5.0, mode=1, hdt=False)
    dyaw = math.atan2(math.sin(_last_yaw(node) - truth.yaw), math.cos(_last_yaw(node) - truth.yaw))
    assert abs(math.degrees(dyaw)) < 1.0
    last = node.car_state_pub.msgs[-1]
    yaw_sigma = math.sqrt(last.pose.covariance[35])
    assert yaw_sigma == pytest.approx(0.02 + node.gyro_yaw_drift_rate * 5.0, abs=0.01)
    # Past gyro_only_heading_timeout the gyro-carried heading is refused ->
    # the raw rung is unavailable -> hold, then fault.
    t = _run(node, truth, t, node.gyro_only_heading_timeout + 1.0, mode=1, hdt=False)
    n = len(node.car_state_pub.msgs)
    _run(node, truth, t, 1.0, mode=1, hdt=False)
    assert len(node.car_state_pub.msgs) == n
    assert node.health_pub.msgs[-1].status[0].level == DiagnosticStatus.ERROR


def test_no_raw_no_wheels_faults_and_blind_gap_is_free(node):
    """Nothing to fall back on: fault as before; the first message after the
    gap carries a huge sigma so the graph does not trust the delta."""
    truth = _Truth(node, v=2.0, yaw_rate=0.0)
    t = _prime(node, truth, raw=False, imu=False)
    n = len(node.car_state_pub.msgs)
    t = _run(node, truth, t, 2.0, mode=1, raw=False, imu=False)
    assert len(node.car_state_pub.msgs) == n
    assert node.health_pub.msgs[-1].status[0].level == DiagnosticStatus.ERROR
    _run(node, truth, t, 0.2, mode=4, raw=False, imu=False)
    first = node.car_state_pub.msgs[n]
    assert math.sqrt(first.pose.covariance[0]) == pytest.approx(1.0e3)
    later = node.car_state_pub.msgs[-1]
    assert math.sqrt(later.pose.covariance[0]) == pytest.approx(node.odom_sigma_mode4)


def test_hold_is_time_limited(node):
    """mode 4 with velocity invalid and no fallback: hold for held_max_sec
    (huge sigma, last speed carried, NOT zero), then fault."""
    truth = _Truth(node, v=4.0, yaw_rate=0.0)
    t = _prime(node, truth, raw=False, imu=False)
    n = len(node.car_state_pub.msgs)
    x0, _ = _last_xy(node)
    rate = 25.0
    for i in range(int(3.0 * rate)):
        ti = t + i / rate
        node.on_euler(_euler(ti, yaw_enu=truth.yaw))
        nav = _nav(ti, 4, ve=truth.v)
        nav.status.velocity_valid = False
        node.on_nav(nav)
    held = node.car_state_pub.msgs[n:]
    assert held  # some hold messages went out
    assert len(held) <= int(node.held_max_sec * rate) + 2  # ...then silence
    assert all(math.sqrt(m.pose.covariance[0]) == pytest.approx(1.0e3) for m in held)
    assert all(m.pose.pose.position.x == pytest.approx(x0) for m in held)
    assert held[-1].twist.twist.linear.x == pytest.approx(truth.v, abs=1e-6)
    assert held[-1].twist.covariance[0] > 1.0e5
    assert node.health_pub.msgs[-1].status[0].level == DiagnosticStatus.ERROR


def test_wheels_dead_reckon_at_mode1_with_gyro_heading(node):
    """Wheels + gyro-carried heading (from HDT) keep DR alive even at mode 1."""
    truth = _Truth(node, v=3.0, yaw_rate=math.radians(15.0))
    t = _prime(node, truth, wheels=True)
    x_ref, y_ref = _last_xy(node)
    tx_ref, ty_ref = truth.x, truth.y
    # No raw velocity/position (GNSS receiver dark), but HDT + gyro + wheels.
    _run(node, truth, t, 8.0, mode=1, raw=False, wheels=True)
    x, y = _last_xy(node)
    err = math.hypot((x - x_ref) - (truth.x - tx_ref), (y - y_ref) - (truth.y - ty_ref))
    assert err < 0.5, err
    last = node.car_state_pub.msgs[-1]
    assert math.sqrt(last.pose.covariance[0]) == pytest.approx(node.odom_sigma_mode2)
    st = node.health_pub.msgs[-1].status[0]
    assert dict((kv.key, kv.value) for kv in st.values)["motion_source"] == "wheels"


def test_raw_start_before_ekf_alignment_with_pinned_offset(node):
    """EKF never aligns (mode 1) but RTK + HDT are up and the offset is
    pinned: the bridge starts on the raw rung instead of sitting silent."""
    truth = _Truth(node, v=2.0, yaw_rate=math.radians(5.0))
    _reset(node)
    node._set_origin(_LAT0, _LON0)
    saved = node.hdt_yaw_offset_deg
    node.hdt_yaw_offset_deg = _HDT_OFFSET_DEG
    node._hdt_offset = math.radians(_HDT_OFFSET_DEG)
    try:
        _run(node, truth, 100.0, 3.0, mode=1)
        assert node._started and not node._georeferenced
        assert node.car_state_pub.msgs
        st = node.health_pub.msgs[-1].status[0]
        assert dict((kv.key, kv.value) for kv in st.values)["motion_source"] == "raw_gnss_rtk"
    finally:
        node.hdt_yaw_offset_deg = saved
