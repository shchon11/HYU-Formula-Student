# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Synthetic-feed checks for the SBG bridge's dead-reckoning degradation.

Messages are fed straight into the callbacks (no executor) and publishes are
captured through a stub, mirroring test_wheel_odometry.py. What is pinned
here is the fault-transition contract that used to kill SLAM:

  * mode 2 (AHRS) must NOT stop the relative odometry — wheel speeds + the
    still-valid AHRS heading keep the same integrator advancing, so the
    cone-anchored SLAM pose never free-runs on a frozen motion input;
  * the integrated position must stay continuous through mode flapping
    (4 -> 2 -> 4), with no per-tick step larger than the physical motion,
    even when the absolute fix jumps on reacquisition;
  * the dead-reckoning tier must report itself: sigma widens to
    odom_sigma_mode2 so the graph downweights the edges, and health degrades
    to WARN, not ERROR;
  * with no usable fallback (stale wheels, or mode <= 1 where the heading
    itself drifts) the bridge faults exactly as before: publication stops.
"""
import importlib.util
import math
import os

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sbg_driver.msg")

from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402
from eufs_msgs.msg import WheelSpeedsStamped  # noqa: E402
from sbg_driver.msg import SbgEkfEuler, SbgEkfNav  # noqa: E402

_MODULE_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "sbg_odometry_bridge.py"
)
_RADIUS = 0.2525
_RPM_TO_RAD_S = 2.0 * math.pi / 60.0
_LAT0, _LON0 = 37.0, 127.0


class _StubPub:
    """Captures publishes instead of sending them anywhere."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        """Record the message."""
        self.msgs.append(msg)


def _stamp(t):
    sec = int(t)
    return TimeMsg(sec=sec, nanosec=int(round((t - sec) * 1e9)))


def _euler(t, yaw_enu=0.0, valid=True):
    """Build an SbgEkfEuler on the NED wire (bridge default convention)."""
    msg = SbgEkfEuler()
    msg.header.stamp = _stamp(t)
    # ENU yaw -> NED heading (the bridge inverts this back).
    msg.angle.z = math.atan2(
        math.sin(0.5 * math.pi - yaw_enu), math.cos(0.5 * math.pi - yaw_enu)
    )
    msg.accuracy.z = 0.02
    msg.status.heading_valid = valid
    return msg


def _nav(t, mode, ve=0.0, vn=0.0, lat=_LAT0, lon=_LON0):
    """Build an SbgEkfNav with the validity flags the EKF ties to the mode."""
    msg = SbgEkfNav()
    msg.header.stamp = _stamp(t)
    msg.status.solution_mode = mode
    msg.status.position_valid = mode >= 4
    msg.status.velocity_valid = mode >= 3
    msg.latitude = lat
    msg.longitude = lon
    # NED wire order: velocity.x = north, velocity.y = east.
    msg.velocity.x = vn
    msg.velocity.y = ve
    msg.position_accuracy.x = 0.02
    msg.position_accuracy.y = 0.02
    return msg


def _speeds(t, v):
    msg = WheelSpeedsStamped()
    msg.header.stamp = _stamp(t)
    rpm = v / (_RADIUS * _RPM_TO_RAD_S)
    msg.speeds.lb_speed = rpm
    msg.speeds.rb_speed = rpm
    return msg


def _reset(node):
    node._origin_lat = None
    node._origin_lon = None
    node._odom_xy = None
    node._odom_yaw = 0.0
    node._last_int_t = None
    node._last_vel_enu = None
    node._started = False
    node._last_pub_nav_t = None
    node._yaw = None
    node._yaw_sigma = node.default_heading_sigma
    node._yaw_stamp = None
    node._heading_valid = False
    node._wheel_speed = None
    node._wheel_stamp = None
    node._last_marker_t = None
    node._last_board_t = None
    node.car_state_pub = _StubPub()
    node.gnss_odom_pub = _StubPub()
    node.health_pub = _StubPub()
    node.marker_pub = None
    node.overlay_pub = None


@pytest.fixture(scope="module")
def node():
    """One SbgOdometryBridge instance, reset per test by the helper."""
    rclpy.init()
    spec = importlib.util.spec_from_file_location(
        "sbg_odometry_bridge", _MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    instance = module.SbgOdometryBridge()
    yield instance
    instance.destroy_node()
    rclpy.try_shutdown()


def _feed(node, t, mode, v, yaw_enu=0.0, wheels=True, heading_valid=True):
    """One EKF tick: euler + (optionally) wheels + nav, all at stamp t."""
    node.on_euler(_euler(t, yaw_enu=yaw_enu, valid=heading_valid))
    if wheels:
        node.on_wheel_speeds(_speeds(t, v))
    node.on_nav(_nav(t, mode, ve=v * math.cos(yaw_enu), vn=v * math.sin(yaw_enu)))


def _drive(node, t0, duration, rate, mode, v, **kwargs):
    for i in range(int(round(duration * rate))):
        _feed(node, t0 + i / rate, mode, v, **kwargs)
    return t0 + duration


def test_mode2_dead_reckons_instead_of_stopping(node):
    """AHRS mode keeps odometry flowing on wheels + heading."""
    _reset(node)
    rate, v = 20.0, 5.0
    t = _drive(node, 100.0, 1.0, rate, mode=4, v=v)
    n_mode4 = len(node.car_state_pub.msgs)
    x_mode4 = node.car_state_pub.msgs[-1].pose.pose.position.x

    t = _drive(node, t, 2.0, rate, mode=2, v=v)

    out = node.car_state_pub.msgs
    assert len(out) > n_mode4  # publication did NOT stop
    # Eastbound at 5 m/s for 2 s of AHRS: the wheels carried the integration.
    assert out[-1].pose.pose.position.x - x_mode4 == pytest.approx(
        v * 2.0, abs=0.5
    )
    assert abs(out[-1].pose.pose.position.y) < 0.1
    # The dead-reckoning tier reports itself in the covariance.
    assert math.sqrt(out[-1].pose.covariance[0]) == pytest.approx(
        node.odom_sigma_mode2
    )


def test_flapping_position_is_continuous(node):
    """4 -> 2 -> 4 with an absolute-fix jump: no step in relative odometry."""
    _reset(node)
    rate, v = 20.0, 5.0
    t = _drive(node, 100.0, 1.0, rate, mode=4, v=v)
    t = _drive(node, t, 1.0, rate, mode=2, v=v)
    # Reacquisition: the absolute fix steps ~15 m east of the datum, which
    # must NOT leak into the integrated (relative) channel.
    lon_jump = _LON0 + 15.0 / (
        node._prime_vertical_radius * math.cos(node._origin_lat)
    ) * 180.0 / math.pi
    for i in range(int(rate)):
        ti = t + i / rate
        node.on_euler(_euler(ti))
        node.on_wheel_speeds(_speeds(ti, v))
        node.on_nav(_nav(ti, 4, ve=v, lon=lon_jump))

    xs = [m.pose.pose.position.x for m in node.car_state_pub.msgs]
    ys = [m.pose.pose.position.y for m in node.car_state_pub.msgs]
    steps = [
        math.hypot(xs[i + 1] - xs[i], ys[i + 1] - ys[i])
        for i in range(len(xs) - 1)
    ]
    # Physical motion per tick is v/rate; nothing may step further.
    assert max(steps) < 2.0 * v / rate
    # ...and the full 3 s of driving is all accounted for.
    assert xs[-1] == pytest.approx(3.0 * v, abs=1.0)


def test_mode2_without_wheels_still_faults(node):
    """No fallback source -> the old contract: stop feeding SLAM."""
    _reset(node)
    rate, v = 20.0, 5.0
    _drive(node, 100.0, 1.0, rate, mode=4, v=v)
    n = len(node.car_state_pub.msgs)

    # Wheels go stale (none fed for > wheel_speed_timeout worth of ticks).
    _drive(node, 101.0 + 1.0, 1.0, rate, mode=2, v=v, wheels=False)
    assert len(node.car_state_pub.msgs) == n
    assert node.health_pub.msgs  # health kept reporting the fault
    from diagnostic_msgs.msg import DiagnosticStatus
    assert node.health_pub.msgs[-1].status[0].level == DiagnosticStatus.ERROR


def test_mode1_never_dead_reckons(node):
    """VERTICAL_GYRO: the heading itself drifts; wheels must not be trusted."""
    _reset(node)
    rate, v = 20.0, 5.0
    _drive(node, 100.0, 1.0, rate, mode=4, v=v)
    n = len(node.car_state_pub.msgs)
    _drive(node, 101.0, 1.0, rate, mode=1, v=v, wheels=True)
    assert len(node.car_state_pub.msgs) == n


def test_hold_branch_publishes_huge_sigma(node):
    """Velocity invalid + stale wheels: the held pose must not look tight."""
    _reset(node)
    rate, v = 20.0, 5.0
    t = _drive(node, 100.0, 1.0, rate, mode=4, v=v)
    n = len(node.car_state_pub.msgs)
    x0 = node.car_state_pub.msgs[-1].pose.pose.position.x
    # A 1 s gap lets the wheel samples go stale; velocity is then flagged
    # invalid on the wire, so neither integration source is usable.
    for i in range(int(rate)):
        ti = t + 1.0 + i / rate
        node.on_euler(_euler(ti))
        nav = _nav(ti, 4, ve=v)
        nav.status.velocity_valid = False
        node.on_nav(nav)
    out = node.car_state_pub.msgs
    assert len(out) > n  # mode 4 holds and keeps publishing (no fault)
    assert out[-1].pose.pose.position.x == pytest.approx(x0)  # pose frozen
    assert math.sqrt(out[-1].pose.covariance[0]) == pytest.approx(1.0e3)
    # Heading was still fed fresh, so its sigma stays tight.
    assert math.sqrt(out[-1].pose.covariance[35]) == pytest.approx(0.02)


def test_velocity_dropout_at_mode3_uses_wheels(node):
    """A momentary velocity_valid=false gap is bridged by the same fallback."""
    _reset(node)
    rate, v = 20.0, 5.0
    t = _drive(node, 100.0, 1.0, rate, mode=4, v=v)
    x0 = node.car_state_pub.msgs[-1].pose.pose.position.x
    # mode 3 but with velocity flagged invalid on the wire.
    for i in range(int(rate)):
        ti = t + i / rate
        node.on_euler(_euler(ti))
        node.on_wheel_speeds(_speeds(ti, v))
        nav = _nav(ti, 3, ve=v)
        nav.status.velocity_valid = False
        node.on_nav(nav)
    x1 = node.car_state_pub.msgs[-1].pose.pose.position.x
    assert x1 - x0 == pytest.approx(v * 1.0, abs=0.5)
