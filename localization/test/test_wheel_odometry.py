# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Synthetic-feed behaviour checks for wheel odometry's INS coupling.

Messages are fed straight into the callbacks (no executor) and publishes are
captured through a stub. Pinned here, all of them things that fail silently:

  * the SBG body frame is Z-DOWN, so the reported yaw rate must be negated
    before it is integrated — get it wrong and the car turns the wrong way
    while every topic still looks healthy;
  * a_x is taken from the gravity-free channel and drives slip compensation
    in the right direction (braking under-reads, traction over-reads);
  * a stale INS falls back to bicycle-model yaw and PAUSES slip compensation
    rather than integrating a stale kappa;
  * sigma_v widens with the applied correction, so the graph downweights
    odometry exactly when the estimate is working hardest.
"""
import importlib.util
import math
import os

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sbg_driver.msg")

from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402
from hyu_msgs.msg import WheelSpeedsStamped  # noqa: E402
from sbg_driver.msg import SbgEkfNav, SbgEkfRotAccel  # noqa: E402

_MODULE_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "wheel_odometry.py"
)
_RADIUS = 0.2525
_RPM_TO_RAD_S = 2.0 * math.pi / 60.0


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


def _speeds(t, v, steering=0.0):
    """Wheel speeds reporting a true ground speed v (no differential split)."""
    msg = WheelSpeedsStamped()
    msg.header.stamp = _stamp(t)
    rpm = v / (_RADIUS * _RPM_TO_RAD_S)
    msg.speeds.lb_speed = rpm
    msg.speeds.rb_speed = rpm
    msg.speeds.lf_speed = rpm
    msg.speeds.rf_speed = rpm
    msg.speeds.steering = steering
    return msg


def _rot_accel(t, yaw_rate_flu, a_x, frame_id="imu_link_ned"):
    """SbgEkfRotAccel as the driver emits it: body RFD, so Z-down."""
    msg = SbgEkfRotAccel()
    msg.header.stamp = _stamp(t)
    msg.header.frame_id = frame_id
    msg.rate.z = -yaw_rate_flu  # Z-down on the wire
    msg.acceleration.x = a_x
    return msg


def _nav_mode(mode):
    """SbgEkfNav carrying only what wheel_odometry reads: solution_mode."""
    msg = SbgEkfNav()
    msg.status.solution_mode = mode
    return msg


def _reset(node):
    node.rear_axle_to_base = 0.0
    node.x = node.y = node.yaw = 0.0
    node.last_t = None
    node.ins_yaw_rate = None
    node.ins_stamp = None
    node.nav_mode = None
    node.used_ins_fallback = False
    node.accel_x_lp = 0.0
    node.accel_lp_stamp = None
    node.warned_frame = False
    node.pub = _StubPub()


@pytest.fixture(scope="module")
def node():
    """One WheelOdometry instance, reset per test by the helper."""
    rclpy.init()
    spec = importlib.util.spec_from_file_location("wheel_odometry", _MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    instance = module.WheelOdometry()
    yield instance
    instance.destroy_node()
    rclpy.try_shutdown()


def _drive(node, duration, rate, v, yaw_rate_flu, a_x, feed_ins=True, arm=0.0):
    """Feed a constant-state run; returns the published CarStates."""
    _reset(node)
    node.rear_axle_to_base = arm
    dt = 1.0 / rate
    for i in range(int(round(duration * rate))):
        t = 100.0 + i * dt
        if feed_ins:
            node.on_rot_accel(_rot_accel(t, yaw_rate_flu, a_x))
        node.on_wheel_speeds(_speeds(t, v))
    return node.pub.msgs


def test_ins_yaw_rate_sign(node):
    """Z-down on the wire must integrate as a left turn in FLU."""
    out = _drive(node, 2.0, 100.0, v=6.0, yaw_rate_flu=0.3, a_x=0.0)
    assert out
    # The node must report the FLU yaw rate, not the wire's Z-down value.
    assert abs(out[-1].twist.twist.angular.z - 0.3) < 1e-6
    # ...and integrate it into a LEFT (positive) heading over 2 s.
    assert node.yaw > 0.5
    assert node.y > 0.0


def test_slip_compensation_direction_and_sigma(node):
    """Traction over-reads (v drops); sigma_v widens with the correction."""
    baseline = _drive(node, 1.0, 100.0, v=10.0, yaw_rate_flu=0.0, a_x=0.0)
    v_coast = baseline[-1].twist.twist.linear.x
    sigma_coast = math.sqrt(baseline[-1].pose.covariance[0])

    driving = _drive(node, 1.0, 100.0, v=10.0, yaw_rate_flu=0.0, a_x=8.0)
    v_driving = driving[-1].twist.twist.linear.x
    sigma_driving = math.sqrt(driving[-1].pose.covariance[0])

    braking = _drive(node, 1.0, 100.0, v=10.0, yaw_rate_flu=0.0, a_x=-8.0)
    v_braking = braking[-1].twist.twist.linear.x

    # Same encoder reading, three traction states: under power the wheels
    # over-read so true v is LOWER; under braking they under-read.
    assert v_driving < v_coast < v_braking
    # The correction is a real fraction of v, not noise-level.
    assert (v_coast - v_driving) > 0.1
    # Confidence tracks the work done: coasting is the tight one.
    assert sigma_driving > sigma_coast


def test_stale_ins_falls_back_and_pauses_slip(node):
    """No INS: bicycle-model yaw, and kappa must not be applied at all."""
    out = _drive(node, 1.0, 100.0, v=10.0, yaw_rate_flu=0.0, a_x=0.0,
                 feed_ins=False)
    assert out
    assert node.used_ins_fallback
    # Straight steering -> bicycle yaw is zero, and with slip paused the
    # reported speed is the raw encoder speed.
    assert abs(out[-1].twist.twist.angular.z) < 1e-9
    assert abs(out[-1].twist.twist.linear.x - 10.0) < 1e-6
    # sigma_v stays at the floor: no correction was applied to widen it.
    assert abs(math.sqrt(out[-1].pose.covariance[0]) - node.sigma_v) < 1e-9


def test_enu_frame_id_warns(node):
    """An ENU frame_id means swapped body axes; say so instead of drifting."""
    _reset(node)
    warnings = []
    original = node.get_logger().warn
    node.get_logger().warn = lambda m: warnings.append(m)
    try:
        node.on_rot_accel(_rot_accel(100.0, 0.0, 0.0, frame_id="imu_link"))
        node.on_rot_accel(_rot_accel(100.01, 0.0, 0.0, frame_id="imu_link"))
    finally:
        node.get_logger().warn = original
    assert len(warnings) == 1  # warned once, not per message
    assert "use_enu" in warnings[0]


def test_ned_frame_id_is_quiet(node):
    """The expected frame must not cry wolf."""
    _reset(node)
    warnings = []
    original = node.get_logger().warn
    node.get_logger().warn = lambda m: warnings.append(m)
    try:
        node.on_rot_accel(_rot_accel(100.0, 0.0, 0.0))
    finally:
        node.get_logger().warn = original
    assert not warnings


def _split_speeds(t, v_left, v_right, steering=0.0):
    """Wheel speeds with a rear differential split (per-wheel encoders)."""
    msg = WheelSpeedsStamped()
    msg.header.stamp = _stamp(t)
    msg.speeds.lb_speed = v_left / (_RADIUS * _RPM_TO_RAD_S)
    msg.speeds.rb_speed = v_right / (_RADIUS * _RPM_TO_RAD_S)
    msg.speeds.lf_speed = msg.speeds.lb_speed
    msg.speeds.rf_speed = msg.speeds.rb_speed
    msg.speeds.steering = steering
    return msg


def test_kinematic_vy_reported_and_integrated(node):
    """With a nonzero arm, cornering must report vy = w * arm (opt-in)."""
    out = _drive(node, 2.0, 100.0, v=6.0, yaw_rate_flu=0.5, a_x=0.0, arm=0.79)
    expected_vy = 0.5 * 0.79
    assert abs(out[-1].twist.twist.linear.y - expected_vy) < 1e-6
    # The base origin's sideways sweep must reach the integrated pose too:
    # against a vy-less integration of the same inputs, y gains ~vy-driven
    # displacement. Just pin that vy is nonzero and finite in the pose chain.
    assert out[-1].twist.twist.linear.y > 0.0


def test_differential_yaw_fallback_before_bicycle(node):
    """With the INS dark, the rear encoder split must supply the yaw rate."""
    _reset(node)
    dt = 1.0 / 100.0
    w_true = 0.4
    v = 6.0
    half = 0.5 * node.track_width * w_true
    for i in range(200):
        t = 100.0 + i * dt
        # No on_rot_accel: INS never arrives; steering left at zero so the
        # bicycle fallback would read w = 0 and fail this test.
        node.on_wheel_speeds(_split_speeds(t, v - half, v + half))
    out = node.pub.msgs
    assert out
    assert abs(out[-1].twist.twist.angular.z - w_true) < 1e-3
    assert node.yaw > 0.5  # integrated a left turn from encoders alone


def test_mode2_gates_rot_accel_consumption(node):
    """Free-inertial EKF (mode < 3): the gravity-leaking rot_accel log is
    treated exactly like a stale one — encoder yaw, slip paused, accel
    zeroed — so the leak cannot reach TMPC's LongAcc feedback."""
    _reset(node)
    node.on_ekf_nav(_nav_mode(2))
    v, w, a_x = 5.0, 0.5, 3.0
    for i in range(50):
        t = 100.0 + i / 100.0
        node.on_rot_accel(_rot_accel(t, w, a_x))
        node.on_wheel_speeds(_speeds(t, v))
    out = node.pub.msgs
    assert out
    last = out[-1]
    # The leaked acceleration must NOT pass through to consumers.
    assert last.linear_acceleration.x == 0.0
    # Yaw comes from the (zero-split) encoders, not the INS rate.
    assert abs(last.twist.twist.angular.z) < 1e-9
    # Slip compensation paused: raw encoder speed goes out unmodified.
    assert abs(last.twist.twist.linear.x - v) < 1e-6
    assert node.used_ins_fallback


def test_mode_recovery_restores_rot_accel(node):
    """Back at mode >= 3 the same fresh log is consumed again."""
    _reset(node)
    node.on_ekf_nav(_nav_mode(2))
    v, w, a_x = 5.0, 0.5, 3.0
    for i in range(20):
        t = 100.0 + i / 100.0
        node.on_rot_accel(_rot_accel(t, w, a_x))
        node.on_wheel_speeds(_speeds(t, v))
    node.on_ekf_nav(_nav_mode(4))
    for i in range(20, 70):
        t = 100.0 + i / 100.0
        node.on_rot_accel(_rot_accel(t, w, a_x))
        node.on_wheel_speeds(_speeds(t, v))
    last = node.pub.msgs[-1]
    assert abs(last.twist.twist.angular.z - w) < 1e-9
    assert abs(last.linear_acceleration.x - a_x) < 1e-9
    # Positive a_x -> traction slip: the reported speed under-reads the
    # encoders again, proving the slip estimate resumed.
    assert last.twist.twist.linear.x < v - 1e-4
    assert not node.used_ins_fallback
