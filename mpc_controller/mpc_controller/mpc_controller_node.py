"""MPC drive controller node — drop-in replacement for pure pursuit.

Identical I/O contract to pure_pursuit_controller_node: consumes the
selector-owned path, its validity, the stop request, and SLAM ego odometry;
publishes /cmd at a fixed rate; brakes (speed 0, -5 m/s^2) whenever ANY input
gate fails, exactly like the pure pursuit safety chain.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import WaypointArrayStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from mpc_controller.mpc_core import EgoState, MpcConfig, PathPoint, solve


def yaw_from_quaternion(x, y, z, w):
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        return None
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return None
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class MpcControllerNode(Node):

    def __init__(self):
        super().__init__("mpc_controller_node")
        config = MpcConfig()
        config.wheelbase_m = self.declare_parameter(
            "wheelbase_m", config.wheelbase_m).value
        config.horizon_steps = int(self.declare_parameter(
            "horizon_steps", config.horizon_steps).value)
        config.horizon_dt_sec = self.declare_parameter(
            "horizon_dt_sec", config.horizon_dt_sec).value
        config.max_steering_rad = self.declare_parameter(
            "max_steering_rad", config.max_steering_rad).value
        config.max_steering_rate_radps = self.declare_parameter(
            "max_steering_rate_radps", config.max_steering_rate_radps).value
        config.max_speed_mps = self.declare_parameter(
            "max_speed_mps", config.max_speed_mps).value
        config.min_acceleration_mps2 = self.declare_parameter(
            "min_acceleration_mps2", config.min_acceleration_mps2).value
        config.max_acceleration_mps2 = self.declare_parameter(
            "max_acceleration_mps2", config.max_acceleration_mps2).value
        config.q_lateral = self.declare_parameter("q_lateral", config.q_lateral).value
        config.q_longitudinal = self.declare_parameter(
            "q_longitudinal", config.q_longitudinal).value
        config.q_heading = self.declare_parameter("q_heading", config.q_heading).value
        config.q_speed = self.declare_parameter("q_speed", config.q_speed).value
        config.q_steering = self.declare_parameter("q_steering", config.q_steering).value
        config.r_steering_rate = self.declare_parameter(
            "r_steering_rate", config.r_steering_rate).value
        config.r_acceleration = self.declare_parameter(
            "r_acceleration", config.r_acceleration).value
        config.s_steering_rate = self.declare_parameter(
            "s_steering_rate", config.s_steering_rate).value
        config.s_acceleration_rate = self.declare_parameter(
            "s_acceleration_rate", config.s_acceleration_rate).value
        config.command_rate_hz = self.declare_parameter(
            "command_rate_hz", config.command_rate_hz).value
        config.input_timeout_sec = self.declare_parameter(
            "input_timeout_sec", config.input_timeout_sec).value
        config.actuation_delay_sec = self.declare_parameter(
            "actuation_delay_sec", config.actuation_delay_sec).value
        config.terminal_weight = self.declare_parameter(
            "terminal_weight", config.terminal_weight).value
        # Lateral-error integral trim: removes any residual steady-state
        # offset (feedforward bias, receding-horizon deferral). Clamped hard.
        self.integral_gain = self.declare_parameter("integral_gain", 0.12).value
        self.integral_limit_rad = self.declare_parameter(
            "integral_limit_rad", 0.08).value
        self.config = config

        self.path = []
        self.path_frame_valid = False
        self.path_time = None
        self.validity = False
        self.validity_time = None
        self.stop_requested = True
        self.stop_time = None
        self.ego = None
        self.odom_frame_valid = False
        self.odom_time = None
        self.previous_control = None
        # The simulator slews the actual steering toward the commanded target
        # at max_steering_rate; integrate the same law to estimate where the
        # actuator really is.
        self.commanded_steering = 0.0
        self.estimated_steering = 0.0
        self.steering_trim = 0.0
        self.last_tick = time.monotonic()

        qos = QoSProfile(depth=10)
        self.command_pub = self.create_publisher(AckermannDriveStamped, "/cmd", qos)
        self.create_subscription(
            WaypointArrayStamped, "/path_waypoints", self.on_path, qos)
        self.create_subscription(
            Bool, "/planning/selected_path_valid", self.on_validity, qos)
        self.create_subscription(Bool, "/planning/stop_request", self.on_stop, qos)
        self.create_subscription(
            Odometry, "/localization/ego_odom", self.on_odom, qos)
        self.create_timer(1.0 / config.command_rate_hz, self.on_timer)
        self.get_logger().info(
            f"MPC controller up: N={config.horizon_steps} dt={config.horizon_dt_sec}s "
            f"limits: steer {config.max_steering_rad} rad, "
            f"a [{config.min_acceleration_mps2}, {config.max_acceleration_mps2}] m/s^2")

    # ------------------------------------------------------------------
    def on_path(self, msg):
        self.path_frame_valid = msg.header.frame_id == "map"
        points = []
        for waypoint in msg.waypoints:
            speed = waypoint.vx_mps if (
                math.isfinite(waypoint.vx_mps) and waypoint.vx_mps > 0.0
            ) else waypoint.speed
            points.append(PathPoint(
                waypoint.position.x, waypoint.position.y,
                speed if math.isfinite(speed) else 0.0))
        self.path = points
        self.path_time = time.monotonic()

    def on_validity(self, msg):
        self.validity = msg.data
        self.validity_time = time.monotonic()

    def on_stop(self, msg):
        self.stop_requested = msg.data
        self.stop_time = time.monotonic()

    def on_odom(self, msg):
        self.odom_frame_valid = (
            msg.header.frame_id == "map" and msg.child_frame_id == "base_footprint")
        yaw = yaw_from_quaternion(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        if yaw is None:
            self.ego = None
        else:
            self.ego = EgoState(
                msg.pose.pose.position.x, msg.pose.pose.position.y,
                yaw, msg.twist.twist.linear.x)
        self.odom_time = time.monotonic()

    # ------------------------------------------------------------------
    def fresh(self, stamp):
        return stamp is not None and (
            time.monotonic() - stamp) <= self.config.input_timeout_sec

    def on_timer(self):
        now = time.monotonic()
        elapsed = max(now - self.last_tick, 1e-3)
        self.last_tick = now
        previous_estimate = self.estimated_steering
        step = self.config.max_steering_rate_radps * elapsed
        difference = self.commanded_steering - self.estimated_steering
        self.estimated_steering += max(-step, min(step, difference))
        steering_rate_estimate = (self.estimated_steering - previous_estimate) / elapsed

        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "map"

        # The exact pure-pursuit safety chain: brake unless every input is
        # present, fresh, frame-correct, valid, and no stop is requested.
        gates_ok = (
            self.path_frame_valid and self.odom_frame_valid and
            self.validity and not self.stop_requested and
            self.fresh(self.path_time) and self.fresh(self.validity_time) and
            self.fresh(self.stop_time) and self.fresh(self.odom_time) and
            self.ego is not None and len(self.path) >= 2)

        solution = None
        if gates_ok:
            previous = (
                (steering_rate_estimate, self.previous_control[1])
                if self.previous_control is not None else None)
            solution = solve(
                self.path, self.ego, self.config, previous,
                current_steering=self.estimated_steering)

        if solution is None:
            command.drive.speed = 0.0
            command.drive.acceleration = -5.0
            command.drive.steering_angle = 0.0
            self.previous_control = None
            self.commanded_steering = 0.0
            self.steering_trim = 0.0
        else:
            # Leaky integral: stale trim must wash out fast when the track
            # curvature flips (a right-circle bias would push the car off a
            # following left circle); the leak costs only ~4 cm of residual.
            self.steering_trim *= max(0.0, 1.0 - 0.3 * elapsed)
            # Anti-windup: the trim only serves small steady-state residuals.
            if abs(solution.lateral_error_m) < 0.5:
                self.steering_trim -= (
                    self.integral_gain * solution.lateral_error_m * elapsed)
            self.steering_trim = max(
                -self.integral_limit_rad,
                min(self.integral_limit_rad, self.steering_trim))
            steering = max(
                -self.config.max_steering_rad,
                min(self.config.max_steering_rad,
                    solution.steering_rad + self.steering_trim))
            command.drive.speed = solution.speed_mps
            command.drive.acceleration = solution.acceleration_mps2
            command.drive.steering_angle = steering
            self.previous_control = (steering, solution.acceleration_mps2)
            self.commanded_steering = steering
        self.command_pub.publish(command)


def main():
    rclpy.init()
    node = MpcControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
