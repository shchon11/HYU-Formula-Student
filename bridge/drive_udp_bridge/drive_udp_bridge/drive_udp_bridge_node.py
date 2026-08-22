# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""Bidirectional ROS 2 command and wheel-RPM Speedgoat UDP bridge."""

from pathlib import Path
import threading
import time
from typing import Optional

from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from drive_udp_bridge.config import BridgeConfig, validate_config
from drive_udp_bridge.protocol import (
    AutonomousStateWatchdog,
    CommandWatchdog,
    DEFAULT_FEEDBACK_VALUE_TYPE,
    feedback_format,
    pack_command,
    unpack_encoder_feedback,
)
from drive_udp_bridge.steering_conversion import SteeringWheelConverter
from drive_udp_bridge.udp_receiver import UdpReceiver
from drive_udp_bridge.udp_sender import UdpSender
from drive_udp_bridge.wheel_speeds import (
    WheelSpeedConfig,
    WheelSpeedConverter,
    WheelSpeedEstimate,
)
from hyu_msgs.msg import CanState, WheelSpeedsStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


def _resolve_steering_calibration_path(configured_path: str) -> Path:
    """Resolve relative calibration names below the installed config folder."""
    path = Path(configured_path).expanduser()
    if path.is_absolute():
        return path
    try:
        package_share = get_package_share_directory('drive_udp_bridge')
    except LookupError as error:
        raise ValueError(
            'cannot locate the installed drive_udp_bridge package share'
        ) from error
    return Path(package_share) / 'config' / path


class DriveUdpBridge(Node):
    """Send safe commands and publish ECU-derived per-wheel linear speeds."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            'drive_udp_bridge',
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter('command_topic', '/vehicle/cmd')
        self.declare_parameter('auto_state_topic', '/vehicle/as_state')
        self.declare_parameter('ecu_ip', '')
        self.declare_parameter('ecu_port', 0)
        self.declare_parameter('local_bind_ip', '0.0.0.0')
        self.declare_parameter('local_bind_port', 0)
        self.declare_parameter('send_rate_hz', 100.0)
        self.declare_parameter('command_timeout_sec', 0.2)
        self.declare_parameter('auto_state_timeout_sec', 0.5)
        self.declare_parameter(
            'steering_calibration_csv',
            'steering_kinematics.csv',
        )
        self.declare_parameter('max_steering_wheel_angle_deg', 90.0)
        self.declare_parameter('feedback_bind_ip', '')
        self.declare_parameter('feedback_port', 0)
        self.declare_parameter('feedback_poll_rate_hz', 200.0)
        self.declare_parameter('feedback_timeout_sec', 0.2)
        # Four per-wheel RPM values per datagram; element type as the ECU
        # sends it (float32/float64/int16/uint16/int32/uint32).
        self.declare_parameter('feedback_value_type', DEFAULT_FEEDBACK_VALUE_TYPE)
        # ECU revolutions per tire revolution (1.0 = wheel-side RPM).
        self.declare_parameter('rpm_gear_ratio', 1.0)
        self.declare_parameter('tire_diameter_m', 0.4572)
        self.declare_parameter('max_wheel_speed_mps', 50.0)
        self.declare_parameter('wheel_speeds_topic', '/vehicle/wheel_speeds')
        self.declare_parameter('wheel_speeds_frame_id', 'base_footprint')
        # Feedback datagrams from any other source address are dropped.
        # '' = ecu_ip; '0.0.0.0' = accept any source (bench with a PC sender).
        self.declare_parameter('feedback_source_ip', '')
        # OFF->ON edge of the autonomous switch: reset the SLAM map first and
        # raise the autonomous-enable byte only once that succeeded, so every
        # run starts from a clean map the instant the car is released. ON->OFF
        # drops the byte immediately, no service involved. require_map_reset
        # false = raise immediately (bench, no SLAM running).
        self.declare_parameter('map_reset_service', '/graph_slam/reset')
        self.declare_parameter('map_reset_timeout_sec', 5.0)
        self.declare_parameter('require_map_reset', True)

        self._config = BridgeConfig(
            command_topic=str(self.get_parameter('command_topic').value).strip(),
            auto_state_topic=str(
                self.get_parameter('auto_state_topic').value
            ).strip(),
            ecu_ip=str(self.get_parameter('ecu_ip').value).strip(),
            ecu_port=int(self.get_parameter('ecu_port').value),
            local_bind_ip=str(self.get_parameter('local_bind_ip').value).strip(),
            local_bind_port=int(self.get_parameter('local_bind_port').value),
            send_rate_hz=float(self.get_parameter('send_rate_hz').value),
            command_timeout_sec=float(
                self.get_parameter('command_timeout_sec').value
            ),
            auto_state_timeout_sec=float(
                self.get_parameter('auto_state_timeout_sec').value
            ),
            steering_calibration_csv=str(
                self.get_parameter('steering_calibration_csv').value
            ).strip(),
            max_steering_wheel_angle_deg=float(
                self.get_parameter('max_steering_wheel_angle_deg').value
            ),
            feedback_bind_ip=str(
                self.get_parameter('feedback_bind_ip').value
            ).strip(),
            feedback_port=int(self.get_parameter('feedback_port').value),
            feedback_poll_rate_hz=float(
                self.get_parameter('feedback_poll_rate_hz').value
            ),
            feedback_timeout_sec=float(
                self.get_parameter('feedback_timeout_sec').value
            ),
            feedback_value_type=str(
                self.get_parameter('feedback_value_type').value
            ).strip(),
            rpm_gear_ratio=float(self.get_parameter('rpm_gear_ratio').value),
            tire_diameter_m=float(
                self.get_parameter('tire_diameter_m').value
            ),
            max_wheel_speed_mps=float(
                self.get_parameter('max_wheel_speed_mps').value
            ),
            wheel_speeds_topic=str(
                self.get_parameter('wheel_speeds_topic').value
            ).strip(),
            wheel_speeds_frame_id=str(
                self.get_parameter('wheel_speeds_frame_id').value
            ).strip(),
            feedback_source_ip=str(
                self.get_parameter('feedback_source_ip').value
            ).strip(),
        )
        validate_config(self._config)
        self._map_reset_service = str(
            self.get_parameter('map_reset_service').value
        ).strip()
        self._map_reset_timeout_sec = float(
            self.get_parameter('map_reset_timeout_sec').value
        )
        self._require_map_reset = bool(
            self.get_parameter('require_map_reset').value
        )
        if self._require_map_reset and not self._map_reset_service:
            raise ValueError('map_reset_service must not be empty when require_map_reset')
        if self._map_reset_timeout_sec <= 0.0:
            raise ValueError('map_reset_timeout_sec must be greater than zero')

        calibration_path = _resolve_steering_calibration_path(
            self._config.steering_calibration_csv
        )
        self._steering_converter = SteeringWheelConverter.from_csv(
            calibration_path,
            self._config.max_steering_wheel_angle_deg,
        )

        self._lock = threading.Lock()
        # Autonomous-enable byte actually sent: follows the switch down at
        # once, follows it up only through a successful map reset.
        self._autonomous_armed = False
        self._reset_future = None
        self._reset_requested_at = float('-inf')
        self._reset_client = (
            self.create_client(Trigger, self._map_reset_service)
            if self._require_map_reset else None
        )
        self._last_reset_log_time = float('-inf')
        self._watchdog = CommandWatchdog(self._config.command_timeout_sec)
        self._auto_state_watchdog = AutonomousStateWatchdog(
            self._config.auto_state_timeout_sec
        )
        self._sender: Optional[UdpSender] = None
        self._receiver: Optional[UdpReceiver] = None
        self._wheel_speed_converter: Optional[WheelSpeedConverter] = None
        self._wheel_speeds_publisher = None
        self._last_send_error_log_time = float('-inf')
        self._last_steering_clip_log_time = float('-inf')
        self._last_feedback_error_log_time = float('-inf')
        self._feedback_started_at = time.monotonic()
        self._last_feedback_receive_time = None
        self._feedback_timed_out = False

        try:
            self._sender = UdpSender(
                ecu_ip=self._config.ecu_ip,
                ecu_port=self._config.ecu_port,
                local_bind_ip=self._config.local_bind_ip,
                local_bind_port=self._config.local_bind_port,
            )
            if self._config.feedback_enabled:
                self._receiver = UdpReceiver(
                    self._config.feedback_bind_ip,
                    self._config.feedback_port,
                )
        except Exception:
            if self._sender is not None:
                self._sender.close()
                self._sender = None
            raise

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self._command_subscription = self.create_subscription(
            AckermannDriveStamped,
            self._config.command_topic,
            self._on_command,
            qos,
        )

        state_qos = QoSProfile(depth=5)
        state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        state_qos.durability = DurabilityPolicy.VOLATILE
        self._auto_state_subscription = self.create_subscription(
            CanState,
            self._config.auto_state_topic,
            self._on_auto_state,
            state_qos,
        )

        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._send_timer = self.create_timer(
            1.0 / self._config.send_rate_hz,
            self._on_send_timer,
            clock=self._steady_clock,
        )

        self._feedback_timer = None
        if self._receiver is not None:
            self._wheel_speed_converter = WheelSpeedConverter(
                WheelSpeedConfig(
                    tire_diameter_m=self._config.tire_diameter_m,
                    rpm_gear_ratio=self._config.rpm_gear_ratio,
                    max_wheel_speed_mps=self._config.max_wheel_speed_mps,
                )
            )
            wheel_speeds_qos = QoSProfile(depth=10)
            wheel_speeds_qos.reliability = ReliabilityPolicy.RELIABLE
            wheel_speeds_qos.durability = DurabilityPolicy.VOLATILE
            self._wheel_speeds_publisher = self.create_publisher(
                WheelSpeedsStamped,
                self._config.wheel_speeds_topic,
                wheel_speeds_qos,
            )
            self._feedback_timer = self.create_timer(
                1.0 / self._config.feedback_poll_rate_hz,
                self._on_feedback_timer,
                clock=self._steady_clock,
            )

        local_endpoint = self._sender.local_endpoint
        self.get_logger().info(
            'Sending %s -> %s:%d at %.3f Hz; auto state %s; '
            'local UDP bind %s:%d; command timeout %.3f s; '
            'auto-state timeout %.3f s; autonomous enable on OFF->ON %s'
            % (
                self._config.command_topic,
                self._config.ecu_ip,
                self._config.ecu_port,
                self._config.send_rate_hz,
                self._config.auto_state_topic,
                local_endpoint[0],
                local_endpoint[1],
                self._config.command_timeout_sec,
                self._config.auto_state_timeout_sec,
                ('after %s succeeds (timeout %.1f s)'
                 % (self._map_reset_service, self._map_reset_timeout_sec))
                if self._require_map_reset else 'immediately (no map reset)',
            )
        )
        if self._receiver is None:
            self.get_logger().warning(
                'ECU encoder feedback is disabled; set feedback_bind_ip and '
                'feedback_port to enable wheel-speed publication'
            )
        else:
            feedback_endpoint = self._receiver.local_endpoint
            source_filter = self._config.feedback_source_filter
            self.get_logger().info(
                'Receiving %s wheel-RPM feedback (FL FR RL RR, 4 x %s) on %s:%d '
                'from %s; publishing four wheel speeds in m/s on %s '
                '(%.6f m/s per RPM = pi * %.4f m / 60 / gear ratio %.3f; '
                'receive times from %s)'
                % (
                    feedback_format(self._config.feedback_value_type),
                    self._config.feedback_value_type,
                    feedback_endpoint[0],
                    feedback_endpoint[1],
                    source_filter if source_filter is not None else 'any source',
                    self._config.wheel_speeds_topic,
                    self._wheel_speed_converter.mps_per_rpm,
                    self._config.tire_diameter_m,
                    self._config.rpm_gear_ratio,
                    'kernel arrival timestamps'
                    if self._receiver.kernel_timestamps else 'poll time',
                )
            )

    def _on_command(self, message: AckermannDriveStamped) -> None:
        speed = float(message.drive.speed)
        steering_angle = float(message.drive.steering_angle)
        with self._lock:
            accepted = self._watchdog.update(speed, steering_angle)

        if not accepted:
            self.get_logger().warning(
                'Rejected non-finite or out-of-range command; sending disabled zeros'
            )

    def _on_auto_state(self, message: CanState) -> None:
        auto_enabled = message.as_state == CanState.AS_DRIVING
        with self._lock:
            self._auto_state_watchdog.update(auto_enabled)

    def _log_reset(self, level: str, message: str, period_sec: float = 2.0) -> None:
        now = time.monotonic()
        if now - self._last_reset_log_time < period_sec:
            return
        self._last_reset_log_time = now
        getattr(self.get_logger(), level)(message)

    def _autonomous_byte(self, switch_on: bool) -> bool:
        """
        Return the autonomous-enable byte for the current switch state.

        Switch OFF (or stale): drop immediately and forget any pending reset.
        Switch ON: already armed -> stay armed; else arm only once the SLAM map
        reset service answered success (retried after map_reset_timeout_sec
        while the switch stays ON), or at once when require_map_reset is off.
        """
        if not switch_on:
            if self._autonomous_armed:
                self.get_logger().info('autonomous switch OFF -> autonomous enable 0')
            self._autonomous_armed = False
            self._reset_future = None
            return False
        if self._autonomous_armed:
            return True
        if not self._require_map_reset:
            self._autonomous_armed = True
            self.get_logger().info('autonomous switch ON -> autonomous enable 1 (no map reset)')
            return True

        now = time.monotonic()
        if self._reset_future is not None:
            if self._reset_future.done():
                response = None
                try:
                    response = self._reset_future.result()
                except Exception as error:  # noqa: BLE001 - report and retry
                    self._log_reset('error', 'map reset call failed: %s' % error)
                self._reset_future = None
                if response is not None and response.success:
                    self._autonomous_armed = True
                    self.get_logger().info(
                        'autonomous switch ON: map reset OK (%s) -> autonomous enable 1'
                        % (response.message or self._map_reset_service)
                    )
                    return True
                if response is not None:
                    self._log_reset(
                        'error',
                        'map reset refused (%s); autonomous enable stays 0, retrying'
                        % response.message,
                    )
            elif now - self._reset_requested_at > self._map_reset_timeout_sec:
                self._log_reset(
                    'error',
                    'map reset %s not answered within %.1f s; autonomous enable '
                    'stays 0, retrying' % (self._map_reset_service, self._map_reset_timeout_sec),
                )
                self._reset_future = None
            else:
                return False
        if self._reset_future is None and now - self._reset_requested_at > 0.5:
            # (re)issue the reset
            self._reset_requested_at = now
            if self._reset_client.service_is_ready():
                self._reset_future = self._reset_client.call_async(Trigger.Request())
                self.get_logger().info(
                    'autonomous switch ON: resetting the SLAM map (%s) before enabling'
                    % self._map_reset_service
                )
            else:
                self._log_reset(
                    'error',
                    'autonomous switch ON but %s is not available; autonomous enable '
                    'stays 0 (is graph SLAM up? require_map_reset:=false to bypass)'
                    % self._map_reset_service,
                )
        return False

    def _on_send_timer(self) -> None:
        with self._lock:
            command = self._watchdog.snapshot()
            switch_on = self._auto_state_watchdog.enabled()

        auto_enabled = self._autonomous_byte(switch_on)
        autonomous_enable = int(auto_enabled)
        motion_command_valid = auto_enabled and command.enable == 1
        speed = command.speed if motion_command_valid else 0.0
        steering_wheel_angle_rad = 0.0
        if motion_command_valid:
            conversion = self._steering_converter.convert(command.steering_angle)
            steering_wheel_angle_rad = conversion.steering_wheel_angle_rad
            if conversion.clipped:
                now = time.monotonic()
                if now - self._last_steering_clip_log_time >= 1.0:
                    self.get_logger().warning(
                        'Clipped bicycle steering %.6f rad to active input '
                        '[%.6f, %.6f] rad; sending steering-wheel %.6f rad'
                        % (
                            command.steering_angle,
                            self._steering_converter.negative_input_limit_rad,
                            self._steering_converter.positive_input_limit_rad,
                            steering_wheel_angle_rad,
                        )
                    )
                    self._last_steering_clip_log_time = now

        packet = pack_command(
            speed,
            steering_wheel_angle_rad,
            1,
            autonomous_enable,
        )
        try:
            if self._sender is not None:
                self._sender.send(packet)
        except OSError as error:
            now = time.monotonic()
            if now - self._last_send_error_log_time >= 1.0:
                self.get_logger().error('UDP send failed: %s' % error)
                self._last_send_error_log_time = now

    def _feedback_warning(self, message: str) -> None:
        """Log a feedback warning at most once per second."""
        now = time.monotonic()
        if now - self._last_feedback_error_log_time >= 1.0:
            self.get_logger().warning(message)
            self._last_feedback_error_log_time = now

    def _on_feedback_timer(self) -> None:
        if self._receiver is None or self._wheel_speed_converter is None:
            return

        try:
            datagrams = self._receiver.receive_available()
        except OSError as error:
            self._feedback_warning('UDP feedback receive failed: %s' % error)
            datagrams = []

        expected_datagrams = []
        source_filter = self._config.feedback_source_filter
        for packet, source, received_at in datagrams:
            if source_filter is not None and source[0] != source_filter:
                self._feedback_warning(
                    'Ignored ECU feedback from unexpected source %s '
                    '(expected %s; feedback_source_ip 0.0.0.0 accepts any)'
                    % (source[0], source_filter)
                )
                continue
            expected_datagrams.append((packet, received_at))

        # RPM is instantaneous: the newest queued datagram is the current
        # state, anything older in the queue is already stale.
        if expected_datagrams:
            packet, received_at = expected_datagrams[-1]
            try:
                feedback = unpack_encoder_feedback(
                    packet, self._config.feedback_value_type
                )
                estimate = self._wheel_speed_converter.convert(feedback)
            except ValueError as error:
                self._feedback_warning('Rejected ECU feedback: %s' % error)
            else:
                self._last_feedback_receive_time = received_at
                if self._feedback_timed_out:
                    self.get_logger().info('ECU encoder feedback restored')
                    self._feedback_timed_out = False
                self._publish_wheel_speeds(estimate, received_at)

        now = time.monotonic()
        last_receive = self._last_feedback_receive_time
        reference_time = (
            self._feedback_started_at
            if last_receive is None else last_receive
        )
        if (
            not self._feedback_timed_out
            and now - reference_time > self._config.feedback_timeout_sec
        ):
            self._feedback_timed_out = True
            self._feedback_warning(
                'ECU encoder feedback timed out; wheel-speed publication paused'
            )

    def _publish_wheel_speeds(
        self,
        estimate: WheelSpeedEstimate,
        received_at: float,
    ) -> None:
        """
        Publish four RPM-derived wheel speeds in metres per second.

        The header stamp is the datagram's arrival time expressed on the node
        clock (now minus the monotonic age of the sample), not the poll-timer
        tick, so consumers that fuse the sample by its stamp see the instant
        the ECU reported.
        """
        message = WheelSpeedsStamped()
        age_sec = max(0.0, time.monotonic() - received_at)
        stamp = self.get_clock().now() - Duration(seconds=age_sec)
        message.header.stamp = stamp.to_msg()
        message.header.frame_id = self._config.wheel_speeds_frame_id
        message.speeds.lf_speed = estimate.front_left_mps
        message.speeds.rf_speed = estimate.front_right_mps
        message.speeds.lb_speed = estimate.rear_left_mps
        message.speeds.rb_speed = estimate.rear_right_mps

        if self._wheel_speeds_publisher is not None:
            self._wheel_speeds_publisher.publish(message)

    def destroy_node(self):
        """Close the UDP socket before releasing ROS entities."""
        if self._receiver is not None:
            self._receiver.close()
            self._receiver = None
        if self._sender is not None:
            self._sender.close()
            self._sender = None
        return super().destroy_node()


def main(args=None) -> None:
    """Run the drive UDP bridge until ROS shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = DriveUdpBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError) as error:
        rclpy.logging.get_logger('drive_udp_bridge').fatal(
            'Startup failed: %s' % error
        )
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
