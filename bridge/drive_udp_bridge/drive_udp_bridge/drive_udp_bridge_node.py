# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""ROS 2 node that forwards Ackermann commands to a Speedgoat ECU over UDP."""

import threading
import time
from typing import Optional

from ackermann_msgs.msg import AckermannDriveStamped
from drive_udp_bridge.config import BridgeConfig, validate_config
from drive_udp_bridge.protocol import (
    AutonomousStateWatchdog,
    CommandWatchdog,
    pack_command,
)
from drive_udp_bridge.udp_sender import UdpSender
from hyu_msgs.msg import CanState
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


class DriveUdpBridge(Node):
    """Continuously send the newest safe Ackermann command at a fixed rate."""

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
        self._sender: Optional[UdpSender] = UdpSender(
            ecu_ip=self._config.ecu_ip,
            ecu_port=self._config.ecu_port,
            local_bind_ip=self._config.local_bind_ip,
            local_bind_port=self._config.local_bind_port,
        )
        self._last_send_error_log_time = float('-inf')

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
        """Autonomous-enable to send for the current switch state.

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
        steering_angle = command.steering_angle if motion_command_valid else 0.0

        packet = pack_command(
            speed,
            steering_angle,
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

    def destroy_node(self):
        """Close the UDP socket before releasing ROS entities."""
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
