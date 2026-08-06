# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""ROS 2 node that forwards Ackermann commands to a Speedgoat ECU over UDP."""

import threading
import time
from typing import Optional

from ackermann_msgs.msg import AckermannDriveStamped
from drive_udp_bridge.config import BridgeConfig, validate_config
from drive_udp_bridge.protocol import CommandWatchdog, pack_command
from drive_udp_bridge.udp_sender import UdpSender
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class DriveUdpBridge(Node):
    """Continuously send the newest safe Ackermann command at a fixed rate."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            'drive_udp_bridge',
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter('command_topic', '/vehicle/cmd')
        self.declare_parameter('ecu_ip', '')
        self.declare_parameter('ecu_port', 0)
        self.declare_parameter('local_bind_ip', '0.0.0.0')
        self.declare_parameter('local_bind_port', 0)
        self.declare_parameter('send_rate_hz', 100.0)
        self.declare_parameter('command_timeout_sec', 0.2)

        self._config = BridgeConfig(
            command_topic=str(self.get_parameter('command_topic').value).strip(),
            ecu_ip=str(self.get_parameter('ecu_ip').value).strip(),
            ecu_port=int(self.get_parameter('ecu_port').value),
            local_bind_ip=str(self.get_parameter('local_bind_ip').value).strip(),
            local_bind_port=int(self.get_parameter('local_bind_port').value),
            send_rate_hz=float(self.get_parameter('send_rate_hz').value),
            command_timeout_sec=float(
                self.get_parameter('command_timeout_sec').value
            ),
        )
        validate_config(self._config)

        self._lock = threading.Lock()
        self._watchdog = CommandWatchdog(self._config.command_timeout_sec)
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

        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._send_timer = self.create_timer(
            1.0 / self._config.send_rate_hz,
            self._on_send_timer,
            clock=self._steady_clock,
        )

        local_endpoint = self._sender.local_endpoint
        self.get_logger().info(
            'Sending %s -> %s:%d at %.3f Hz; local UDP bind %s:%d; timeout %.3f s'
            % (
                self._config.command_topic,
                self._config.ecu_ip,
                self._config.ecu_port,
                self._config.send_rate_hz,
                local_endpoint[0],
                local_endpoint[1],
                self._config.command_timeout_sec,
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

    def _on_send_timer(self) -> None:
        with self._lock:
            command = self._watchdog.snapshot()

        packet = pack_command(
            command.speed,
            command.steering_angle,
            command.enable,
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
