# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""
Publish a canned /vehicle/cmd pattern for ECU drive tests -- no stack needed.

    ros2 run drive_udp_bridge cmd_pattern [options]              # pattern below
    ros2 run drive_udp_bridge cmd_pattern SPEED [STEER] [options]  # constant command

With a positional SPEED the speed is held constant (m/s, no ramp, no stop);
an optional STEER (rad) holds the steering constant too -- otherwise the
steering sine below still runs (--steer-amp 0 for straight). Without them:
steering is a sine wave; speed ramps linearly from 0, hard-stops (steps to 0)
the moment it reaches the peak, holds zero for a while and repeats:

    steer  = steer_amp * sin(2*pi*t / steer_period)          rad, +left
    speed  = accel * (t mod cycle)   while t mod cycle < ramp  (ramp = max_speed/accel)
           = 0                       otherwise                 (for stop_hold s)

Defaults: +-0.3 rad over 4 s, 0 -> 3 m/s at 1 m/s^2 (3 s), 2 s stop, forever.
Nothing here drives the ECU on its own: drive_udp_bridge still forwards
these commands only while /vehicle/as_state is AS_DRIVING (AS button ON in
the bench rig, `ros2 launch drive_udp_bridge drive_udp_bridge.launch.py
bench:=true`). Ctrl+C publishes zero commands for a moment before exiting so
the ECU sees an explicit stop rather than a watchdog timeout.

    ros2 run drive_udp_bridge cmd_pattern --max-speed 0            # steering only
    ros2 run drive_udp_bridge cmd_pattern --steer-amp 0 --accel 2  # speed only, 1.5 s ramp
    ros2 run drive_udp_bridge cmd_pattern --cycles 3               # three ramps then stop
    ros2 run drive_udp_bridge cmd_pattern 1                        # 1 m/s constant, sine steer
    ros2 run drive_udp_bridge cmd_pattern 1 0.2                    # 1 m/s, 0.2 rad, both constant
"""

import argparse
import math
import signal
import sys
import threading
import time
from typing import List, Optional, Tuple

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

STOP_PUBLISH_SEC = 0.5


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    """Parse the pattern options (ROS args are stripped by rclpy first)."""
    parser = argparse.ArgumentParser(
        prog='cmd_pattern',
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        'speed', nargs='?', type=float, default=None,
        help='hold this speed constant, m/s (no ramp/stop); omit for the ramp pattern',
    )
    parser.add_argument(
        'steer', nargs='?', type=float, default=None,
        help='hold this steering constant, rad (+left); omit for the steering sine',
    )
    parser.add_argument('--topic', default='/vehicle/cmd', help='command topic')
    parser.add_argument('--rate', type=float, default=50.0, help='publish rate, Hz')
    parser.add_argument('--frame-id', default='base_footprint', help='header.frame_id')
    parser.add_argument(
        '--steer-amp', type=float, default=0.3,
        help='steering sine amplitude, rad (vehicle lock is 0.52)',
    )
    parser.add_argument(
        '--steer-period', type=float, default=4.0, help='steering sine period, s',
    )
    parser.add_argument(
        '--max-speed', type=float, default=3.0,
        help='speed at which the ramp hard-stops, m/s (0 = never move)',
    )
    parser.add_argument(
        '--accel', type=float, default=1.0, help='ramp slope, m/s^2',
    )
    parser.add_argument(
        '--stop-hold', type=float, default=2.0,
        help='seconds to hold speed 0 after each hard stop',
    )
    parser.add_argument(
        '--cycles', type=int, default=0,
        help='number of ramp/stop cycles before exiting (0 = forever)',
    )
    args = parser.parse_args(argv)
    if args.rate <= 0.0:
        parser.error('--rate must be > 0')
    if args.steer_period <= 0.0:
        parser.error('--steer-period must be > 0')
    if args.steer_amp < 0.0:
        parser.error('--steer-amp must be >= 0')
    if args.max_speed < 0.0:
        parser.error('--max-speed must be >= 0')
    if args.accel <= 0.0:
        parser.error('--accel must be > 0')
    if args.stop_hold < 0.0:
        parser.error('--stop-hold must be >= 0')
    if args.cycles < 0:
        parser.error('--cycles must be >= 0')
    if args.steer is not None and args.speed is None:
        parser.error('STEER needs SPEED first')
    if args.speed is not None and not math.isfinite(args.speed):
        parser.error('SPEED must be finite')
    if args.steer is not None and not math.isfinite(args.steer):
        parser.error('STEER must be finite')
    return args


class CommandPattern:
    """Pure function of elapsed time -> (speed, steering_angle, phase, cycle)."""

    def __init__(
        self,
        steer_amp: float,
        steer_period: float,
        max_speed: float,
        accel: float,
        stop_hold: float,
        const_speed: Optional[float] = None,
        const_steer: Optional[float] = None,
    ) -> None:
        self.const_speed = const_speed
        self.const_steer = const_steer
        self.steer_amp = steer_amp
        self.steer_period = steer_period
        self.max_speed = max_speed
        self.accel = accel
        self.stop_hold = stop_hold
        self.ramp_sec = max_speed / accel
        self.cycle_sec = self.ramp_sec + stop_hold

    def steering(self, t: float) -> float:
        """Steering angle at time t, rad, positive = left."""
        if self.const_steer is not None:
            return self.const_steer
        return self.steer_amp * math.sin(2.0 * math.pi * t / self.steer_period)

    def speed(self, t: float) -> Tuple[float, str, int]:
        """Speed at time t plus the phase label and 0-based cycle index."""
        if self.const_speed is not None:
            return self.const_speed, 'CONST', 0
        if self.cycle_sec <= 0.0:
            return 0.0, 'IDLE', 0
        cycle = int(t // self.cycle_sec)
        tc = t - cycle * self.cycle_sec
        if tc < self.ramp_sec:
            return min(self.accel * tc, self.max_speed), 'RAMP', cycle
        return 0.0, 'STOP', cycle


class CmdPatternNode(Node):
    """Publish the pattern at a fixed rate and print a status line."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('cmd_pattern')
        self.args = args
        self.pattern = CommandPattern(
            steer_amp=args.steer_amp,
            steer_period=args.steer_period,
            max_speed=args.max_speed,
            accel=args.accel,
            stop_hold=args.stop_hold,
            const_speed=args.speed,
            const_steer=args.steer,
        )
        self.publisher = self.create_publisher(AckermannDriveStamped, args.topic, 10)
        self.done = threading.Event()
        self.t0 = time.monotonic()
        self.last_print = 0.0
        self.timer = self.create_timer(1.0 / args.rate, self._tick)
        self._announce()

    def _announce(self) -> None:
        p = self.pattern
        if p.const_speed is not None:
            steer_desc = (
                '%.3f rad constant' % p.const_steer if p.const_steer is not None
                else '%.2f rad sine / %.1f s' % (p.steer_amp, p.steer_period)
            )
            self.get_logger().info(
                'publishing %s at %.0f Hz: speed %.2f m/s CONSTANT, steer %s '
                '(Ctrl+C to stop)'
                % (self.args.topic, self.args.rate, p.const_speed, steer_desc)
            )
            return
        self.get_logger().info(
            'publishing %s at %.0f Hz: steer %.2f rad sine / %.1f s, '
            'speed 0->%.1f m/s at %.1f m/s^2 (%.1f s ramp) then 0 for %.1f s, '
            '%s' % (
                self.args.topic, self.args.rate, p.steer_amp, p.steer_period,
                p.max_speed, p.accel, p.ramp_sec, p.stop_hold,
                'forever' if self.args.cycles == 0 else '%d cycles' % self.args.cycles,
            )
        )

    def _message(self, speed: float, steering: float) -> AckermannDriveStamped:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        return msg

    def _tick(self) -> None:
        if self.done.is_set():
            return
        t = time.monotonic() - self.t0
        speed, phase, cycle = self.pattern.speed(t)
        if self.args.cycles and phase != 'CONST' and cycle >= self.args.cycles:
            self.done.set()
            return
        steering = self.pattern.steering(t)
        self.publisher.publish(self._message(speed, steering))
        if t - self.last_print >= 0.1:
            self.last_print = t
            sys.stdout.write(
                '\r t=%7.2f s  cycle %d  %-5s  speed %5.2f m/s  steer %+.3f rad (%+6.1f deg)   '
                % (t, cycle + 1, phase, speed, steering, math.degrees(steering))
            )
            sys.stdout.flush()

    def publish_stop(self, seconds: float = STOP_PUBLISH_SEC) -> None:
        """Stream zero commands for a moment so the ECU gets an explicit stop."""
        self.done.set()
        period = 1.0 / self.args.rate
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.publisher.publish(self._message(0.0, 0.0))
            time.sleep(period)
        sys.stdout.write('\n')
        sys.stdout.flush()
        self.get_logger().info('stopped: zero command published for %.1f s' % seconds)


def main(args=None) -> None:
    """Run the pattern publisher until Ctrl+C or the cycle count is reached."""
    ros_args = args if args is not None else sys.argv[1:]
    user_args = rclpy.utilities.remove_ros_args(ros_args)
    options = parse_args(user_args)

    # Own the SIGINT handling so the stop burst can still go out on the wire
    # after Ctrl+C (rclpy's default handler tears the context down first).
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    interrupted = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: interrupted.set())

    node = CmdPatternNode(options)
    try:
        while rclpy.ok() and not interrupted.is_set() and not node.done.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
