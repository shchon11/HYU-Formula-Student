#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Keyboard teleop for the EUFS simulator with driving-game control shaping.

Techniques borrowed from driving simulators to make keyboard control
comfortable:

- Cruise-style speed target: W/S adjust a target speed instead of a raw
  pedal, and an internal jerk-limited P controller produces the
  acceleration command. No need to hold the throttle key.
- Speed-sensitive steering: maximum steering angle and per-tap step shrink
  as speed rises, preventing high-speed spins.
- Auto-centering: while driving, steering relaxes toward center at a rate
  proportional to speed (Live-for-Speed keyboard-stabilized style), so
  releasing the keys naturally straightens the car.
- Counter-steer boost: steering input against the current angle acts
  faster than input away from center, so corrections feel immediate.
- Expo response: fine control near center, softer response near full lock.
- Deadman: commands publish continuously at 50 Hz; SPACE is a full brake
  and Q always leaves the car braking straight.
"""

import math
import os
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import CanState, CarState
from eufs_msgs.srv import SetCanState

HELP = (
    "W/↑ faster   S/↓ slower/brake   A/← D/→ steer   SPACE stop   "
    "C center   R reverse   M mission   +/- speed cap   Q quit"
)


class TeleopNode(Node):
    def __init__(self):
        super().__init__('eufs_teleop')

        self.declare_parameter('cmd_topic', '/cmd')
        self.declare_parameter('car_state_topic', '/wheel_odometry/car_state')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('max_steering', 0.52)      # rad, vehicle lock
        self.declare_parameter('max_speed', 8.0)          # m/s speed cap
        self.declare_parameter('max_accel', 3.0)          # m/s^2 command limit
        self.declare_parameter('max_brake', 8.0)          # m/s^2 command limit
        self.declare_parameter('max_jerk', 12.0)          # m/s^3 command slew
        self.declare_parameter('speed_step', 0.4)         # m/s per W tap
        self.declare_parameter('steer_step', 0.28)        # fraction per tap
        self.declare_parameter('steer_expo', 1.15)        # response curve
        self.declare_parameter('counter_steer_gain', 2.2)
        self.declare_parameter('auto_center_rate', 0.15)  # 1/s per (m/s)
        self.declare_parameter('speed_steer_falloff', 0.07)  # per m/s
        self.declare_parameter('speed_kp', 1.6)
        self.declare_parameter('auto_mission', True)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_accel = float(self.get_parameter('max_accel').value)
        self.max_brake = float(self.get_parameter('max_brake').value)
        self.max_jerk = float(self.get_parameter('max_jerk').value)
        self.speed_step = float(self.get_parameter('speed_step').value)
        self.steer_step = float(self.get_parameter('steer_step').value)
        self.steer_expo = float(self.get_parameter('steer_expo').value)
        self.counter_gain = float(self.get_parameter('counter_steer_gain').value)
        self.center_rate = float(self.get_parameter('auto_center_rate').value)
        self.falloff = float(self.get_parameter('speed_steer_falloff').value)
        self.speed_kp = float(self.get_parameter('speed_kp').value)

        # Control state. steer is a normalized [-1, 1] "wheel position".
        self.steer = 0.0
        self.target_speed = 0.0
        self.gear = 1                 # 1 forward, -1 reverse
        self.speed = 0.0              # measured, signed (m/s)
        self.last_accel_cmd = 0.0
        self.mission_sent = False
        self.brake_hold = True        # start stopped
        self.status = 'waiting for keys'
        self.lock = threading.Lock()

        self.cmd_pub = self.create_publisher(AckermannDriveStamped, self.cmd_topic, 1)
        self.create_subscription(
            CarState,
            self.get_parameter('car_state_topic').value,
            self.on_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.mission_client = self.create_client(SetCanState, '/ros_can/set_mission')
        self.create_timer(1.0 / self.rate_hz, self.on_tick)

        if bool(self.get_parameter('auto_mission').value):
            self.create_timer(0.5, self.try_send_mission)

    # ------------------------------------------------------------------ ROS

    def on_state(self, msg):
        # Signed longitudinal speed: the bicycle model reports body-frame v_x.
        self.speed = msg.twist.twist.linear.x

    def try_send_mission(self):
        if self.mission_sent or not self.mission_client.service_is_ready():
            return
        req = SetCanState.Request()
        req.ami_state = CanState.AMI_MANUAL
        self.mission_client.call_async(req)
        self.mission_sent = True
        self.status = 'MANUAL mission requested'

    # ------------------------------------------------------------ shaping

    def steer_limit(self):
        """Speed-sensitive fraction of full lock available right now."""
        return 1.0 / (1.0 + self.falloff * abs(self.speed))

    def apply_steer_tap(self, direction):
        """One steering key event. direction +1 = left, -1 = right."""
        with self.lock:
            step = self.steer_step
            # Counter-steer boost: pressing against the current angle
            # responds faster than steering further into the turn.
            if self.steer * direction < 0.0:
                step *= self.counter_gain
            # Speed-sensitive taps: finer steps at speed.
            step *= self.steer_limit()
            self.steer = max(-1.0, min(1.0, self.steer + direction * step))

    def apply_speed_tap(self, delta):
        with self.lock:
            sign = 1.0 if self.gear > 0 else -1.0
            target = self.target_speed + sign * delta
            if self.gear > 0:
                self.target_speed = max(0.0, min(self.max_speed, target))
            else:
                self.target_speed = min(0.0, max(-0.5 * self.max_speed, target))
            self.brake_hold = False

    def handle_key(self, key):
        if key in ('w', 'UP'):
            self.apply_speed_tap(self.speed_step)
            self.status = 'accelerate'
        elif key in ('s', 'DOWN'):
            self.apply_speed_tap(-2.0 * self.speed_step)
            if abs(self.target_speed) < 0.05 and abs(self.speed) < 0.3:
                self.brake_hold = True
            self.status = 'slow down'
        elif key in ('a', 'LEFT'):
            self.apply_steer_tap(+1.0)
            self.status = 'steer left'
        elif key in ('d', 'RIGHT'):
            self.apply_steer_tap(-1.0)
            self.status = 'steer right'
        elif key == ' ':
            with self.lock:
                self.target_speed = 0.0
                self.brake_hold = True
            self.status = 'FULL BRAKE'
        elif key == 'c':
            with self.lock:
                self.steer = 0.0
            self.status = 'center steering'
        elif key == 'r':
            with self.lock:
                if abs(self.speed) < 0.5:
                    self.gear = -self.gear
                    self.target_speed = 0.0
                    self.status = 'gear: ' + ('D' if self.gear > 0 else 'R')
                else:
                    self.status = 'slow down before shifting'
        elif key == 'm':
            self.mission_sent = False
            self.try_send_mission()
        elif key == '+':
            self.max_speed = min(30.0, self.max_speed + 1.0)
            self.status = f'speed cap {self.max_speed:.0f} m/s'
        elif key == '-':
            self.max_speed = max(1.0, self.max_speed - 1.0)
            with self.lock:
                self.target_speed = min(self.target_speed, self.max_speed)
            self.status = f'speed cap {self.max_speed:.0f} m/s'

    # ---------------------------------------------------------------- tick

    def on_tick(self):
        dt = 1.0 / self.rate_hz

        with self.lock:
            # Auto-centering, stronger with speed and with steering angle so
            # small corrections persist while big angles return quickly;
            # disabled near standstill so full lock holds in tight maneuvers.
            v = abs(self.speed)
            if v > 0.5:
                decay = self.center_rate * v * (0.3 + abs(self.steer)) * dt
                if abs(self.steer) <= decay:
                    self.steer = 0.0
                else:
                    self.steer -= math.copysign(decay, self.steer)

            # Expo response + speed-sensitive lock.
            shaped = math.copysign(abs(self.steer) ** self.steer_expo, self.steer)
            steering_angle = shaped * self.steer_limit() * self.max_steering

            # Cruise controller with jerk-limited acceleration command.
            if self.brake_hold:
                accel = -self.max_brake if abs(self.speed) > 0.05 else -1.0
            else:
                accel = self.speed_kp * (self.target_speed - self.speed)
            accel = max(-self.max_brake, min(self.max_accel, accel))
            max_step = self.max_jerk * dt
            accel = self.last_accel_cmd + max(
                -max_step, min(max_step, accel - self.last_accel_cmd))
            self.last_accel_cmd = accel

            target_speed = self.target_speed

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.steering_angle = steering_angle
        cmd.drive.acceleration = accel
        cmd.drive.speed = target_speed
        self.cmd_pub.publish(cmd)

    # ----------------------------------------------------------------- HUD

    def hud(self):
        with self.lock:
            steer, target = self.steer, self.target_speed
            gear = 'D' if self.gear > 0 else 'R'
            brake = self.brake_hold
        width = 21
        pos = int(round((steer + 1.0) * 0.5 * (width - 1)))
        bar = ''.join('█' if i == pos else ('|' if i == width // 2 else '·')
                      for i in range(width))
        mode = 'BRAKE' if brake else f'{target:+5.1f} m/s'
        return (
            f'\r\x1b[2K [{gear}] v {self.speed:+5.1f} → {mode}  '
            f'steer {bar} {steer * self.steer_limit() * self.max_steering:+.2f} rad  '
            f'cap {self.max_speed:.0f}  {self.status:<24.24}'
        )


def read_keys(handle_key, stop_event):
    """Raw-terminal key reader; decodes arrow-key escape sequences."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    arrows = {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}
    try:
        while not stop_event.is_set():
            if not select.select([sys.stdin], [], [], 0.05)[0]:
                continue
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    seq = sys.stdin.read(1)
                    if seq == '[' and select.select([sys.stdin], [], [], 0.01)[0]:
                        code = sys.stdin.read(1)
                        key = arrows.get(code)
                        if key:
                            handle_key(key)
                continue
            if ch in ('q', 'Q', '\x03'):
                stop_event.set()
                break
            handle_key(ch.lower())
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    if not sys.stdin.isatty():
        print('eufs_teleop needs an interactive terminal', file=sys.stderr)
        return 1

    # The EUFS launcher forces ROS_LOCALHOST_ONLY=1 on the simulator; a
    # terminal without it lands in a separate DDS domain and /cmd silently
    # never reaches the sim. Match it unless the user set it explicitly.
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')

    rclpy.init()
    node = TeleopNode()
    stop_event = threading.Event()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    key_thread = threading.Thread(
        target=read_keys, args=(node.handle_key, stop_event), daemon=True)
    key_thread.start()

    print('EUFS teleop — ' + HELP)
    try:
        while not stop_event.is_set():
            print(node.hud(), end='', flush=True)
            stop_event.wait(0.1)
    except KeyboardInterrupt:
        stop_event.set()

    # Leave the car braking straight before exiting.
    for _ in range(5):
        cmd = AckermannDriveStamped()
        cmd.drive.acceleration = -5.0
        node.cmd_pub.publish(cmd)
    print('\nteleop exit — car left braking')

    rclpy.try_shutdown()
    spin_thread.join(timeout=1.0)
    key_thread.join(timeout=1.0)
    return 0


if __name__ == '__main__':
    sys.exit(main())
