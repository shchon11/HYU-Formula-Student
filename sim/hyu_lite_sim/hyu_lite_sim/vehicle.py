"""Kinematic bicycle vehicle with actuator dynamics and per-wheel speeds.

Reference point is the REAR AXLE (the stack's Pure Pursuit assumes the pose
it drives is the rear axle); base_footprint -- the frame every sensor output
is expressed in -- sits `rear_axle_x` metres behind it along the body x axis
(base_footprint is the ground point under the ZED's stereo centre, see
hyu_sensor_bringup; 2026-07-26: camera ~0.91 m behind the axle line the old
base marked).

The ECU model is what the Speedgoat does with the bridge's packet: a speed
setpoint tracked by a P loop with acceleration/brake limits and a first-order
lag, and a steering-wheel setpoint tracked with a rate limit and a lag.
"""
import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple


@dataclass
class VehicleParams:
    wheelbase_m: float = 1.58          # hyu_pure_pursuit.yaml wheelbase_m
    track_width_m: float = 1.20
    rear_axle_x_m: float = 0.91        # rear axle position in base_footprint (+x forward)
    max_steer_rad: float = 0.335       # bicycle-equivalent lock: steering wheel +-90 deg through the bridge table
    steer_rate_rad_s: float = 2.0      # actuator slew at the road wheel
    steer_tau_s: float = 0.12          # actuator first-order lag
    accel_max_mps2: float = 3.0
    brake_max_mps2: float = 6.0
    speed_kp: float = 2.0              # ECU speed loop gain [1/s]
    speed_tau_s: float = 0.25          # drivetrain lag
    drag_per_mps: float = 0.05         # rolling/aero decel per m/s (coast-down)
    max_speed_mps: float = 20.0
    tire_diameter_m: float = 0.4572    # drive_udp_bridge.yaml tire_diameter_m
    # Per-wheel effective rolling-radius scale FL FR RL RR (encoder vs tape).
    wheel_scale: List[float] = field(default_factory=lambda: [1.000, 0.997, 1.002, 0.999])
    allow_reverse: bool = True


@dataclass
class VehicleState:
    x: float = 0.0        # rear axle, world
    y: float = 0.0
    yaw: float = 0.0      # ENU, CCW from +x
    v: float = 0.0        # longitudinal speed at the rear axle (signed)
    a: float = 0.0        # longitudinal acceleration (realised)
    delta: float = 0.0    # road-wheel (bicycle) steering angle
    yaw_rate: float = 0.0
    yaw_acc: float = 0.0


class BicycleVehicle:
    def __init__(self, params: VehicleParams):
        self.p = params
        self.s = VehicleState()
        self.v_cmd = 0.0
        self.delta_cmd = 0.0
        self.enabled = False   # ECU autonomous enable: False -> brake to rest, steer hold

    # --- commands ----------------------------------------------------------
    def set_command(self, speed_mps: float, steer_rad: float, enabled: bool) -> None:
        self.enabled = bool(enabled)
        if not self.enabled:
            speed_mps = 0.0
        if not self.p.allow_reverse:
            speed_mps = max(0.0, speed_mps)
        self.v_cmd = max(-self.p.max_speed_mps, min(self.p.max_speed_mps, float(speed_mps)))
        self.delta_cmd = max(-self.p.max_steer_rad, min(self.p.max_steer_rad, float(steer_rad)))

    def teleport(self, x: float, y: float, yaw: float) -> None:
        self.s = VehicleState(x=x, y=y, yaw=yaw)
        self.v_cmd = 0.0
        self.delta_cmd = 0.0

    # --- integration ---------------------------------------------------------
    def step(self, dt: float) -> None:
        p, s = self.p, self.s
        if dt <= 0.0:
            return
        # Steering: slew + lag toward the setpoint (or hold when disabled).
        target = self.delta_cmd if self.enabled else s.delta
        d_lag = (target - s.delta) / max(p.steer_tau_s, 1e-3)
        d_max = p.steer_rate_rad_s
        d_dot = max(-d_max, min(d_max, d_lag))
        s.delta = max(-p.max_steer_rad, min(p.max_steer_rad, s.delta + d_dot * dt))

        # Speed: P loop with accel/brake limits, through a drivetrain lag.
        # Limits follow the direction of motion: accelerating away from rest
        # is bounded by accel_max, slowing toward rest by brake_max.
        if self.enabled:
            a_req = p.speed_kp * (self.v_cmd - s.v)
        else:
            a_req = -p.brake_max_mps2 * (1.0 if s.v > 0.05 else (-1.0 if s.v < -0.05 else 0.0))
        direction = s.v if abs(s.v) > 0.05 else self.v_cmd
        if direction >= 0.0:
            lo, hi = -p.brake_max_mps2, p.accel_max_mps2
        else:
            lo, hi = -p.accel_max_mps2, p.brake_max_mps2
        a_req = max(lo, min(hi, a_req))
        s.a += (a_req - s.a) * min(1.0, dt / max(p.speed_tau_s, 1e-3))
        a_total = s.a - p.drag_per_mps * s.v
        v_new = s.v + a_total * dt
        # Braking never carries the car through zero.
        if abs(self.v_cmd) < 0.01 or not self.enabled:
            if v_new * s.v < 0.0:
                v_new = 0.0
                s.a = 0.0
            if abs(v_new) < 0.01:
                v_new = 0.0
        s.v = v_new

        # Kinematics (rear-axle reference).
        yaw_rate = s.v * math.tan(s.delta) / p.wheelbase_m
        s.yaw_acc = (yaw_rate - s.yaw_rate) / dt
        s.yaw_rate = yaw_rate
        s.x += s.v * math.cos(s.yaw) * dt
        s.y += s.v * math.sin(s.yaw) * dt
        s.yaw = (s.yaw + yaw_rate * dt + math.pi) % (2.0 * math.pi) - math.pi

    # --- derived quantities ----------------------------------------------------
    def base_pose(self) -> Tuple[float, float, float]:
        """base_footprint (x, y, yaw) in the world frame."""
        s, d = self.s, -self.p.rear_axle_x_m
        return (s.x + d * math.cos(s.yaw), s.y + d * math.sin(s.yaw), s.yaw)

    def body_point_velocity(self, rx: float, ry: float) -> Tuple[float, float]:
        """Velocity of a body point (rx, ry from the REAR AXLE, x fwd y left), body frame."""
        s = self.s
        return (s.v - s.yaw_rate * ry, s.yaw_rate * rx)

    def body_point_acceleration(self, rx: float, ry: float) -> Tuple[float, float]:
        """Acceleration of a body point, body frame: a_axle + alpha x r + w x (w x r)."""
        s = self.s
        w, al = s.yaw_rate, s.yaw_acc
        ax = s.a - al * ry - w * w * rx
        ay = s.v * w + al * rx - w * w * ry
        return (ax, ay)

    def base_twist(self) -> Tuple[float, float, float]:
        """(vx, vy, wz) of base_footprint in its own frame."""
        vx, vy = self.body_point_velocity(-self.p.rear_axle_x_m, 0.0)
        return (vx, vy, self.s.yaw_rate)

    def world_point(self, rx: float, ry: float) -> Tuple[float, float]:
        """World position of a body point (offsets from the rear axle)."""
        s = self.s
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        return (s.x + c * rx - sn * ry, s.y + sn * rx + c * ry)

    def world_velocity(self, rx: float, ry: float) -> Tuple[float, float]:
        vx, vy = self.body_point_velocity(rx, ry)
        c, sn = math.cos(self.s.yaw), math.sin(self.s.yaw)
        return (c * vx - sn * vy, sn * vx + c * vy)

    def wheel_speeds(self) -> Tuple[float, float, float, float]:
        """Rolling speeds FL FR RL RR [m/s], signed, Ackermann geometry, no slip."""
        p, s = self.p, self.s
        L, h = p.wheelbase_m, 0.5 * p.track_width_m
        t = math.tan(s.delta)
        # inner/outer road-wheel angles
        dl = math.atan2(L * t, L - h * t) if abs(L - h * t) > 1e-6 else math.copysign(math.pi / 2, t)
        dr = math.atan2(L * t, L + h * t) if abs(L + h * t) > 1e-6 else math.copysign(math.pi / 2, t)
        out = []
        for (rx, ry, di) in ((L, h, dl), (L, -h, dr), (0.0, h, 0.0), (0.0, -h, 0.0)):
            vx, vy = self.body_point_velocity(rx, ry)
            out.append(vx * math.cos(di) + vy * math.sin(di))
        return tuple(out)  # type: ignore[return-value]

    def wheel_rpm(self, rng=None, noise_rpm: float = 0.0, quant_rpm: float = 0.0,
                  stiction_mps: float = 0.02) -> Tuple[float, float, float, float]:
        """What the ECU's encoders report: RPM per wheel with radius scale, noise, quantisation."""
        circ = math.pi * self.p.tire_diameter_m
        out = []
        for v, k in zip(self.wheel_speeds(), self.p.wheel_scale):
            if abs(v) < stiction_mps:
                out.append(0.0)
                continue
            rpm = v / (circ * k) * 60.0
            if rng is not None and noise_rpm > 0.0:
                rpm += rng.gauss(0.0, noise_rpm)
            if quant_rpm > 0.0:
                rpm = round(rpm / quant_rpm) * quant_rpm
            out.append(rpm)
        return tuple(out)  # type: ignore[return-value]
