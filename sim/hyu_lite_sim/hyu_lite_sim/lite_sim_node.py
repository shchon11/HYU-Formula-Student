#!/usr/bin/env python3
"""hyu_lite_sim: the car, its ECU, its SBG and its perception -- without Gazebo.

One node (named race_car so the run scripts' waits see it) that, on the
wall clock:

  * integrates a bicycle-model car driven by the ECU command
      udp mode (default): the REAL drive_udp_bridge sends its '<ffBB'
        datagrams here (127.0.0.1), this node answers with the four-wheel RPM
        feedback the bridge turns into /vehicle/wheel_speeds -- the whole
        vehicle path of the car is exercised, enable bytes included
      ros  mode: subscribes /vehicle/cmd + /vehicle/as_state directly and
        publishes /vehicle/wheel_speeds itself (no bridge)
  * emulates the SBG Ellipse-D raw outputs sbg_raw_ekf fuses:
      /sbg/imu_data 25 Hz, /sbg/gps_pos /gps_vel /gps_hdt 5 Hz, receiver
      epochs delivered ~105 ms late, device clock in uint32 us, noise and
      status fields as on the 2026-08-01 bags (see sbg_emu.py)
  * emulates perception: /perception/cones (base_footprint) from the track
      csv plus off-track CLUTTER that appears as unknown-colour cones, with
      range/FOV limits, colour drop-out, noise, covariance tiers, latency
  * publishes ground truth for evaluation/RViz in the world frame, which with
      the EKF given the same datum IS the 'odom' frame:
      /ground_truth/state (CarState), /ground_truth/track, /ground_truth/clutter,
      /ground_truth/cones (visible, noise-free, base_footprint),
      /sim/debug/world, /sim/debug/car, /sim/debug/live_cones (markers)
  * services: /vehicle/reset_vehicle_pos (teleport to the start line, what
      'mission reset' calls), /sim/reset_clutter (new clutter, next seed)

The AS/AMI state (/vehicle/as_state, set_mission, ...) is NOT here: the car's
own vehicle_state.py provides it, exactly as on the vehicle. See
launch/lite_sim.launch.py for the whole step-1 assembly.
"""
import math
import os
import random
import time
from dataclasses import MISSING, fields
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from hyu_msgs.msg import (CanState, CarState, ConeArrayWithCovariance, ConeWithCovariance,
                          WheelSpeedsStamped)
from sbg_driver.msg import SbgGpsHdt, SbgGpsPos, SbgGpsVel, SbgImuData
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .ecu_udp import FakeEcu, SteeringTable
from .geo import LocalProjection
from .perception_emu import PerceptionEmulator, PerceptionParams
from .sbg_emu import SbgEmulator, SbgParams
from .track import (Clutter, Cone, generate_clutter, load_clutter, load_track_csv,
                    resolve_track_path, save_clutter)
from .vehicle import BicycleVehicle, VehicleParams

COLOUR_RGBA = {
    'blue': (0.1, 0.3, 1.0, 1.0),
    'yellow': (1.0, 0.9, 0.0, 1.0),
    'orange': (1.0, 0.5, 0.0, 1.0),
    'big_orange': (1.0, 0.35, 0.0, 1.0),
    'unknown': (0.65, 0.65, 0.65, 1.0),
}
CONE_FIELDS = {'blue': 'blue_cones', 'yellow': 'yellow_cones', 'orange': 'orange_cones',
               'big_orange': 'big_orange_cones', 'unknown': 'unknown_color_cones'}


def _declare_dataclass(node: Node, prefix: str, cls):
    """Declare one ROS parameter per dataclass field under `prefix.` and build the instance."""
    kwargs = {}
    for f in fields(cls):
        if f.default is not MISSING:
            default = f.default
        elif f.default_factory is not MISSING:  # type: ignore[misc]
            default = f.default_factory()  # type: ignore[misc]
        else:
            raise ValueError(f'{cls.__name__}.{f.name} has no default')
        val = node.declare_parameter(f'{prefix}.{f.name}', default).value
        if isinstance(default, list):
            val = [float(v) for v in val]
        elif isinstance(default, bool):
            val = bool(val)
        elif isinstance(default, int):
            val = int(val)
        elif isinstance(default, float):
            val = float(val)
        else:
            val = str(val)
        kwargs[f.name] = val
    return cls(**kwargs)


class LiteSimNode(Node):
    def __init__(self):
        super().__init__('race_car')
        p = self.declare_parameter
        # --- world ---------------------------------------------------------------
        track = str(p('track', 'small_track').value)
        self.datum_lat = float(p('datum_latitude', 37.5552263).value)   # 0801 site
        self.datum_lon = float(p('datum_longitude', 127.0454965).value)
        start_x = float(p('start_x', math.nan).value)
        start_y = float(p('start_y', math.nan).value)
        start_yaw_deg = float(p('start_yaw_deg', math.nan).value)
        self.world_frame = str(p('world_frame', 'odom').value)
        # 실차의 datum=첫fix 처럼, 월드 원점을 car_start(=출발 포즈)에 두고
        # +x를 출발 전진방향으로 정렬한다. graph_slam 이 첫 포즈를 절대 odom
        # 좌표에 고정하므로(map 재원점화 없음), skidpad_director 의 "map 원점 =
        # 출발 포즈, +x 전진" 계약은 월드 자체를 그렇게 잡아야 성립한다.
        self.origin_at_start = bool(p('world_origin_at_start', True).value)
        self.base_frame = str(p('base_frame', 'base_footprint').value)
        # --- ECU interface ----------------------------------------------------------
        self.ecu_mode = str(p('ecu_mode', 'udp').value).strip().lower()
        udp_listen_ip = str(p('udp_listen_ip', '127.0.0.1').value)
        udp_listen_port = int(p('udp_listen_port', 15000).value)
        udp_fb_ip = str(p('udp_feedback_ip', '127.0.0.1').value)
        udp_fb_port = int(p('udp_feedback_port', 15001).value)
        self.feedback_rate = float(p('feedback_rate_hz', 100.0).value)
        self.rpm_noise = float(p('rpm_noise', 0.8).value)
        self.rpm_quant = float(p('rpm_quant', 0.0).value)
        steering_table = str(p('steering_table', 'bridge').value)   # bridge | linear | /path.csv
        steering_ratio = float(p('steering_linear_ratio', 0.2).value)
        self.cmd_timeout = float(p('cmd_timeout_s', 0.25).value)
        # --- geometry of the sensors in base_footprint -------------------------------
        self.antenna_x = float(p('antenna_offset_x', 1.25).value)   # == sbg_raw_ekf default
        self.antenna_y = float(p('antenna_offset_y', 0.0).value)
        self.imu_x = float(p('imu_offset_x', 1.07).value)
        self.imu_y = float(p('imu_offset_y', -0.06).value)
        # --- clutter ------------------------------------------------------------------
        self.clutter_count = int(p('clutter_count', 60).value)
        self.clutter_seed = int(p('clutter_seed', 1).value)
        clutter_file = str(p('clutter_file', '').value)
        clutter_save = str(p('clutter_save', '').value)
        self.clutter_min_dist = float(p('clutter_min_dist_m', 2.5).value)
        self.clutter_margin = float(p('clutter_margin_m', 25.0).value)
        self.clutter_max_dist = float(p('clutter_max_dist_m', 30.0).value)
        # --- rates ----------------------------------------------------------------------
        self.physics_rate = float(p('physics_rate_hz', 100.0).value)
        gt_rate = float(p('ground_truth_rate_hz', 25.0).value)
        self.publish_gt = bool(p('publish_ground_truth', True).value)
        noise_seed = int(p('noise_seed', 0).value)
        # --- models ----------------------------------------------------------------
        self.vp = _declare_dataclass(self, 'vehicle', VehicleParams)
        self.sp = _declare_dataclass(self, 'sbg', SbgParams)
        self.pp = _declare_dataclass(self, 'perception', PerceptionParams)
        # Per-run wheel effective-radius bias (tyre wear / pressure): a common
        # bias across all wheels (pressure) + per-wheel variation (wear). Drawn
        # once per run so wheel odometry carries a systematic scale error the
        # EKF's wheel_scale/wheel fusion must cope with.
        self.wheel_radius_bias_pct = float(p('wheel_radius_bias_pct', 2.0).value)
        self.wheel_radius_random_pct = float(p('wheel_radius_random_pct', 1.0).value)

        # --- world -------------------------------------------------------------------
        self.track_path = resolve_track_path(track)
        self.cones, csv_start = load_track_csv(self.track_path)
        sx, sy, syaw = csv_start if csv_start else (0.0, 0.0, 0.0)
        if not math.isnan(start_x):
            sx = start_x
        if not math.isnan(start_y):
            sy = start_y
        if not math.isnan(start_yaw_deg):
            syaw = math.radians(start_yaw_deg)
        if self.origin_at_start:
            # 콘·클러터를 출발 프레임으로 이동/회전: p' = R(-syaw)·(p - start)
            import math as _m
            c, s_ = _m.cos(-syaw), _m.sin(-syaw)

            def _xf(x, y):
                dx, dy = x - sx, y - sy
                return (c * dx - s_ * dy, s_ * dx + c * dy)

            def _rot(x, y):
                return (c * x - s_ * y, s_ * x + c * y)

            for _cn in self.cones:
                _cn.x, _cn.y = _xf(_cn.x, _cn.y)
            self.start_pose = (0.0, 0.0, 0.0)
        else:
            self.start_pose = (sx, sy, syaw)
        self.clutter_path = clutter_file
        if clutter_file:
            self.clutter = load_clutter(clutter_file)
            if self.origin_at_start:
                # A saved clutter file is in the track's csv frame; move it into
                # the start frame too (offsets are relative -> rotate only).
                for _cl in self.clutter:
                    _cl.x, _cl.y = _xf(_cl.x, _cl.y)
                    _cl.offsets = [_rot(ox, oy) for ox, oy in _cl.offsets]
        else:
            self.clutter = generate_clutter(self.cones, self.clutter_count, self.clutter_seed,
                                            self.clutter_min_dist, self.clutter_margin,
                                            self.clutter_max_dist)
        if clutter_save:
            save_clutter(clutter_save, self.clutter)
        self.proj = LocalProjection(self.datum_lat, self.datum_lon)
        rng_seed = noise_seed if noise_seed else int(time.time()) & 0xFFFF
        self.rng = random.Random(rng_seed)

        # --- vehicle ------------------------------------------------------------------
        if self.wheel_radius_bias_pct > 0.0 or self.wheel_radius_random_pct > 0.0:
            common = self.rng.uniform(-self.wheel_radius_bias_pct, self.wheel_radius_bias_pct) / 100.0
            self.vp.wheel_scale = [
                sc * (1.0 + common + self.rng.uniform(-self.wheel_radius_random_pct,
                                                      self.wheel_radius_random_pct) / 100.0)
                for sc in self.vp.wheel_scale]
        self.car = BicycleVehicle(self.vp)
        self._teleport_to_start()
        self.sbg = SbgEmulator(self.sp, self.proj, random.Random(rng_seed + 1))
        self.perc = PerceptionEmulator(self.pp, self.cones, self.clutter, random.Random(rng_seed + 2))

        # --- ECU ------------------------------------------------------------------------
        self.ecu: Optional[FakeEcu] = None
        self.ros_cmd = None           # (speed, steer, t_mono)
        self.ros_as_driving = False
        self.ros_as_t = -1.0
        if self.ecu_mode == 'udp':
            if steering_table == 'bridge':
                try:
                    table = SteeringTable.from_bridge_share()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f'steering table from drive_udp_bridge not available ({exc}); '
                                           f'using linear ratio {steering_ratio}')
                    table = SteeringTable.linear(steering_ratio)
            elif steering_table == 'linear':
                table = SteeringTable.linear(steering_ratio)
            else:
                table = SteeringTable.from_csv(os.path.expanduser(steering_table))
            self.ecu = FakeEcu(udp_listen_ip, udp_listen_port, udp_fb_ip, udp_fb_port, table)
        elif self.ecu_mode == 'ros':
            cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(AckermannDriveStamped, '/vehicle/cmd', self._on_cmd, cmd_qos)
            as_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(CanState, '/vehicle/as_state', self._on_as_state, as_qos)
            self.pub_wheels = self.create_publisher(WheelSpeedsStamped, '/vehicle/wheel_speeds', 10)
        else:
            raise ValueError(f"ecu_mode must be 'udp' or 'ros', got {self.ecu_mode!r}")

        # --- publishers ------------------------------------------------------------------
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_imu = self.create_publisher(SbgImuData, '/sbg/imu_data', rel)
        self.pub_pos = self.create_publisher(SbgGpsPos, '/sbg/gps_pos', rel)
        self.pub_vel = self.create_publisher(SbgGpsVel, '/sbg/gps_vel', rel)
        self.pub_hdt = self.create_publisher(SbgGpsHdt, '/sbg/gps_hdt', rel)
        self.pub_cones = self.create_publisher(ConeArrayWithCovariance, '/perception/cones', rel)
        self.pub_gt_state = self.create_publisher(CarState, '/ground_truth/state', rel)
        self.pub_gt_track = self.create_publisher(ConeArrayWithCovariance, '/ground_truth/track', latched)
        self.pub_gt_clutter = self.create_publisher(ConeArrayWithCovariance, '/ground_truth/clutter', latched)
        self.pub_gt_cones = self.create_publisher(ConeArrayWithCovariance, '/ground_truth/cones', rel)
        self.pub_world_mk = self.create_publisher(MarkerArray, '/sim/debug/world', latched)
        self.pub_car_mk = self.create_publisher(MarkerArray, '/sim/debug/car', rel)
        self.pub_live_mk = self.create_publisher(MarkerArray, '/sim/debug/live_cones', rel)
        self.pub_status = self.create_publisher(String, '/sim/status', rel)

        self.create_service(Trigger, '/vehicle/reset_vehicle_pos', self._srv_reset_pos)
        self.create_service(Trigger, '/sim/reset_clutter', self._srv_reset_clutter)

        # --- timing state -----------------------------------------------------------------
        self.t0_mono = time.monotonic()
        self.last_phys = self.t0_mono
        self.imu_tick = -1
        self.phys_tick = 0
        self.fb_every = max(1, int(round(self.physics_rate / max(1.0, self.feedback_rate))))
        self.queue: List = []       # (due_mono, kind, payload)
        self.last_cmd_log = 0.0
        self.enabled = False
        self.cmd_speed = 0.0
        self.cmd_steer = 0.0

        self.create_timer(1.0 / self.physics_rate, self._physics)
        self.create_timer(self.sbg.imu_dt, self._imu)
        self.create_timer(1.0 / self.pp.rate_hz, self._perception)
        if self.publish_gt:
            self.create_timer(1.0 / gt_rate, self._ground_truth)
            self.create_timer(0.1, self._car_marker)
            self.create_timer(2.0, self._world_markers)
        self.create_timer(1.0, self._status)
        self._world_markers()
        self._publish_static_truth()

        self.get_logger().info(
            f"lite sim: track {os.path.basename(self.track_path)} ({len(self.cones)} cones), "
            f"{len(self.clutter)} clutter objects (seed {self.clutter_seed}"
            f"{', file ' + clutter_file if clutter_file else ''}), start "
            f"({sx:.1f}, {sy:.1f}, {math.degrees(syaw):.0f} deg), datum ({self.datum_lat:.7f}, "
            f"{self.datum_lon:.7f}), world frame '{self.world_frame}'")
        if self.ecu:
            self.get_logger().info(
                f"ECU: udp -- commands on {udp_listen_ip}:{udp_listen_port} ('<ffBB' from "
                f"drive_udp_bridge), RPM feedback x4 float32 -> {udp_fb_ip}:{udp_fb_port} at "
                f"{self.feedback_rate:.0f} Hz; the car moves only while enable AND autonomous_enable are 1")
        else:
            self.get_logger().info("ECU: ros -- /vehicle/cmd + /vehicle/as_state (AS_DRIVING) in, "
                                   "/vehicle/wheel_speeds out (no bridge)")
        self.get_logger().info(
            f"SBG: imu {self.sp.imu_rate_hz:.0f} Hz, receiver {self.sp.gps_rate_hz:.0f} Hz "
            f"(+{self.sp.receiver_latency_s * 1e3:.0f} ms), fix {self.sp.fix}"
            f"{' schedule ' + self.sp.fix_schedule if self.sp.fix_schedule else ''}; antenna at "
            f"({self.antenna_x:+.2f}, {self.antenna_y:+.2f}) in {self.base_frame} -- sbg_raw_ekf "
            f"antenna_offset_x/y MUST match; datum_latitude/longitude too for GT == odom")
        self.get_logger().info(
            f"perception: {self.pp.rate_hz:.0f} Hz +{self.pp.latency_s * 1e3:.0f} ms, range "
            f"{self.pp.min_range_m:.1f}-{self.pp.max_range_m:.0f} m, lidar fov {self.pp.lidar_fov_deg:.0f}, "
            f"colour <= {self.pp.colour_max_range_m:.0f} m in camera fov {self.pp.camera_fov_deg:.0f}")

    # ------------------------------------------------------------------ helpers
    def _now_msg(self):
        return self.get_clock().now().to_msg()

    def _t_sim(self) -> float:
        return time.monotonic() - self.t0_mono

    def _teleport_to_start(self):
        sx, sy, syaw = self.start_pose
        # csv car_start is the base_footprint pose; the model's reference is the rear axle
        ax = sx + self.vp.rear_axle_x_m * math.cos(syaw)
        ay = sy + self.vp.rear_axle_x_m * math.sin(syaw)
        self.car.teleport(ax, ay, syaw)

    def _body_offsets(self, x_base: float, y_base: float):
        """base_footprint offsets -> offsets from the rear axle (model reference)."""
        return (x_base - self.vp.rear_axle_x_m, y_base)

    # ------------------------------------------------------------------ inputs
    def _on_cmd(self, msg: AckermannDriveStamped):
        self.ros_cmd = (float(msg.drive.speed), float(msg.drive.steering_angle), time.monotonic())

    def _on_as_state(self, msg: CanState):
        self.ros_as_driving = msg.as_state == CanState.AS_DRIVING
        self.ros_as_t = time.monotonic()

    # ------------------------------------------------------------------ physics
    def _physics(self):
        now = time.monotonic()
        dt = min(0.05, max(0.0, now - self.last_phys))
        self.last_phys = now
        if self.ecu is not None:
            self.ecu.poll()
            speed, steer, enabled = self.ecu.command(self.cmd_timeout)
        else:
            fresh = self.ros_cmd is not None and (now - self.ros_cmd[2]) <= self.cmd_timeout
            as_ok = self.ros_as_driving and (now - self.ros_as_t) <= 0.5
            enabled = fresh and as_ok
            speed, steer = (self.ros_cmd[0], self.ros_cmd[1]) if (fresh and enabled) else (0.0, 0.0)
        if enabled != self.enabled:
            self.get_logger().info('drive ENABLED (ECU autonomous enable 1)' if enabled
                                   else 'drive disabled -> braking to rest, steering held')
        self.enabled, self.cmd_speed, self.cmd_steer = enabled, speed, steer
        self.car.set_command(speed, steer, enabled)
        self.car.step(dt)
        self.phys_tick += 1
        if self.phys_tick % self.fb_every == 0:
            rpm = self.car.wheel_rpm(self.rng, self.rpm_noise, self.rpm_quant)
            if self.ecu is not None:
                self.ecu.send_feedback(rpm)
            else:
                self._publish_wheel_speeds(rpm)
        self._deliver(now)

    def _publish_wheel_speeds(self, rpm):
        m = WheelSpeedsStamped()
        m.header.stamp = self._now_msg()
        m.header.frame_id = self.base_frame
        k = math.pi * self.vp.tire_diameter_m / 60.0
        m.speeds.lf_speed, m.speeds.rf_speed, m.speeds.lb_speed, m.speeds.rb_speed = [float(r * k) for r in rpm]
        self.pub_wheels.publish(m)

    def _deliver(self, now: float):
        if not self.queue:
            return
        keep = []
        for due, kind, payload in self.queue:
            if due > now:
                keep.append((due, kind, payload))
                continue
            if kind == 'gps':
                stamp = self._now_msg()          # the driver stamps receiver logs on arrival
                pos, vel, hdt = payload
                pos.header.stamp = stamp
                vel.header.stamp = stamp
                hdt.header.stamp = stamp
                self.pub_pos.publish(pos)
                self.pub_vel.publish(vel)
                self.pub_hdt.publish(hdt)
            elif kind == 'cones':
                msg, live = payload
                self.pub_cones.publish(msg)
                self.pub_live_mk.publish(live)
        self.queue = keep

    # ------------------------------------------------------------------ SBG
    def _imu(self):
        t = self._t_sim()
        tick = int(round(t / self.sbg.imu_dt))
        if tick <= self.imu_tick:
            return                                   # timer fired early; keep the device clock monotonic
        self.imu_tick = tick
        stamp = self._now_msg()
        rx, ry = self._body_offsets(self.imu_x, self.imu_y)
        a_body = self.car.body_point_acceleration(rx, ry)
        self.pub_imu.publish(self.sbg.imu_msg(stamp, tick, a_body, self.car.s.yaw_rate, self.car.s.v))
        if tick % self.sbg.gps_every == 0:
            ax, ay = self._body_offsets(self.antenna_x, self.antenna_y)
            ant_xy = self.car.world_point(ax, ay)
            ant_v = self.car.world_velocity(ax, ay)
            msgs = self.sbg.gps_msgs(stamp, tick, t, ant_xy, ant_v, self.car.s.yaw)
            self.queue.append((time.monotonic() + self.sp.receiver_latency_s, 'gps', msgs))

    # ------------------------------------------------------------------ perception
    def _perception(self):
        stamp = self._now_msg()
        base = self.car.base_pose()
        obs = self.perc.observe(base, self.car.s.v)
        msg = ConeArrayWithCovariance()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        live = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        live.markers.append(clear)
        i = 0
        for colour, field in CONE_FIELDS.items():
            arr = getattr(msg, field)
            for (x, y, cov) in obs[colour]:
                c = ConeWithCovariance()
                c.point.x, c.point.y, c.point.z = x, y, 0.0
                c.covariance = [float(v) for v in cov]
                arr.append(c)
                live.markers.append(self._cone_marker('live', i, self.base_frame, stamp, x, y, colour, 0.9))
                i += 1
        self.queue.append((time.monotonic() + self.pp.latency_s, 'cones', (msg, live)))
        if self.publish_gt:
            gt = ConeArrayWithCovariance()
            gt.header.stamp = stamp
            gt.header.frame_id = self.base_frame
            for colour, field in CONE_FIELDS.items():
                arr = getattr(gt, field)
                for (x, y) in self.perc.truth_visible(base)[colour]:
                    c = ConeWithCovariance()
                    c.point.x, c.point.y = x, y
                    arr.append(c)
            self.pub_gt_cones.publish(gt)

    # ------------------------------------------------------------------ ground truth
    def _ground_truth(self):
        bx, by, byaw = self.car.base_pose()
        vx, vy, wz = self.car.base_twist()
        rx, ry = self._body_offsets(0.0, 0.0)
        ax, ay = self.car.body_point_acceleration(rx, ry)
        m = CarState()
        m.header.stamp = self._now_msg()
        m.header.frame_id = self.world_frame
        m.child_frame_id = self.base_frame
        m.pose.pose.position.x, m.pose.pose.position.y = bx, by
        m.pose.pose.orientation.z = math.sin(0.5 * byaw)
        m.pose.pose.orientation.w = math.cos(0.5 * byaw)
        m.twist.twist.linear.x, m.twist.twist.linear.y = vx, vy
        m.twist.twist.angular.z = wz
        m.linear_acceleration.x, m.linear_acceleration.y = ax, ay
        self.pub_gt_state.publish(m)

    def _publish_static_truth(self):
        stamp = self._now_msg()
        track = ConeArrayWithCovariance()
        track.header.stamp = stamp
        track.header.frame_id = self.world_frame
        for c in self.cones:
            cc = ConeWithCovariance()
            cc.point.x, cc.point.y = c.x, c.y
            getattr(track, CONE_FIELDS[c.tag]).append(cc)
        self.pub_gt_track.publish(track)
        cl = ConeArrayWithCovariance()
        cl.header.stamp = stamp
        cl.header.frame_id = self.world_frame
        for c in self.clutter:
            for (x, y) in [(c.x, c.y)] + [(c.x + ox, c.y + oy) for ox, oy in c.offsets]:
                cc = ConeWithCovariance()
                cc.point.x, cc.point.y = x, y
                cl.unknown_color_cones.append(cc)
        self.pub_gt_clutter.publish(cl)

    # ------------------------------------------------------------------ markers
    def _cone_marker(self, ns, i, frame, stamp, x, y, colour, alpha=1.0, scale=None):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = i
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        big = colour == 'big_orange'
        s = scale or (0.3 if big else 0.22)
        h = 0.5 if big else 0.32
        m.scale.x = m.scale.y = s
        m.scale.z = h
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, h / 2.0
        m.pose.orientation.w = 1.0
        r, g, b, _ = COLOUR_RGBA[colour]
        m.color = ColorRGBA(r=r, g=g, b=b, a=alpha)
        return m

    def _world_markers(self):
        stamp = self._now_msg()
        arr = MarkerArray()
        for i, c in enumerate(self.cones):
            arr.markers.append(self._cone_marker('track', i, self.world_frame, stamp, c.x, c.y, c.tag))
        j = 0
        for c in self.clutter:
            for (x, y) in [(c.x, c.y)] + [(c.x + ox, c.y + oy) for ox, oy in c.offsets]:
                m = Marker()
                m.header.frame_id = self.world_frame
                m.header.stamp = stamp
                m.ns = 'clutter'
                m.id = j
                j += 1
                m.type = Marker.CUBE if c.kind in ('fence', 'car') else Marker.CYLINDER
                m.action = Marker.ADD
                m.scale.x = m.scale.y = c.size_m
                m.scale.z = 0.6 if c.kind != 'person' else 1.7
                m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, m.scale.z / 2.0
                m.pose.orientation.w = 1.0
                m.color = ColorRGBA(r=0.45, g=0.45, b=0.45, a=0.8)
                arr.markers.append(m)
        # start line
        sx, sy, syaw = self.start_pose
        m = Marker()
        m.header.frame_id = self.world_frame
        m.header.stamp = stamp
        m.ns = 'start'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = 1.5, 0.15, 0.15
        m.pose.position.x, m.pose.position.y, m.pose.position.z = sx, sy, 0.05
        m.pose.orientation.z = math.sin(0.5 * syaw)
        m.pose.orientation.w = math.cos(0.5 * syaw)
        m.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.8)
        arr.markers.append(m)
        self.pub_world_mk.publish(arr)

    def _car_marker(self):
        stamp = self._now_msg()
        bx, by, byaw = self.car.base_pose()
        arr = MarkerArray()
        # body box centred mid-wheelbase
        cx = self.vp.rear_axle_x_m + 0.5 * self.vp.wheelbase_m
        body = Marker()
        body.header.frame_id = self.world_frame
        body.header.stamp = stamp
        body.ns = 'car'
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.scale.x, body.scale.y, body.scale.z = self.vp.wheelbase_m + 1.2, self.vp.track_width_m + 0.2, 0.5
        body.pose.position.x = bx + cx * math.cos(byaw)
        body.pose.position.y = by + cx * math.sin(byaw)
        body.pose.position.z = 0.3
        body.pose.orientation.z = math.sin(0.5 * byaw)
        body.pose.orientation.w = math.cos(0.5 * byaw)
        col = (0.1, 0.9, 0.3, 0.6) if self.enabled else (0.9, 0.2, 0.2, 0.6)
        body.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=col[3])
        arr.markers.append(body)
        ant = Marker()
        ant.header = body.header
        ant.ns = 'antenna'
        ant.id = 0
        ant.type = Marker.SPHERE
        ant.action = Marker.ADD
        ant.scale.x = ant.scale.y = ant.scale.z = 0.2
        ax, ay = self.car.world_point(*self._body_offsets(self.antenna_x, self.antenna_y))
        ant.pose.position.x, ant.pose.position.y, ant.pose.position.z = ax, ay, 1.0
        ant.pose.orientation.w = 1.0
        ant.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        arr.markers.append(ant)
        self.pub_car_mk.publish(arr)

    def _status(self):
        s = self.car.s
        bx, by, byaw = self.car.base_pose()
        ecu = 'udp' if self.ecu else 'ros'
        extra = ''
        if self.ecu:
            age = (time.monotonic() - self.ecu.last_rx_t) if self.ecu.last_rx_t > 0 else float('inf')
            extra = f' rx={self.ecu.n_rx} age={age:.2f}s bad={self.ecu.n_bad}'
        txt = (f"ecu={ecu}{extra} enable={int(self.enabled)} cmd=({self.cmd_speed:.2f} m/s, "
               f"{math.degrees(self.cmd_steer):.1f} deg) v={s.v:.2f} m/s delta={math.degrees(s.delta):.1f} deg "
               f"pose=({bx:.1f}, {by:.1f}, {math.degrees(byaw):.0f}) fix={self.sbg.fix_name} "
               f"clutter={len(self.clutter)} t={self._t_sim():.0f}s")
        self.pub_status.publish(String(data=txt))

    # ------------------------------------------------------------------ services
    def _srv_reset_pos(self, req, res):
        self._teleport_to_start()
        sx, sy, syaw = self.start_pose
        res.success = True
        res.message = (f'vehicle teleported to the start ({sx:.1f}, {sy:.1f}, {math.degrees(syaw):.0f} deg); '
                       f'restart the INS (mission reset does)')
        self.get_logger().warn(res.message)
        return res

    def _srv_reset_clutter(self, req, res):
        self.clutter_seed += 1
        self.clutter = generate_clutter(self.cones, self.clutter_count, self.clutter_seed,
                                        self.clutter_min_dist, self.clutter_margin, self.clutter_max_dist)
        self.perc.set_world(self.cones, self.clutter)
        self._world_markers()
        self._publish_static_truth()
        res.success = True
        res.message = f'{len(self.clutter)} clutter objects regenerated (seed {self.clutter_seed})'
        self.get_logger().info(res.message)
        return res

    def destroy_node(self):
        if self.ecu:
            self.ecu.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = LiteSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
