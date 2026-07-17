#!/usr/bin/env python3
"""Skidpad mission director: phase-gated cone feed for the local planner.

The skidpad figure-eight cannot be driven by reactive cone-following alone —
at the centre junction the blue-left/yellow-right corridors of BOTH circles
and the entry/exit lane all cross, so the planner has no way to pick the
mission-correct branch. This node owns that choice. It republishes the SLAM
cone map with only the cones that belong to the current mission phase, and the
unchanged local planner drives whatever corridor it is fed:

  ENTRY        entry-lane orange cones, recoloured blue(left)/yellow(right)
  RIGHT_CIRCLE right-circle annulus blue/yellow only  (right_laps turns, CW)
  LEFT_CIRCLE  left-circle annulus blue/yellow only   (left_laps turns, CCW)
  EXIT         exit-lane orange cones, recoloured
  STOP         empty map -> local path invalid -> controller brakes

Geometry is expressed in the SLAM map frame, whose origin is the car's start
pose (x forward along the entry lane). The standard FS layout puts the circle
centres at (entry_length_m, ±circle_offset_m).

Lap counts are parameters (right_laps / left_laps) per mission tuning.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from hyu_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import String

PHASE_ENTRY = "ENTRY"
PHASE_RIGHT = "RIGHT_CIRCLE"
PHASE_LEFT = "LEFT_CIRCLE"
PHASE_EXIT = "EXIT"
PHASE_STOP = "STOP"

TWO_PI = 2.0 * math.pi


class SkidpadDirector(Node):

    def __init__(self):
        super().__init__("skidpad_director")

        self.cone_map_topic = self.declare_parameter(
            "cone_map_topic", "/localization/cone_map").value
        self.ego_odom_topic = self.declare_parameter(
            "ego_odom_topic", "/localization/ego_odom").value
        self.output_topic = self.declare_parameter(
            "output_cone_map_topic", "/planning/skidpad/cone_map").value
        self.phase_topic = self.declare_parameter(
            "phase_topic", "/planning/skidpad/phase").value

        # Mission tuning: laps per circle.
        self.right_laps = int(self.declare_parameter("right_laps", 2).value)
        self.left_laps = int(self.declare_parameter("left_laps", 2).value)

        # FS skidpad geometry in the map frame (origin = start pose).
        self.entry_length_m = self.declare_parameter("entry_length_m", 14.4).value
        self.circle_offset_m = self.declare_parameter("circle_offset_m", 9.125).value
        self.annulus_min_m = self.declare_parameter("annulus_min_m", 6.5).value
        self.annulus_max_m = self.declare_parameter("annulus_max_m", 13.5).value
        # Ring-fit classification (FS standard radii). A cone belongs to a
        # circle when it sits on that circle's inner or outer cone ring; a
        # plain radial band around the active centre also captures the OTHER
        # circle's rings near the junction and extends the corridor into it
        # (the L-shaped path that drags the car off mission).
        self.inner_radius_m = self.declare_parameter("inner_radius_m", 7.625).value
        self.outer_radius_m = self.declare_parameter("outer_radius_m", 10.625).value
        self.ring_tolerance_m = self.declare_parameter("ring_tolerance_m", 1.5).value
        # Shared junction cones fit both circles; keep them for the active
        # circle unless the other circle fits better by more than this margin.
        self.ring_share_margin_m = self.declare_parameter("ring_share_margin_m", 0.75).value
        # Radially displace the fed circle cones outward from the active
        # centre. Both rings shift equally, so the corridor width and the
        # planner's width gates are untouched, but the midline (and the car)
        # moves outward by the same amount — extra clearance to the inner
        # ring, which the pursuit's corner-cut otherwise shaves toward.
        self.circle_outward_bias_m = self.declare_parameter("circle_outward_bias_m", 0.5).value
        # Switch ENTRY->RIGHT this far before the junction so the circle
        # corridor is already in the planner ROI when the lane cones end.
        self.switch_lead_m = self.declare_parameter("switch_lead_m", 8.0).value
        self.junction_zone_x_m = self.declare_parameter("junction_zone_x_m", 3.5).value
        self.junction_zone_y_m = self.declare_parameter("junction_zone_y_m", 3.0).value
        # Angle slack: a lap counts as complete within this much of N*2pi so
        # the junction-zone test, not angle quantisation, decides the switch.
        self.lap_margin_rad = self.declare_parameter("lap_margin_rad", 0.35).value
        self.exit_stop_forward_m = self.declare_parameter(
            "exit_stop_forward_m", 12.0).value
        # Entry/exit lanes are fixed by the FS spec relative to the start pose
        # (= map origin), so synthesize their corridor instead of waiting for
        # the young SLAM map to confirm the sparse lane cones.
        self.lane_half_width_m = self.declare_parameter("lane_half_width_m", 1.55).value
        self.lane_pair_spacing_m = self.declare_parameter("lane_pair_spacing_m", 2.0).value

        self.phase = PHASE_ENTRY
        self.ego_x = 0.0
        self.ego_y = 0.0
        self.have_ego = False
        self.cum_angle = 0.0
        self.prev_angle = None

        self.right_centre = (self.entry_length_m, -self.circle_offset_m)
        self.left_centre = (self.entry_length_m, self.circle_offset_m)

        # The local planner's slam_map subscription is reliable+transient_local
        # (it expects a latched map); match it or nothing is delivered.
        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(
            ConeArrayWithCovariance, self.output_topic, latched)
        self.phase_pub = self.create_publisher(String, self.phase_topic, latched)

        self.create_subscription(Odometry, self.ego_odom_topic, self.on_odom, 10)
        self.create_subscription(
            ConeArrayWithCovariance, self.cone_map_topic, self.on_cone_map, 10)

        self.publish_phase()
        self.get_logger().info(
            f"skidpad director up: right_laps={self.right_laps} "
            f"left_laps={self.left_laps} entry={self.entry_length_m}m "
            f"offset={self.circle_offset_m}m")

    # ------------------------------------------------------------------
    def publish_phase(self):
        self.phase_pub.publish(String(data=self.phase))

    def set_phase(self, phase):
        if phase == self.phase:
            return
        self.get_logger().info(
            f"phase {self.phase} -> {phase} at ego=({self.ego_x:.1f},{self.ego_y:.1f}) "
            f"turned={abs(self.cum_angle) / TWO_PI:.2f} laps")
        self.phase = phase
        self.cum_angle = 0.0
        self.prev_angle = None
        self.publish_phase()

    def in_junction_zone(self):
        return (abs(self.ego_x - self.entry_length_m) < self.junction_zone_x_m and
                abs(self.ego_y) < self.junction_zone_y_m)

    def active_centre(self):
        return self.right_centre if self.phase == PHASE_RIGHT else self.left_centre

    def accumulate_angle(self):
        cx, cy = self.active_centre()
        dx, dy = self.ego_x - cx, self.ego_y - cy
        radius = math.hypot(dx, dy)
        if not (self.annulus_min_m - 1.0 <= radius <= self.annulus_max_m + 1.0):
            self.prev_angle = None
            return
        angle = math.atan2(dy, dx)
        if self.prev_angle is not None:
            delta = math.atan2(
                math.sin(angle - self.prev_angle), math.cos(angle - self.prev_angle))
            self.cum_angle += delta
        self.prev_angle = angle

    def on_odom(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.have_ego = True

        if self.phase == PHASE_ENTRY:
            if self.ego_x > self.entry_length_m - self.switch_lead_m:
                self.set_phase(PHASE_RIGHT)
        elif self.phase in (PHASE_RIGHT, PHASE_LEFT):
            self.accumulate_angle()
            laps = self.right_laps if self.phase == PHASE_RIGHT else self.left_laps
            target = laps * TWO_PI - self.lap_margin_rad
            if abs(self.cum_angle) >= target and self.in_junction_zone():
                self.set_phase(
                    PHASE_LEFT if self.phase == PHASE_RIGHT else PHASE_EXIT)
        elif self.phase == PHASE_EXIT:
            if self.ego_x > self.entry_length_m + self.exit_stop_forward_m:
                self.set_phase(PHASE_STOP)

    # ------------------------------------------------------------------
    @staticmethod
    def _position(cone):
        return cone.point.x, cone.point.y

    def _virtual_lane(self, out, x_from, x_to):
        """Synthesize a straight blue/yellow corridor along the lane spec."""
        x = x_from
        while x <= x_to:
            for y, sink in ((self.lane_half_width_m, out.blue_cones),
                            (-self.lane_half_width_m, out.yellow_cones)):
                cone = ConeWithCovariance()
                cone.point.x = float(x)
                cone.point.y = float(y)
                cone.covariance = [0.01, 0.0, 0.0, 0.01]
                sink.append(cone)
            x += self.lane_pair_spacing_m

    def _ring_fit(self, x, y, centre):
        radius = math.hypot(x - centre[0], y - centre[1])
        return min(abs(radius - self.inner_radius_m), abs(radius - self.outer_radius_m))

    def _circle_filter(self, msg, out, centre):
        other = self.left_centre if centre is self.right_centre else self.right_centre
        for source, sink in ((msg.blue_cones, out.blue_cones),
                             (msg.yellow_cones, out.yellow_cones)):
            for cone in source:
                x, y = self._position(cone)
                fit = self._ring_fit(x, y, centre)
                if fit > self.ring_tolerance_m:
                    continue
                if fit > self._ring_fit(x, y, other) + self.ring_share_margin_m:
                    continue  # clearly the other circle's cone
                shifted = ConeWithCovariance()
                radius = math.hypot(x - centre[0], y - centre[1])
                scale = (radius + self.circle_outward_bias_m) / radius
                shifted.point.x = centre[0] + (x - centre[0]) * scale
                shifted.point.y = centre[1] + (y - centre[1]) * scale
                shifted.covariance = list(cone.covariance)
                sink.append(shifted)

    def on_cone_map(self, msg):
        out = ConeArrayWithCovariance()
        out.header = msg.header

        if not self.have_ego:
            self.map_pub.publish(out)
            return

        if self.phase == PHASE_ENTRY:
            self._virtual_lane(out, 0.5, self.entry_length_m + 2.0)
        elif self.phase == PHASE_RIGHT:
            self._circle_filter(msg, out, self.right_centre)
        elif self.phase == PHASE_LEFT:
            self._circle_filter(msg, out, self.left_centre)
        elif self.phase == PHASE_EXIT:
            self._virtual_lane(
                out, self.entry_length_m - 2.0,
                self.entry_length_m + self.exit_stop_forward_m + 6.0)
        # PHASE_STOP publishes the empty array: no cones -> no local path ->
        # selected path invalid -> the controller brakes to a stop.

        self.map_pub.publish(out)


def main():
    rclpy.init()
    node = SkidpadDirector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
