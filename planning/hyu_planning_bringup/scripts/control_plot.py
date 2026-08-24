#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
control_plot — real-time CONTROL sparklines at the bottom of the RViz view.

RViz has no stock time-series plot overlay, so this draws three scrolling
sparklines with Unicode block glyphs (▁▂▃▄▅▆▇█) in a TextOverlay, one
character per sample, newest on the right:

    v act   measured speed (ins_odom twist, else ego_odom)      green
    v cmd   commanded speed (/vehicle/cmd drive.speed)          yellow
    steer   commanded steering angle (/vehicle/cmd), signed,     cyan
            centred (▄ = 0), full-scale ±steer_scale_deg

plus the live numbers on the left and a BRAKE tag when the controller is
braking (speed 0 with a negative acceleration). Both speed rows share one
scale (0..speed_scale_mps, auto-grown to the observed maximum) so the gap
between act and cmd is readable. Window = columns / sample_hz seconds.
"""

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from hyu_msgs.msg import CarState
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA

try:
    from rviz_2d_overlay_msgs.msg import OverlayText
    HAVE_OVERLAY = True
except ImportError:
    HAVE_OVERLAY = False

GREEN = "rgb(110,235,130)"
YELLOW = "rgb(255,205,80)"
CYAN = "rgb(120,205,255)"
DIM = "rgb(128,128,128)"
TEXT = "rgb(232,232,232)"
ERR = "rgb(255,92,92)"
BLOCKS = "▁▂▃▄▅▆▇█"
NBSP = " "
LABEL_W = 8
FONT_PT = 10.0
PX_PER_CHAR = FONT_PT * 96.0 / 72.0 * 0.66


def esc(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def span(text, color):
    return f'<span style="color: {color};">{esc(text)}</span>'


def pad(label, width=LABEL_W):
    return (label + NBSP * width)[:width] if len(label) < width else label[:width]


def spark(values, lo, hi, columns):
    """Sparkline string: NaN -> space, else a block glyph on [lo, hi]."""
    out = []
    vals = list(values)[-columns:]
    out.append(NBSP * (columns - len(vals)))
    for v in vals:
        if v is None or not math.isfinite(v):
            out.append(NBSP)
            continue
        f = 0.0 if hi <= lo else (v - lo) / (hi - lo)
        f = min(1.0, max(0.0, f))
        out.append(BLOCKS[min(7, int(round(f * 7)))])
    return "".join(out)


class ControlPlot(Node):

    def __init__(self):
        super().__init__("control_plot")
        p = self.declare_parameter
        self.topic = p("plot_topic", "/planning/control_plot").value
        cmd_topic = p("cmd_topic", "/vehicle/cmd").value
        ins_topic = p("ins_odom_topic", "/localization/ins_odom").value
        ego_topic = p("ego_odom_topic", "/localization/ego_odom").value
        self.columns = int(p("columns", 80).value)
        self.sample_hz = float(p("sample_hz", 5.0).value)
        self.speed_scale = float(p("speed_scale_mps", 6.0).value)
        self.steer_scale = math.radians(float(p("steer_scale_deg", 20.0).value))

        self.cmd = None
        self.cmd_rx = None
        self.v_ins = None
        self.v_ins_rx = None
        self.v_ego = None
        self.v_ego_rx = None
        n = self.columns
        self.h_act = deque(maxlen=n)
        self.h_cmd = deque(maxlen=n)
        self.h_steer = deque(maxlen=n)
        self.v_max_seen = 0.0

        be = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(AckermannDriveStamped, cmd_topic, self._on_cmd, be)
        self.create_subscription(CarState, ins_topic, self._on_ins, be)
        self.create_subscription(Odometry, ego_topic, self._on_ego, be)
        if HAVE_OVERLAY:
            self.pub = self.create_publisher(OverlayText, self.topic, latched)
        else:
            self.pub = None
            self.get_logger().warn("rviz_2d_overlay_msgs not found; control plot disabled")
        self.create_timer(1.0 / self.sample_hz, self._sample)
        self.create_timer(0.2, self._render)
        self.get_logger().info(
            f"control plot on '{self.topic}': {self.columns} cols @ {self.sample_hz:.0f} Hz "
            f"= {self.columns / self.sample_hz:.0f} s window")

    def _on_cmd(self, msg):
        self.cmd = msg
        self.cmd_rx = time.monotonic()

    def _on_ins(self, msg):
        self.v_ins = msg.twist.twist.linear.x
        self.v_ins_rx = time.monotonic()

    def _on_ego(self, msg):
        self.v_ego = msg.twist.twist.linear.x
        self.v_ego_rx = time.monotonic()

    def _actual(self):
        now = time.monotonic()
        if self.v_ins_rx is not None and now - self.v_ins_rx < 1.0:
            return self.v_ins
        if self.v_ego_rx is not None and now - self.v_ego_rx < 1.0:
            return self.v_ego
        return float("nan")

    def _sample(self):
        now = time.monotonic()
        v = self._actual()
        if self.cmd is not None and now - self.cmd_rx < 1.0:
            vc = self.cmd.drive.speed
            st = self.cmd.drive.steering_angle
        else:
            vc = st = float("nan")
        self.h_act.append(v)
        self.h_cmd.append(vc)
        self.h_steer.append(st)
        for x in (v, vc):
            if math.isfinite(x):
                self.v_max_seen = max(self.v_max_seen, abs(x))

    def _render(self):
        if self.pub is None:
            return
        now = time.monotonic()
        v = self._actual()
        have_cmd = self.cmd is not None and now - self.cmd_rx < 1.0
        vc = self.cmd.drive.speed if have_cmd else float("nan")
        st = self.cmd.drive.steering_angle if have_cmd else float("nan")
        braking = have_cmd and self.cmd.drive.speed == 0.0 and self.cmd.drive.acceleration < -1.0
        vs = max(self.speed_scale, math.ceil(self.v_max_seen))
        win = self.columns / self.sample_hz

        def num(x, fmt):
            return fmt % x if math.isfinite(x) else "  -- "

        head = (span("CONTROL", TEXT) + NBSP * 2
                + span(f"act {num(v, '%4.2f')}", GREEN) + NBSP * 2
                + span(f"cmd {num(vc, '%4.2f')} m/s", YELLOW) + NBSP * 2
                + span(f"steer {num(math.degrees(st) if math.isfinite(st) else st, '%+5.1f')}°", CYAN)
                + NBSP * 2 + span(f"scale 0–{vs:.0f} m/s · ±{math.degrees(self.steer_scale):.0f}° · {win:.0f}s", DIM)
                + (NBSP * 2 + span("BRAKE", ERR) if braking else ""))
        if self.cmd is None and not math.isfinite(v):
            head += NBSP * 2 + span("waiting for /vehicle/cmd, odom", DIM)
        rows = [
            head,
            pad("v act") + span(spark(self.h_act, 0.0, vs, self.columns), GREEN),
            pad("v cmd") + span(spark(self.h_cmd, 0.0, vs, self.columns), YELLOW),
            pad("steer") + span(spark(self.h_steer, -self.steer_scale, self.steer_scale, self.columns), CYAN),
        ]
        width = int(24 + PX_PER_CHAR * (LABEL_W + self.columns + 2))
        try:
            ov = OverlayText()
            ov.action = OverlayText.ADD
            ov.width = width
            ov.height = 20 + 17 * len(rows)
            ov.horizontal_distance = 0
            ov.vertical_distance = 10
            ov.horizontal_alignment = OverlayText.CENTER
            ov.vertical_alignment = OverlayText.BOTTOM
            ov.bg_color = ColorRGBA(r=0.04, g=0.04, b=0.07, a=0.62)
            ov.fg_color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=1.0)
            ov.line_width = 2
            ov.text_size = FONT_PT
            ov.font = "DejaVu Sans Mono"
            ov.text = "\n".join(rows)
            self.pub.publish(ov)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"overlay publish failed ({exc})")


def main():
    rclpy.init()
    node = ControlPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
