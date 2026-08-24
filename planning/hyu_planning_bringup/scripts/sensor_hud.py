#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
sensor_hud — the SENSORS board of the RViz HUD (top-right).

Replaces the old GNSS-only overlay. One fixed-width line per input the stack
depends on, each with its rate and health, quiet when fine:

    CAMERA   ZED left/right camera_info rate (the images themselves are not
             subscribed: too heavy for a HUD; info publishes in lock-step)
    LIDAR    RS-16 cloud rate + point count
    IMU      SBG imu_data rate, |accel|, |gyro|, temperature
    GPS      SBG gps_pos: fix type, satellites, reported accuracy, epoch age
    HDT      dual-antenna heading: validity, drop-out ratio, heading, accuracy
    EKF      sbg_raw_ekf diagnostics: motion source (raw_ekf / zupt / no_hdt /
             coast / fault), position & yaw sigma, fix age, wheel fusion state
    WHEELS   ECU encoder feedback (via drive_udp_bridge): rate + 4 speeds
    ECU      packet flow both ways: /vehicle/cmd rate (stack -> bridge -> ECU)
             and wheel feedback rate (ECU -> bridge), plus the AS state that
             gates the bridge's autonomous-enable byte

Rates are measured on receive (wall clock, 3 s window). A line is DIM before
the first message, OK at/above ~60 % of its nominal rate, WARN when slow and
ERR when silent. Lines are clipped to a character budget and the panel
width is computed from it, so nothing wraps.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray
from hyu_msgs.msg import CanState, WheelSpeedsStamped
from sbg_driver.msg import SbgGpsHdt, SbgGpsPos, SbgImuData
from sensor_msgs.msg import CameraInfo, PointCloud2
from std_msgs.msg import ColorRGBA

try:
    from rviz_2d_overlay_msgs.msg import OverlayText
    HAVE_OVERLAY = True
except ImportError:
    HAVE_OVERLAY = False

OK = "rgb(110,235,130)"
WARN = "rgb(255,205,80)"
ERR = "rgb(255,92,92)"
DIM = "rgb(128,128,128)"
INFO = "rgb(120,205,255)"
TEXT = "rgb(232,232,232)"
B_OK, B_WARN, B_ERR, B_DIM = "●", "▲", "✕", "○"
NBSP = " "
LABEL_W = 8
MAX_CHARS = 58
FONT_PT = 12.0
PX_PER_CHAR = FONT_PT * 96.0 / 72.0 * 0.66

FIX_NAMES = {0: "NO_SOL", 1: "UNKNOWN", 2: "SINGLE", 3: "PSRDIFF", 4: "SBAS",
             5: "OMNISTAR", 6: "RTK_FLOAT", 7: "RTK_FIXED", 8: "PPP_FLOAT",
             9: "PPP_INT", 10: "FIXED"}
AS_NAMES = {0: "AS:OFF", 1: "AS:READY", 2: "AS:DRIVING", 3: "AS:EBS", 4: "AS:FINISHED"}


def esc(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def span(text, color):
    return f'<span style="color: {color};">{esc(text)}</span>'


def pad(label, width=LABEL_W):
    return (label + NBSP * width)[:width] if len(label) < width else label[:width]


def clip(text, budget):
    return text if len(text) <= budget else text[: max(0, budget - 1)] + "…"


class Stream:
    """Latest message, receive time and a 3 s rate window."""

    def __init__(self, nominal_hz):
        self.nominal = nominal_hz
        self.msg = None
        self.rx = None
        self._t = []

    def put(self, msg):
        self.msg = msg
        self.rx = time.monotonic()
        self._t.append(self.rx)
        if len(self._t) > 8 and self._t[0] < self.rx - 3.0:
            self._t = [t for t in self._t if t >= self.rx - 3.0]

    def age(self):
        return math.inf if self.rx is None else time.monotonic() - self.rx

    def hz(self):
        now = time.monotonic()
        return sum(1 for t in self._t if now - t <= 3.0) / 3.0

    def health(self, silent_after=1.5):
        """-> (bullet, colour) from presence/rate."""
        if self.rx is None:
            return B_DIM, DIM
        if self.age() > silent_after:
            return B_ERR, ERR
        if self.nominal > 0 and self.hz() < 0.6 * self.nominal:
            return B_WARN, WARN
        return B_OK, OK


class SensorHud(Node):

    def __init__(self):
        super().__init__("sensor_hud")
        p = self.declare_parameter
        self.hud_topic = p("hud_topic", "/sensors/hud").value
        cam_l = p("camera_left_info_topic", "/sensors/zed/left/color/rect/camera_info").value
        cam_r = p("camera_right_info_topic", "/sensors/zed/right/color/rect/camera_info").value
        lidar = p("lidar_topic", "/sensors/lidar/points").value
        imu = p("imu_topic", "/sbg/imu_data").value
        gps = p("gps_pos_topic", "/sbg/gps_pos").value
        hdt = p("gps_hdt_topic", "/sbg/gps_hdt").value
        ekf = p("ekf_status_topic", "/sbg_bridge/status").value
        wheels = p("wheel_speeds_topic", "/vehicle/wheel_speeds").value
        cmd = p("cmd_topic", "/vehicle/cmd").value
        can = p("can_state_topic", "/vehicle/as_state").value
        self.cam_l = Stream(p("camera_hz", 30.0).value)
        self.cam_r = Stream(self.cam_l.nominal)
        self.lidar = Stream(p("lidar_hz", 10.0).value)
        self.imu = Stream(p("imu_hz", 25.0).value)
        self.gps = Stream(p("gps_hz", 5.0).value)
        self.hdt = Stream(self.gps.nominal)
        self.ekf = Stream(5.0)
        self.wheels = Stream(p("wheel_hz", 100.0).value)
        self.cmd = Stream(p("cmd_hz", 20.0).value)
        self.can = Stream(20.0)
        self._hdt_window = []   # (rx, valid)

        be = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.create_subscription
        sub(CameraInfo, cam_l, self.cam_l.put, be)
        sub(CameraInfo, cam_r, self.cam_r.put, be)
        sub(PointCloud2, lidar, self.lidar.put, be)
        sub(SbgImuData, imu, self.imu.put, be)
        sub(SbgGpsPos, gps, self.gps.put, be)
        sub(SbgGpsHdt, hdt, self._on_hdt, be)
        sub(DiagnosticArray, ekf, self.ekf.put, be)
        sub(WheelSpeedsStamped, wheels, self.wheels.put, be)
        sub(AckermannDriveStamped, cmd, self.cmd.put, be)
        sub(CanState, can, self.can.put, be)

        if HAVE_OVERLAY:
            self.pub = self.create_publisher(OverlayText, self.hud_topic, latched)
        else:
            self.pub = None
            self.get_logger().warn("rviz_2d_overlay_msgs not found; sensor HUD disabled")
        self.create_timer(0.25, self._render)
        self.get_logger().info(f"sensor HUD on '{self.hud_topic}'")

    def _on_hdt(self, msg):
        self.hdt.put(msg)
        now = time.monotonic()
        self._hdt_window.append((now, (msg.status & 0x3F) == 0))
        self._hdt_window = [w for w in self._hdt_window if now - w[0] <= 5.0]

    # ------------------------------------------------------------------ lines

    def _line(self, label, bullet, color, body, body_color=None):
        body = clip(body, MAX_CHARS - LABEL_W - 2)
        return pad(label) + span(f"{bullet} ", color) + span(body, body_color or color)

    @staticmethod
    def _diag_kv(msg):
        out = {}
        for st in msg.status:
            for kv in st.values:
                out[kv.key] = kv.value
            out["_level"] = st.level
            out["_message"] = st.message
        return out

    def _render(self):
        if self.pub is None:
            return
        L = []
        L.append(span("SENSORS", TEXT) + NBSP * 2 + span(time.strftime("%H:%M:%S"), DIM))

        # CAMERA
        b, c = self.cam_l.health()
        if self.cam_l.rx is None and self.cam_r.rx is None:
            L.append(self._line("CAMERA", B_DIM, DIM, "none (no camera_info)"))
        else:
            sides = ("L" if self.cam_l.age() < 1.5 else "-") + "+" + ("R" if self.cam_r.age() < 1.5 else "-")
            body = f"{self.cam_l.hz():4.1f}Hz  {sides}"
            if self.cam_l.age() > 1.5:
                body = f"silent {self.cam_l.age():.1f}s"
            L.append(self._line("CAMERA", b, c, body))

        # LIDAR
        b, c = self.lidar.health()
        if self.lidar.rx is None:
            L.append(self._line("LIDAR", B_DIM, DIM, "none"))
        elif self.lidar.age() > 1.5:
            L.append(self._line("LIDAR", b, c, f"silent {self.lidar.age():.1f}s"))
        else:
            m = self.lidar.msg
            npts = m.width * m.height
            L.append(self._line("LIDAR", b, c, f"{self.lidar.hz():4.1f}Hz  {npts / 1000:.1f}k pts"))

        # IMU
        b, c = self.imu.health()
        if self.imu.rx is None:
            L.append(self._line("IMU", B_DIM, DIM, "none"))
        elif self.imu.age() > 1.5:
            L.append(self._line("IMU", b, c, f"silent {self.imu.age():.1f}s"))
        else:
            m = self.imu.msg
            a = math.sqrt(m.accel.x ** 2 + m.accel.y ** 2 + m.accel.z ** 2)
            w = math.degrees(math.sqrt(m.gyro.x ** 2 + m.gyro.y ** 2 + m.gyro.z ** 2))
            L.append(self._line("IMU", b, c,
                                f"{self.imu.hz():4.1f}Hz  |a|{a:5.2f}  |ω|{w:5.1f}°/s  {m.temp:.0f}°C"))

        # GPS
        b, c = self.gps.health()
        if self.gps.rx is None:
            L.append(self._line("GPS", B_DIM, DIM, "none"))
        elif self.gps.age() > 1.5:
            L.append(self._line("GPS", b, c, f"silent {self.gps.age():.1f}s"))
        else:
            m = self.gps.msg
            fix = FIX_NAMES.get(m.status.type, str(m.status.type))
            valid = m.status.status == 0 and m.status.type >= 2
            acc = max(m.position_accuracy.x, m.position_accuracy.y)
            col = c
            if not valid:
                col, b = ERR, B_ERR
            elif m.status.type < 6:
                col, b = WARN, B_WARN        # no RTK: metre-class
            elif m.status.type == 6:
                col = INFO                   # RTK float
            body = f"{self.gps.hz():3.1f}Hz  {fix}  sv{m.num_sv_used:<2d} acc {acc:.3f}m"
            L.append(self._line("GPS", b, col, body))

        # HDT — judged on the drop-out RATIO, not the latest epoch (3.9 % of
        # epochs are invalid even on a healthy unit); shows the last valid heading.
        if self.hdt.rx is None:
            L.append(self._line("HDT", B_DIM, DIM, "none"))
        elif self.hdt.age() > 1.5:
            L.append(self._line("HDT", B_ERR, ERR, f"silent {self.hdt.age():.1f}s"))
        else:
            m = self.hdt.msg
            valid = (m.status & 0x3F) == 0
            if valid:
                self._hdt_last_valid = m
            n = len(self._hdt_window)
            drop = 0.0 if n == 0 else 100.0 * sum(1 for _t, v in self._hdt_window if not v) / n
            lv = getattr(self, "_hdt_last_valid", None)
            hdg = f"hdg {lv.true_heading:5.1f}° acc {lv.true_heading_acc:.2f}°" if lv else "no valid heading yet"
            if drop >= 50.0 or lv is None:
                bb, col = B_ERR, ERR
            elif drop >= 20.0:
                bb, col = B_WARN, WARN
            else:
                bb, col = B_OK, OK
            L.append(self._line("HDT", bb, col, f"{hdg}  drop {drop:.0f}%" + ("" if valid else " (now invalid)")))

        # EKF (sbg_raw_ekf diagnostics)
        if self.ekf.rx is None:
            L.append(self._line("EKF", B_DIM, DIM, "none (sbg_raw_ekf?)"))
        elif self.ekf.age() > 2.0:
            L.append(self._line("EKF", B_ERR, ERR, f"status silent {self.ekf.age():.1f}s"))
        else:
            kv = self._diag_kv(self.ekf.msg)
            src = kv.get("motion_source", "?")
            lvl = kv.get("_level", 0)
            col = {0: OK, 1: WARN, 2: ERR}.get(int(lvl) if isinstance(lvl, int) else 0, DIM)
            bb = {OK: B_OK, WARN: B_WARN, ERR: B_ERR}.get(col, B_DIM)
            body = (f"{src} σp{kv.get('sigma_pos', '-')}m σψ{kv.get('sigma_yaw_deg', '-')}° "
                    f"fix{kv.get('pos_age', '-')}s")
            if "wheel" in kv:
                body += f" whl:{kv['wheel']}"
            if "wheel_scale" in kv and kv.get("wheel_scale") not in ("", "-"):
                body += f" ws{kv['wheel_scale']}"
            L.append(self._line("EKF", bb, col, body))

        # WHEELS
        b, c = self.wheels.health()
        if self.wheels.rx is None:
            L.append(self._line("WHEELS", B_DIM, DIM, "none (ECU feedback)"))
        elif self.wheels.age() > 1.5:
            L.append(self._line("WHEELS", b, c, f"silent {self.wheels.age():.1f}s — ECU feedback?"))
        else:
            s = self.wheels.msg.speeds
            L.append(self._line("WHEELS", b, c,
                                f"{self.wheels.hz():5.1f}Hz  FL{s.lf_speed:5.2f} FR{s.rf_speed:5.2f} "
                                f"RL{s.lb_speed:5.2f} RR{s.rb_speed:5.2f}"))

        # ECU packet flow
        as_state = self.can.msg.as_state if (self.can.msg and self.can.age() < 1.5) else None
        as_txt = AS_NAMES.get(as_state, "AS:?") if as_state is not None else "AS:none"
        tx = "—" if self.cmd.rx is None else (f"{self.cmd.hz():4.1f}Hz" if self.cmd.age() < 1.5 else "silent")
        rx = "—" if self.wheels.rx is None else (f"{self.wheels.hz():4.1f}Hz" if self.wheels.age() < 1.5 else "silent")
        if self.cmd.rx is None and self.wheels.rx is None:
            bb, col = B_DIM, DIM
        elif "silent" in (tx, rx):
            bb, col = B_ERR, ERR
        elif as_state == 2:
            bb, col = B_OK, OK
        else:
            bb, col = B_OK, INFO
        L.append(self._line("ECU", bb, col, f"cmd→ {tx}   fb← {rx}   {as_txt}"))

        width = int(24 + PX_PER_CHAR * (MAX_CHARS + 3))
        try:
            ov = OverlayText()
            ov.action = OverlayText.ADD
            ov.width = width
            ov.height = 22 + 19 * len(L)
            ov.horizontal_distance = 12
            ov.vertical_distance = 12
            ov.horizontal_alignment = OverlayText.RIGHT
            ov.vertical_alignment = OverlayText.TOP
            ov.bg_color = ColorRGBA(r=0.04, g=0.04, b=0.07, a=0.62)
            ov.fg_color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=1.0)
            ov.line_width = 2
            ov.text_size = FONT_PT
            ov.font = "DejaVu Sans Mono"
            ov.text = "\n".join(L)
            self.pub.publish(ov)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"overlay publish failed ({exc})")


def main():
    rclpy.init()
    node = SensorHud()
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
