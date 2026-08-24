#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
stack_hud — the PLANNING board of the RViz HUD (top-left).

One TextOverlay, one fixed-width line per stage, coloured by health. Quiet
when healthy, and says WHY when a stage blocks:

    PERCEPTION  status only: silent / slow / uncoloured (fusion) / no bboxes.
                Cone counts per colour are deliberately not shown.
    SLAM        lifecycle + map size.
    SELECTOR    LOCAL  GLOBAL  -> candidate   (the active source is bright,
                the other dim) plus a detail line: while on LOCAL, what keeps
                the car off the raceline (mapping lap, global not converged,
                handoff gate, raceline generator reason); on GLOBAL, the
                racing state; when nothing is selected, the selector's reason.
    MISSION / DSSI / TRACKING as before.

No banner any more and no CONTROL line: speed/steering live on the
control_plot.py sparklines at the bottom; sensors on sensor_hud.py
(top-right). Lines are clipped to a character budget and the panel width is
computed from it, so nothing ever wraps.

Needs ros-humble-rviz-2d-overlay-plugins in RViz; degrades to a warning
without the message package.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from hyu_msgs.msg import BoundingBoxes, CanState, ConeArrayWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, ColorRGBA, Float32, String

try:
    from rviz_2d_overlay_msgs.msg import OverlayText
    HAVE_OVERLAY = True
except ImportError:
    HAVE_OVERLAY = False

# Panel palette (QTextDocument rich-text colors; the overlay plugin skips its
# own fg wrap whenever the text carries "color:...;" styles).
OK = "rgb(110,235,130)"
WARN = "rgb(255,205,80)"
ERR = "rgb(255,92,92)"
DIM = "rgb(128,128,128)"
INFO = "rgb(120,205,255)"
TEXT = "rgb(232,232,232)"
ACCENT = "rgb(255,150,40)"

B_OK, B_WARN, B_ERR, B_DIM = "●", "▲", "✕", "○"

NBSP = " "
LABEL_W = 11          # "PERCEPTION " column
MAX_CHARS = 62        # visible characters per line, label included
FONT_PT = 12.0
# DejaVu Sans Mono advance ~0.6 em; 1 pt = 96/72 px. Generous so the QText
# layout never wraps even with the bullet/arrow glyphs.
PX_PER_CHAR = FONT_PT * 96.0 / 72.0 * 0.66

AS_NAMES = {0: "AS:OFF", 1: "AS:READY", 2: "AS:DRIVING", 3: "AS:EBS", 4: "AS:FINISHED"}
AMI_NAMES = {
    10: "NOT_SELECTED", 11: "ACCEL", 12: "SKIDPAD", 13: "AUTOCROSS",
    14: "TRACKDRIVE", 15: "AD_DEMO", 16: "INSPECTION", 17: "ADS_EBS",
    18: "DDT_A", 19: "DDT_B", 20: "JOYSTICK", 21: "MANUAL",
}


def esc(text):
    return (
        str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    )


def span(text, color):
    return f'<span style="color: {color};">{esc(text)}</span>'


def pad(label, width=LABEL_W):
    return (label + NBSP * width)[:width] if len(label) < width else label[:width]


def clip(text, budget):
    return text if len(text) <= budget else text[: max(0, budget - 1)] + "…"


class TopicAge:
    """Latest message + monotonic receive time for staleness checks."""

    def __init__(self):
        self.msg = None
        self.rx = None
        self._times = []

    def put(self, msg):
        self.msg = msg
        self.rx = time.monotonic()
        self._times.append(self.rx)
        cut = self.rx - 3.0
        if len(self._times) > 4 and self._times[0] < cut:
            self._times = [t for t in self._times if t >= cut]

    def age(self):
        return math.inf if self.rx is None else time.monotonic() - self.rx

    def fresh(self, timeout):
        return self.age() <= timeout

    def hz(self):
        now = time.monotonic()
        ts = [t for t in self._times if now - t <= 3.0]
        return len(ts) / 3.0


class StackHud(Node):

    def __init__(self):
        super().__init__("stack_hud")
        p = self.declare_parameter
        self.hud_topic = p("hud_topic", "/planning/stack_hud").value
        cones_topic = p("cones_topic", "/perception/cones").value
        bbox_topic = p("bbox_topic", "/perception/bounding_boxes").value
        cone_map_topic = p("cone_map_topic", "/localization/cone_map").value
        slam_status_topic = p("slam_status_topic", "/localization/status").value
        ego_odom_topic = p("ego_odom_topic", "/localization/ego_odom").value
        planning_debug_topic = p("planning_debug_topic", "/planning/debug").value
        selector_debug_topic = p(
            "selector_debug_topic", "/planning/hyu_path_selector/debug").value
        local_valid_topic = p(
            "local_path_valid_topic", "/planning/local_path_valid").value
        local_reason_topic = p(
            "local_path_reason_topic", "/planning/local_path_reason").value
        global_valid_topic = p(
            "global_path_valid_topic", "/planning/global_path_valid").value
        global_reason_topic = p(
            "global_path_reason_topic", "/planning/global_path_reason").value
        selected_valid_topic = p(
            "selected_path_valid_topic", "/planning/selected_path_valid").value
        can_state_topic = p("can_state_topic", "/vehicle/as_state").value
        dssi_topic = p("dssi_topic", "/vehicle/dssi").value
        cte_topic = p("cte_topic", "/planning/cte").value
        cte_rmse_topic = p("cte_rmse_topic", "/planning/cte_rmse").value
        self.target_laps = p("target_lap_count", 4).value
        # Kept for callers that still pass it; a banner is not published.
        p("banner_topic", "")

        self.cones = TopicAge()
        self.bbox = TopicAge()
        self.cone_map = TopicAge()
        self.slam_status = TopicAge()
        self.ego = TopicAge()
        self.debug = TopicAge()
        self.sel_debug = TopicAge()
        self.local_valid = TopicAge()
        self.local_reason = TopicAge()
        self.global_valid = TopicAge()
        self.global_reason = TopicAge()
        self.selected_valid = TopicAge()
        self.can = TopicAge()
        self.dssi = TopicAge()
        self.cte = TopicAge()
        self.cte_rmse = TopicAge()
        # uncoloured-ratio window for the perception status
        self._cone_mix = []   # (rx_time, coloured, unknown)

        be = QoSProfile(
            depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        reliable = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        sub = self.create_subscription
        sub(ConeArrayWithCovariance, cones_topic, self._on_cones, be)
        sub(BoundingBoxes, bbox_topic, self.bbox.put, be)
        sub(ConeArrayWithCovariance, cone_map_topic, self.cone_map.put, latched)
        sub(String, slam_status_topic, self.slam_status.put, latched)
        sub(Odometry, ego_odom_topic, self.ego.put, be)
        sub(String, planning_debug_topic, self.debug.put, reliable)
        sub(String, selector_debug_topic, self.sel_debug.put, reliable)
        sub(Bool, local_valid_topic, self.local_valid.put, reliable)
        sub(String, local_reason_topic, self.local_reason.put, reliable)
        sub(Bool, global_valid_topic, self.global_valid.put, reliable)
        sub(String, global_reason_topic, self.global_reason.put, latched)
        sub(Bool, selected_valid_topic, self.selected_valid.put, reliable)
        # NOTE: the sim publishes /vehicle/as_state only while it has
        # subscribers — this subscription is what makes it flow.
        sub(CanState, can_state_topic, self.can.put, be)
        sub(String, dssi_topic, self.dssi.put, reliable)
        sub(Float32, cte_topic, self.cte.put, reliable)
        sub(Float32, cte_rmse_topic, self.cte_rmse.put, reliable)

        if HAVE_OVERLAY:
            self.hud_pub = self.create_publisher(OverlayText, self.hud_topic, latched)
        else:
            self.hud_pub = None
            self.get_logger().warn(
                "rviz_2d_overlay_msgs not found; install "
                "ros-humble-rviz-2d-overlay-plugins for the stack HUD.")

        self.create_timer(0.2, self._render)
        self.get_logger().info(f"stack HUD (planning board) on '{self.hud_topic}'")

    # ------------------------------------------------------------------ input

    # Colour is only expected where the camera can see the cone: inside its
    # FOV and within the stereo range. Far cones and off-track clutter are
    # uncoloured by design, so they must not count against the fusion.
    COLOUR_RANGE_M = 8.0
    COLOUR_HALF_FOV_DEG = 50.0

    def _colourable(self, c):
        r = math.hypot(c.point.x, c.point.y)
        if r < 1.5 or r > self.COLOUR_RANGE_M:
            return False
        return abs(math.degrees(math.atan2(c.point.y, c.point.x))) <= self.COLOUR_HALF_FOV_DEG

    def _on_cones(self, msg):
        self.cones.put(msg)
        coloured = sum(1 for f in (msg.blue_cones, msg.yellow_cones, msg.orange_cones,
                                    msg.big_orange_cones) for c in f if self._colourable(c))
        unknown = sum(1 for c in msg.unknown_color_cones if self._colourable(c))
        now = time.monotonic()
        self._cone_mix.append((now, coloured, unknown))
        self._cone_mix = [m for m in self._cone_mix if now - m[0] <= 3.0]

    @staticmethod
    def _kv(topic_age):
        """Parse a 'k=v k=v ...' debug string into a dict."""
        if topic_age.msg is None:
            return {}
        return dict(
            item.split("=", 1)
            for item in topic_age.msg.data.split()
            if "=" in item)

    @staticmethod
    def _cone_count(msg):
        return (
            len(msg.blue_cones) + len(msg.yellow_cones) + len(msg.orange_cones)
            + len(msg.big_orange_cones) + len(msg.unknown_color_cones))

    # ----------------------------------------------------------------- render

    def _render(self):
        if self.hud_pub is None:
            return
        debug = self._kv(self.debug)
        sel = self._kv(self.sel_debug) if self.sel_debug.fresh(2.0) else {}
        lines = self._board_lines(debug, sel)
        html = "\n".join(h for h, _n in lines)
        width = int(24 + PX_PER_CHAR * (MAX_CHARS + 3))
        self._publish(
            html, width=width, height=22 + 19 * len(lines),
            h_align=OverlayText.LEFT, v_align=OverlayText.TOP,
            h_dist=12, v_dist=12, size=FONT_PT)

    # --- line builders: each returns (html, visible_len) and never exceeds MAX_CHARS

    def _line(self, label, bullet, color, body, body_color=None):
        body_color = body_color or color
        budget = MAX_CHARS - LABEL_W - 2
        body = clip(body, budget)
        html = pad(label) + span(f"{bullet} ", color) + span(body, body_color)
        return html, LABEL_W + 2 + len(body)

    def _detail(self, text, color=DIM):
        """Indented continuation line under a stage."""
        budget = MAX_CHARS - LABEL_W - 2
        text = clip(text, budget)
        return pad("") + NBSP * 2 + span(text, color), LABEL_W + 2 + len(text)

    def _board_lines(self, debug, sel):
        lines = []
        head = span("PLANNING", TEXT) + NBSP * 2 + span(time.strftime("%H:%M:%S"), DIM)
        lines.append((head, 18))

        # ---------------------------------------------------------- PERCEPTION
        lines.append(self._perception_line())

        # ---------------------------------------------------------------- SLAM
        slam = self.slam_status.msg.data if self.slam_status.msg else ""
        map_n = self._cone_count(self.cone_map.msg) if self.cone_map.msg else 0
        if slam and self.ego.rx is not None and not self.ego.fresh(1.5):
            style = (B_ERR, ERR, f"ego odom silent {self.ego.age():.0f}s — slam down?")
        else:
            style = {
                "mapping": (B_OK, WARN, f"MAPPING  map {map_n}"),
                "mapping_converged": (B_OK, INFO, f"CONVERGED  map {map_n}"),
                "localization": (B_OK, OK, f"LOCALIZATION  map {map_n}"),
            }.get(slam, (B_DIM, DIM, "waiting for graph_slam"))
        lines.append(self._line("SLAM", *style))

        # ------------------------------------------------------------ SELECTOR
        lines.extend(self._selector_lines(debug, sel, slam))

        # ------------------------------------------------------------- MISSION
        as_state = self.can.msg.as_state if self.can.msg else None
        driving = as_state == 2
        self.target_laps = debug.get("target_lap_count", self.target_laps)
        lap = self._display_lap(debug)
        if self.can.msg is None:
            mis = (B_DIM, DIM, "waiting for /vehicle/as_state")
        else:
            as_name = AS_NAMES.get(as_state, f"AS:{as_state}").replace("AS:", "")
            ami = AMI_NAMES.get(self.can.msg.ami_state, str(self.can.msg.ami_state))
            body = f"{as_name} {ami} lap {lap}/{self.target_laps}"
            laps = self._lap_times(debug)
            if laps:
                body += f"  {laps}"
            if driving:
                mis = (B_OK, OK, body)
            elif as_state == 3:
                mis = (B_ERR, ERR, body)
            elif as_state == 4:
                mis = (B_OK, INFO, body)
            else:
                mis = (B_WARN, WARN, f"{body} — arm mission")
        lines.append(self._line("MISSION", *mis))

        # ---------------------------------------------------------------- DSSI
        blink_on = (time.monotonic() % 1.0) < 0.5
        if self.dssi.msg is None or not self.dssi.fresh(1.5):
            dssi_line = (B_DIM, DIM, "waiting for /vehicle/dssi")
        else:
            dssi_line = {
                "off": (B_DIM, DIM, "OFF"),
                "yellow_flashing": (
                    B_OK if blink_on else B_DIM, WARN, "YELLOW FLASHING — system check"),
                "yellow_continuous": (B_OK, WARN, "YELLOW — RTAD"),
                "blue_continuous": (B_OK, INFO, "BLUE — driverless"),
            }.get(self.dssi.msg.data, (B_ERR, ERR, f"unknown '{self.dssi.msg.data}'"))
        lines.append(self._line("DSSI", *dssi_line))

        # ------------------------------------------------------------ TRACKING
        if self.cte.msg is not None and self.cte.fresh(1.5):
            d_val = self.cte.msg.data
            rmse = self.cte_rmse.msg.data if self.cte_rmse.msg else float("nan")
            try:
                s_val = f"{float(debug.get('current_s')):.1f}m"
            except (TypeError, ValueError):
                s_val = "?"
            color = OK if abs(d_val) < 0.35 else (WARN if abs(d_val) < 0.8 else ERR)
            body = f"d{d_val:+5.2f}m  rmse {rmse:.2f}  s {s_val}"
            body = clip(body, MAX_CHARS - LABEL_W - 2)
            html = pad("TRACKING") + span(f"{B_OK} ", color) + span(body, TEXT)
            lines.append((html, LABEL_W + 2 + len(body)))
        else:
            lines.append(self._line("TRACKING", B_DIM, DIM, "no frenet data"))
        return lines

    # ---------------------------------------------------------- perception

    def _perception_line(self):
        """Status only. Healthy -> 'OK 9.8Hz'. Otherwise the first thing a
        driver would want to know: silent / slow / uncoloured / no bboxes."""
        if self.cones.rx is None:
            return self._line("PERCEPTION", B_DIM, DIM, "waiting for /perception/cones")
        if not self.cones.fresh(1.5):
            return self._line("PERCEPTION", B_ERR, ERR,
                              f"SILENT {self.cones.age():.1f}s — perception down?")
        hz = self.cones.hz()
        issues = []
        level = OK
        if hz < 5.0:
            issues.append(f"slow {hz:.1f}Hz")
            level = WARN
        coloured = sum(m[1] for m in self._cone_mix)
        unknown = sum(m[2] for m in self._cone_mix)
        total = coloured + unknown
        # Symptom first (mostly uncoloured cones), then the likely cause. A
        # healthy colour mix stays quiet whatever the bbox stream does.
        if total >= 20 and unknown / total > 0.75:
            if self.bbox.rx is None or not self.bbox.fresh(1.0):
                cause = "no bboxes — YOLO down?"
            elif self.bbox.hz() < 5.0:
                cause = f"bbox {self.bbox.hz():.1f}Hz stale — fusion lag"
            else:
                cause = "camera/fusion?"
            issues.append(f"{100 * unknown / total:.0f}% near cones uncoloured: {cause}")
            level = WARN if level == OK else level
        if total == 0:
            issues.append("no cones in view")
            level = WARN if level == OK else level
        if not issues:
            return self._line("PERCEPTION", B_OK, OK, f"OK  {hz:.1f}Hz")
        bullet = B_WARN if level == WARN else B_ERR
        return self._line("PERCEPTION", bullet, level, f"{hz:.1f}Hz  " + "; ".join(issues))

    # ------------------------------------------------------------- selector

    def _selector_lines(self, debug, sel, slam):
        path_source = debug.get("path_source", "")
        state = debug.get("state", "")
        s_valid = bool(self.selected_valid.msg and self.selected_valid.msg.data
                       and self.selected_valid.fresh(1.5))
        l_valid = bool(self.local_valid.msg and self.local_valid.msg.data
                       and self.local_valid.fresh(1.5))
        g_valid = bool(self.global_valid.msg and self.global_valid.msg.data
                       and self.global_valid.fresh(1.5))
        l_reason = self.local_reason.msg.data if self.local_reason.msg else ""
        g_reason = self.global_reason.msg.data if self.global_reason.msg else ""
        failure = sel.get("selection_failure", "")
        candidate = sel.get("selected_candidate", "")
        on_global = path_source.startswith("GLOBAL")
        on_local = path_source in ("", "LOCAL") or path_source.startswith("LOCAL")

        # --- header line: LOCAL / GLOBAL coloured by who is driving and whether valid
        def word(name, active, valid, rx_seen):
            if active:
                return span(name, OK if valid else ERR)
            if not rx_seen:
                return span(name, DIM)
            return span(name, DIM if valid else "rgb(170,120,60)")

        local_w = word("LOCAL", on_local and not on_global, l_valid, self.local_valid.rx is not None)
        global_w = word("GLOBAL", on_global, g_valid, self.global_valid.rx is not None)
        if self.selected_valid.rx is None:
            bullet, bcol, tail = B_DIM, DIM, "waiting for hyu_path_selector"
        elif not self.selected_valid.fresh(1.5):
            bullet, bcol, tail = B_ERR, ERR, f"heartbeat lost {self.selected_valid.age():.0f}s"
        elif s_valid:
            bullet, bcol = B_OK, OK
            tail = f"→ {candidate or path_source or '?'}"
        elif failure == "stop_requested":
            bullet, bcol, tail = B_DIM, DIM, "stop requested"
        else:
            bullet, bcol = B_ERR, ERR
            detail = failure or "no path"
            cont = sel.get("continuity_failure", "none")
            if detail in ("global_unavailable", "handoff_not_ready") and cont != "none":
                detail += f" ({cont})"
            tail = detail
        tail_budget = MAX_CHARS - LABEL_W - 2 - len("LOCAL  GLOBAL  ")
        tail = clip(tail, max(8, tail_budget))
        html = (pad("SELECTOR") + span(f"{bullet} ", bcol) + local_w + NBSP * 2 + global_w
                + NBSP * 2 + span(tail, bcol if bullet != B_OK else TEXT))
        lines = [(html, LABEL_W + 2 + len("LOCAL  GLOBAL  ") + len(tail))]

        # --- detail line: why this source, what blocks the other
        if self.selected_valid.rx is None:
            return lines
        if state == "STOP":
            lines.append(self._detail("stop state", DIM))
        elif on_global:
            fin = "final lap" if path_source == "GLOBAL_FINAL_STOP" else "racing raceline"
            lines.append(self._detail(fin + ("" if l_valid else " · local idle"), INFO))
        else:
            # On LOCAL: name the thing that keeps the car off the raceline.
            if slam in ("", "mapping"):
                why = "mapping lap 1 — global needs a converged map"
                col = WARN
            elif slam == "mapping_converged":
                why = "map converged — waiting for localization (gate/seam)"
                col = WARN
            elif slam == "localization":
                if not g_valid:
                    why = f"global invalid: {g_reason or 'no raceline yet'}"
                    col = ERR if self.global_valid.rx is not None else WARN
                else:
                    why = f"localized · {self._handoff_gate(debug, sel)}"
                    col = INFO
            else:
                why = f"slam {slam}"
                col = DIM
            if not l_valid and on_local and self.local_valid.rx is not None:
                why = f"LOCAL invalid: {l_reason or 'no path'} · " + why
                col = ERR
            lines.append(self._detail(why, col))
        return lines

    # ---------------------------------------------------------------- misc

    @staticmethod
    def _lap_times(debug):
        try:
            last = float(debug.get("lap_time_last", "0"))
            best = float(debug.get("lap_time_best", "0"))
        except ValueError:
            return ""
        if last <= 0.0:
            return ""
        return f"last {last:.1f} best {best:.1f}"

    def _display_lap(self, debug):
        """Current lap (1..target) from the state machine's COMPLETED count."""
        completed = debug.get("lap_count", "?")
        try:
            return min(int(completed) + 1, int(self.target_laps))
        except (TypeError, ValueError):
            return completed

    def _handoff_gate(self, debug, sel):
        """Name the LOCAL->GLOBAL entry blocker (state machine's authoritative
        global_entry_reason, with the selector's continuity detail)."""
        reason = debug.get("global_entry_reason", "")
        if reason in ("", "ready"):
            return "switching to GLOBAL"
        if reason == "handoff_not_ready":
            cont = sel.get("continuity_failure", "")
            if cont and cont != "none":
                return f"handoff not ready: {cont}"
        return "gate: " + reason.replace("_", " ")

    def _publish(self, text, width, height, h_align, v_align, h_dist, v_dist, size):
        try:
            ov = OverlayText()
            ov.action = OverlayText.ADD
            ov.width = width
            ov.height = height
            ov.horizontal_distance = h_dist
            ov.vertical_distance = v_dist
            ov.horizontal_alignment = h_align
            ov.vertical_alignment = v_align
            ov.bg_color = ColorRGBA(r=0.04, g=0.04, b=0.07, a=0.62)
            ov.fg_color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=1.0)
            ov.line_width = 2
            ov.text_size = size
            ov.font = "DejaVu Sans Mono"
            ov.text = text
            self.hud_pub.publish(ov)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"overlay publish failed ({exc})")


def main():
    rclpy.init()
    node = StackHud()
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
