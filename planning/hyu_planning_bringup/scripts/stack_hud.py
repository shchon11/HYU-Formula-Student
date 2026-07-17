#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
stack_hud — one-glance RViz debugging HUD for the whole FSK pipeline.

Aggregates every stage (perception -> SLAM -> global/local planners ->
selector -> controller -> mission) into two TextOverlay panels:

- /planning/stack_hud        : left board, one colored line per stage; a
                               blocking stage shows its failure REASON inline
                               (planner reasons come from the new
                               /planning/{local,global}_path_reason topics,
                               selector reasons from its debug string).
- /planning/stack_hud_banner : big top-center banner answering "what is the
                               car doing right now / why is it not moving".

Health coloring is load-bearing-aware: the local planner being invalid while
the car races the GLOBAL path is dim, not red. Freshness uses
time.monotonic() at receive so sim/wall clock mismatches cannot fake
staleness. Needs ros-humble-rviz-2d-overlay-plugins in RViz; degrades to a
warning without the message package (same pattern as ate_monitor).
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import CanState, ConeArrayWithCovariance
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
DIM = "rgb(148,148,148)"
INFO = "rgb(120,205,255)"
TEXT = "rgb(232,232,232)"
ACCENT = "rgb(255,150,40)"

B_OK, B_WARN, B_ERR, B_DIM = "●", "▲", "✕", "○"

NBSP = " "

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


def pad(label, width=11):
    return (label + NBSP * width)[:width] if len(label) < width else label


class TopicAge:
    """Latest message + monotonic receive time for staleness checks."""

    def __init__(self):
        self.msg = None
        self.rx = None

    def put(self, msg):
        self.msg = msg
        self.rx = time.monotonic()

    def age(self):
        return math.inf if self.rx is None else time.monotonic() - self.rx

    def fresh(self, timeout):
        return self.age() <= timeout


class StackHud(Node):

    def __init__(self):
        super().__init__("stack_hud")
        p = self.declare_parameter
        self.hud_topic = p("hud_topic", "/planning/stack_hud").value
        self.banner_topic = p(
            "banner_topic", "/planning/stack_hud_banner").value

        cones_topic = p("cones_topic", "/cones").value
        cone_map_topic = p("cone_map_topic", "/localization/cone_map").value
        slam_status_topic = p("slam_status_topic", "/graph_slam/status").value
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
        cmd_topic = p("cmd_topic", "/cmd").value
        can_state_topic = p("can_state_topic", "/ros_can/state").value
        cte_topic = p("cte_topic", "/planning/cte").value
        cte_rmse_topic = p("cte_rmse_topic", "/planning/cte_rmse").value
        self.target_laps = p("target_lap_count", 4).value

        # Receive-side state. Every entry is (latest msg, monotonic rx time).
        self.cones = TopicAge()
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
        self.cmd = TopicAge()
        self.can = TopicAge()
        self.cte = TopicAge()
        self.cte_rmse = TopicAge()
        self.cones_rx_window = []  # monotonic stamps for rate estimation

        # Best-effort matches both reliable and best-effort publishers.
        be = QoSProfile(
            depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        reliable = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        sub = self.create_subscription
        sub(ConeArrayWithCovariance, cones_topic, self._on_cones, be)
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
        sub(AckermannDriveStamped, cmd_topic, self.cmd.put, be)
        # NOTE: the sim publishes /ros_can/state only while it has
        # subscribers — this subscription is what makes it flow.
        sub(CanState, can_state_topic, self.can.put, be)
        sub(Float32, cte_topic, self.cte.put, reliable)
        sub(Float32, cte_rmse_topic, self.cte_rmse.put, reliable)

        if HAVE_OVERLAY:
            self.hud_pub = self.create_publisher(
                OverlayText, self.hud_topic, latched)
            self.banner_pub = self.create_publisher(
                OverlayText, self.banner_topic, latched)
        else:
            self.hud_pub = None
            self.banner_pub = None
            self.get_logger().warn(
                "rviz_2d_overlay_msgs not found; install "
                "ros-humble-rviz-2d-overlay-plugins for the stack HUD.")

        self.create_timer(0.2, self._render)  # 5 Hz
        self.get_logger().info(
            f"stack HUD: board on '{self.hud_topic}', banner on "
            f"'{self.banner_topic}'")

    # ------------------------------------------------------------------ input

    def _on_cones(self, msg):
        self.cones.put(msg)
        now = time.monotonic()
        self.cones_rx_window.append(now)
        self.cones_rx_window = [
            t for t in self.cones_rx_window if now - t <= 3.0]

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
        # Selector claims (failure reasons) go stale with the node; drop them
        # so a dead selector reads as "silent", not its last excuse.
        sel = self._kv(self.sel_debug) if self.sel_debug.fresh(2.0) else {}
        lines, statuses = self._board_lines(debug, sel)
        # Board sits BELOW the top-centre banner (v_dist 8, height 44 -> y8..52).
        # On a narrow viewport the centred banner slides left over this left
        # board; clearing it vertically keeps the banner readable. GNSS HUD
        # (sbg_odometry_bridge.py) is pinned just under this board's bottom, so
        # keep the two in sync when moving either.
        self._publish(
            self.hud_pub, "\n".join(lines),
            width=480, height=24 + 19 * len(lines),
            h_align=OverlayText.LEFT, v_align=OverlayText.TOP,
            h_dist=12, v_dist=56, size=12.0)

        text, color = self._banner(debug, sel, statuses)
        self._publish(
            self.banner_pub, span(text, color),
            width=760, height=44,
            h_align=OverlayText.CENTER, v_align=OverlayText.TOP,
            h_dist=0, v_dist=8, size=19.0)

    def _board_lines(self, debug, sel):
        """Build the per-stage board. Returns (html lines, status dict)."""
        st = {}
        lines = [span("FSK STACK", TEXT) + NBSP * 2 + span(
            time.strftime("%H:%M:%S"), DIM)]

        # PERCEPTION — live /cones flow.
        if self.cones.rx is None:
            perc = (B_DIM, DIM, "waiting for /cones")
        elif not self.cones.fresh(1.5):
            perc = (B_ERR, ERR, f"SILENT {self.cones.age():.1f}s — check perception")
        else:
            m = self.cones.msg
            rate = len(self.cones_rx_window) / 3.0
            perc = (B_OK, OK,
                    f"{rate:4.1f}Hz  B{len(m.blue_cones)} Y{len(m.yellow_cones)} "
                    f"O{len(m.orange_cones) + len(m.big_orange_cones)} "
                    f"U{len(m.unknown_color_cones)}")
        st["perception_ok"] = perc[0] == B_OK
        lines.append(self._stage("PERCEPTION", *perc))

        # SLAM — lifecycle + map size. The status topic is latched, so
        # liveness comes from the ego-odom stream graph_slam re-publishes.
        slam = self.slam_status.msg.data if self.slam_status.msg else ""
        map_n = self._cone_count(self.cone_map.msg) if self.cone_map.msg else 0
        if slam and self.ego.rx is not None and not self.ego.fresh(1.5):
            style = (
                B_ERR, ERR,
                f"ego odom silent {self.ego.age():.0f}s — slam down?")
        else:
            style = {
                "mapping": (B_OK, WARN, f"MAPPING  map {map_n}"),
                "mapping_converged": (B_OK, INFO, f"CONVERGED  map {map_n}"),
                "localization": (B_OK, OK, f"LOCALIZATION  map {map_n}"),
            }.get(slam, (B_DIM, DIM, "waiting for graph_slam"))
        st["slam"] = slam
        lines.append(self._stage("SLAM", *style))

        path_source = debug.get("path_source", "")
        state = debug.get("state", "")

        # GLOBAL planner — validity + reason; dim while mapping (expected).
        g_valid = bool(self.global_valid.msg and self.global_valid.msg.data
                       and self.global_valid.fresh(1.5))
        g_reason = self.global_reason.msg.data if self.global_reason.msg else ""
        if g_valid:
            glob = (B_OK, OK, "valid")
        elif self.global_valid.rx is None:
            glob = (B_DIM, DIM, "no heartbeat yet")
        elif not self.global_valid.fresh(1.5):
            glob = (B_ERR, ERR, f"heartbeat lost {self.global_valid.age():.0f}s")
        elif slam != "localization":
            glob = (B_DIM, DIM, g_reason or "waiting for localization")
        else:
            glob = (B_ERR, ERR, g_reason or "invalid")
        st["global_valid"] = g_valid
        lines.append(self._stage("GLOBAL", *glob))

        # LOCAL planner — load-bearing only while path_source == LOCAL.
        l_valid = bool(self.local_valid.msg and self.local_valid.msg.data
                       and self.local_valid.fresh(1.5))
        l_reason = self.local_reason.msg.data if self.local_reason.msg else ""
        local_bearing = path_source in ("", "LOCAL")
        if l_valid:
            loc = (B_OK, OK, "valid")
        elif self.local_valid.rx is None:
            loc = (B_DIM, DIM, "no heartbeat yet")
        elif not self.local_valid.fresh(1.5):
            loc = (B_ERR, ERR, f"heartbeat lost {self.local_valid.age():.0f}s")
        elif local_bearing:
            loc = (B_ERR, ERR, l_reason or "invalid")
        else:
            loc = (B_DIM, DIM, "idle (on global)")
        st["local_valid"] = l_valid
        lines.append(self._stage("LOCAL", *loc))

        # SELECTOR — which candidate feeds the controller, and why not.
        s_valid = bool(self.selected_valid.msg and self.selected_valid.msg.data
                       and self.selected_valid.fresh(1.5))
        failure = sel.get("selection_failure", "")
        candidate = sel.get("selected_candidate", "?")
        if s_valid:
            pick = (B_OK, OK, f"{path_source or '?'} → {candidate}")
        elif self.selected_valid.rx is None:
            pick = (B_DIM, DIM, "waiting for hyu_path_selector")
        elif not self.selected_valid.fresh(1.5):
            pick = (B_ERR, ERR,
                    f"heartbeat lost {self.selected_valid.age():.0f}s")
        elif failure == "stop_requested":
            pick = (B_DIM, DIM, "stop requested")
        else:
            detail = failure or "invalid"
            cont = sel.get("continuity_failure", "none")
            if detail in ("global_unavailable", "handoff_not_ready") and cont != "none":
                detail += f" ({cont})"
            pick = (B_ERR, ERR, detail)
        st["selected_valid"] = s_valid
        st["selection_failure"] = failure
        lines.append(self._stage("SELECTOR", *pick))

        # CONTROL — target vs actual speed, steering; brake detection.
        as_state = self.can.msg.as_state if self.can.msg else None
        driving = as_state == 2
        st["as_state"] = as_state
        actual = self.ego.msg.twist.twist.linear.x if self.ego.msg else float("nan")
        st["actual_speed"] = actual
        if self.cmd.rx is None:
            ctl = (B_DIM, DIM, "waiting for /cmd")
        elif not self.cmd.fresh(1.5):
            ctl = (B_ERR, ERR, f"/cmd silent {self.cmd.age():.0f}s — controller down?")
        else:
            d = self.cmd.msg.drive
            braking = d.speed == 0.0 and d.acceleration < -1.0
            body = (
                f"{actual:4.1f}→{d.speed:3.1f}m/s  "
                f"δ{d.steering_angle:+5.2f}rad")
            if braking and driving:
                ctl = (B_WARN, WARN, f"BRAKE  {body}")
            elif braking:
                ctl = (B_DIM, DIM, f"hold  {body}")
            else:
                ctl = (B_OK, OK, body)
        lines.append(self._stage("CONTROL", *ctl))

        # MISSION — sim AS/AMI + lap progress. The lap target comes from the
        # state machine's debug stream so the HUD always shows the value the
        # mission actually uses; the parameter is only a pre-boot fallback.
        lap = debug.get("lap_count", "?")
        self.target_laps = debug.get("target_lap_count", self.target_laps)
        if self.can.msg is None:
            mis = (B_DIM, DIM, "waiting for /ros_can/state")
        else:
            as_name = AS_NAMES.get(as_state, f"AS:{as_state}")
            ami = AMI_NAMES.get(self.can.msg.ami_state, str(self.can.msg.ami_state))
            body = f"{as_name} {ami}  lap {lap}/{self.target_laps}"
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
        lines.append(self._stage("MISSION", *mis))

        # TRACKING — cross-track error + Frenet position (needs ate_monitor).
        if self.cte.msg is not None and self.cte.fresh(1.5):
            d_val = self.cte.msg.data
            rmse = self.cte_rmse.msg.data if self.cte_rmse.msg else float("nan")
            try:
                s_val = f"{float(debug.get('current_s')):.1f}m"
            except (TypeError, ValueError):
                s_val = "?"
            color = OK if abs(d_val) < 0.35 else (WARN if abs(d_val) < 0.8 else ERR)
            lines.append(
                pad("TRACKING") + span(f"d{d_val:+5.2f}m", color)
                + span(f"  rmse {rmse:.2f}  s {s_val}", DIM))
        else:
            lines.append(self._stage("TRACKING", B_DIM, DIM, "no frenet data"))
        return lines, st

    @staticmethod
    def _stage(label, bullet, color, body):
        # The overlay budgets one visual row per line; clip long reason
        # strings so they can't wrap and overflow the fixed panel height.
        if len(body) > 60:
            body = body[:59] + "…"
        return pad(label) + span(f"{bullet} {body}", color)

    @staticmethod
    def _lap_times(debug):
        """'last 42.3s best 41.8s' from the orange-gate lap timer keys."""
        try:
            last = float(debug.get("lap_time_last", "0"))
            best = float(debug.get("lap_time_best", "0"))
        except ValueError:
            return ""
        if last <= 0.0:
            return ""
        return f"last {last:.1f}s best {best:.1f}s"

    @staticmethod
    def _lap_elapsed(debug):
        try:
            elapsed = float(debug.get("lap_elapsed", "-1"))
        except ValueError:
            return ""
        return f" · ⏱{elapsed:.0f}s" if elapsed >= 0.0 else ""

    def _banner(self, debug, sel, st):
        """One line answering: what is the car doing / why is it stuck."""
        state = debug.get("state", "")
        path_source = debug.get("path_source", "")
        slam = st.get("slam", "")
        lap = debug.get("lap_count", "?")
        v = st.get("actual_speed", float("nan"))
        v_txt = f"{v:.1f} m/s" if math.isfinite(v) else "- m/s"
        driving = st.get("as_state") == 2

        if self.debug.rx is None or not self.debug.fresh(2.0):
            return "✕ PLANNING GRAPH SILENT — is hyu_planning_bringup up?", ERR
        if st.get("as_state") == 3:
            return "✕ EMERGENCY BRAKE (AS:EBS)", ERR
        if state == "STOP" or st.get("as_state") == 4:
            return f"⚑ FINISHED — {lap} laps", TEXT
        if not driving:
            return "WAITING MISSION — run: race (auto-arms) or set_mission", WARN
        # From here on the car is armed & driving: diagnose why it may be stuck.
        if self.cmd.rx is not None and not self.cmd.fresh(1.5):
            return "✕ CONTROLLER SILENT — /cmd stopped", ERR
        if not st.get("selected_valid"):
            cause = st.get("selection_failure")
            if not cause or cause == "none":
                cause = ("hyu_path_selector silent"
                         if not self.selected_valid.fresh(1.5) else "no path")
            return f"✕ NO PATH → BRAKING — {cause}", ERR
        if not st.get("perception_ok") and slam != "localization":
            return "✕ PERCEPTION SILENT — SLAM starving", ERR
        if slam in ("mapping", "mapping_converged") or state == "LOCAL":
            if slam == "localization":
                gate = self._handoff_gate(debug)
                return f"LOCALIZED · awaiting handoff ({gate})", INFO
            label = "MAPPING" if slam != "mapping_converged" else "MAP CONVERGED"
            return f"LAP 1 · {label} · LOCAL · {v_txt}", WARN
        if path_source == "GLOBAL_FINAL_STOP":
            return (
                f"FINAL LAP {lap}/{self.target_laps} · {v_txt}"
                f"{self._lap_elapsed(debug)}", ACCENT)
        if state == "GLOBAL":
            return (
                f"RACING · GLOBAL · lap {lap}/{self.target_laps} "
                f"· {v_txt}{self._lap_elapsed(debug)}", OK)
        return f"{state or '?'} · {path_source or '?'} · {v_txt}", TEXT

    def _handoff_gate(self, debug):
        """Name the first failing LOCAL->GLOBAL entry gate for the banner."""
        if debug.get("global_path_valid") == "false":
            return "global path invalid"
        if debug.get("frenet_fresh") == "false":
            return "frenet odom stale"
        try:
            d = abs(float(debug.get("current_d", "0")))
            if d > 2.0:
                return f"|d|={d:.1f}m > 2.0 gate"
        except ValueError:
            pass
        for key in ("global_handoff_dwell_ready", "global_handoff_ready",
                    "global_path_ready"):
            if debug.get(key) == "false":
                return key.replace("global_", "").replace("_", " ")
        return "selector dwell"

    def _publish(self, pub, text, width, height, h_align, v_align,
                 h_dist, v_dist, size):
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
            pub.publish(ov)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"overlay publish failed ({exc})")


def main():
    rclpy.init()
    node = StackHud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
