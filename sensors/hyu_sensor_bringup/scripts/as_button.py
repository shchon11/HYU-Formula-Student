#!/usr/bin/env python3
"""AS button on the Jetson 40-pin header -> /vehicle/as_button (std_msgs/Bool).

The physical autonomous-system switch: a push button that TOGGLES the AS
state on every press (mode 'toggle', default) or a latching switch whose
position IS the state (mode 'level'). vehicle_state.py turns that into
/vehicle/as_state (AS_DRIVING while ON and a mission is selected), the ECU
bridge into the autonomous-enable byte (after a SLAM map reset on OFF->ON).

Wiring assumed by the defaults: button between header pin 31 (tegra234
PAA.00, gpiochip1 line 0) and GND (pin 30 next to it), read active-low --
pressed = 0. Pin 31 (like 32 and 33) has a built-in pull-up, so it idles at
1; pin 15 does NOT (it idles at 0 and a button to GND is invisible there --
measured 2026-08-21). Jetson.GPIO cannot set pulls (the pinmux does), so
pick a pull-up pin or add a ~10 kOhm pull-up to 3.3 V; a button wired to
3.3 V instead works with active_low:=false.

Safety: the state starts OFF and a press is only honoured after the button
has first been seen RELEASED for debounce_ms, so a stuck/shorted line at boot
can never arm the car; it is reported instead.

    ros2 run hyu_sensor_bringup as_button.py
    ros2 run hyu_sensor_bringup as_button.py --ros-args -p mode:=level -p active_low:=false
Publishes /vehicle/as_button (state, latched + 20 Hz) and
/vehicle/as_button_pressed (raw debounced level, for wiring checks).
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool


class AsButton(Node):
    def __init__(self):
        super().__init__("as_button")
        p = self.declare_parameter
        self.pin = int(p("pin", 31).value)                   # Jetson.GPIO BOARD numbering
        self.active_low = bool(p("active_low", True).value)  # pressed reads 0
        self.mode = str(p("mode", "toggle").value).strip().lower()
        self.debounce_s = float(p("debounce_ms", 40.0).value) / 1000.0
        self.poll_hz = float(p("poll_rate_hz", 200.0).value)
        self.pub_hz = float(p("publish_rate_hz", 20.0).value)
        topic = str(p("topic", "/vehicle/as_button").value)
        if self.mode not in ("toggle", "level"):
            raise ValueError("mode must be 'toggle' or 'level'")

        try:
            import Jetson.GPIO as GPIO
        except ImportError as exc:  # pragma: no cover - hardware dependent
            raise SystemExit(f"as_button: Jetson.GPIO not available ({exc}); "
                             "no button driver (vehicle_state keeps OFF, use "
                             "'mission go' / /vehicle/set_as_button)") from exc
        self.GPIO = GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.IN)

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(Bool, topic, latched)
        self.pub_pressed = self.create_publisher(Bool, "/vehicle/as_button_pressed", 10)

        self.state = False            # AS ON/OFF (what vehicle_state consumes)
        self.pressed = None           # debounced level
        self.armed = False            # saw RELEASED once -> presses count
        self._raw_last = None
        self._raw_since = time.monotonic()
        self._held_warned = False
        self._t0 = time.monotonic()
        self.create_timer(1.0 / self.poll_hz, self._poll)
        self.create_timer(1.0 / self.pub_hz, self._publish)
        self.get_logger().info(
            f"AS button on header pin {self.pin} ({'active-low: pressed=0' if self.active_low else 'active-high: pressed=1'}), "
            f"mode {self.mode}, debounce {self.debounce_s*1e3:.0f} ms -> {topic} (starts OFF)")

    def _read_pressed(self) -> bool:
        level = self.GPIO.input(self.pin)
        return (level == 0) if self.active_low else (level == 1)

    def _poll(self):
        raw = self._read_pressed()
        now = time.monotonic()
        if raw != self._raw_last:
            self._raw_last = raw
            self._raw_since = now
            return
        if now - self._raw_since < self.debounce_s or raw == self.pressed:
            # not stable long enough, or no change after debounce
            if self.pressed is None and now - self._raw_since >= self.debounce_s:
                pass
            else:
                self._stuck_check(now)
                return
        prev = self.pressed
        self.pressed = raw
        if prev is None:
            # first stable reading
            if raw:
                self.get_logger().warn(
                    "button reads PRESSED at start -- held, shorted, or the line has no "
                    "pull-up (pin idle must read the released level). Ignoring until released.")
            else:
                self.armed = True
            return
        if self.mode == "level":
            if self.state != raw:
                self.state = raw
                self.get_logger().info(f"AS switch {'ON' if raw else 'OFF'}")
            return
        # toggle mode: act on the RELEASED -> PRESSED edge, only once armed
        if raw and self.armed:
            self.state = not self.state
            self.get_logger().info(f"AS button pressed -> {'ON' if self.state else 'OFF'}")
        elif not raw and not self.armed:
            self.armed = True
            self.get_logger().info("button released -- presses are now honoured")

    def _stuck_check(self, now):
        if (self.pressed and not self.armed and not self._held_warned
                and now - self._raw_since > 5.0):
            self._held_warned = True
            self.get_logger().error(
                "button line has read PRESSED for 5 s since start: check wiring "
                "(use a pull-up pin: 31/32/33 idle high; pin 15 does not; or wire to 3.3 V and set active_low:=false)")

    def _publish(self):
        m = Bool(); m.data = bool(self.state); self.pub.publish(m)
        if self.pressed is not None:
            r = Bool(); r.data = bool(self.pressed); self.pub_pressed.publish(r)

    def destroy_node(self):
        try:
            self.GPIO.cleanup(self.pin)
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = AsButton()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
