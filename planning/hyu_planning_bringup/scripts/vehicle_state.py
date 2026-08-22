#!/usr/bin/env python3
"""Vehicle-side AS/AMI state -- what the simulator's race-car plugin does in sim.

Publishes /vehicle/as_state (hyu_msgs/CanState) and /vehicle/as_state_str at a
fixed rate and serves the /vehicle/* contract that mission.sh and the planning
stack already rely on, so step 3 ('mission <name>') arms on the car exactly as
it does in the simulator:

    /vehicle/set_mission        hyu_msgs/srv/SetCanState  select the AMI (once;
                                                          /vehicle/reset clears it)
    /vehicle/reset              std_srvs/Trigger          AMI -> NOT_SELECTED, EBS and
                                                          finished latches cleared
    /vehicle/ebs                std_srvs/Trigger          latch AS_EMERGENCY_BRAKE
    /vehicle/reset_vehicle_pos  std_srvs/Trigger          sim teleport -- no-op here
    /vehicle/set_as_button      std_srvs/SetBool          bench stand-in for the button

AS state, in priority order:

    AS_EMERGENCY_BRAKE  while an EBS is latched
    AS_FINISHED         once /vehicle/mission_completed said true (until reset)
    AS_DRIVING          AS button ON and a mission selected   -> ECU autonomous enable
    AS_READY            mission selected, button OFF
    AS_OFF              no mission selected

The AS button is the physical momentary button on the car; as_button.py
(hyu_sensor_bringup, header pin 31) toggles ON/OFF on every press and
publishes /vehicle/as_button (std_msgs/Bool) at 20 Hz. The default is OFF:
the car boots in manual, and the button stream must stay FRESH -- when it
stops for as_button_timeout_sec (driver dead, wire off) the button reads OFF
again, so a crashed driver can never leave the car armed. 'mission go' /
'mission halt' set the same switch through the service (bench use, latched).
drive_udp_bridge turns AS_DRIVING into the autonomous-enable byte for the
ECU and, on the OFF->ON edge, resets the SLAM map before raising it.

    ros2 run hyu_planning_bringup vehicle_state.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from hyu_msgs.msg import CanState
from hyu_msgs.srv import SetCanState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

AS_NAMES = {
    CanState.AS_OFF: "OFF",
    CanState.AS_READY: "READY",
    CanState.AS_DRIVING: "DRIVING",
    CanState.AS_EMERGENCY_BRAKE: "EMERGENCY",
    CanState.AS_FINISHED: "FINISHED",
}
AMI_NAMES = {
    CanState.AMI_NOT_SELECTED: "NOT_SELECTED",
    CanState.AMI_ACCELERATION: "ACCELERATION",
    CanState.AMI_SKIDPAD: "SKIDPAD",
    CanState.AMI_AUTOCROSS: "AUTOCROSS",
    CanState.AMI_TRACK_DRIVE: "TRACK_DRIVE",
    CanState.AMI_AUTONOMOUS_DEMO: "AUTONOMOUS_DEMO",
    CanState.AMI_ADS_INSPECTION: "ADS_INSPECTION",
    CanState.AMI_ADS_EBS: "ADS_EBS",
    CanState.AMI_DDT_INSPECTION_A: "DDT_INSPECTION_A",
    CanState.AMI_DDT_INSPECTION_B: "DDT_INSPECTION_B",
    CanState.AMI_JOYSTICK: "JOYSTICK",
    CanState.AMI_MANUAL: "MANUAL",
}


class VehicleState(Node):
    def __init__(self):
        super().__init__("vehicle_state")
        p = self.declare_parameter
        as_topic = str(p("as_state_topic", "/vehicle/as_state").value)
        as_str_topic = str(p("as_state_str_topic", "/vehicle/as_state_str").value)
        button_topic = str(p("as_button_topic", "/vehicle/as_button").value)
        completed_topic = str(p("mission_completed_topic", "/vehicle/mission_completed").value)
        rate = float(p("publish_rate_hz", 20.0).value)
        # Start with the button as it physically is at boot: OFF. A bench
        # override is possible (-p as_button_initial:=true) but never the
        # default -- the car must not come up armed.
        self.button_on = bool(p("as_button_initial", False).value)
        # The GPIO driver publishes its state at 20 Hz; silence longer than
        # this means the driver (or its wire) is gone -> button OFF.
        self.button_timeout = float(p("as_button_timeout_sec", 1.0).value)
        self.button_source = "initial"   # 'topic' (watched) | 'service' (latched)
        self.button_topic_t = None
        self._button_stale_warned = False

        self.ami = CanState.AMI_NOT_SELECTED
        # BENCH ONLY: pre-select a mission so the AS button alone takes the
        # state to AS_DRIVING (ECU comms test with 'bridge', no planning stack
        # to call /vehicle/set_mission). Name as in CanState (TRACK_DRIVE,
        # AUTOCROSS, SKIDPAD, ACCELERATION, ADS_INSPECTION, ...) or the AMI
        # number. Default "" = unarmed, as the car must boot.
        initial_mission = str(p("initial_mission", "").value).strip()
        if initial_mission:
            self.ami = self._parse_mission(initial_mission)
            self.get_logger().warn(
                f"BENCH: mission {AMI_NAMES[self.ami]} pre-selected (initial_mission) -- "
                "the AS button alone now arms AS_DRIVING. Never use this on the car.")
        self.ebs = False
        self.finished = False
        self._last_as = None

        # Best-effort like the sim's publisher: every consumer (bridge, state
        # machine, dssi_state, HUD) subscribes best-effort and just wants the
        # freshest value. Published continuously -- the bridge's watchdog
        # drops the autonomous enable 0.5 s after the stream stops.
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(CanState, as_topic, qos)
        self.pub_str = self.create_publisher(String, as_str_topic, 1)
        self.create_subscription(Bool, button_topic, self._on_button, 10)
        self.create_subscription(Bool, completed_topic, self._on_completed, 10)
        self.create_service(SetCanState, "/vehicle/set_mission", self._srv_set_mission)
        self.create_service(Trigger, "/vehicle/reset", self._srv_reset)
        self.create_service(Trigger, "/vehicle/ebs", self._srv_ebs)
        self.create_service(Trigger, "/vehicle/reset_vehicle_pos", self._srv_reset_pos)
        self.create_service(SetBool, "/vehicle/set_as_button", self._srv_set_button)
        self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(
            f"vehicle state: {as_topic} @ {rate:.0f} Hz from the AS button "
            f"({button_topic} / /vehicle/set_as_button; initial "
            f"{'ON' if self.button_on else 'OFF'}); /vehicle/set_mission, reset, ebs served")

    # --- inputs ------------------------------------------------------------
    def _on_button(self, msg: Bool):
        self.button_topic_t = self.get_clock().now().nanoseconds * 1e-9
        self.button_source = "topic"
        self._button_stale_warned = False
        self._set_button(bool(msg.data), "topic")

    def _button_fresh(self) -> bool:
        """Topic-sourced button must be fresh; the bench service is latched."""
        if self.button_source != "topic":
            return True
        age = self.get_clock().now().nanoseconds * 1e-9 - (self.button_topic_t or 0.0)
        return age <= self.button_timeout

    def _set_button(self, on: bool, source: str):
        if on == self.button_on:
            return
        self.button_on = on
        if on and self.ami == CanState.AMI_NOT_SELECTED:
            self.get_logger().warn(
                "AS button ON but no mission selected -- staying AS_OFF "
                "(arm one: 'mission <name>')")
        self.get_logger().info(f"AS button {'ON' if on else 'OFF'} ({source})")

    def _on_completed(self, msg: Bool):
        if msg.data and not self.finished:
            self.finished = True
            self.get_logger().info("mission completed -> AS_FINISHED (until /vehicle/reset)")

    # --- services -----------------------------------------------------------
    def _srv_set_mission(self, req, res):
        if self.ami != CanState.AMI_NOT_SELECTED:
            res.success = False
            res.message = (f"mission already set ({AMI_NAMES.get(self.ami, self.ami)}); "
                           "call /vehicle/reset first")
            self.get_logger().warn(res.message)
            return res
        if req.ami_state not in AMI_NAMES or req.ami_state == CanState.AMI_NOT_SELECTED:
            res.success = False
            res.message = f"unknown ami_state {req.ami_state}"
            return res
        self.ami = int(req.ami_state)
        res.success = True
        res.message = f"mission {AMI_NAMES[self.ami]} selected"
        self.get_logger().info(res.message + (" -- AS button is ON: driving enabled now"
                                              if self.button_on else ""))
        return res

    def _srv_reset(self, req, res):
        self.ami = CanState.AMI_NOT_SELECTED
        self.ebs = False
        self.finished = False
        res.success = True
        res.message = "vehicle state reset (mission cleared)"
        self.get_logger().info(res.message)
        return res

    def _srv_ebs(self, req, res):
        self.ebs = True
        res.success = True
        res.message = "EBS latched: AS_EMERGENCY_BRAKE until /vehicle/reset"
        self.get_logger().error(res.message)
        return res

    def _srv_reset_pos(self, req, res):
        res.success = True
        res.message = "no vehicle pose to reset on the car"
        return res

    def _srv_set_button(self, req, res):
        self.button_source = "service"
        self._set_button(bool(req.data), "service")
        res.success = True
        res.message = f"AS button {'ON' if self.button_on else 'OFF'}"
        return res

    @staticmethod
    def _parse_mission(text: str) -> int:
        """CanState AMI name ('TRACK_DRIVE', 'trackdrive') or number -> AMI value."""
        key = text.strip().upper().replace("-", "_")
        if key.isdigit() and int(key) in AMI_NAMES:
            return int(key)
        for value, name in AMI_NAMES.items():
            if key in (name, name.replace("_", "")) or key == f"AMI_{name}":
                return value
        raise ValueError(
            f"initial_mission '{text}' unknown; use one of "
            f"{', '.join(n for n in AMI_NAMES.values() if n != 'NOT_SELECTED')}")

    # --- output -------------------------------------------------------------
    def _as_state(self) -> int:
        if self.ebs:
            return CanState.AS_EMERGENCY_BRAKE
        if self.finished:
            return CanState.AS_FINISHED
        if self.ami == CanState.AMI_NOT_SELECTED:
            return CanState.AS_OFF
        if self.button_on and not self._button_fresh():
            if not self._button_stale_warned:
                self._button_stale_warned = True
                self.get_logger().error(
                    f"AS button stream silent for > {self.button_timeout:.1f} s -- treating the "
                    "button as OFF (driver/wire down?)")
            return CanState.AS_READY
        return CanState.AS_DRIVING if self.button_on else CanState.AS_READY

    def _tick(self):
        msg = CanState()
        msg.as_state = self._as_state()
        msg.ami_state = self.ami
        self.pub.publish(msg)
        s = String()
        s.data = f"AS:{AS_NAMES.get(msg.as_state, msg.as_state)} AMI:{AMI_NAMES.get(self.ami, self.ami)}"
        self.pub_str.publish(s)
        if msg.as_state != self._last_as:
            self._last_as = msg.as_state
            self.get_logger().info(f"-> {s.data}")


def main():
    rclpy.init()
    node = VehicleState()
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
