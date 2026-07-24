#!/usr/bin/env python3
# Copyright 2026 shchon11
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
NTRIP client -> RTCM corrections for the SBG Ellipse-D RTK solution.

The sbg_ros2_driver already knows how to take RTK corrections: with
``rtcm.subscribe: true`` it subscribes to ``ntrip_client/rtcm``
(``rtcm_msgs/Message``) and pipes the bytes straight into the device over the
sbgECom link (sbg_device.cpp writeRtcmMessageToDevice). What it does NOT do is
FETCH those corrections -- that is this node. Without it the Ellipse-D runs on
SBAS/autonomous and tops out at PSRDIFF (sub-metre); fed a carrier-phase RTCM3
stream from a base within ~10-40 km it resolves RTK float -> fixed (cm).

  NTRIP caster ──HTTP/ICY stream──► this node ──rtcm_msgs/Message──► /ntrip_client/rtcm
                                                       │
                                         sbg_driver (rtcm.subscribe:true) ──► Ellipse-D

VRS support: virtual-reference-station mounts (e.g. Korea's NGII network) need
the rover to uplink its approximate position as an NMEA GGA sentence, or they
send nothing. This node builds the GGA from the live INS position
(``/sbg/ekf_nav``) and sends it on connect and every ``gga_interval`` s. A
fixed ``initial_latitude/longitude`` seeds the very first GGA before any fix
exists; single-base mounts that ignore GGA can leave ``send_gga`` false.

Only the standard library is used for the socket/NTRIP protocol (no external
NTRIP dependency to vendor). The blocking socket runs on its own thread; RTCM
frames are handed to rclpy through the node's publisher, which is thread-safe.

  ros2 run hyu_localization ntrip_client --ros-args \
      -p host:=RTS2.ngii.go.kr -p port:=2101 -p mountpoint:=VRS-RTCM32 \
      -p username:=ID -p password:=PW -p send_gga:=true
"""

import base64
import math
import socket
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from rtcm_msgs.msg import Message as RtcmMessage

try:
    from sbg_driver.msg import SbgEkfNav
except ImportError as exc:  # pragma: no cover - depends on runtime environment
    raise SystemExit(
        "sbg_driver messages not found. Source the workspace that provides the "
        "sbg_driver package before running this node."
    ) from exc


def _nmea_gga(lat_deg, lon_deg, alt_m, quality, num_sv, t_utc):
    """Build an NMEA GGA sentence (VRS uplink) from a geodetic fix."""
    hh = int(t_utc // 3600) % 24
    mm = int(t_utc // 60) % 60
    ss = t_utc % 60.0
    lat_hemi = "N" if lat_deg >= 0 else "S"
    lon_hemi = "E" if lon_deg >= 0 else "W"
    lat = abs(lat_deg)
    lon = abs(lon_deg)
    lat_deg_i = int(lat)
    lon_deg_i = int(lon)
    lat_min = (lat - lat_deg_i) * 60.0
    lon_min = (lon - lon_deg_i) * 60.0
    body = (
        f"GPGGA,{hh:02d}{mm:02d}{ss:05.2f},"
        f"{lat_deg_i:02d}{lat_min:08.5f},{lat_hemi},"
        f"{lon_deg_i:03d}{lon_min:08.5f},{lon_hemi},"
        f"{quality},{num_sv:02d},1.0,{alt_m:.2f},M,0.0,M,,"
    )
    checksum = 0
    for ch in body:
        checksum ^= ord(ch)
    return f"${body}*{checksum:02X}\r\n"


class NtripClient(Node):
    def __init__(self):
        super().__init__("ntrip_client")

        self.host = self.declare_parameter("host", "").value
        self.port = int(self.declare_parameter("port", 2101).value)
        self.mountpoint = self.declare_parameter("mountpoint", "").value
        self.username = self.declare_parameter("username", "").value
        self.password = self.declare_parameter("password", "").value
        # NTRIP protocol revision the caster speaks (1 = ICY, 2 = HTTP). Most
        # modern casters accept 2; a few legacy VRS mounts still want 1.
        self.ntrip_version = int(self.declare_parameter("ntrip_version", 2).value)
        # Absolute default so it matches the driver's rtcm.subscribe topic
        # (rtcm.namespace=ntrip_client + rtcm.topic_name=rtcm) regardless of how
        # this node is launched (node name != namespace; a relative "rtcm" would
        # land on /rtcm and the driver would never receive it).
        self.rtcm_topic = self.declare_parameter("rtcm_topic", "/ntrip_client/rtcm").value

        # VRS: uplink the rover position as GGA. Off for single-base mounts.
        self.send_gga = bool(self.declare_parameter("send_gga", True).value)
        self.gga_interval = float(self.declare_parameter("gga_interval", 10.0).value)
        self.nav_topic = self.declare_parameter("nav_topic", "/sbg/ekf_nav").value
        # Seed GGA before the first live fix (rough site coordinates). NaN =
        # wait for a live fix before sending any GGA.
        self.init_lat = float(self.declare_parameter("initial_latitude", float("nan")).value)
        self.init_lon = float(self.declare_parameter("initial_longitude", float("nan")).value)
        self.init_alt = float(self.declare_parameter("initial_altitude", 50.0).value)

        self.reconnect_delay = float(self.declare_parameter("reconnect_delay", 5.0).value)
        self.socket_timeout = float(self.declare_parameter("socket_timeout", 15.0).value)

        if not self.host or not self.mountpoint:
            raise ValueError("ntrip_client requires 'host' and 'mountpoint' params")

        self.pub = self.create_publisher(RtcmMessage, self.rtcm_topic, 10)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(SbgEkfNav, self.nav_topic, self._on_nav, sensor_qos)

        # Latest live fix for the GGA uplink (lat, lon, alt, quality, sats).
        self._fix = None
        if math.isfinite(self.init_lat) and math.isfinite(self.init_lon):
            self._fix = (self.init_lat, self.init_lon, self.init_alt, 1, 12)

        self._stop = threading.Event()
        self._rtcm_bytes = 0
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        self.create_timer(10.0, self._log_health)
        self.get_logger().info(
            f"NTRIP client: {self.host}:{self.port}/{self.mountpoint} "
            f"-> {self.rtcm_topic} (GGA uplink {'on' if self.send_gga else 'off'})"
        )

    # --- position for the VRS GGA uplink ---------------------------------

    def _on_nav(self, msg):
        # solution_mode 4 = valid absolute position; below it the lat/lon is a
        # drifting integration and would send the VRS a wrong reference.
        if msg.status.solution_mode >= 4 and msg.status.position_valid:
            # GGA quality: 1 single / 2 DGPS / 4 RTK fixed / 5 RTK float. The
            # caster only needs a rough position, so a coarse tier is fine.
            self._fix = (msg.latitude, msg.longitude, msg.altitude, 1, 12)

    def _current_gga(self):
        if self._fix is None:
            return None
        lat, lon, alt, q, sv = self._fix
        return _nmea_gga(lat, lon, alt, q, sv, time.time() % 86400.0)

    # --- NTRIP socket loop -----------------------------------------------

    def _build_request(self):
        auth = base64.b64encode(
            f"{self.username}:{self.password}".encode()
        ).decode()
        if self.ntrip_version >= 2:
            lines = [
                f"GET /{self.mountpoint} HTTP/1.1",
                f"Host: {self.host}:{self.port}",
                "Ntrip-Version: Ntrip/2.0",
                "User-Agent: NTRIP hyu_localization/1.0",
                f"Authorization: Basic {auth}",
                "Connection: close",
            ]
        else:
            lines = [
                f"GET /{self.mountpoint} HTTP/1.0",
                "User-Agent: NTRIP hyu_localization/1.0",
                f"Authorization: Basic {auth}",
            ]
        return ("\r\n".join(lines) + "\r\n\r\n").encode()

    def _run(self):
        while not self._stop.is_set() and rclpy.ok():
            try:
                self._session()
            except (OSError, socket.timeout) as exc:
                self.get_logger().warn(f"NTRIP connection lost: {exc}")
            if self._stop.is_set():
                break
            time.sleep(self.reconnect_delay)

    def _session(self):
        sock = socket.create_connection(
            (self.host, self.port), timeout=self.socket_timeout
        )
        sock.settimeout(self.socket_timeout)
        sock.sendall(self._build_request())
        # Read the response header (until blank line).
        header = b""
        while b"\r\n\r\n" not in header and b"\n\n" not in header:
            chunk = sock.recv(256)
            if not chunk:
                raise OSError("caster closed during handshake")
            header += chunk
            if len(header) > 4096:
                break
        head_txt = header.decode("latin-1", "replace")
        if not ("200" in head_txt.split("\r\n")[0] or head_txt.startswith("ICY 200")):
            raise OSError(f"caster rejected: {head_txt.splitlines()[0] if head_txt else 'no reply'}")
        self.get_logger().info("NTRIP stream open, receiving RTCM")

        # Any bytes past the header are already RTCM.
        sep = b"\r\n\r\n" if b"\r\n\r\n" in header else b"\n\n"
        leftover = header.split(sep, 1)[1] if sep in header else b""
        if leftover:
            self._publish(leftover)

        last_gga = 0.0
        # Send an initial GGA immediately so a VRS mount starts streaming.
        if self.send_gga:
            self._maybe_send_gga(sock, force=True)
            last_gga = time.time()

        while not self._stop.is_set() and rclpy.ok():
            try:
                data = sock.recv(4096)
            except socket.timeout:
                # No RTCM for a while: keep the VRS mount alive with a GGA.
                if self.send_gga:
                    self._maybe_send_gga(sock, force=True)
                    last_gga = time.time()
                continue
            if not data:
                raise OSError("caster closed the stream")
            self._publish(data)
            now = time.time()
            if self.send_gga and (now - last_gga) >= self.gga_interval:
                self._maybe_send_gga(sock)
                last_gga = now
        try:
            sock.close()
        except OSError:
            pass

    def _maybe_send_gga(self, sock, force=False):
        gga = self._current_gga()
        if gga is None:
            if force:
                self.get_logger().warn(
                    "VRS wants a GGA but no position yet "
                    "(set initial_latitude/longitude or wait for a fix)"
                )
            return
        try:
            sock.sendall(gga.encode())
        except OSError as exc:
            self.get_logger().warn(f"GGA uplink failed: {exc}")

    def _publish(self, data):
        msg = RtcmMessage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.message = list(data)
        self.pub.publish(msg)
        self._rtcm_bytes += len(data)

    def _log_health(self):
        rate = self._rtcm_bytes / 10.0
        self._rtcm_bytes = 0
        if rate > 0:
            self.get_logger().info(f"RTCM {rate:.0f} B/s -> device")
        else:
            self.get_logger().warn("no RTCM received in the last 10 s")

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main():
    rclpy.init(args=sys.argv)
    node = NtripClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
