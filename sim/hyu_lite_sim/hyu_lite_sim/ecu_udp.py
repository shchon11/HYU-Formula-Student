"""Fake Speedgoat ECU on UDP -- the other end of drive_udp_bridge.

Command datagram (bridge -> ECU), little endian '<ffBB':
    float32 speed [m/s], float32 steering-WHEEL angle [rad], uint8 enable,
    uint8 autonomous_enable
Feedback datagram (ECU -> bridge): four float32 RPM, FL FR RL RR.

The bridge converts the stack's bicycle steering angle to a steering-wheel
angle through its suspension-kinematics table (steering_kinematics.csv:
steering wheel deg -> toe left/right deg, equivalent = (right-left)/2). The
fake ECU applies the same table FORWARD to recover the road-wheel angle, so
what the model steers is what the stack asked for (up to the bridge's clip).
"""
import csv
import math
import socket
import struct
import time
from bisect import bisect_right
from typing import Optional, Sequence, Tuple

CMD_FMT = '<ffBB'
CMD_SIZE = struct.calcsize(CMD_FMT)
FB_FMT = '<ffff'


class SteeringTable:
    """steering-wheel angle [rad] -> equivalent bicycle angle [rad] (forward table)."""

    def __init__(self, wheel_deg: Sequence[float], equiv_rad: Sequence[float]):
        self.wheel_deg = list(wheel_deg)
        self.equiv_rad = list(equiv_rad)

    @classmethod
    def linear(cls, ratio: float = 0.2):
        """Fallback: road angle = ratio * wheel angle."""
        return cls([-180.0, 180.0], [math.radians(-180.0 * ratio), math.radians(180.0 * ratio)])

    @classmethod
    def from_csv(cls, path: str):
        wheel, equiv = [], []
        with open(path, newline='', encoding='utf-8-sig') as f:
            reader = csv.reader(f, delimiter='\t')
            next(reader)
            for row in reader:
                if len(row) != 3:
                    continue
                w, l, r = (float(v) for v in row)
                wheel.append(w)
                equiv.append(math.radians((r - l) / 2.0))
        if len(wheel) < 2:
            raise ValueError(f'{path}: fewer than two calibration rows')
        return cls(wheel, equiv)

    @classmethod
    def from_bridge_share(cls, name: str = 'steering_kinematics.csv'):
        from ament_index_python.packages import get_package_share_directory
        import os
        return cls.from_csv(os.path.join(get_package_share_directory('drive_udp_bridge'), 'config', name))

    def wheel_rad_to_bicycle(self, wheel_rad: float) -> float:
        w = math.degrees(wheel_rad)
        xs, ys = self.wheel_deg, self.equiv_rad
        if w <= xs[0]:
            return ys[0]
        if w >= xs[-1]:
            return ys[-1]
        j = bisect_right(xs, w)
        i = j - 1
        f = (w - xs[i]) / (xs[j] - xs[i])
        return ys[i] + f * (ys[j] - ys[i])


class FakeEcu:
    def __init__(self, listen_ip: str, listen_port: int, feedback_ip: str, feedback_port: int,
                 table: Optional[SteeringTable] = None):
        self.table = table or SteeringTable.linear()
        self.listen = (listen_ip, listen_port)
        self.feedback = (feedback_ip, feedback_port)
        # No SO_REUSEADDR on purpose: a second simulator instance must fail
        # loudly ("Address already in use") instead of silently stealing the
        # bridge's datagrams from the first one.
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(self.listen)
        self.rx.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_cmd: Optional[Tuple[float, float, int, int]] = None
        self.last_rx_t: float = -1.0
        self.last_source = None
        self.n_rx = 0
        self.n_bad = 0

    def close(self):
        self.rx.close()
        self.tx.close()

    def poll(self) -> None:
        """Drain the socket; keep the newest well-formed command."""
        while True:
            try:
                pkt, src = self.rx.recvfrom(2048)
            except BlockingIOError:
                return
            except OSError:
                return
            if len(pkt) != CMD_SIZE:
                self.n_bad += 1
                continue
            speed, wheel_rad, enable, auto = struct.unpack(CMD_FMT, pkt)
            if not (math.isfinite(speed) and math.isfinite(wheel_rad)):
                self.n_bad += 1
                continue
            self.last_cmd = (speed, wheel_rad, int(enable), int(auto))
            self.last_rx_t = time.monotonic()
            self.last_source = src
            self.n_rx += 1

    def command(self, timeout_s: float = 0.25):
        """-> (speed_mps, bicycle_steer_rad, drive_enabled) as the ECU would apply it.

        Drive is enabled only while packets are fresh and BOTH bytes are 1 --
        the same fail-closed rule the Speedgoat model uses.
        """
        if self.last_cmd is None or (time.monotonic() - self.last_rx_t) > timeout_s:
            return 0.0, 0.0, False
        speed, wheel_rad, enable, auto = self.last_cmd
        enabled = enable == 1 and auto == 1
        return (speed if enabled else 0.0), self.table.wheel_rad_to_bicycle(wheel_rad), enabled

    def send_feedback(self, rpm: Sequence[float]) -> None:
        try:
            self.tx.sendto(struct.pack(FB_FMT, *[float(v) for v in rpm]), self.feedback)
        except OSError:
            pass
