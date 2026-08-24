import math
import socket
import struct
import time

from hyu_lite_sim.ecu_udp import CMD_FMT, FakeEcu, SteeringTable


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_steering_table_from_csv_inverts_bridge_convention(tmp_path):
    # bridge: equivalent = (right_toe - left_toe)/2 deg for a steering-wheel angle
    csv = tmp_path / 'k.csv'
    csv.write_text('"steering_displacements.angle_front"\t"suspension_kinematics.toe_left_front"\t"suspension_kinematics.toe_right_front"\n'
                   '-90\t20\t-20\n0\t0\t0\n90\t-20\t20\n')
    t = SteeringTable.from_csv(str(csv))
    assert abs(t.wheel_rad_to_bicycle(math.radians(90.0)) - math.radians(20.0)) < 1e-9
    assert abs(t.wheel_rad_to_bicycle(math.radians(-45.0)) - math.radians(-10.0)) < 1e-9
    assert abs(t.wheel_rad_to_bicycle(math.radians(500.0)) - math.radians(20.0)) < 1e-9   # clamps


def test_fake_ecu_roundtrip():
    rx_port, fb_port = _free_port(), _free_port()
    ecu = FakeEcu('127.0.0.1', rx_port, '127.0.0.1', fb_port, SteeringTable.linear(0.5))
    fb = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fb.bind(('127.0.0.1', fb_port))
    fb.settimeout(1.0)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        assert ecu.command() == (0.0, 0.0, False)
        tx.sendto(struct.pack(CMD_FMT, 2.5, 0.4, 1, 1), ('127.0.0.1', rx_port))
        time.sleep(0.05)
        ecu.poll()
        speed, steer, en = ecu.command()
        assert en and abs(speed - 2.5) < 1e-6 and abs(steer - 0.2) < 1e-6
        # autonomous byte 0 -> no drive, even with a speed
        tx.sendto(struct.pack(CMD_FMT, 2.5, 0.4, 1, 0), ('127.0.0.1', rx_port))
        time.sleep(0.05)
        ecu.poll()
        assert ecu.command()[2] is False and ecu.command()[0] == 0.0
        # bad size is counted, not applied
        tx.sendto(b'\x00' * 5, ('127.0.0.1', rx_port))
        time.sleep(0.05)
        ecu.poll()
        assert ecu.n_bad == 1
        ecu.send_feedback((10.0, 11.0, 12.0, 13.0))
        pkt, _ = fb.recvfrom(64)
        assert struct.unpack('<ffff', pkt) == (10.0, 11.0, 12.0, 13.0)
        # staleness
        time.sleep(0.3)
        assert ecu.command(timeout_s=0.25)[2] is False
    finally:
        ecu.close()
        fb.close()
        tx.close()
