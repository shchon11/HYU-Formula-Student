# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

import socket

from drive_udp_bridge.protocol import pack_command
from drive_udp_bridge.udp_sender import UdpSender


def test_udp_sender_delivers_one_exact_datagram_over_loopback():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0))
    receiver.settimeout(1.0)
    receiver_port = receiver.getsockname()[1]

    sender = UdpSender('127.0.0.1', receiver_port, '127.0.0.1', 0)
    packet = pack_command(7.25, -0.4, 1)

    try:
        sender.send(packet)
        received, source = receiver.recvfrom(64)
    finally:
        sender.close()
        receiver.close()

    assert received == packet
    assert source[0] == '127.0.0.1'
