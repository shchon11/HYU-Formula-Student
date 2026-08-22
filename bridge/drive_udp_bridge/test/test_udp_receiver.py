# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

import socket
import time

from drive_udp_bridge.udp_receiver import UdpReceiver


def test_udp_receiver_drains_available_loopback_datagrams():
    receiver = UdpReceiver('127.0.0.1', 0)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        sender.sendto(b'first', receiver.local_endpoint)
        sender.sendto(b'second', receiver.local_endpoint)

        deadline = time.monotonic() + 1.0
        datagrams = []
        while not datagrams and time.monotonic() < deadline:
            datagrams = receiver.receive_available()

        assert [item[0] for item in datagrams] == [b'first', b'second']
        assert all(item[1][0] == '127.0.0.1' for item in datagrams)
        assert all(item[2] > 0.0 for item in datagrams)
    finally:
        sender.close()
        receiver.close()


def test_udp_receiver_timestamps_are_arrival_times_on_the_monotonic_clock():
    receiver = UdpReceiver('127.0.0.1', 0)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        before_send = time.monotonic()
        sender.sendto(b'stamped', receiver.local_endpoint)
        # Let the datagram sit in the socket queue: a kernel arrival stamp
        # must then be clearly earlier than the drain time.
        time.sleep(0.05)
        deadline = time.monotonic() + 1.0
        datagrams = []
        while not datagrams and time.monotonic() < deadline:
            datagrams = receiver.receive_available()
        after_drain = time.monotonic()

        assert len(datagrams) == 1
        received_at = datagrams[0][2]
        assert before_send <= received_at <= after_drain
        if receiver.kernel_timestamps:
            assert after_drain - received_at > 0.03
    finally:
        sender.close()
        receiver.close()
