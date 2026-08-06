# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""Small IPv4 UDP transport used by the ROS bridge and loopback tests."""

import socket


class UdpSender:
    """Own a bound non-blocking UDP socket and fixed destination endpoint."""

    def __init__(
        self,
        ecu_ip: str,
        ecu_port: int,
        local_bind_ip: str,
        local_bind_port: int,
    ) -> None:
        self._destination = (ecu_ip, ecu_port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind((local_bind_ip, local_bind_port))
            self._socket.setblocking(False)
        except Exception:
            self._socket.close()
            raise

    @property
    def local_endpoint(self):
        """Return the actual local address, including an OS-selected port."""
        return self._socket.getsockname()

    def send(self, packet: bytes) -> None:
        """Send exactly one UDP datagram to the configured ECU endpoint."""
        bytes_sent = self._socket.sendto(packet, self._destination)
        if bytes_sent != len(packet):
            raise OSError(
                f'partial UDP datagram send: {bytes_sent} of {len(packet)} bytes'
            )

    def close(self) -> None:
        """Close the underlying socket."""
        self._socket.close()
