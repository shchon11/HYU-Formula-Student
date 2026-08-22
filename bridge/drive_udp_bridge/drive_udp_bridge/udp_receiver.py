# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

"""
Non-blocking IPv4 UDP receiver for Speedgoat encoder feedback.

Every datagram is returned with a receive time on the ``time.monotonic()``
clock. On Linux the kernel's own arrival timestamp (``SO_TIMESTAMPNS``, taken
when the frame entered the network stack) is used and mapped onto the
monotonic clock, so the interval between two samples is the true arrival
interval and not a multiple of the caller's poll period. Where that socket
option is unavailable the drain time is used instead.
"""

import socket
import struct
import sys
import time
from typing import List, Optional, Tuple


ReceivedDatagram = Tuple[bytes, Tuple[str, int], float]

# Linux SO_TIMESTAMPNS (== SCM_TIMESTAMPNS): the Python socket module does not
# export it. On 64-bit Linux the ancillary payload is a native ``struct
# timespec`` (two int64: seconds, nanoseconds) in CLOCK_REALTIME.
_SO_TIMESTAMPNS = getattr(socket, 'SO_TIMESTAMPNS', 35)
_SCM_TIMESTAMPNS = getattr(socket, 'SCM_TIMESTAMPNS', _SO_TIMESTAMPNS)
_TIMESPEC_FORMAT = 'qq'
_TIMESPEC_SIZE = struct.calcsize(_TIMESPEC_FORMAT)
_ANCILLARY_BUFFER = socket.CMSG_SPACE(64) if hasattr(socket, 'CMSG_SPACE') else 0


class UdpReceiver:
    """Own a bound non-blocking socket and drain available datagrams."""

    def __init__(self, bind_ip: str, bind_port: int, kernel_timestamps: bool = True) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._kernel_timestamps = False
        try:
            self._socket.bind((bind_ip, bind_port))
            self._socket.setblocking(False)
            if kernel_timestamps and sys.platform.startswith('linux') and _ANCILLARY_BUFFER:
                try:
                    self._socket.setsockopt(socket.SOL_SOCKET, _SO_TIMESTAMPNS, 1)
                    self._kernel_timestamps = True
                except OSError:
                    self._kernel_timestamps = False
        except Exception:
            self._socket.close()
            raise

    @property
    def local_endpoint(self):
        """Return the actual local address and port."""
        return self._socket.getsockname()

    @property
    def kernel_timestamps(self) -> bool:
        """Return whether receive times come from the kernel arrival stamp."""
        return self._kernel_timestamps

    @staticmethod
    def _arrival_realtime(ancillary) -> Optional[float]:
        for level, kind, data in ancillary:
            if level != socket.SOL_SOCKET or kind != _SCM_TIMESTAMPNS:
                continue
            if len(data) < _TIMESPEC_SIZE:
                continue
            seconds, nanoseconds = struct.unpack(_TIMESPEC_FORMAT, data[:_TIMESPEC_SIZE])
            return float(seconds) + float(nanoseconds) * 1.0e-9
        return None

    def receive_available(self, max_datagrams: int = 64) -> List[ReceivedDatagram]:
        """Return currently queued datagrams with monotonic receive times."""
        received = []
        # One realtime<->monotonic anchor per drain; the pair is sampled
        # back-to-back so its skew is microseconds.
        anchor_monotonic = time.monotonic()
        anchor_realtime = time.time()
        for _ in range(max_datagrams):
            try:
                if self._kernel_timestamps:
                    packet, ancillary, _flags, source = self._socket.recvmsg(
                        65535, _ANCILLARY_BUFFER)
                else:
                    packet, source = self._socket.recvfrom(65535)
                    ancillary = ()
            except BlockingIOError:
                break
            received_at = None
            if ancillary:
                arrival_realtime = self._arrival_realtime(ancillary)
                if arrival_realtime is not None:
                    lag = anchor_realtime - arrival_realtime
                    # A negative lag means the wall clock stepped between the
                    # arrival and the drain; fall back to the drain time.
                    if 0.0 <= lag < 60.0:
                        received_at = anchor_monotonic - lag
            if received_at is None:
                received_at = time.monotonic()
            received.append((packet, source, received_at))
        return received

    def close(self) -> None:
        """Close the underlying socket."""
        self._socket.close()
