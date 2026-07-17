"""Small timestamp-indexed buffer used for sensor message synchronization."""

from bisect import bisect_left
from dataclasses import dataclass
from operator import index
from typing import Generic, List, Optional, TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class TimestampedValue(Generic[T]):
    """A value paired with its integer nanosecond timestamp."""

    stamp_ns: int
    value: T


class TimestampBuffer(Generic[T]):
    """
    Keep the newest timestamped values and find the closest match.

    Entries are ordered by timestamp rather than arrival order so delayed or
    out-of-order sensor messages behave predictably.  When two entries are
    equally close to a target, the earlier timestamp wins.
    """

    def __init__(self, maxlen: int):
        self._maxlen = _integer(maxlen, "maxlen")
        if self._maxlen <= 0:
            raise ValueError("maxlen must be greater than zero")
        self._entries: List[TimestampedValue[T]] = []
        self._stamps: List[int] = []

    def __len__(self) -> int:
        return len(self._entries)

    def add(self, stamp_ns: int, value: T) -> None:
        """Add a value, replacing an existing value at the same timestamp."""
        stamp = _integer(stamp_ns, "stamp_ns")
        position = bisect_left(self._stamps, stamp)

        entry = TimestampedValue(stamp_ns=stamp, value=value)
        if position < len(self._stamps) and self._stamps[position] == stamp:
            self._entries[position] = entry
            return

        self._stamps.insert(position, stamp)
        self._entries.insert(position, entry)

        if len(self._entries) > self._maxlen:
            # Timestamp buffers should retain the newest time window even when
            # an old message arrives late.
            del self._stamps[0]
            del self._entries[0]

    def nearest(
        self,
        stamp_ns: int,
        tolerance_ns: int,
    ) -> Optional[TimestampedValue[T]]:
        """Return the closest entry within the inclusive tolerance."""
        match_index = self._nearest_index(stamp_ns, tolerance_ns)
        if match_index is None:
            return None
        return self._entries[match_index]

    def pop_nearest(
        self,
        stamp_ns: int,
        tolerance_ns: int,
    ) -> Optional[TimestampedValue[T]]:
        """Remove and return the closest entry within the tolerance."""
        match_index = self._nearest_index(stamp_ns, tolerance_ns)
        if match_index is None:
            return None
        del self._stamps[match_index]
        return self._entries.pop(match_index)

    def entries(self) -> List[TimestampedValue[T]]:
        """Return a stable timestamp-ordered snapshot of buffered entries."""
        return list(self._entries)

    def remove(self, stamp_ns: int) -> Optional[TimestampedValue[T]]:
        """Remove an entry with exactly ``stamp_ns`` when present."""
        stamp = _integer(stamp_ns, "stamp_ns")
        position = bisect_left(self._stamps, stamp)
        if position >= len(self._stamps) or self._stamps[position] != stamp:
            return None
        del self._stamps[position]
        return self._entries.pop(position)

    def clear(self) -> None:
        self._stamps.clear()
        self._entries.clear()

    def _nearest_index(
        self,
        stamp_ns: int,
        tolerance_ns: int,
    ) -> Optional[int]:
        target = _integer(stamp_ns, "stamp_ns")
        tolerance = _integer(tolerance_ns, "tolerance_ns")
        if tolerance < 0:
            raise ValueError("tolerance_ns must not be negative")
        if not self._stamps:
            return None

        insertion = bisect_left(self._stamps, target)
        candidates = []
        if insertion > 0:
            candidates.append(insertion - 1)
        if insertion < len(self._stamps):
            candidates.append(insertion)

        # stamp is the secondary key, which makes equal-distance matching
        # deterministic and favors the earlier sample.
        best = min(
            candidates,
            key=lambda item: (abs(self._stamps[item] - target), self._stamps[item]),
        )
        if abs(self._stamps[best] - target) > tolerance:
            return None
        return best


def _integer(value, name: str) -> int:
    if isinstance(value, bool):
        raise TypeError("{} must be an integer".format(name))
    try:
        return index(value)
    except TypeError as exc:
        raise TypeError("{} must be an integer".format(name)) from exc
