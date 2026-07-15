"""Stereo halves must be paired by stamp, never by arrival order.

This exists because "keep the newest of each" looked equivalent to "keep a pair"
and is not. No single Python process can deserialise two 2.7 MB image streams at
30 Hz, so the halves reach the node at different effective rates: measured in the
sim, left arrived at 29.41 Hz and right at 15.15 Hz *in the same process*, while
each ran at 29.41 Hz when measured alone. Under latest-wins the node would hold a
left and a right up to 33 ms apart and hand them to ZNCC, which cannot tell ego
motion from depth and would report 0.21 m of the car's own travel as a confident
cone range.
"""
import threading
import unittest

from eufs_perception_baseline.perception_node import PerceptionNode


class _Stamp:
    def __init__(self, ns):
        self.sec, self.nanosec = divmod(ns, 1_000_000_000)


class _Header:
    def __init__(self, ns):
        self.stamp = _Stamp(ns)


class _Image:
    """Just enough Image to be paired: a stamp and an identity."""

    def __init__(self, ns, side):
        self.header = _Header(ns)
        self.side = side

    def __repr__(self):
        return f"{self.side}@{self.header.stamp.nanosec}"


def _node():
    node = object.__new__(PerceptionNode)
    node._lock = threading.Lock()
    node._pending_left = __import__("collections").OrderedDict()
    node._pending_right = __import__("collections").OrderedDict()
    node._pair = None
    node._pair_buffer_size = 8
    return node


def _left(node, ns):
    node._left_image_callback(_Image(ns, "L"))


def _right(node, ns):
    node._right_image_callback(_Image(ns, "R"))


class StereoPairing(unittest.TestCase):
    def test_matching_stamps_form_a_pair_in_left_right_order(self):
        node = _node()
        _left(node, 1_000_000_000)
        self.assertIsNone(node._pair, "a lone left half is not a pair")
        _right(node, 1_000_000_000)
        left, right = node._pair
        self.assertEqual((left.side, right.side), ("L", "R"))

    def test_pair_order_holds_when_right_arrives_first(self):
        node = _node()
        _right(node, 5_000_000_000)
        _left(node, 5_000_000_000)
        left, right = node._pair
        self.assertEqual((left.side, right.side), ("L", "R"),
                         "arrival order must not decide which half is which")

    def test_mismatched_stamps_never_pair(self):
        node = _node()
        _left(node, 1_000_000_000)
        _right(node, 1_033_000_000)          # 33 ms later: a different instant
        self.assertIsNone(
            node._pair,
            "33 ms apart is 0.21 m of ego motion at 6.5 m/s; that is not a pair")

    def test_a_half_finds_its_partner_across_dropped_frames(self):
        # The realistic case: right drops frames, so left runs ahead. The pair
        # for the stamp right *did* deliver must still form.
        node = _node()
        for i in range(4):
            _left(node, 1_000_000_000 + i * 33_000_000)
        self.assertIsNone(node._pair)
        _right(node, 1_000_000_000 + 2 * 33_000_000)
        left, right = node._pair
        self.assertEqual(left.header.stamp.nanosec, right.header.stamp.nanosec)

    def test_pair_rate_holds_up_under_the_worst_case_drop_pattern(self):
        # Adversarial on purpose: each side drops a third, and the drops are
        # perfectly anti-aligned, so a stamp survives only when neither side
        # dropped it. That is the floor, not the typical case -- real drops are
        # not synchronised against each other. Even here 30 Hz of halves yields
        # exactly 10 Hz of pairs, which is the LiDAR's rate, so the output the
        # cloud paces never goes hungry.
        node, pairs = _node(), 0
        for i in range(90):                              # 3 s at 30 Hz
            ns = 1_000_000_000 + i * 33_000_000
            if i % 3 != 0:
                _left(node, ns)
            if i % 3 != 1:
                _right(node, ns)
            if node._pair and self._pair_stamp(node) == ns:
                pairs += 1
                node._pair = None
        self.assertGreaterEqual(pairs, 30, "worst case must still reach 10 Hz")

    @staticmethod
    def _pair_stamp(node):
        stamp = node._pair[0].header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def test_unmatched_halves_do_not_grow_without_bound(self):
        node = _node()
        for i in range(200):                 # right never arrives
            _left(node, 1_000_000_000 + i * 33_000_000)
        self.assertLessEqual(len(node._pending_left), node._pair_buffer_size)

    def test_a_formed_pair_clears_the_backlog(self):
        node = _node()
        for i in range(5):
            _left(node, 1_000_000_000 + i * 33_000_000)
        _right(node, 1_000_000_000 + 4 * 33_000_000)
        self.assertFalse(node._pending_left, "stale halves cannot find partners")
        self.assertFalse(node._pending_right)


if __name__ == "__main__":
    unittest.main()
