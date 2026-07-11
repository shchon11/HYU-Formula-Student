import unittest

from eufs_perception_baseline.sync_buffer import TimestampBuffer


class TimestampBufferTest(unittest.TestCase):
    def test_nearest_handles_out_of_order_input(self):
        buffer = TimestampBuffer(maxlen=4)
        buffer.add(10_040_000_000, "late")
        buffer.add(10_000_000_000, "exact")
        buffer.add(10_010_000_000, "near")

        match = buffer.nearest(10_008_000_000, 50_000_000)

        self.assertEqual(match.stamp_ns, 10_010_000_000)
        self.assertEqual(match.value, "near")

    def test_tolerance_boundary_is_inclusive(self):
        buffer = TimestampBuffer(maxlen=2)
        buffer.add(1_000_000_000, "value")
        self.assertIsNotNone(buffer.nearest(1_050_000_000, 50_000_000))
        self.assertIsNone(buffer.nearest(1_050_000_001, 50_000_000))

    def test_pop_nearest_consumes_message_once(self):
        buffer = TimestampBuffer(maxlen=2)
        buffer.add(1_000_000_000, "value")
        self.assertEqual(
            buffer.pop_nearest(1_000_000_000, 1).value,
            "value",
        )
        self.assertIsNone(buffer.pop_nearest(1_000_000_000, 1))

    def test_duplicate_stamp_replaces_stale_value(self):
        buffer = TimestampBuffer(maxlen=2)
        buffer.add(1, "old")
        buffer.add(1, "new")
        self.assertEqual(buffer.nearest(1, 0).value, "new")
        self.assertEqual(len(buffer), 1)

    def test_entries_snapshot_and_exact_remove(self):
        buffer = TimestampBuffer(maxlen=3)
        buffer.add(3, "three")
        buffer.add(1, "one")
        self.assertEqual([entry.stamp_ns for entry in buffer.entries()], [1, 3])
        self.assertEqual(buffer.remove(1).value, "one")
        self.assertIsNone(buffer.remove(1))


if __name__ == "__main__":
    unittest.main()
