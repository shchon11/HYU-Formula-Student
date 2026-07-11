import sys
import unittest
from types import SimpleNamespace

import numpy as np
from std_msgs.msg import Header

from eufs_perception_baseline.ros_image_utils import (
    image_message_to_numpy,
    numpy_to_image_message,
)


class RosImageUtilsTest(unittest.TestCase):
    def _message(self, array, encoding, step=None, is_bigendian=0):
        return SimpleNamespace(
            height=array.shape[0],
            width=array.shape[1],
            encoding=encoding,
            is_bigendian=is_bigendian,
            step=step if step is not None else array.strides[0],
            data=array.tobytes(),
        )

    def test_bgr8_round_trip_without_cv_bridge(self):
        array = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
        converted = image_message_to_numpy(self._message(array, "bgr8"), "bgr8")
        np.testing.assert_array_equal(converted, array)

    def test_rgb8_to_bgr8(self):
        rgb = np.asarray([[[1, 2, 3]]], dtype=np.uint8)
        converted = image_message_to_numpy(self._message(rgb, "rgb8"), "bgr8")
        np.testing.assert_array_equal(converted, [[[3, 2, 1]]])

    def test_mono8_conversion(self):
        mono = np.asarray([[10, 20]], dtype=np.uint8)
        converted = image_message_to_numpy(self._message(mono, "mono8"), "mono8")
        np.testing.assert_array_equal(converted, mono)

    def test_row_padding_is_removed(self):
        raw = np.asarray([[1, 2, 3, 4], [5, 6, 7, 8]], dtype=np.uint8)
        msg = SimpleNamespace(
            height=2,
            width=3,
            encoding="mono8",
            is_bigendian=0,
            step=4,
            data=raw.tobytes(),
        )
        np.testing.assert_array_equal(
            image_message_to_numpy(msg, "mono8"),
            [[1, 2, 3], [5, 6, 7]],
        )

    def test_unsupported_encoding_is_rejected(self):
        msg = SimpleNamespace(
            height=1,
            width=1,
            encoding="32FC1",
            is_bigendian=int(sys.byteorder == "little"),
            step=4,
            data=b"\0\0\0\0",
        )
        with self.assertRaises(ValueError):
            image_message_to_numpy(msg, "bgr8")

    def test_numpy_to_ros_image_round_trip(self):
        array = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        header = Header()
        message = numpy_to_image_message(array, header)
        self.assertEqual(message.encoding, "bgr8")
        self.assertEqual(message.step, 9)
        np.testing.assert_array_equal(
            image_message_to_numpy(message, "bgr8"),
            array,
        )


if __name__ == "__main__":
    unittest.main()
