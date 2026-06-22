import math
import struct
from dataclasses import dataclass
import unittest

from eufs_perception_baseline.point_cloud2_compat import read_points


@dataclass
class Field:
    name: str
    offset: int
    datatype: int
    count: int = 1


@dataclass
class Cloud:
    fields: list
    data: bytes
    width: int
    height: int = 1
    point_step: int = 12
    is_bigendian: bool = False


class PointCloud2CompatTest(unittest.TestCase):
    def test_read_points_xyz_skips_nans(self):
        cloud = Cloud(
            fields=[
                Field("x", 0, 7),
                Field("y", 4, 7),
                Field("z", 8, 7),
            ],
            width=2,
            data=struct.pack("<ffffff", 1.0, 2.0, 3.0, math.nan, 5.0, 6.0),
        )

        points = list(
            read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
        )

        self.assertEqual(points, [(1.0, 2.0, 3.0)])

    def test_read_points_reports_missing_fields(self):
        cloud = Cloud(
            fields=[Field("x", 0, 7)],
            width=1,
            data=struct.pack("<f", 1.0),
        )

        with self.assertRaisesRegex(ValueError, "missing requested fields"):
            list(read_points(cloud, field_names=("x", "y")))
