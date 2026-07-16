import unittest

from eufs_msgs.msg import ConeArrayWithCovariance, ConeWithCovariance
from visualization_msgs.msg import Marker

from eufs_perception_baseline.perception_baseline_node import PerceptionBaselineNode


class PerceptionMarkerVizTest(unittest.TestCase):
    def _cone(self, x, y):
        cone = ConeWithCovariance()
        cone.point.x = x
        cone.point.y = y
        cone.point.z = 0.0
        return cone

    def test_cone_array_is_converted_to_rviz_marker_array(self):
        cones = ConeArrayWithCovariance()
        cones.header.frame_id = "base_footprint"
        cones.blue_cones.append(self._cone(1.0, 2.0))
        cones.yellow_cones.append(self._cone(3.0, 4.0))

        markers = PerceptionBaselineNode._cones_to_markers(cones).markers

        self.assertEqual(markers[0].action, Marker.DELETEALL)
        self.assertEqual(markers[1].ns, "blue")
        self.assertEqual(markers[1].type, Marker.SPHERE_LIST)
        self.assertEqual(markers[1].points[0].x, 1.0)
        self.assertEqual(markers[1].points[0].y, 2.0)
        self.assertEqual(markers[1].points[0].z, 0.125)
        self.assertEqual(markers[2].ns, "yellow")
        self.assertEqual(markers[2].points[0].x, 3.0)
        self.assertEqual(markers[2].points[0].y, 4.0)
        self.assertEqual(markers[1].header.frame_id, "base_footprint")

    def test_cone_marker_scale_can_be_configured(self):
        cones = ConeArrayWithCovariance()
        cones.header.frame_id = "base_footprint"
        cones.big_orange_cones.append(self._cone(1.0, -1.0))

        markers = PerceptionBaselineNode._cones_to_markers(
            cones,
            marker_scale=0.5,
        ).markers

        self.assertEqual(markers[4].scale.x, 0.5)
        self.assertEqual(markers[4].scale.y, 0.5)
        self.assertEqual(markers[4].scale.z, 0.5)
        self.assertEqual(markers[4].points[0].z, 0.25)

    def test_viz_topic_tracks_output_cone_topic(self):
        self.assertEqual(
            PerceptionBaselineNode._viz_topic_for("/cones"),
            "/cones/viz",
        )
        self.assertEqual(
            PerceptionBaselineNode._viz_topic_for("/debug/cones/"),
            "/debug/cones/viz",
        )
