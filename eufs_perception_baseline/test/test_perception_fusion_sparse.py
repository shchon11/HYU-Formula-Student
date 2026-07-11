import unittest

import numpy as np
from sensor_msgs.msg import CameraInfo

from eufs_perception_baseline.perception_baseline_node import (
    Cluster,
    Detection,
    PerceptionBaselineNode,
)


class PerceptionFusionSparseTest(unittest.TestCase):
    def _node(self):
        node = object.__new__(PerceptionBaselineNode)
        node.roi_min_x = 0.0
        node.roi_max_x = 30.0
        node.roi_abs_y = 15.0
        node.roi_min_z = -1.0
        node.roi_max_z = 2.0
        node.ground_min_z = -1.0
        node.self_mask_enabled = True
        node.self_mask_min_x = -1.5
        node.self_mask_max_x = 1.8
        node.self_mask_abs_y = 0.8
        node.self_mask_min_z = -0.4
        node.self_mask_max_z = 1.4
        node.projection_model = "pinhole"
        node.min_project_depth = 0.2
        node.clip_projected_points_to_image = False
        node.cluster_max_width = 0.90
        node.min_projected_points = 1
        node.sparse_association_enabled = True
        node.sparse_bbox_margin_px = 4.0
        node.sparse_bbox_margin_ratio = 0.15
        node.sparse_near_range_m = 6.0
        node.sparse_far_range_m = 12.0
        node.sparse_near_min_points = 4
        node.sparse_mid_min_points = 2
        node.sparse_far_min_points = 2
        node.sparse_far_min_probability = 0.25
        node.sparse_far_min_bbox_width_px = 4.0
        node.sparse_far_min_bbox_height_px = 4.0
        node.sparse_max_depth_span_m = 1.0
        node.sparse_max_depth_span_ratio = 0.35
        node.sparse_max_width = 0.90
        return node

    def _camera_info(self):
        info = CameraInfo()
        info.width = 100
        info.height = 100
        info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]
        return info

    def test_self_mask_rejects_vehicle_points_but_keeps_forward_points(self):
        node = self._node()
        points_base = np.asarray(
            [
                [1.0, 0.0, 0.5],
                [2.0, 0.0, 0.5],
                [1.0, 1.2, 0.5],
            ],
            dtype=np.float64,
        )

        mask = node._roi_mask(points_base)

        self.assertFalse(mask[0])
        self.assertTrue(mask[1])
        self.assertTrue(mask[2])

    def test_sparse_association_accepts_coherent_near_support(self):
        node = self._node()
        detection = Detection(
            color="blue",
            probability=0.9,
            xmin=45.0,
            ymin=45.0,
            xmax=55.0,
            ymax=55.0,
        )
        points_base = np.asarray(
            [
                [5.0, 0.00, 0.20],
                [5.1, 0.05, 0.22],
                [5.0, -0.05, 0.25],
                [5.2, 0.02, 0.21],
            ],
            dtype=np.float64,
        )
        points_camera = np.asarray(
            [
                [0.00, 0.00, 5.0],
                [0.03, 0.02, 5.1],
                [-0.02, -0.02, 5.0],
                [0.01, 0.00, 5.2],
            ],
            dtype=np.float64,
        )

        assignments = node._associate_sparse_detections(
            [detection],
            points_base,
            points_camera,
            self._camera_info(),
            set(),
            set(),
        )

        self.assertEqual(len(assignments), 1)
        self.assertEqual(assignments[0].source, "sparse")
        self.assertEqual(assignments[0].support_count, 4)
        self.assertAlmostEqual(assignments[0].cluster.centroid_base[0], 5.075)

    def test_sparse_association_accepts_two_point_mid_range_support(self):
        node = self._node()
        detection = Detection(
            color="blue",
            probability=0.9,
            xmin=45.0,
            ymin=45.0,
            xmax=55.0,
            ymax=55.0,
        )
        points_base = np.asarray(
            [
                [6.052, 0.02, 0.20],
                [6.052, -0.02, 0.22],
            ],
            dtype=np.float64,
        )
        points_camera = np.asarray(
            [
                [0.01, 0.00, 6.052],
                [-0.01, 0.00, 6.052],
            ],
            dtype=np.float64,
        )

        assignments = node._associate_sparse_detections(
            [detection],
            points_base,
            points_camera,
            self._camera_info(),
            set(),
            set(),
        )

        self.assertEqual(len(assignments), 1)
        self.assertEqual(assignments[0].source, "sparse")
        self.assertEqual(assignments[0].support_count, 2)
        self.assertGreaterEqual(assignments[0].cluster.range_m, node.sparse_near_range_m)
        self.assertLess(assignments[0].cluster.range_m, node.sparse_far_range_m)
        self.assertAlmostEqual(assignments[0].cluster.range_m, 6.052)

    def test_sparse_association_rejects_zero_support_bbox(self):
        node = self._node()
        detection = Detection(
            color="yellow",
            probability=0.9,
            xmin=80.0,
            ymin=80.0,
            xmax=90.0,
            ymax=90.0,
        )
        points_base = np.asarray(
            [[5.0, 0.0, 0.2], [5.1, 0.1, 0.2], [5.2, -0.1, 0.2], [5.0, 0.2, 0.2]],
            dtype=np.float64,
        )
        points_camera = np.asarray(
            [[0.0, 0.0, 5.0], [0.02, 0.0, 5.1], [-0.02, 0.0, 5.2], [0.0, 0.02, 5.0]],
            dtype=np.float64,
        )

        assignments = node._associate_sparse_detections(
            [detection],
            points_base,
            points_camera,
            self._camera_info(),
            set(),
            set(),
        )

        self.assertEqual(assignments, [])

    def test_sparse_association_deduplicates_shared_support(self):
        node = self._node()
        detections = [
            Detection("orange", 0.6, 45.0, 45.0, 55.0, 55.0),
            Detection("big_orange", 0.9, 44.0, 44.0, 56.0, 56.0),
        ]
        points_base = np.asarray(
            [
                [5.0, 0.00, 0.20],
                [5.1, 0.05, 0.22],
                [5.0, -0.05, 0.25],
                [5.2, 0.02, 0.21],
            ],
            dtype=np.float64,
        )
        points_camera = np.asarray(
            [
                [0.00, 0.00, 5.0],
                [0.03, 0.02, 5.1],
                [-0.02, -0.02, 5.0],
                [0.01, 0.00, 5.2],
            ],
            dtype=np.float64,
        )

        assignments = node._associate_sparse_detections(
            detections,
            points_base,
            points_camera,
            self._camera_info(),
            set(),
            set(),
        )

        self.assertEqual(len(assignments), 1)
        self.assertEqual(assignments[0].detection.color, "big_orange")

    def test_cluster_association_prefers_higher_detection_confidence(self):
        node = self._node()
        points_camera = np.asarray(
            [[0.0, 0.0, 5.0], [0.02, 0.0, 5.0]],
            dtype=np.float64,
        )
        points_base = np.asarray(
            [[5.0, 0.0, 0.2], [5.0, -0.02, 0.2]],
            dtype=np.float64,
        )
        cluster = Cluster(
            points_base=points_base,
            points_camera=points_camera,
            centroid_base=points_base.mean(axis=0),
            range_m=5.0,
            indices=np.asarray([0, 1]),
        )
        detections = [
            Detection("blue", 0.2, 45.0, 45.0, 55.0, 55.0),
            Detection("yellow", 0.9, 45.0, 45.0, 55.0, 55.0),
        ]

        supported_detections = set()
        assignments = node._associate_detections_to_clusters(
            detections,
            [cluster],
            self._camera_info(),
            supported_detection_indices=supported_detections,
        )

        self.assertEqual(len(assignments), 1)
        self.assertEqual(assignments[0].detection.color, "yellow")
        self.assertEqual(supported_detections, {0, 1})

    def test_cluster_assignment_uses_only_bbox_supported_points(self):
        node = self._node()
        points_camera = np.asarray(
            [
                [0.0, 0.0, 5.0],
                [0.1, 0.0, 5.2],
                [2.0, 0.0, 5.0],
            ],
            dtype=np.float64,
        )
        points_base = np.asarray(
            [
                [5.0, 0.0, 0.20],
                [5.2, -0.1, 0.25],
                [5.0, 2.0, 0.20],
            ],
            dtype=np.float64,
        )
        cluster = Cluster(
            points_base=points_base,
            points_camera=points_camera,
            centroid_base=points_base.mean(axis=0),
            range_m=5.0,
            indices=np.asarray([0, 1, 2]),
        )
        detection = Detection("blue", 0.9, 45.0, 45.0, 55.0, 55.0)

        assignment = node._associate_detections_to_clusters(
            [detection],
            [cluster],
            self._camera_info(),
        )[0]

        self.assertEqual(assignment.support_count, 2)
        np.testing.assert_allclose(
            assignment.cluster.centroid_base,
            [5.1, -0.05, 0.225],
        )
        np.testing.assert_allclose(
            assignment.cluster.points_camera[:, 2],
            [5.0, 5.2],
        )
        np.testing.assert_array_equal(
            assignment.cluster.consumed_indices,
            [0, 1, 2],
        )


if __name__ == "__main__":
    unittest.main()
