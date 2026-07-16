import tempfile
import unittest

import numpy as np

from eufs_perception_baseline.rektnet import (
    REKTNET_KEYPOINT_NAMES,
    PnPPose,
    RektNet,
    RektNetInference,
    RektNetPrediction,
    bbox_from_keypoints,
    cone_keypoint_template,
    estimate_rektnet_stereo_depth,
    prepare_rektnet_crop,
    project_keypoints_to_right,
    rectified_right_from_left,
    solve_rektnet_pnp,
)


try:
    import cv2
except ImportError:
    cv2 = None

try:
    import torch
except ImportError:
    torch = None


class _FixedEstimator:
    def __init__(self, prediction):
        self.prediction = prediction

    def predict(self, _image, _bbox):
        return self.prediction


class RektNetArchitectureTest(unittest.TestCase):
    @unittest.skipUnless(torch is not None, "PyTorch is unavailable")
    def test_reference_architecture_outputs_seven_normalized_keypoints(self):
        model = RektNet().eval()
        with torch.inference_mode():
            heatmaps, points = model(torch.zeros((2, 3, 80, 80)))

        self.assertEqual(tuple(heatmaps.shape), (2, 7, 80, 80))
        self.assertEqual(tuple(points.shape), (2, 7, 2))
        np.testing.assert_allclose(
            heatmaps.reshape(2, 7, -1).sum(dim=2).numpy(),
            np.ones((2, 7)),
            atol=1.0e-5,
        )
        self.assertEqual(
            REKTNET_KEYPOINT_NAMES,
            (
                "top",
                "mid_L_top",
                "mid_R_top",
                "mid_L_bot",
                "mid_R_bot",
                "bot_L",
                "bot_R",
            ),
        )

    @unittest.skipUnless(torch is not None, "PyTorch is unavailable")
    def test_public_checkpoint_dictionary_contract_loads_strictly(self):
        model = RektNet()
        with tempfile.NamedTemporaryFile(suffix=".pt") as checkpoint:
            torch.save({"model": model.state_dict()}, checkpoint.name)
            estimator = RektNetInference(checkpoint.name, "cpu")

        self.assertEqual(estimator.device.type, "cpu")

    def test_crop_matches_public_direct_resize_contract(self):
        image = np.zeros((30, 40, 3), dtype=np.uint8)
        image[5:25, 10:30, 2] = 200
        prepared = prepare_rektnet_crop(image, (10.2, 5.2, 29.8, 24.8))

        self.assertIsNotNone(prepared)
        crop, bbox = prepared
        self.assertEqual(crop.shape, (80, 80, 3))
        self.assertEqual(bbox, (10, 5, 30, 25))
        self.assertEqual(int(crop[40, 40, 2]), 200)


@unittest.skipUnless(cv2 is not None, "OpenCV is unavailable")
class RektNetGeometryTest(unittest.TestCase):
    @staticmethod
    def _camera_matrix():
        return np.asarray(
            [[400.0, 0.0, 190.0], [0.0, 400.0, 90.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    @staticmethod
    def _template():
        return cone_keypoint_template(1.5, 0.6)

    @staticmethod
    def _pose():
        return PnPPose(
            rotation_vector=np.zeros((3, 1), dtype=np.float64),
            translation_vector=np.asarray([[0.0], [-0.75], [6.0]]),
            reprojection_error_px=0.0,
            used_keypoint_indices=tuple(range(7)),
        )

    def _left_points(self):
        points, _ = cv2.projectPoints(
            self._template(),
            self._pose().rotation_vector,
            self._pose().translation_vector,
            self._camera_matrix(),
            np.zeros(5),
        )
        return points.reshape(-1, 2)

    def test_parameterized_template_has_public_semantic_topology(self):
        template = cone_keypoint_template(0.3, 0.12)

        np.testing.assert_allclose(template[0], [0.0, 0.0, 0.0])
        np.testing.assert_allclose(template[1], [-0.04, 0.1, 0.0])
        np.testing.assert_allclose(template[2], [0.04, 0.1, 0.0])
        np.testing.assert_allclose(template[5], [-0.12, 0.3, 0.0])
        np.testing.assert_allclose(template[6], [0.12, 0.3, 0.0])

    def test_pnp_recovers_a_perfect_seven_point_projection(self):
        pose = solve_rektnet_pnp(
            self._template(),
            self._left_points(),
            self._camera_matrix(),
            np.zeros(5),
            max_reprojection_error_px=0.1,
        )

        self.assertIsNotNone(pose)
        self.assertEqual(len(pose.used_keypoint_indices), 7)
        self.assertLess(pose.reprojection_error_px, 1.0e-5)

    def test_planar_pnp_selects_right_projection_consistent_pose(self):
        rotation = np.asarray([[0.30], [-0.20], [0.08]], dtype=np.float64)
        translation = np.asarray([[0.15], [-0.70], [6.0]], dtype=np.float64)
        left_points, _ = cv2.projectPoints(
            self._template(),
            rotation,
            translation,
            self._camera_matrix(),
            np.zeros(5),
        )
        pose = solve_rektnet_pnp(
            self._template(),
            left_points.reshape(-1, 2),
            self._camera_matrix(),
            np.zeros(5),
            max_reprojection_error_px=0.1,
        )
        right_from_left = np.eye(4, dtype=np.float64)
        right_from_left[0, 3] = -0.12
        self.assertIsNotNone(pose)
        estimated_right = project_keypoints_to_right(
            self._template(),
            pose,
            right_from_left,
            self._camera_matrix(),
            np.zeros(5),
        )
        expected_right, _ = cv2.projectPoints(
            self._template(),
            rotation,
            translation + np.asarray([[-0.12], [0.0], [0.0]]),
            self._camera_matrix(),
            np.zeros(5),
        )
        np.testing.assert_allclose(
            estimated_right,
            expected_right.reshape(-1, 2),
            atol=1.0e-4,
        )

    def test_pnp_drops_one_bad_keypoint_as_public_robustness_rule(self):
        points = self._left_points()
        points[4] += np.asarray([30.0, -20.0])
        pose = solve_rektnet_pnp(
            self._template(),
            points,
            self._camera_matrix(),
            np.zeros(5),
            max_reprojection_error_px=0.5,
        )

        self.assertIsNotNone(pose)
        self.assertEqual(len(pose.used_keypoint_indices), 6)
        self.assertNotIn(4, pose.used_keypoint_indices)
        self.assertLess(pose.reprojection_error_px, 0.5)

    def test_right_projection_uses_negative_rectified_baseline(self):
        left_projection = [
            400.0, 0.0, 190.0, 0.0,
            0.0, 400.0, 90.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        right_projection = [
            400.0, 0.0, 190.0, -48.0,
            0.0, 400.0, 90.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        transform = rectified_right_from_left(
            left_projection,
            right_projection,
        )
        right_points = project_keypoints_to_right(
            self._template(),
            self._pose(),
            transform,
            self._camera_matrix(),
            np.zeros(5),
        )

        self.assertAlmostEqual(transform[0, 3], -0.12)
        np.testing.assert_allclose(
            self._left_points()[:, 0] - right_points[:, 0],
            np.full(7, 8.0),
            atol=1.0e-8,
        )
        self.assertIsNotNone(bbox_from_keypoints(right_points, (380, 180)))

    @unittest.skipUnless(
        cv2 is not None and hasattr(cv2, "SIFT_create"),
        "OpenCV build does not provide SIFT",
    )
    def test_full_stage_orders_pnp_projection_before_sift_disparity(self):
        rng = np.random.default_rng(31)
        texture = rng.integers(0, 256, size=(104, 84), dtype=np.uint8)
        texture = cv2.GaussianBlur(texture, (3, 3), 0)
        left = np.zeros((180, 380, 3), dtype=np.uint8)
        right = np.zeros_like(left)
        left[38:142, 148:232] = texture[:, :, np.newaxis]
        right[38:142, 140:224] = texture[:, :, np.newaxis]
        prediction = RektNetPrediction(
            points=self._left_points(),
            heatmap_peaks=np.ones(7, dtype=np.float64),
        )
        transform = np.eye(4, dtype=np.float64)
        transform[0, 3] = -0.12

        result = estimate_rektnet_stereo_depth(
            _FixedEstimator(prediction),
            left,
            right,
            (148, 38, 232, 142),
            self._template(),
            self._camera_matrix(),
            np.zeros(5),
            self._camera_matrix(),
            np.zeros(5),
            transform,
            baseline_m=0.12,
            min_depth_m=2.0,
            max_depth_m=15.0,
            max_reprojection_error_px=0.5,
            right_roi_padding_ratio=0.08,
            epipolar_tolerance_px=1.0,
            slender_fraction=0.5,
            ratio_threshold=0.75,
            min_matches=1,
        )

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.depth.depth_m, 6.0, delta=0.35)
        self.assertEqual(result.depth.match_count, 1)
        self.assertEqual(result.pnp_keypoint_count, 7)


if __name__ == "__main__":
    unittest.main()
