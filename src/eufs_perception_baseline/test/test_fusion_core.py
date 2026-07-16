import math
import unittest

import numpy as np

from eufs_perception_baseline.fusion_core import (
    baseline_from_projection,
    camera_point_from_depth,
    classify_cone_condition,
    disparity_to_depth,
    estimate_stereo_depth,
    monocular_depth_from_bbox,
    remove_ground_ransac,
    slender_bbox,
)


def _sift_available():
    try:
        import cv2
    except ImportError:
        return False
    return callable(getattr(cv2, "SIFT_create", None))


class GroundPlaneTest(unittest.TestCase):
    def test_ransac_removes_sloped_ground_and_preserves_cone(self):
        rng = np.random.default_rng(7)
        ground_xy = rng.uniform([-2.0, -4.0], [15.0, 4.0], size=(500, 2))
        ground_z = (
            0.06 * ground_xy[:, 0]
            + 0.015 * ground_xy[:, 1]
            + rng.normal(0.0, 0.003, size=ground_xy.shape[0])
        )
        ground = np.column_stack((ground_xy, ground_z))

        cone_z = np.linspace(0.06, 0.42, 12)
        cone_xy = np.column_stack(
            (
                np.full(cone_z.shape, 6.0) + rng.normal(0.0, 0.025, cone_z.shape),
                np.full(cone_z.shape, 0.5) + rng.normal(0.0, 0.025, cone_z.shape),
            )
        )
        local_ground = 0.06 * cone_xy[:, 0] + 0.015 * cone_xy[:, 1]
        cone = np.column_stack((cone_xy, local_ground + cone_z))
        points = np.vstack((ground, cone))

        result = remove_ground_ransac(
            points,
            distance_threshold=0.03,
            max_iterations=200,
            max_tilt_degrees=20.0,
            min_inliers=100,
            seed=7,
        )

        self.assertIsNotNone(result.plane)
        self.assertGreaterEqual(np.mean(result.ground_mask[: len(ground)]), 0.95)
        self.assertTrue(np.all(result.non_ground_mask[len(ground):]))

    def test_ransac_rejects_vertical_wall_even_when_wall_has_more_points(self):
        rng = np.random.default_rng(11)
        ground_xy = rng.uniform([-2.0, -4.0], [12.0, 4.0], size=(220, 2))
        ground = np.column_stack(
            (ground_xy, rng.normal(0.0, 0.002, size=ground_xy.shape[0]))
        )
        wall_yz = rng.uniform([-3.0, 0.0], [3.0, 2.0], size=(400, 2))
        wall = np.column_stack(
            (np.full(400, 8.0) + rng.normal(0.0, 0.002, 400), wall_yz)
        )

        result = remove_ground_ransac(
            np.vstack((ground, wall)),
            distance_threshold=0.02,
            max_iterations=300,
            max_tilt_degrees=15.0,
            min_inliers=100,
            seed=11,
        )

        self.assertIsNotNone(result.plane)
        normal = result.plane[:3]
        tilt = math.degrees(math.acos(float(np.clip(normal[2], -1.0, 1.0))))
        self.assertLessEqual(tilt, 15.0)
        self.assertGreaterEqual(np.mean(result.ground_mask[: len(ground)]), 0.95)
        self.assertLess(np.mean(result.ground_mask[len(ground):]), 0.10)

    def test_degenerate_input_fails_open_and_is_deterministic(self):
        points = np.asarray([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]])
        result = remove_ground_ransac(points, seed=3)
        self.assertIsNone(result.plane)
        self.assertTrue(np.all(result.non_ground_mask))
        self.assertFalse(np.any(result.ground_mask))


class MonocularDepthTest(unittest.TestCase):
    def test_bbox_height_curve_uses_normalized_image_height(self):
        depth = monocular_depth_from_bbox(72.0, 720.0)
        self.assertAlmostEqual(depth, 0.498 * (0.1 ** -0.954), places=9)

    def test_curve_is_resolution_invariant(self):
        self.assertAlmostEqual(
            monocular_depth_from_bbox(72.0, 720.0),
            monocular_depth_from_bbox(108.0, 1080.0),
            places=9,
        )

    def test_invalid_height_returns_none(self):
        self.assertIsNone(monocular_depth_from_bbox(0.0, 720.0))
        self.assertIsNone(monocular_depth_from_bbox(10.0, 0.0))

    def test_camera_ray_is_back_projected_at_optical_depth(self):
        point = camera_point_from_depth(
            60.0,
            45.0,
            5.0,
            np.asarray([[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0, 0, 1]]),
        )
        np.testing.assert_allclose(point, [0.5, -0.25, 5.0])

    def test_good_horizontal_clip_bad_policy_is_explicit(self):
        self.assertEqual(
            classify_cone_condition((40, 20, 60, 80), (100, 100)),
            "good",
        )
        self.assertEqual(
            classify_cone_condition((20, 40, 80, 60), (100, 100)),
            "bad",
        )
        self.assertEqual(
            classify_cone_condition((0, 20, 20, 80), (100, 100)),
            "horizontal_clip",
        )
        self.assertEqual(
            classify_cone_condition((40, 0, 60, 60), (100, 100)),
            "bad",
        )


class StereoDepthTest(unittest.TestCase):
    def test_disparity_to_depth(self):
        self.assertAlmostEqual(disparity_to_depth(8.0, 400.0, 0.12), 6.0)
        self.assertIsNone(disparity_to_depth(0.0, 400.0, 0.12))

    def test_baseline_is_derived_from_projection_matrices(self):
        left = [400.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        right = [400.0, 0.0, 320.0, -48.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.assertAlmostEqual(baseline_from_projection(left, right), 0.12)

    def test_slender_bbox_excludes_outer_edges(self):
        self.assertEqual(slender_bbox((10, 20, 90, 100), 0.5), (30, 20, 70, 100))

    @unittest.skipUnless(_sift_available(), "OpenCV build does not provide SIFT")
    def test_feature_stereo_recovers_known_horizontal_shift(self):
        import cv2

        rng = np.random.default_rng(21)
        patch = rng.integers(0, 256, size=(100, 80), dtype=np.uint8)
        patch = cv2.GaussianBlur(patch, (3, 3), 0)
        left = np.zeros((180, 300), dtype=np.uint8)
        right = np.zeros_like(left)
        left[40:140, 150:230] = patch
        right[40:140, 142:222] = patch

        estimate = estimate_stereo_depth(
            left,
            right,
            (150, 40, 230, 140),
            (142, 40, 222, 140),
            fx=400.0,
            baseline_m=0.12,
            min_depth_m=2.0,
            max_depth_m=15.0,
            epipolar_tolerance_px=1.0,
        )

        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(estimate.depth_m, 6.0, delta=0.35)
        self.assertGreaterEqual(estimate.match_count, 1)

    @unittest.skipUnless(_sift_available(), "OpenCV build does not provide SIFT")
    def test_feature_stereo_corrects_principal_point_offset(self):
        import cv2

        rng = np.random.default_rng(22)
        patch = rng.integers(0, 256, size=(100, 80), dtype=np.uint8)
        patch = cv2.GaussianBlur(patch, (3, 3), 0)
        left = np.zeros((180, 300), dtype=np.uint8)
        right = np.zeros_like(left)
        # Raw shift is 10 px, but cx_left - cx_right is 2 px.  The corrected
        # disparity is therefore 8 px and the optical depth is 6 m.
        left[40:140, 150:230] = patch
        right[40:140, 140:220] = patch

        estimate = estimate_stereo_depth(
            left,
            right,
            (150, 40, 230, 140),
            (140, 40, 220, 140),
            fx=400.0,
            baseline_m=0.12,
            min_depth_m=2.0,
            max_depth_m=15.0,
            epipolar_tolerance_px=1.0,
            principal_point_offset_px=2.0,
        )

        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(estimate.disparity_px, 8.0, delta=0.4)
        self.assertAlmostEqual(estimate.depth_m, 6.0, delta=0.35)


if __name__ == "__main__":
    unittest.main()
