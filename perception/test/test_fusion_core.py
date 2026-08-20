import math
import unittest

import numpy as np

from hyu_perception.fusion_core import (
    remove_ground_polar_grid,
    bbox_height_disparity_prior,
    bearing_aligned_covariance,
    camera_point_from_depth,
    monocular_depth_from_bbox,
    monocular_relative_depth_sigma,
    remove_ground_ransac,
    stereo_relative_depth_sigma,
    zncc_disparity,
)


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


class VisionCovarianceTest(unittest.TestCase):
    """The error model behind the tiers' reported covariance.

    A vision cone's error is an ellipse along the line of sight: weak in depth,
    strong in bearing.  These check the two sigmas and the rotation that puts
    them on the right axes.
    """

    def test_monocular_sigma_is_the_curve_derivative_and_ignores_coefficient(self):
        # sigma_D/D = |e| * sigma_h / h, straight out of D = c*h_n^e.
        self.assertAlmostEqual(
            monocular_relative_depth_sigma(10.0, -0.7555, 1.5),
            0.7555 * 1.5 / 10.0,
            places=12,
        )

    def test_monocular_sigma_grows_as_the_box_shrinks(self):
        near = monocular_relative_depth_sigma(48.0, -0.7555, 1.5)
        far = monocular_relative_depth_sigma(12.0, -0.7555, 1.5)
        # A quarter of the pixel height is four times the fractional error --
        # which is exactly the far-cone problem the ellipse exists to report.
        self.assertAlmostEqual(far / near, 4.0, places=9)

    def test_stereo_sigma_is_linear_in_depth(self):
        self.assertAlmostEqual(
            stereo_relative_depth_sigma(15.0, 448.13, 0.12, 0.25),
            15.0 * 0.25 / (448.13 * 0.12),
            places=12,
        )
        self.assertAlmostEqual(
            stereo_relative_depth_sigma(30.0, 448.13, 0.12, 0.25)
            / stereo_relative_depth_sigma(15.0, 448.13, 0.12, 0.25),
            2.0,
            places=9,
        )

    def test_invalid_inputs_return_none(self):
        self.assertIsNone(monocular_relative_depth_sigma(0.0, -0.7555, 1.5))
        self.assertIsNone(monocular_relative_depth_sigma(10.0, float("nan"), 1.5))
        self.assertIsNone(stereo_relative_depth_sigma(15.0, 448.0, 0.0, 0.25))
        self.assertIsNone(stereo_relative_depth_sigma(-1.0, 448.0, 0.12, 0.25))
        self.assertIsNone(bearing_aligned_covariance(1.0, 0.0, -1.0, 0.1))
        self.assertIsNone(bearing_aligned_covariance(float("inf"), 0.0, 1.0, 0.1))

    def test_cone_straight_ahead_puts_depth_error_on_x(self):
        xx, xy, yx, yy = bearing_aligned_covariance(14.0, 0.0, 1.6, 0.05)
        self.assertAlmostEqual(xx, 1.6 ** 2, places=12)
        self.assertAlmostEqual(yy, 0.05 ** 2, places=12)
        self.assertAlmostEqual(xy, 0.0, places=12)
        self.assertEqual(xy, yx)

    def test_cone_abeam_swaps_the_axes(self):
        # At 90 deg the line of sight is +y, so the weak axis must move to yy.
        xx, _xy, _yx, yy = bearing_aligned_covariance(0.0, 14.0, 1.6, 0.05)
        self.assertAlmostEqual(xx, 0.05 ** 2, places=12)
        self.assertAlmostEqual(yy, 1.6 ** 2, places=12)

    def test_off_axis_cone_needs_the_off_diagonal(self):
        # The whole reason an axis-aligned matrix cannot express this: at 45 deg
        # the ellipse's major axis lies on neither x nor y.
        xx, xy, _yx, yy = bearing_aligned_covariance(10.0, 10.0, 1.6, 0.05)
        self.assertAlmostEqual(xx, yy, places=12)
        self.assertGreater(abs(xy), 0.5)
        # Rotation preserves the eigenvalues, so the ellipse is the same shape.
        trace, det = xx + yy, xx * yy - xy * xy
        self.assertAlmostEqual(trace, 1.6 ** 2 + 0.05 ** 2, places=12)
        self.assertAlmostEqual(det, (1.6 * 0.05) ** 2, places=12)

    def test_covariance_is_positive_definite_at_every_bearing(self):
        for degrees in range(-180, 180, 7):
            theta = math.radians(degrees)
            x, y = 12.0 * math.cos(theta), 12.0 * math.sin(theta)
            xx, xy, _yx, yy = bearing_aligned_covariance(x, y, 1.6, 0.05)
            self.assertGreater(xx, 0.0)
            self.assertGreater(xx * yy - xy * xy, 0.0, f"singular at {degrees} deg")

    def test_origin_falls_back_to_isotropic(self):
        # atan2(0, 0) would otherwise pick an arbitrary axis for the ellipse.
        xx, xy, _yx, yy = bearing_aligned_covariance(0.0, 0.0, 1.6, 0.05)
        self.assertAlmostEqual(xx, 1.6 ** 2, places=12)
        self.assertAlmostEqual(yy, 1.6 ** 2, places=12)
        self.assertAlmostEqual(xy, 0.0, places=12)


class ZnccDisparityTest(unittest.TestCase):
    """The stereo cross-check that replaced per-crop SIFT.

    Its job is not precision for its own sake. Monocular depth depends on the
    cone's assumed size and ZNCC depends on matching, so they fail differently:
    a correlation peak near the disparity the box implies is physical evidence
    that something geometrically consistent is there, and background texture
    cannot produce it. These check both halves -- that it finds a real cone, and
    that it refuses when there is nothing to find.
    """

    @staticmethod
    def _pair(disparity, width=400, height=200, x=200, y=80, box_h=40, seed=0):
        """A textured patch on textured background, shifted by `disparity`."""
        rng = np.random.default_rng(seed)
        box_w = int(box_h / 2.5)
        patch = rng.normal(200, 30, (box_h, box_w))
        left = rng.normal(120, 12, (height, width))
        left[y:y + box_h, x:x + box_w] = patch
        right = rng.normal(120, 12, (height, width))
        shifted = x - int(round(disparity))
        right[y:y + box_h, shifted:shifted + box_w] = patch
        return left, right, (x, y, x + box_w, y + box_h)

    def test_recovers_a_known_disparity(self):
        for truth in (2, 3, 6, 10, 15):
            with self.subTest(disparity=truth):
                left, right, bbox = self._pair(truth)
                match = zncc_disparity(left, right, bbox, max(0, truth - 4),
                                       truth + 4)
                self.assertIsNotNone(match)
                # Sub-pixel matters more than it looks: at 15 m the whole
                # disparity is ~2.5 px, so the parabola IS the measurement.
                self.assertAlmostEqual(match.disparity_px, truth, delta=0.1)

    def test_the_prior_needs_no_distance_model(self):
        # h_px = fy*H/D and d = fx*B/D share the D, so it cancels: the prior is
        # similar triangles, independent of the fitted curve it checks.
        fx = fy = 448.13
        baseline, cone_height = 0.12, 0.450
        for depth in (5.0, 10.0, 15.0):
            with self.subTest(depth=depth):
                height_px = fy * cone_height / depth
                self.assertAlmostEqual(
                    bbox_height_disparity_prior(height_px, baseline, cone_height),
                    fx * baseline / depth,
                    places=9,
                )

    def test_prior_is_the_baseline_over_the_cone(self):
        # 0.12/0.450 = 0.267 per pixel of box height for THIS car. A 325 mm
        # competition cone would give 0.369, which is 40 % wrong here.
        self.assertAlmostEqual(
            bbox_height_disparity_prior(100.0, 0.12, 0.450), 26.667, places=3)

    def test_a_flat_patch_has_nothing_to_match_and_is_refused(self):
        # It would correlate with everything equally; returning the argmax of
        # numerical noise would be worse than returning nothing.
        flat = np.full((200, 400), 128.0)
        self.assertIsNone(zncc_disparity(flat, flat.copy(), (100, 80, 120, 120),
                                         0, 12))

    def test_background_texture_cannot_fake_a_cone(self):
        # This is the false-positive guard: nothing geometrically consistent
        # exists at any disparity in the window, so there must be no peak.
        rng = np.random.default_rng(3)
        left = rng.normal(120, 12, (200, 400))
        right = rng.normal(120, 12, (200, 400))
        self.assertIsNone(zncc_disparity(left, right, (180, 80, 200, 120),
                                         0, 12, min_score=0.5))

    def test_a_window_off_the_image_is_refused_not_wrapped(self):
        left, right, _bbox = self._pair(2)
        # A box at the left edge cannot be shifted further left.
        self.assertIsNone(
            zncc_disparity(left, right, (1, 80, 9, 120), 20, 40))

    def test_invalid_inputs_return_none(self):
        left, right, bbox = self._pair(4)
        self.assertIsNone(zncc_disparity(left[:, :10], right, bbox, 0, 8))
        self.assertIsNone(zncc_disparity(left, right, bbox, -1.0, 8.0))
        self.assertIsNone(zncc_disparity(left, right, bbox, 8.0, 2.0))
        self.assertIsNone(zncc_disparity(left, right, (5, 5, 5, 5), 0, 8))


class PolarGridGroundTest(unittest.TestCase):
    """Local per-cell floor: ground that is not one plane (slope, undulation,
    body roll) is removed everywhere, cones on it survive."""

    @staticmethod
    def _ground(fn, n=6000, seed=0):
        rng = np.random.default_rng(seed)
        r = rng.uniform(1.5, 30.0, n)
        th = rng.uniform(-math.pi / 2, math.pi / 2, n)
        x, y = r * np.cos(th), r * np.sin(th)
        z = fn(x, y) + rng.normal(0.0, 0.01, n)
        return np.column_stack([x, y, z])

    @staticmethod
    def _cone(cx, cy, ground_z, n=12, seed=1):
        rng = np.random.default_rng(seed)
        h = rng.uniform(0.02, 0.32, n)
        rad = 0.11 * (1.0 - h / 0.32)
        ang = rng.uniform(0, 2 * math.pi, n)
        return np.column_stack([cx + rad * np.cos(ang), cy + rad * np.sin(ang), ground_z + h])

    def _check(self, ground_fn):
        ground = self._ground(ground_fn)
        cones = [self._cone(6.0, 1.5, ground_fn(6.0, 1.5)),
                 self._cone(15.0, -3.0, ground_fn(15.0, -3.0), seed=2),
                 self._cone(24.0, 4.0, ground_fn(24.0, 4.0), seed=3)]
        pts = np.vstack([ground] + cones)
        mask = remove_ground_polar_grid(pts, cell_range_m=1.0, cell_angle_deg=5.0,
                                        height_margin_m=0.05, height_slope_per_m=0.004)
        n_ground = ground.shape[0]
        ground_kept = mask[:n_ground].mean()
        self.assertLess(ground_kept, 0.02, f"ground leaked: {ground_kept:.3f}")
        off = n_ground
        for cone in cones:
            kept = mask[off:off + cone.shape[0]]
            tall = cone[:, 2] - ground_fn(cone[0, 0], cone[0, 1]) > 0.14  # above margin at 30 m
            self.assertTrue(kept[tall].all(), "cone body removed")
            self.assertGreaterEqual(kept.sum(), 4)
            off += cone.shape[0]

    def test_sloped_and_undulating_ground(self):
        # 1.5 deg pitch + 2 deg roll + a 10 cm swell every 12 m: a single plane
        # cannot fit this; the RANSAC slab left 5-25 cm of it "non-ground".
        self._check(lambda x, y: 0.026 * x + 0.035 * y + 0.10 * np.sin(x / 12.0 * 2 * math.pi))

    def test_flat_ground(self):
        self._check(lambda x, y: np.zeros_like(x) if isinstance(x, np.ndarray) else 0.0)

    def test_empty_and_degenerate_inputs(self):
        self.assertEqual(remove_ground_polar_grid(np.empty((0, 3))).shape, (0,))
        one = remove_ground_polar_grid(np.array([[3.0, 0.0, 0.2]]))
        self.assertEqual(one.shape, (1,))
        self.assertFalse(one[0])  # alone in its cell it IS the floor
