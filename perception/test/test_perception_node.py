"""The node's policy: what gets published, from what, and how much it is trusted.

The maths lives in test_fusion_core. What is checked here is the part that is a
*decision* rather than a calculation -- the priority order, the gates that stop a
box from fusing whatever lies behind it, and the covariance handed to SLAM.
"""
import math
import unittest

import numpy as np

from hyu_perception.perception_node import (
    Cone,
    Detection,
    PerceptionNode,
    PROV_CLUSTER,
    PROV_CLUSTER_ONLY,
    PROV_MONOCULAR,
    PROV_SPARSE,
    Scene,
)


def _node(**overrides):
    """A node with the shipped defaults, without a ROS context."""
    node = object.__new__(PerceptionNode)
    node.sparse_enabled = True
    node.sparse_near_range_m, node.sparse_far_range_m = 8.0, 15.0
    node.sparse_near_min_points, node.sparse_mid_min_points = 3, 2
    node.sparse_far_min_points = 2
    node.sparse_max_width_m = 0.90
    node.sparse_max_depth_span_m, node.sparse_max_depth_span_ratio = 1.0, 0.35
    node.sparse_far_min_probability = 0.5
    node.sparse_far_min_bbox_width_px = 4.0
    node.sparse_far_min_bbox_height_px = 4.0
    node.bbox_match_margin_px, node.bbox_match_margin_ratio = 4.0, 0.10
    node.min_cluster_support_px = 1
    node.monocular_enabled = False
    node.vision_dedup_radius_m = 0.5
    node.min_variance = 1.0e-4
    node.range_variance_scale = 0.0
    node.lidar_variance_x = node.lidar_variance_y = 0.04
    node.lidar_only_variance_x = node.lidar_only_variance_y = 0.20
    node.sparse_sigma_lat_m, node.sparse_sigma_lon_m = 0.10, 0.25
    node.vision_lateral_floor_m = 0.25
    # No deskew twist cached: the speed-proportional timing variance term
    # contributes zero, keeping the covariance assertions exact.
    node._twist = None
    node.lidar_timing_sigma_per_mps = 0.02
    node.cluster_range_bias_m = 0.0
    # 0.0 = uncapped, the declare_parameter default; the shipped config caps
    # at 12 m, but these gates are asserted range-by-range without it.
    node.sparse_max_range_m = 0.0
    for name, value in overrides.items():
        setattr(node, name, value)
    return node


def _scene(points_base, pixels, clusters):
    return Scene(points_base=np.asarray(points_base, dtype=np.float64),
                 pixels=np.asarray(pixels, dtype=np.float64),
                 cluster_indices=[np.asarray(c) for c in clusters])


DETECTION = Detection("blue", 0.9, 100.0, 100.0, 120.0, 160.0)
INSIDE = [110.0, 130.0]       # a pixel inside DETECTION's box
OUTSIDE = [9999.0, 9999.0]
# Three returns off one cone at ~6 m: one depth, cone-sized.
CONE_POINTS = [[6.0, 0.0, 0.2], [6.05, 0.05, 0.3], [5.98, -0.03, 0.15]]
NO_CLUSTER = np.empty((0,), dtype=np.int64)


class PriorityOrderTest(unittest.TestCase):
    """LiDAR position always beats a vision position, and a cluster always
    publishes -- confirmed by the camera or not."""

    def test_a_cluster_the_camera_confirms_carries_its_colour(self):
        node = _node()
        scene = _scene(CONE_POINTS, [INSIDE] * 3, [[0, 1, 2]])
        cones = node._build_cones([DETECTION], scene, None, np.eye(3), None,
                                  None, None)
        self.assertEqual(len(cones), 1)
        self.assertEqual(cones[0].provenance, PROV_CLUSTER)
        self.assertEqual(cones[0].color, "blue")

    def test_a_cluster_the_camera_missed_still_publishes_as_unknown(self):
        # The backbone does not need the camera's permission to exist.
        node = _node()
        scene = _scene(CONE_POINTS, [OUTSIDE] * 3, [[0, 1, 2]])
        cones = node._build_cones([DETECTION], scene, None, np.eye(3), None,
                                  None, None)
        self.assertEqual(len(cones), 1)
        self.assertEqual(cones[0].provenance, PROV_CLUSTER_ONLY)
        self.assertEqual(cones[0].color, "unknown")

    def test_sparse_does_not_duplicate_a_cone_a_cluster_already_claimed(self):
        node = _node()
        scene = _scene(CONE_POINTS, [INSIDE] * 3, [[0, 1, 2]])
        cones = node._build_cones([DETECTION], scene, None, np.eye(3), None,
                                  None, None)
        self.assertEqual([c.provenance for c in cones], [PROV_CLUSTER])

    def test_a_detection_with_no_cluster_falls_through_to_sparse(self):
        node = _node()
        scene = _scene(CONE_POINTS, [INSIDE] * 3, [])
        cones = node._build_cones([DETECTION], scene, None, np.eye(3), None,
                                  None, None)
        self.assertEqual(len(cones), 1)
        self.assertEqual(cones[0].provenance, PROV_SPARSE)
        self.assertEqual(cones[0].color, "blue")

    def test_no_lidar_at_all_publishes_nothing_from_the_backbone(self):
        node = _node()
        cones = node._build_cones([DETECTION], None, None, np.eye(3), None,
                                  None, None)
        self.assertEqual(cones, [])


class SparseGateTest(unittest.TestCase):
    """Sparse turns raw LiDAR inside a box into a cone. These are the checks
    that stop it turning whatever lies behind the box into one."""

    def test_accepts_real_thin_support(self):
        cone = _node()._sparse_cone(
            DETECTION, _scene(CONE_POINTS, [INSIDE] * 3, []), NO_CLUSTER)
        self.assertIsNotNone(cone)
        self.assertEqual(cone.provenance, PROV_SPARSE)
        self.assertAlmostEqual(cone.range_m, 6.01, places=2)
        # A LiDAR position keeps a LiDAR covariance: no ellipse.
        self.assertIsNone(cone.relative_depth_sigma)

    def test_rejects_a_depth_spread_because_one_cone_is_at_one_depth(self):
        # 6 m to 9 m inside one box is a wall, or a cone and the ground behind.
        wall = [[6.0, 0.0, 0.2], [7.5, 0.05, 0.3], [9.0, -0.03, 0.15]]
        self.assertIsNone(_node()._sparse_cone(
            DETECTION, _scene(wall, [INSIDE] * 3, []), NO_CLUSTER))

    def test_rejects_a_blob_wider_than_a_cone(self):
        wide = [[6.0, -0.8, 0.2], [6.05, 0.0, 0.3], [5.98, 0.8, 0.15]]
        self.assertIsNone(_node()._sparse_cone(
            DETECTION, _scene(wide, [INSIDE] * 3, []), NO_CLUSTER))

    def test_rejects_too_few_points_for_the_range_it_claims(self):
        # Under 8 m the LiDAR should see 3 returns; 2 means it is not a cone.
        near = CONE_POINTS[:2]
        self.assertIsNone(_node()._sparse_cone(
            DETECTION, _scene(near, [INSIDE] * 2, []), NO_CLUSTER))

    def test_cannot_re_use_points_a_cluster_already_consumed(self):
        clustered = np.asarray([0, 1, 2])
        self.assertIsNone(_node()._sparse_cone(
            DETECTION, _scene(CONE_POINTS, [INSIDE] * 3, [clustered]), clustered))

    def test_far_and_unconfident_is_refused(self):
        # Far and thin is where a false positive is cheapest to make.
        far_points = [[16.0, 0.0, 0.2], [16.05, 0.05, 0.3]]
        unsure = Detection("blue", 0.3, 100.0, 100.0, 120.0, 160.0)
        self.assertIsNone(_node()._sparse_cone(
            unsure, _scene(far_points, [INSIDE] * 2, []), NO_CLUSTER))
        # The same points with a confident detection are accepted.
        self.assertIsNotNone(_node()._sparse_cone(
            DETECTION, _scene(far_points, [INSIDE] * 2, []), NO_CLUSTER))

    def test_ignores_points_outside_the_box(self):
        self.assertIsNone(_node()._sparse_cone(
            DETECTION, _scene(CONE_POINTS, [OUTSIDE] * 3, []), NO_CLUSTER))


class ClusteringTest(unittest.TestCase):
    """DBSCAN's contract. Speed is worthless if the partition changes: every
    covariance constant in perception.yaml was fitted against these clusters."""

    def _cluster_node(self):
        return _node(cluster_eps=0.35, cluster_min_points=3)

    def test_two_cones_a_corridor_apart_stay_two_clusters(self):
        node = self._cluster_node()
        left = np.random.default_rng(1).normal([5.0, 1.5], 0.05, size=(12, 2))
        right = np.random.default_rng(2).normal([5.0, -1.5], 0.05, size=(12, 2))
        labels = node._dbscan_xy(np.vstack([left, right]))
        self.assertEqual(len({int(x) for x in labels if x >= 0}), 2)
        # and they are not mixed
        self.assertEqual(len(set(labels[:12])), 1)
        self.assertEqual(len(set(labels[12:])), 1)
        self.assertNotEqual(labels[0], labels[12])

    def test_a_point_alone_is_noise_not_a_cone(self):
        # min_samples is what stops one stray return becoming a cone.
        node = self._cluster_node()
        labels = node._dbscan_xy(np.array([[5.0, 0.0], [20.0, 9.0]]))
        self.assertTrue(all(x < 0 for x in labels))

    def test_eps_is_what_joins_returns_into_one_cone(self):
        node = self._cluster_node()
        # Three returns 10 cm apart: one cone.
        tight = np.array([[5.0, 0.0], [5.1, 0.0], [5.05, 0.05]])
        self.assertEqual(len({int(x) for x in node._dbscan_xy(tight) if x >= 0}), 1)
        # The same three a metre apart: nothing, they are not a blob.
        loose = np.array([[5.0, 0.0], [6.0, 0.0], [7.0, 0.0]])
        self.assertTrue(all(x < 0 for x in node._dbscan_xy(loose)))

    def test_an_empty_cloud_does_not_explode(self):
        labels = self._cluster_node()._dbscan_xy(np.empty((0, 2)))
        self.assertEqual(labels.shape, (0,))


class CovarianceTest(unittest.TestCase):
    """What SLAM is told. Understating this corrupts the map rather than merely
    adding noise, because the optimizer believes it."""

    def _vision_node(self):
        return _node(monocular_sigma_u_px=10.0, sigma_h_px=4.0)

    def test_a_vision_cone_is_precise_across_the_track_and_vague_along_the_ray(self):
        node = self._vision_node()
        cone = Cone(x=15.0, y=0.0, color="blue", provenance=PROV_MONOCULAR,
                    range_m=15.0, relative_depth_sigma=0.12, sigma_u_px=10.0,
                    fx_px=448.13)
        xx, xy, yx, yy = node._covariance(cone)
        self.assertAlmostEqual(math.sqrt(xx), 15.0 * 0.12, places=9)
        # Lateral is the pixel term and the floor in quadrature. The floor is
        # what the measured error has and the pixel term does not: rms showed no
        # range trend across 0-20 m while the pixel term tripled.
        self.assertAlmostEqual(math.sqrt(yy),
                               math.hypot(0.25, 15.0 * 10.0 / 448.13), places=9)
        self.assertAlmostEqual(xy, 0.0, places=12)
        self.assertEqual(xy, yx)
        # The point of the ellipse survives the floor: still a sliver on the ray.
        self.assertGreater(xx, 4.0 * yy)

    def test_the_lateral_floor_is_what_stops_a_near_vision_cone_lying(self):
        # The pixel term is a straight line through the origin, so without a
        # floor a 1 m cone claims ~2 cm of lateral certainty. Measured lateral
        # rms in the nearest band was 0.136 m -- z^2 = 4.47, over-confident by
        # the factor that corrupts a map rather than merely adding noise.
        node = self._vision_node()
        cone = Cone(x=1.0, y=0.0, color="blue", provenance=PROV_MONOCULAR,
                    range_m=1.0, relative_depth_sigma=0.12, sigma_u_px=10.0,
                    fx_px=448.13)
        _xx, _xy, _yx, yy = node._covariance(cone)
        pixel_only = 1.0 * 10.0 / 448.13
        self.assertAlmostEqual(math.sqrt(yy), math.hypot(0.25, pixel_only),
                               places=9)
        self.assertGreater(math.sqrt(yy), 10.0 * pixel_only)

    def test_the_lateral_floor_does_not_set_the_far_field(self):
        # A floor that dominated at range would be a constant wearing a model's
        # clothes. At 20 m the pixel term is 0.45 m and the floor moves it 15 %.
        node = self._vision_node()
        cone = Cone(x=20.0, y=0.0, color="blue", provenance=PROV_MONOCULAR,
                    range_m=20.0, relative_depth_sigma=0.12, sigma_u_px=10.0,
                    fx_px=448.13)
        _xx, _xy, _yx, yy = node._covariance(cone)
        pixel_only = 20.0 * 10.0 / 448.13
        self.assertLess(math.sqrt(yy), 1.2 * pixel_only)

    def test_an_off_axis_vision_cone_needs_the_off_diagonal(self):
        node = self._vision_node()
        theta = math.radians(45.0)
        cone = Cone(x=10.0 * math.cos(theta), y=10.0 * math.sin(theta),
                    color="blue", provenance=PROV_MONOCULAR, range_m=10.0,
                    relative_depth_sigma=0.12, sigma_u_px=10.0, fx_px=448.13)
        xx, xy, _yx, yy = node._covariance(cone)
        # At 45 degrees the ellipse lies on neither axis, so a diagonal-only
        # matrix would draw a circle where the truth is a sliver.
        self.assertGreater(abs(xy), 0.0)
        self.assertAlmostEqual(xx, yy, places=9)

    def test_a_cluster_keeps_its_measured_constant_because_it_is_a_circle(self):
        # A cluster has the returns to find the cone's CENTRE, and measures it
        # near-isotropically: lat/lon rms 0.041 / 0.035 m (n=5203). One constant
        # really is the truth for it -- unlike sparse, below.
        node = _node()
        for provenance, expected in ((PROV_CLUSTER, 0.04),
                                     (PROV_CLUSTER_ONLY, 0.20)):
            with self.subTest(provenance=provenance):
                cone = Cone(x=10.0, y=0.0, color="blue",
                            provenance=provenance, range_m=10.0)
                self.assertEqual(list(node._covariance(cone)),
                                 [expected, 0.0, 0.0, expected])

    def test_a_sparse_cone_is_an_ellipse_pointing_along_the_ray(self):
        # 2-3 returns on the front face: which ray hit fixes the BEARING, how
        # far down the face they landed leaves the RANGE noisy. Measured lateral
        # 0.050 m stable, longitudinal 0.047-0.125 m. An isotropic constant read
        # lon z^2 = 2.42 -- over-confident along the ray.
        node = _node()
        cone = Cone(x=10.0, y=0.0, color="blue", provenance=PROV_SPARSE,
                    range_m=10.0)
        xx, xy, _yx, yy = node._covariance(cone)
        # Dead ahead, so the ray is +x: xx is the range axis, yy the bearing.
        self.assertAlmostEqual(math.sqrt(xx), 0.25, places=9)
        self.assertAlmostEqual(math.sqrt(yy), 0.10, places=9)
        self.assertAlmostEqual(xy, 0.0, places=12)
        self.assertGreater(xx, 4.0 * yy)

    def test_an_off_axis_sparse_cone_rotates_with_the_bearing(self):
        # The ellipse follows the RAY, not the x axis. A diagonal-only matrix
        # would point a sparse cone's uncertainty at the wrong thing entirely.
        node = _node()
        theta = math.radians(45.0)
        cone = Cone(x=10.0 * math.cos(theta), y=10.0 * math.sin(theta),
                    color="blue", provenance=PROV_SPARSE, range_m=10.0)
        xx, xy, _yx, yy = node._covariance(cone)
        self.assertGreater(abs(xy), 0.0)
        self.assertAlmostEqual(xx, yy, places=9)

    def test_range_term_is_added_to_the_lidar_constants(self):
        node = _node(range_variance_scale=0.01)
        cone = Cone(x=10.0, y=0.0, color="blue", provenance=PROV_CLUSTER,
                    range_m=10.0)
        xx, _xy, _yx, yy = node._covariance(cone)
        self.assertAlmostEqual(xx, 0.04 + 0.1, places=9)
        self.assertAlmostEqual(yy, 0.04 + 0.1, places=9)

    def test_minimum_variance_clamps_without_rotating_the_ellipse(self):
        # Lifting the axes can only make the ellipse rounder, which is safe.
        # Scaling the off-diagonal to match would swing it off the bearing.
        node = _node(monocular_sigma_u_px=10.0, sigma_h_px=4.0,
                     min_variance=0.01)
        cone = Cone(x=2.0, y=2.0, color="blue", provenance=PROV_MONOCULAR,
                    range_m=math.hypot(2.0, 2.0), relative_depth_sigma=0.02,
                    sigma_u_px=1.0, fx_px=448.13)
        xx, xy, _yx, yy = node._covariance(cone)
        self.assertGreaterEqual(xx, 0.01)
        self.assertGreaterEqual(yy, 0.01)
        self.assertGreater(xx * yy - xy * xy, 0.0)

    def test_an_unusable_vision_model_falls_back_rather_than_dropping_the_cone(self):
        # A missing measurement costs the ellipse, never the detection.
        node = self._vision_node()
        cone = Cone(x=15.0, y=0.0, color="blue", provenance=PROV_MONOCULAR,
                    range_m=15.0, relative_depth_sigma=None)
        self.assertEqual(list(node._covariance(cone)), [0.20, 0.0, 0.0, 0.20])


class CovarianceMarkerTest(unittest.TestCase):
    """The ellipse drawn in RViz must be the one SLAM was given."""

    def test_axes_and_angle_follow_the_covariance(self):
        node = _node()
        from hyu_perception.fusion_core import bearing_aligned_covariance
        for degrees in (0, 30, 45, -60, 90):
            with self.subTest(degrees=degrees):
                theta = math.radians(degrees)
                x, y = 14.0 * math.cos(theta), 14.0 * math.sin(theta)
                covariance = list(bearing_aligned_covariance(x, y, 1.6, 0.05))
                major, minor, yaw = node._covariance_axes(covariance)
                self.assertAlmostEqual(major, 1.6, places=9)
                self.assertAlmostEqual(minor, 0.05, places=9)
                # The major axis lies on the bearing; an ellipse has period pi.
                offset = abs((yaw - theta + math.pi / 2) % math.pi - math.pi / 2)
                self.assertAlmostEqual(offset, 0.0, places=9)

    def test_an_isotropic_covariance_draws_a_disc(self):
        major, minor, _yaw = _node()._covariance_axes([0.04, 0.0, 0.0, 0.04])
        self.assertAlmostEqual(major, minor, places=12)

    def test_a_degenerate_covariance_draws_nothing(self):
        node = _node()
        for bad in ([float("nan"), 0.0, 0.0, 1.0],
                    [0.0, 0.0, 0.0, 0.0],
                    [1.0, 2.0, 2.0, 1.0]):     # not positive-definite
            with self.subTest(covariance=bad):
                self.assertIsNone(node._covariance_axes(bad))


class ProjectionTest(unittest.TestCase):
    """The two directions of the axis permutation must stay inverses. Getting
    this wrong does not raise: it silently puts every vision cone in the wrong
    place, which measured as 74.6 % false positives and ~0 recall."""

    K = np.asarray([[448.13, 0.0, 640.0], [0.0, 448.13, 360.0], [0.0, 0.0, 1.0]])

    def test_back_projection_and_projection_round_trip(self):
        from hyu_perception.fusion_core import camera_point_from_depth
        for model in ("eufs_bbox", "pinhole"):
            node = _node(projection_model=model, min_project_depth=0.1)
            for u, v, depth in ((640.0, 360.0, 10.0), (200.0, 300.0, 7.0),
                                (1100.0, 500.0, 15.0)):
                with self.subTest(model=model, u=u, v=v):
                    point = camera_point_from_depth(u, v, depth, self.K)
                    pixels = node._project(
                        node._to_camera_axes(point).reshape(1, 3), self.K)
                    self.assertAlmostEqual(pixels[0, 0], u, places=6)
                    self.assertAlmostEqual(pixels[0, 1], v, places=6)

    def test_points_behind_the_camera_stay_index_aligned_as_nan(self):
        # Consumers address points by index, so a filtered array would silently
        # shift every cluster onto the wrong points.
        node = _node(projection_model="pinhole", min_project_depth=0.1)
        points = np.asarray([[0.0, 0.0, 10.0], [0.0, 0.0, -5.0]])
        pixels = node._project(points, self.K)
        self.assertEqual(pixels.shape, (2, 2))
        self.assertTrue(np.all(np.isfinite(pixels[0])))
        self.assertTrue(np.all(np.isnan(pixels[1])))


class ColorTest(unittest.TestCase):
    def test_big_orange_is_matched_before_orange(self):
        self.assertEqual(PerceptionNode._normalize_color("big_orange"),
                         "big_orange")
        self.assertEqual(PerceptionNode._normalize_color("orange"), "orange")
        self.assertEqual(PerceptionNode._normalize_color("BigOrange"),
                         "big_orange")

    def test_an_unrecognised_colour_is_unknown_not_a_guess(self):
        self.assertEqual(PerceptionNode._normalize_color("weird"), "unknown")
        self.assertEqual(PerceptionNode._normalize_color(""), "unknown")


class ConeHeightTest(unittest.TestCase):
    """The curve's coefficient carries the cone's size, so a non-standard cone
    rescales it. An unknown class has no scale and must not take the standard
    one -- that is what the deleted stereo tier existed to avoid."""

    def test_each_class_gets_its_own_height(self):
        node = _node(standard_cone_height_m=0.450, big_cone_height_m=0.5255)
        self.assertEqual(node._cone_height_m("blue"), 0.450)
        self.assertEqual(node._cone_height_m("yellow"), 0.450)
        self.assertEqual(node._cone_height_m("orange"), 0.450)
        self.assertEqual(node._cone_height_m("big_orange"), 0.5255)

    def test_an_unknown_class_has_no_scale_and_is_dropped(self):
        node = _node(standard_cone_height_m=0.450, big_cone_height_m=0.5255)
        self.assertIsNone(node._cone_height_m("unknown"))


if __name__ == "__main__":
    unittest.main()
