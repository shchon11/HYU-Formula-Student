"""
Pure geometry helpers for the IIT Bombay-inspired perception pipeline.

The functions in this module intentionally have no ROS dependencies.  Keeping
the numerical operations here makes their failure behaviour explicit and lets
the ROS node decide which perception tier to use next.
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class GroundRemovalResult:
    """
    Result of ground-plane estimation.

    ``plane`` stores normalized ``(a, b, c, d)`` coefficients for
    ``a*x + b*y + c*z + d = 0``.  A missing plane means estimation failed
    open: every input point is retained as non-ground.
    """

    plane: Optional[np.ndarray]
    ground_mask: np.ndarray
    non_ground_mask: np.ndarray


@dataclass(frozen=True)
class StereoDepthEstimate:
    """Robust depth estimate from rectified stereo feature matches."""

    depth_m: float
    disparity_px: float
    match_count: int


def remove_ground_ransac(
    points,
    distance_threshold: float = 0.03,
    max_iterations: int = 200,
    max_tilt_degrees: float = 20.0,
    min_inliers: int = 3,
    seed: int = 0,
) -> GroundRemovalResult:
    """
    Remove a near-horizontal ground plane with deterministic RANSAC.

    Candidate normals are constrained to ``max_tilt_degrees`` from the
    positive z-axis.  This prevents a larger vertical wall from winning the
    consensus vote.  Invalid or insufficient inputs fail open so downstream
    cone detection does not silently discard data.
    """
    array = np.asarray(points, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] < 3:
        count = array.shape[0] if array.ndim >= 1 else 0
        return _failed_ground_result(count)

    xyz = array[:, :3]
    count = xyz.shape[0]
    finite_mask = np.all(np.isfinite(xyz), axis=1)
    finite_indices = np.flatnonzero(finite_mask)

    if (
        finite_indices.size < 3
        or not _positive_finite(distance_threshold)
        or int(max_iterations) <= 0
        or not math.isfinite(float(max_tilt_degrees))
        or float(max_tilt_degrees) < 0.0
        or float(max_tilt_degrees) >= 90.0
        or int(min_inliers) < 3
    ):
        return _failed_ground_result(count)

    required_inliers = int(min_inliers)
    if required_inliers > finite_indices.size:
        return _failed_ground_result(count)

    cos_max_tilt = math.cos(math.radians(float(max_tilt_degrees)))
    threshold = float(distance_threshold)
    rng = np.random.default_rng(seed)

    best_plane = None
    best_inliers = None
    best_score = None
    finite_xyz = xyz[finite_indices]

    for _ in range(int(max_iterations)):
        sample_positions = rng.choice(finite_xyz.shape[0], size=3, replace=False)
        candidate = _plane_from_three_points(finite_xyz[sample_positions])
        if candidate is None or candidate[2] < cos_max_tilt:
            continue

        distances = np.abs(finite_xyz @ candidate[:3] + candidate[3])
        inliers = distances <= threshold
        inlier_count = int(np.count_nonzero(inliers))
        if inlier_count < required_inliers:
            continue

        inlier_distances = distances[inliers]
        score = (
            inlier_count,
            -float(np.median(inlier_distances)),
            -float(np.mean(inlier_distances)),
        )
        if best_score is None or score > best_score:
            best_score = score
            best_plane = candidate
            best_inliers = inliers

    if best_plane is None or best_inliers is None:
        return _failed_ground_result(count)

    # Least-squares refinement reduces sample noise.  Keep the RANSAC model if
    # refinement would violate the ground-normal constraint.
    refined = _fit_plane_svd(finite_xyz[best_inliers])
    if refined is not None and refined[2] >= cos_max_tilt:
        best_plane = refined

    finite_distances = np.abs(finite_xyz @ best_plane[:3] + best_plane[3])
    finite_ground = finite_distances <= threshold
    if int(np.count_nonzero(finite_ground)) < required_inliers:
        return _failed_ground_result(count)

    ground_mask = np.zeros(count, dtype=bool)
    ground_mask[finite_indices] = finite_ground
    return GroundRemovalResult(
        plane=best_plane,
        ground_mask=ground_mask,
        non_ground_mask=~ground_mask,
    )


def monocular_depth_from_bbox(
    bbox_height_px: float,
    image_height_px: float,
    coefficient: float = 0.498,
    exponent: float = -0.954,
) -> Optional[float]:
    """Estimate optical depth using the paper's normalized bbox-height curve."""
    values = (bbox_height_px, image_height_px, coefficient, exponent)
    if not all(_finite(value) for value in values):
        return None
    if bbox_height_px <= 0.0 or image_height_px <= 0.0 or coefficient <= 0.0:
        return None

    normalized_height = float(bbox_height_px) / float(image_height_px)
    if normalized_height <= 0.0:
        return None
    depth = float(coefficient) * normalized_height ** float(exponent)
    return depth if _positive_finite(depth) else None


def classify_cone_condition(
    bbox: Sequence[float],
    image_size: Sequence[int],
    min_height_to_width: float = 1.2,
    border_margin_ratio: float = 0.01,
) -> str:
    """
    Classify a detection as suitable (``good``) for monocular depth.

    A good cone is fully inside the image and visibly upright.  Border-clipped,
    invalid, or wide/fallen detections are explicitly routed to ``bad`` so a
    stereo fallback can handle them.
    """
    parsed_bbox = _parse_bbox(bbox)
    if parsed_bbox is None or len(image_size) < 2:
        return "bad"
    try:
        image_width = float(image_size[0])
        image_height = float(image_size[1])
    except (TypeError, ValueError):
        return "bad"

    if not _positive_finite(image_width) or not _positive_finite(image_height):
        return "bad"
    if not _positive_finite(min_height_to_width):
        return "bad"
    if not _finite(border_margin_ratio) or border_margin_ratio < 0.0:
        return "bad"

    x1, y1, x2, y2 = parsed_bbox
    width = x2 - x1
    height = y2 - y1
    margin_x = image_width * float(border_margin_ratio)
    margin_y = image_height * float(border_margin_ratio)
    fully_visible = (
        x1 > margin_x
        and y1 > margin_y
        and x2 < image_width - margin_x
        and y2 < image_height - margin_y
    )
    upright = height / width >= float(min_height_to_width)
    return "good" if fully_visible and upright else "bad"


def camera_point_from_depth(
    u_px: float,
    v_px: float,
    optical_depth_m: float,
    camera_matrix,
) -> Optional[np.ndarray]:
    """Back-project a pixel at a known optical-axis (camera-z) depth."""
    if not all(_finite(value) for value in (u_px, v_px, optical_depth_m)):
        return None
    if optical_depth_m <= 0.0:
        return None

    matrix = np.asarray(camera_matrix, dtype=np.float64)
    if matrix.size != 9:
        return None
    matrix = matrix.reshape(3, 3)
    if not np.all(np.isfinite(matrix)):
        return None

    try:
        ray = np.linalg.solve(
            matrix,
            np.asarray([float(u_px), float(v_px), 1.0], dtype=np.float64),
        )
    except np.linalg.LinAlgError:
        return None
    if not np.all(np.isfinite(ray)) or abs(float(ray[2])) <= 1e-12:
        return None

    point = ray * (float(optical_depth_m) / float(ray[2]))
    return point if np.all(np.isfinite(point)) else None


def baseline_from_projection(left_projection, right_projection) -> Optional[float]:
    """Derive the positive stereo baseline from rectified 3x4 projections."""
    left = _projection_matrix(left_projection)
    right = _projection_matrix(right_projection)
    if left is None or right is None:
        return None
    if abs(float(left[0, 0])) <= 1e-12 or abs(float(right[0, 0])) <= 1e-12:
        return None

    left_center_x = -float(left[0, 3]) / float(left[0, 0])
    right_center_x = -float(right[0, 3]) / float(right[0, 0])
    baseline = abs(right_center_x - left_center_x)
    return baseline if _positive_finite(baseline) else None


def disparity_to_depth(
    disparity_px: float,
    fx_px: float,
    baseline_m: float,
) -> Optional[float]:
    """Convert positive rectified disparity to optical depth."""
    if not all(_positive_finite(value) for value in (disparity_px, fx_px, baseline_m)):
        return None
    depth = float(fx_px) * float(baseline_m) / float(disparity_px)
    return depth if _positive_finite(depth) else None


def slender_bbox(
    bbox: Sequence[float],
    width_fraction: float = 0.5,
) -> Optional[Tuple[int, int, int, int]]:
    """Return the central, slender horizontal crop of a pixel bbox."""
    parsed = _parse_bbox(bbox)
    if parsed is None or not _finite(width_fraction):
        return None
    if width_fraction <= 0.0 or width_fraction > 1.0:
        return None

    x1, y1, x2, y2 = parsed
    center_x = 0.5 * (x1 + x2)
    half_width = 0.5 * (x2 - x1) * float(width_fraction)
    return (
        int(math.floor(center_x - half_width)),
        int(math.floor(y1)),
        int(math.ceil(center_x + half_width)),
        int(math.ceil(y2)),
    )


def estimate_stereo_depth(
    left_image,
    right_image,
    left_bbox: Sequence[float],
    fx: float,
    baseline_m: float,
    min_depth_m: float = 0.5,
    max_depth_m: float = 30.0,
    epipolar_tolerance_px: float = 2.0,
    slender_fraction: float = 0.5,
    ratio_threshold: float = 0.75,
    min_matches: int = 1,
    principal_point_offset_px: float = 0.0,
) -> Optional[StereoDepthEstimate]:
    """
    Estimate depth from robust SIFT matches in rectified stereo images.

    Feature detection is restricted to a slender cone crop in the left image
    and a physically valid disparity search region in the right image.  Lowe
    ratio, reciprocal-best, epipolar, depth-range, uniqueness, and median/MAD
    checks reject ambiguous matches before disparity is converted to depth.
    """
    if not all(_positive_finite(value) for value in (fx, baseline_m)):
        return None
    if not all(_positive_finite(value) for value in (min_depth_m, max_depth_m)):
        return None
    if min_depth_m > max_depth_m:
        return None
    if not _finite(epipolar_tolerance_px) or epipolar_tolerance_px < 0.0:
        return None
    if not _finite(ratio_threshold) or not 0.0 < ratio_threshold < 1.0:
        return None
    if int(min_matches) < 1:
        return None
    if not _finite(principal_point_offset_px):
        return None

    left_gray = _as_gray_uint8(left_image)
    right_gray = _as_gray_uint8(right_image)
    if left_gray is None or right_gray is None:
        return None

    crop = slender_bbox(left_bbox, slender_fraction)
    if crop is None:
        return None
    left_crop = _clip_bbox(crop, left_gray.shape[1], left_gray.shape[0])
    if left_crop is None:
        return None

    min_disparity = float(fx) * float(baseline_m) / float(max_depth_m)
    max_disparity = float(fx) * float(baseline_m) / float(min_depth_m)
    min_raw_disparity = min_disparity + float(principal_point_offset_px)
    max_raw_disparity = max_disparity + float(principal_point_offset_px)
    lx1, ly1, lx2, ly2 = left_crop
    right_search = _clip_bbox(
        (
            math.floor(lx1 - max_raw_disparity),
            math.floor(ly1 - epipolar_tolerance_px),
            math.ceil(lx2 - min_raw_disparity),
            math.ceil(ly2 + epipolar_tolerance_px),
        ),
        right_gray.shape[1],
        right_gray.shape[0],
    )
    if right_search is None:
        return None

    try:
        import cv2
    except ImportError:
        return None
    sift_factory = getattr(cv2, "SIFT_create", None)
    if sift_factory is None:
        # Do not silently change the paper's descriptor contract.  Runtimes
        # without SIFT must disable this tier or install a SIFT-capable OpenCV.
        return None
    feature_detector = sift_factory()
    descriptor_norm = cv2.NORM_L2

    left_mask = np.zeros(left_gray.shape, dtype=np.uint8)
    right_mask = np.zeros(right_gray.shape, dtype=np.uint8)
    left_mask[ly1:ly2, lx1:lx2] = 255
    rx1, ry1, rx2, ry2 = right_search
    right_mask[ry1:ry2, rx1:rx2] = 255

    left_keypoints, left_descriptors = feature_detector.detectAndCompute(
        left_gray, left_mask
    )
    right_keypoints, right_descriptors = feature_detector.detectAndCompute(
        right_gray, right_mask
    )
    if (
        left_descriptors is None
        or right_descriptors is None
        or len(left_keypoints) == 0
        or len(right_keypoints) == 0
    ):
        return None

    matcher = cv2.BFMatcher(descriptor_norm)
    forward_knn = matcher.knnMatch(left_descriptors, right_descriptors, k=2)
    reverse_best = matcher.match(right_descriptors, left_descriptors)
    reverse_by_query = {match.queryIdx: match.trainIdx for match in reverse_best}

    valid_matches = []
    for neighbours in forward_knn:
        if len(neighbours) < 2:
            continue
        best, second = neighbours
        if second.distance <= 0.0 or best.distance >= ratio_threshold * second.distance:
            continue
        if reverse_by_query.get(best.trainIdx) != best.queryIdx:
            continue

        left_point = left_keypoints[best.queryIdx].pt
        right_point = right_keypoints[best.trainIdx].pt
        disparity = float(
            left_point[0]
            - right_point[0]
            - float(principal_point_offset_px)
        )
        vertical_error = abs(float(left_point[1] - right_point[1]))
        if vertical_error > float(epipolar_tolerance_px):
            continue
        if disparity < min_disparity or disparity > max_disparity:
            continue
        valid_matches.append((best, disparity))

    if not valid_matches:
        return None

    # Enforce one-to-one right features, retaining the strongest descriptor
    # match when multiple left features target the same right feature.
    unique_by_right = {}
    for match, disparity in valid_matches:
        previous = unique_by_right.get(match.trainIdx)
        if previous is None or match.distance < previous[0].distance:
            unique_by_right[match.trainIdx] = (match, disparity)

    disparities = np.asarray(
        [entry[1] for entry in unique_by_right.values()], dtype=np.float64
    )
    if disparities.size == 0:
        return None

    median_disparity = float(np.median(disparities))
    absolute_deviations = np.abs(disparities - median_disparity)
    mad = float(np.median(absolute_deviations))
    robust_tolerance = max(0.5, 3.0 * 1.4826 * mad)
    robust_disparities = disparities[absolute_deviations <= robust_tolerance]
    if robust_disparities.size < int(min_matches):
        return None

    disparity = float(np.median(robust_disparities))
    depth = disparity_to_depth(disparity, fx, baseline_m)
    if depth is None or depth < min_depth_m or depth > max_depth_m:
        return None
    return StereoDepthEstimate(
        depth_m=depth,
        disparity_px=disparity,
        match_count=int(robust_disparities.size),
    )


def _failed_ground_result(count: int) -> GroundRemovalResult:
    ground_mask = np.zeros(max(0, int(count)), dtype=bool)
    return GroundRemovalResult(
        plane=None,
        ground_mask=ground_mask,
        non_ground_mask=~ground_mask,
    )


def _plane_from_three_points(points: np.ndarray) -> Optional[np.ndarray]:
    first, second, third = points
    normal = np.cross(second - first, third - first)
    norm = float(np.linalg.norm(normal))
    if not _positive_finite(norm) or norm <= 1e-12:
        return None
    normal = normal / norm
    if normal[2] < 0.0:
        normal = -normal
    offset = -float(np.dot(normal, first))
    plane = np.asarray([normal[0], normal[1], normal[2], offset])
    return plane if np.all(np.isfinite(plane)) else None


def _fit_plane_svd(points: np.ndarray) -> Optional[np.ndarray]:
    if points.shape[0] < 3:
        return None
    centroid = np.mean(points, axis=0)
    try:
        _, singular_values, vectors = np.linalg.svd(
            points - centroid, full_matrices=False
        )
    except np.linalg.LinAlgError:
        return None
    if singular_values.size < 2 or singular_values[1] <= 1e-12:
        return None
    normal = vectors[-1]
    norm = float(np.linalg.norm(normal))
    if norm <= 1e-12:
        return None
    normal = normal / norm
    if normal[2] < 0.0:
        normal = -normal
    plane = np.asarray(
        [normal[0], normal[1], normal[2], -float(np.dot(normal, centroid))]
    )
    return plane if np.all(np.isfinite(plane)) else None


def _projection_matrix(value) -> Optional[np.ndarray]:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.size != 12:
        return None
    matrix = matrix.reshape(3, 4)
    return matrix if np.all(np.isfinite(matrix)) else None


def _parse_bbox(bbox: Sequence[float]) -> Optional[Tuple[float, float, float, float]]:
    try:
        if len(bbox) < 4:
            return None
        values = tuple(float(value) for value in bbox[:4])
    except (TypeError, ValueError):
        return None
    if not all(_finite(value) for value in values):
        return None
    x1, y1, x2, y2 = values
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def _clip_bbox(bbox, image_width: int, image_height: int):
    parsed = _parse_bbox(bbox)
    if parsed is None or image_width <= 0 or image_height <= 0:
        return None
    x1, y1, x2, y2 = parsed
    clipped = (
        max(0, min(int(math.floor(x1)), image_width)),
        max(0, min(int(math.floor(y1)), image_height)),
        max(0, min(int(math.ceil(x2)), image_width)),
        max(0, min(int(math.ceil(y2)), image_height)),
    )
    if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
        return None
    return clipped


def _as_gray_uint8(image) -> Optional[np.ndarray]:
    array = np.asarray(image)
    if array.ndim == 3 and array.shape[2] in (3, 4):
        try:
            import cv2
        except ImportError:
            return None
        conversion = cv2.COLOR_BGRA2GRAY if array.shape[2] == 4 else cv2.COLOR_BGR2GRAY
        try:
            array = cv2.cvtColor(array, conversion)
        except cv2.error:
            return None
    elif array.ndim != 2:
        return None
    if array.size == 0 or not np.all(np.isfinite(array)):
        return None
    if array.dtype == np.uint8:
        return np.ascontiguousarray(array)
    minimum = float(np.min(array))
    maximum = float(np.max(array))
    if maximum <= minimum:
        return np.zeros(array.shape, dtype=np.uint8)
    scaled = (array.astype(np.float64) - minimum) * (255.0 / (maximum - minimum))
    return np.ascontiguousarray(np.clip(scaled, 0.0, 255.0).astype(np.uint8))


def _finite(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _positive_finite(value) -> bool:
    return _finite(value) and float(value) > 0.0
