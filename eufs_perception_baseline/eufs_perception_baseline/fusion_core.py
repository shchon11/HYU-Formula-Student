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


def monocular_relative_depth_sigma(
    bbox_height_px: float,
    exponent: float,
    sigma_h_px: float,
) -> Optional[float]:
    """
    Fractional depth uncertainty of the bbox-height curve, ``sigma_D / D``.

    Differentiating ``D = c * h_n^e`` gives ``sigma_D/D = |e| * sigma_h/h``
    directly, so no distance model is needed: the pixel height is itself the
    measurement.  ``c`` cancels, which is why this survives a re-fit of the
    coefficient -- only the exponent matters.
    """
    if not all(_finite(value) for value in (bbox_height_px, exponent, sigma_h_px)):
        return None
    if bbox_height_px <= 0.0 or sigma_h_px < 0.0:
        return None
    return abs(float(exponent)) * float(sigma_h_px) / float(bbox_height_px)


def stereo_relative_depth_sigma(
    depth_m: float,
    fx_px: float,
    baseline_m: float,
    sigma_d_px: float,
) -> Optional[float]:
    """
    Fractional depth uncertainty of a disparity measurement, ``sigma_D / D``.

    ``D = fx*B/d`` gives ``sigma_D = D^2 * sigma_d / (fx*B)``, so the fraction
    is ``D * sigma_d / (fx*B)`` -- linear in depth, because disparity shrinks
    as the cone recedes and a fixed pixel error is then a larger share of it.
    """
    values = (depth_m, fx_px, baseline_m, sigma_d_px)
    if not all(_finite(value) for value in values):
        return None
    if depth_m <= 0.0 or fx_px <= 0.0 or baseline_m <= 0.0 or sigma_d_px < 0.0:
        return None
    return float(depth_m) * float(sigma_d_px) / (float(fx_px) * float(baseline_m))


def bearing_aligned_covariance(
    x_m: float,
    y_m: float,
    sigma_lon_m: float,
    sigma_lat_m: float,
) -> Optional[Tuple[float, float, float, float]]:
    """
    Build a 2x2 covariance whose axes follow the line of sight to ``(x, y)``.

    A vision tier's error is not axis-aligned: it is weak *along* the ray
    (depth) and strong *across* it (bearing).  Rotating ``diag(lon^2, lat^2)``
    by the bearing states that honestly, which an axis-aligned matrix cannot do
    for a cone off the camera's centreline.

    Returns ``(xx, xy, yx, yy)`` in eufs_msgs/ConeWithCovariance order.
    """
    values = (x_m, y_m, sigma_lon_m, sigma_lat_m)
    if not all(_finite(value) for value in values):
        return None
    if sigma_lon_m < 0.0 or sigma_lat_m < 0.0:
        return None
    # A cone at the origin has no defined bearing; fall back to an isotropic
    # matrix rather than letting atan2(0, 0) pick an arbitrary axis.
    if math.hypot(float(x_m), float(y_m)) <= 0.0:
        isotropic = max(float(sigma_lon_m), float(sigma_lat_m)) ** 2
        return (isotropic, 0.0, 0.0, isotropic)

    theta = math.atan2(float(y_m), float(x_m))
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    var_lon = float(sigma_lon_m) ** 2
    var_lat = float(sigma_lat_m) ** 2
    xx = var_lon * cos_t * cos_t + var_lat * sin_t * sin_t
    yy = var_lon * sin_t * sin_t + var_lat * cos_t * cos_t
    xy = (var_lon - var_lat) * sin_t * cos_t
    if not all(_finite(value) for value in (xx, xy, yy)):
        return None
    return (xx, xy, xy, yy)


@dataclass(frozen=True)
class ZnccMatch:
    """A disparity found by 1D correlation against a bbox-height prior."""

    disparity_px: float
    score: float                 # ZNCC peak in [-1, 1]
    prior_px: float
    #: peak / prior. Physical evidence lives here: a real cone's disparity sits
    #: near the distance its pixel height implies; background texture does not.
    ratio: float


def bbox_height_disparity_prior(
    bbox_height_px: float,
    baseline_m: float,
    cone_height_m: float,
) -> Optional[float]:
    """Disparity a cone of this pixel height must have, from geometry alone.

    ``h_px = fy*H/D`` and ``d = fx*B/D`` share the ``D``, so it cancels:

        d / h_px = (fx*B) / (fy*H) = B / H     (fx == fy on a rectified pair)

    **No distance model is involved** -- not the fitted curve, not its
    coefficient, not its exponent. The prior is pure similar triangles, which is
    what makes it an independent check on a curve fitted to this detector.

    For this car: 0.12 / 0.450 = 0.267 per pixel of box height. (A 325 mm
    competition cone would give 0.369; using that here is 40 % wrong and misses
    the search window outright.)
    """
    values = (bbox_height_px, baseline_m, cone_height_m)
    if not all(_finite(value) for value in values):
        return None
    if bbox_height_px <= 0.0 or baseline_m <= 0.0 or cone_height_m <= 0.0:
        return None
    return float(bbox_height_px) * float(baseline_m) / float(cone_height_m)


def zncc_disparity(
    left_gray: np.ndarray,
    right_gray: np.ndarray,
    bbox: Sequence[float],
    disparity_min: float,
    disparity_max: float,
    min_score: float = 0.5,
) -> Optional[ZnccMatch]:
    """Match a left-image patch along its own row in the right image.

    On a **rectified** pair this is all the problem is: the scale differs by
    ~1 %, the rotation is zero, and the correspondence is on the same row. A
    descriptor's invariances solve a problem that does not exist here, which is
    why this replaces the per-crop SIFT that cost 150-300 ms and 200 % CPU.

    The search is bounded by a prior from the bbox height (see
    ``bbox_height_disparity_prior``), so it is a handful of integer shifts plus
    a parabola, not a search.

    ``left_gray``/``right_gray`` are 2D arrays. Returns None when the patch has
    no contrast to match on, the window falls outside the image, or the peak is
    weaker than ``min_score``.
    """
    if left_gray.ndim != 2 or right_gray.ndim != 2:
        return None
    if left_gray.shape != right_gray.shape:
        return None
    if not all(_finite(value) for value in (disparity_min, disparity_max)):
        return None
    if disparity_min < 0.0 or disparity_max < disparity_min:
        return None

    height, width = left_gray.shape
    x0 = int(math.floor(bbox[0]))
    y0 = int(math.floor(bbox[1]))
    x1 = int(math.ceil(bbox[2]))
    y1 = int(math.ceil(bbox[3]))
    if x1 <= x0 or y1 <= y0:
        return None
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(width, x1), min(height, y1)
    if x1 - x0 < 2 or y1 - y0 < 2:
        return None

    template = left_gray[y0:y1, x0:x1].astype(np.float64)
    template_centered = template - template.mean()
    template_norm = float(np.sqrt(np.sum(template_centered ** 2)))
    # A flat patch correlates with everything equally; refuse rather than
    # return the arbitrary argmax of numerical noise.
    if template_norm <= 1.0e-6:
        return None

    low = int(math.floor(disparity_min))
    high = int(math.ceil(disparity_max))
    scores = {}
    for disparity in range(low, high + 1):
        # A rectified right image sees the same point shifted LEFT by d.
        left_edge = x0 - disparity
        if left_edge < 0 or left_edge + (x1 - x0) > width:
            continue
        candidate = right_gray[y0:y1, left_edge:left_edge + (x1 - x0)]
        candidate = candidate.astype(np.float64)
        candidate_centered = candidate - candidate.mean()
        candidate_norm = float(np.sqrt(np.sum(candidate_centered ** 2)))
        if candidate_norm <= 1.0e-6:
            continue
        scores[disparity] = float(
            np.sum(template_centered * candidate_centered)
            / (template_norm * candidate_norm))
    if not scores:
        return None

    best = max(scores, key=scores.get)
    peak = scores[best]
    if peak < min_score:
        return None

    # Sub-pixel by fitting a parabola to the peak and its neighbours. At 15 m
    # the whole disparity is only ~2.5 px, so the sub-pixel term is not a
    # refinement here -- it is most of the measurement.
    offset = 0.0
    if best - 1 in scores and best + 1 in scores:
        before, after = scores[best - 1], scores[best + 1]
        denominator = before - 2.0 * peak + after
        if abs(denominator) > 1.0e-12:
            offset = 0.5 * (before - after) / denominator
            # A parabola through a true peak cannot put the vertex outside the
            # neighbouring samples; if it does, the peak is not one.
            if abs(offset) > 1.0:
                offset = 0.0

    disparity = float(best) + offset
    if disparity <= 0.0:
        return None
    prior = 0.5 * (float(disparity_min) + float(disparity_max))
    return ZnccMatch(
        disparity_px=disparity,
        score=peak,
        prior_px=prior,
        ratio=disparity / prior if prior > 0.0 else float("nan"),
    )


def classify_cone_condition(
    bbox: Sequence[float],
    image_size: Sequence[int],
    min_height_to_width: float = 1.2,
    border_margin_ratio: float = 0.01,
) -> str:
    """
    Classify a detection as suitable (``good``) for monocular depth.

    A good cone is fully inside the image and visibly upright.  An upright cone
    clipped only by the left or right image edge is returned as
    ``horizontal_clip`` because its pixel height is still usable by the
    monocular distance model.  Vertically clipped, invalid, or wide/fallen
    detections are routed to ``bad`` so only stereo may recover them.
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
    horizontally_visible = x1 > margin_x and x2 < image_width - margin_x
    vertically_visible = y1 > margin_y and y2 < image_height - margin_y
    upright = height / width >= float(min_height_to_width)
    if horizontally_visible and vertically_visible and upright:
        return "good"
    if not horizontally_visible and vertically_visible and upright:
        return "horizontal_clip"
    return "bad"


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
    right_bbox: Sequence[float],
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

    Feature detection is restricted to slender cone crops in both images.  The
    caller must provide the right bbox propagated by ReKTNet/PnP; this helper no
    longer fabricates a right search window directly from the left bbox.  Lowe
    ratio, reciprocal-best, epipolar, depth-range, and uniqueness checks reject
    invalid candidates.  The best remaining descriptor supplies the single
    disparity used by IIT Bombay's reported highest-accuracy configuration.
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

    left_crop_candidate = slender_bbox(left_bbox, slender_fraction)
    right_crop_candidate = slender_bbox(right_bbox, slender_fraction)
    if left_crop_candidate is None or right_crop_candidate is None:
        return None
    left_crop = _clip_bbox(
        left_crop_candidate,
        left_gray.shape[1],
        left_gray.shape[0],
    )
    right_crop = _clip_bbox(
        right_crop_candidate,
        right_gray.shape[1],
        right_gray.shape[0],
    )
    if left_crop is None or right_crop is None:
        return None

    min_disparity = float(fx) * float(baseline_m) / float(max_depth_m)
    max_disparity = float(fx) * float(baseline_m) / float(min_depth_m)
    lx1, ly1, lx2, ly2 = left_crop

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
    rx1, ry1, rx2, ry2 = right_crop
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

    if len(unique_by_right) < int(min_matches):
        return None
    _, disparity = min(
        unique_by_right.values(),
        key=lambda entry: float(entry[0].distance),
    )
    disparity = float(disparity)
    depth = disparity_to_depth(disparity, fx, baseline_m)
    if depth is None or depth < min_depth_m or depth > max_depth_m:
        return None
    return StereoDepthEstimate(
        depth_m=depth,
        disparity_px=disparity,
        match_count=1,
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
