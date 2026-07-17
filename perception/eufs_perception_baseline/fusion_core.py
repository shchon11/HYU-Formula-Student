"""Pure geometry for cone perception.

No ROS dependencies, on purpose: what is here is measurement and can be checked
against arithmetic, while the node above it owns the plumbing and the policy.
Everything fails by returning None rather than raising or guessing, so the node
decides what a missing measurement means -- which in practice is always "publish
the cone with a wider covariance" or "do not publish it", never "publish it with
a number nobody stands behind".
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


def _finite(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _positive_finite(value) -> bool:
    return _finite(value) and float(value) > 0.0
