#!/usr/bin/env python3
"""Fit the Tier-2 monocular depth curve to the camera actually in use.

Tier 2 estimates depth from the bounding-box height alone:

    D = c * h_n ^ e,    h_n = bbox_height_px / image_height_px

``c`` and ``e`` are camera- and cone-specific.  The IIT Bombay paper reports
c=0.498, e=-0.954 for their ZED 2i and their cones, and states the fit must be
redone whenever camera placement or optics change.  Carrying their constants to
a different rig silently biases every monocular cone -- on this simulator's ZED
they overestimate depth by roughly 50%.

Two ways to use this:

  --analytic
      Print the exact pinhole curve implied by the camera intrinsics and the
      cone height.  For an undistorted (simulated) camera this is the ground
      truth: h_px = fy*H_cone/D, so e = -1 exactly and c = fy*H_cone/H_img.

  --bag / live topics
      Fit c and e empirically by pairing detected bounding boxes with the true
      cone ranges from /ground_truth/cones.  Use this on a real camera, where
      lens distortion and mounting pitch bend the curve away from e = -1.

Examples
--------
    # What should the constants be for this simulator's ZED?
    ./fit_mono_depth_curve.py --analytic --cone-height 0.450

    # Fit against a recorded run.
    ./fit_mono_depth_curve.py --bag ~/bags/track_run --cone-height 0.450
"""
import argparse
import math
import sys


def analytic_curve(fx_px, image_height_px, cone_height_m):
    """Exact pinhole curve: a rigid cone's pixel height is inversely linear in depth."""
    coefficient = fx_px * cone_height_m / image_height_px
    return coefficient, -1.0


def fx_from_fov(width_px, horizontal_fov_rad):
    return (width_px / 2.0) / math.tan(horizontal_fov_rad / 2.0)


def fit_power_law(normalized_heights, depths):
    """Least-squares fit of D = c * h^e, solved linearly in log space."""
    try:
        import numpy as np
    except ImportError:
        sys.exit("numpy is required to fit; pip install numpy")

    h = np.asarray(normalized_heights, dtype=float)
    d = np.asarray(depths, dtype=float)
    keep = (h > 0) & (d > 0) & np.isfinite(h) & np.isfinite(d)
    h, d = h[keep], d[keep]
    if h.size < 2:
        sys.exit(f"need at least 2 valid samples, got {h.size}")

    # log D = log c + e * log h  ->  ordinary least squares on [log h, 1]
    design = np.column_stack([np.log(h), np.ones(h.size)])
    (exponent, log_c), *_ = np.linalg.lstsq(design, np.log(d), rcond=None)
    coefficient = float(np.exp(log_c))

    predicted = coefficient * h ** exponent
    relative_error = np.abs(predicted - d) / d
    return coefficient, float(exponent), h.size, relative_error


def report_fit(coefficient, exponent, count, relative_error):
    import numpy as np

    print(f"\nfitted on {count} cone observations")
    print(f"  monocular_depth_coefficient: {coefficient:.4f}")
    print(f"  monocular_depth_exponent:    {exponent:.4f}")
    print("\nresidual error vs ground truth:")
    print(f"  mean   {100 * float(np.mean(relative_error)):5.2f}%")
    print(f"  median {100 * float(np.median(relative_error)):5.2f}%")
    print(f"  p90    {100 * float(np.percentile(relative_error, 90)):5.2f}%")
    print(f"  max    {100 * float(np.max(relative_error)):5.2f}%")
    print("\nThe paper reports 4.49% mean error for its own fitted curve.")


def collect_from_bag(bag_path, bbox_topic, truth_topic, image_height_px):
    """Pair each detection with the nearest ground-truth cone range."""
    try:
        import rosbag2_py  # noqa: F401
    except ImportError:
        sys.exit(
            "rosbag2_py is required for --bag. Source the ROS workspace, or "
            "use --analytic to compute the curve from intrinsics instead."
        )
    sys.exit(
        "Bag replay is not wired up yet: run the simulator with perception and "
        "record the paired topics, then extend this function to walk the bag. "
        "Until then --analytic gives the exact curve for an undistorted camera."
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--analytic", action="store_true",
                        help="compute the exact curve from camera intrinsics")
    parser.add_argument("--cone-height", type=float, default=0.450,
                        help="cone height in metres (simulator default: 0.450)")
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=720)
    parser.add_argument("--hfov", type=float, default=1.91986,
                        help="horizontal FOV in radians (simulator ZED: 1.91986)")
    parser.add_argument("--fx", type=float, default=None,
                        help="focal length in pixels; overrides --hfov")
    parser.add_argument("--bag", default=None, help="rosbag2 directory to fit against")
    parser.add_argument("--bbox-topic", default="/yolo_bounding_boxes")
    parser.add_argument("--truth-topic", default="/ground_truth/cones")
    args = parser.parse_args()

    fx = args.fx if args.fx else fx_from_fov(args.image_width, args.hfov)

    if args.bag:
        heights, depths = collect_from_bag(
            args.bag, args.bbox_topic, args.truth_topic, args.image_height)
        report_fit(*fit_power_law(heights, depths))
        return

    if not args.analytic:
        parser.error("pass --analytic, or --bag to fit against recorded data")

    coefficient, exponent = analytic_curve(fx, args.image_height, args.cone_height)
    print(f"camera:  {args.image_width}x{args.image_height}, fx = {fx:.2f} px")
    print(f"cone:    {args.cone_height} m tall")
    print("\nexact pinhole curve for this camera:")
    print(f"  monocular_depth_coefficient: {coefficient:.4f}")
    print(f"  monocular_depth_exponent:    {exponent:.1f}")
    print("\n  (the exponent is -1 exactly because a simulated camera has no")
    print("   lens distortion; a real lens bends it slightly, which is what the")
    print("   paper's -0.954 absorbs. Fit with --bag on real hardware.)")

    print(f"\n{'true D':>8} {'bbox h':>8} {'this curve':>12} {'paper curve':>12}")
    for depth in (2.0, 3.0, 5.0, 10.0, 15.0, 20.0):
        h_px = fx * args.cone_height / depth
        h_n = h_px / args.image_height
        ours = coefficient * h_n ** exponent
        paper = 0.498 * h_n ** -0.954
        print(f"{depth:7.1f}m {h_px:7.1f}px {ours:11.2f}m {paper:11.2f}m")


if __name__ == "__main__":
    main()
