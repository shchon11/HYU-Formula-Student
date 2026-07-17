# Keypoints for one detected cone, in the same image frame as the matching
# BoundingBox.  Small cones (blue/yellow/orange) carry 6 keypoints (three
# left/right stripe pairs); big orange cones carry 8 (four pairs).  Semantic
# order is top-down, left-then-right within each row:
#   0,1 = upper L,R   2,3 = mid L,R   4,5 = lower L,R   [6,7 = bottom L,R]
#
# u/v are pixel coordinates in the image named by ConeKeypointsArray.image_header.
# confidence is the per-keypoint visibility/score in [0, 1].
# All three arrays have the same length, which must be 6 or 8.

float64[] u
float64[] v
float64[] confidence
