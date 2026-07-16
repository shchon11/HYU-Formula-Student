# Per-cone keypoints emitted alongside eufs_msgs/BoundingBoxes by a pose-capable
# detector (one inference produces both).  Entry i of `cone_keypoints`
# corresponds to entry i of the BoundingBoxes message published with the same
# header stamp, so the two topics pair one-to-one.
#
# `header` and `image_header` must both carry the source Image.header stamp,
# matching the BoundingBoxes timestamp policy, so the fusion node can pair the
# two streams through its existing timestamp buffers.

std_msgs/Header header
std_msgs/Header image_header
ConeKeypoints[] cone_keypoints
