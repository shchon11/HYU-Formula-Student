// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef EUFS_GRAPH_SLAM__OPTIMIZER_POLICY_HPP_
#define EUFS_GRAPH_SLAM__OPTIMIZER_POLICY_HPP_

#include <cstddef>

namespace eufs_graph_slam
{
namespace optimizer_policy
{

inline bool shouldFixPose(
  std::size_t pose_index,
  std::size_t pose_count,
  int max_active_poses)
{
  if (pose_index == 0U) {
    return true;
  }
  if (max_active_poses <= 0 ||
    pose_count <= static_cast<std::size_t>(max_active_poses))
  {
    return false;
  }
  return pose_index < pose_count - static_cast<std::size_t>(max_active_poses);
}

}  // namespace optimizer_policy
}  // namespace eufs_graph_slam

#endif  // EUFS_GRAPH_SLAM__OPTIMIZER_POLICY_HPP_
