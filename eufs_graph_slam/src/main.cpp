// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <exception>
#include <memory>

#include "eufs_graph_slam/graph_slam_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<eufs_graph_slam::GraphSlamNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("graph_slam_node"),
      "GraphSLAM stopped: %s",
      error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
