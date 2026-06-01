#include "eufs_graph_slam/graph_slam_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eufs_graph_slam::GraphSlamNode>());
  rclcpp::shutdown();
  return 0;
}
