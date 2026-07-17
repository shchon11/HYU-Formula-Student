#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "hyu_local_planner/hyu_local_planner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hyu_local_planner::LocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
