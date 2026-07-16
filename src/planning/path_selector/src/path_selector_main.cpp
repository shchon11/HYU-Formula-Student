#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "path_selector/path_selector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_selector::PathSelectorNode>());
  rclcpp::shutdown();
  return 0;
}
