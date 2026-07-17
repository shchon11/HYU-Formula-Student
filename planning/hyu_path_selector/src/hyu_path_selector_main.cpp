#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "hyu_path_selector/hyu_path_selector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hyu_path_selector::PathSelectorNode>());
  rclcpp::shutdown();
  return 0;
}
