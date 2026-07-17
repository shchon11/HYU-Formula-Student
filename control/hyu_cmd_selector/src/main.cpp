#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tmpc_cmd_selector/tmpc_cmd_selector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tmpc_cmd_selector::TmpcCmdSelectorNode>());
  rclcpp::shutdown();
  return 0;
}
