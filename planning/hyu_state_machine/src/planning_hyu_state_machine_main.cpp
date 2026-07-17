#include "hyu_state_machine/planning_hyu_state_machine_node.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hyu_state_machine::PlanningStateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
