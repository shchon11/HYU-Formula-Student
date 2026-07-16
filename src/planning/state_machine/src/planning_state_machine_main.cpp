#include "state_machine/planning_state_machine_node.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<state_machine::PlanningStateMachineNode>());
  rclcpp::shutdown();
  return 0;
}
