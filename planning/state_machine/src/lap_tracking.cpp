#include "state_machine/planning_state_machine_node.hpp"

namespace state_machine
{

bool PlanningStateMachineNode::detectStartFinishGate() const
{
  // TODO(haejun): Implement start/finish gate detection using local /cones.
  return false;
}

bool PlanningStateMachineNode::hasCrossedStartFinishGate() const
{
  // TODO(haejun): Implement crossing logic using Frenet s or gate pose.
  return false;
}

void PlanningStateMachineNode::updateLapCount()
{
  (void)detectStartFinishGate();
  (void)hasCrossedStartFinishGate();

  // TODO(haejun): Implement lap count update condition later.
  // For now, keep lap_count_ unchanged except initial_lap_count parameter.
  // enable_manual_lap_override_ is declared for a future manual override interface.
}

}
