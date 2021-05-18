# Hybrid Planning
Implementation of MoveIt 2's new hybrid planning architecture. You find more information in the project's issues[#300](https://github.com/ros-planning/moveit2/issues/300), [#433](https://github.com/ros-planning/moveit2/issues/433)  and on the [MoveIt 2 roadmap](https://moveit.ros.org/documentation/contributing/roadmap/).

## Getting started
1. Build moveit2 from source and make sure you use [this fork](https://github.com/sjahr/moveit_msgs) of moveit_msgs for the hybrid planning specific action definitions
2. To start the demo run:
```
ros2 launch moveit_hybrid_planning hybrid_planning.launch.py
```
You can exchange the planner logic plugin in the hybrid_planning_manager.yaml. Currently available demo plugins are:
- planner logic plugins:
  - replan_invalidated_trajectory: Runs the global planner once and starts executing the global solution
   with the local planner. In case the local planner detects a collision the global planner is rerun to update the
   invalidated global trajectory.
  - single_plan_execution: Run the global planner once and starts executing the global solution
    with the local planner
- trajectory operator plugins:
  - simple_sampler: Samples the next global trajectory waypoint as local goal constraint
   based on the current robot state. When the waypoint is reached the index that marks the current local goal constraint
   is updated to the next global trajectory waypoint. Global trajectory updates simply replace the reference trajectory.
- local solver plugins:
  - forward_trajectory: Forwards the next waypoint of the sampled local trajectory.
   Additionally, it is possible to enable collision checking, which lets the robot stop in front of a collision object.
