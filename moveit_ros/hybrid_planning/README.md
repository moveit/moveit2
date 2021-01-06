# Hybrid Planning
Implementation of MoveIt 2's new hybrid planning architecture. You find more information in the [project's issue](https://github.com/ros-planning/moveit2/issues/300) and on the [MoveIt 2 roadmap](https://moveit.ros.org/documentation/contributing/roadmap/).

## Getting started
1. Build moveit2 from source and make sure you use [this fork](https://github.com/sjahr/moveit_msgs) of moveit_msgs for the hybrid planning specific action definitions
2. To start the demo run:
```
ros2 launch moveit_hybrid_planning hybrid_planning.launch.py
```
You can exchange the planner logic plugin in the hybrid_planning_manager.yaml

## Components
- hybrid_planning_manager: Control unit of the architecture. The manager coordinates planners and implements reactive behavior defined by the planner logic plugin.
  - Implementation as ROS 2 component node
  - Planner logic plugin interface
- global_planner: ROS 2 component node that receives global motion planning requests and computes a solution to the global problem
- local_planner: Solves local constraints and can be used to execute the global solution. The is realized with the constraint solver plugin. The local planner also handles trajectory blending and matching with the trajectory operator plugin.
  - Implementation as ROS 2 component node
  - Constraint solver plugin interface
  - Trajectory operator plugin interface
- planner_logic_plugins: Planner logic plugin implementation
  - replan_invalidated_trajectory: Replan the global solution if the local planner triggers a collision detection event
  - single_plan_execution: Call the global planner once and execute the this global solution
- trajectory_operator_plugins: Trajectory operator plugin implementations
  - next_waypoint_sample: Forwards the next three waypoints of the global trajectory to the local solver based on the current robot state. When the global solution is updated, the internal (old) reference trajectory is dumped in favor of the update
- constraint_solver_plugins: Constraint solver plugin implementations
  - handle_imminent_collision: Follows local trajectory and stops in front of collision objects
- test:
  - hybrid_planning_test_node: Simple test demo
