# Hybrid Planning
A Hybrid Planning architecture. You can find more information in the project's issues[#300](https://github.com/moveit/moveit2/issues/300), [#433](https://github.com/moveit/moveit2/issues/433) and on the [MoveIt 2 roadmap](https://moveit.ros.org/documentation/contributing/roadmap/). Furthermore, there is an extensive tutorial available [here](https://github.com/moveit/moveit2_tutorials/pull/97).

## Getting started
To start the demo run:
```
ros2 launch moveit_hybrid_planning hybrid_planning_demo.launch.py
```
You can exchange the planner logic plugin in the hybrid_planning_manager.yaml. Currently available demo plugins are:
- planner logic plugins:
  - replan_invalidated_trajectory: Runs the global planner once and starts executing the global solution
   with the local planner. In case the local planner detects a collision the global planner is rerun to update the
   invalidated global trajectory.
  - single_plan_execution: Run the global planner once and starts executing the global solution
    with the local planner
- global_planner plugins:
  - moveit_planning_pipeline: Global planner plugin that utilizes MoveIt's planning pipeline accessed via the MoveItCpp API.
- trajectory operator plugins:
  - simple_sampler: Samples the next global trajectory waypoint as local goal constraint
   based on the current robot state. When the waypoint is reached the index that marks the current local goal constraint
   is updated to the next global trajectory waypoint. Global trajectory updates simply replace the reference trajectory.
- local solver plugins:
  - forward_trajectory: Forwards the next waypoint of the sampled local trajectory.
   Additionally, it is possible to enable collision checking, which lets the robot stop in front of a collision object.
