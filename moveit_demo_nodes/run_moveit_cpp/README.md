<img src="https://github.com/ros-planning/moveit.ros.org/blob/main/assets/logo/moveit2/moveit_logo-black.png" alt="MoveIt 2 Logo" width="300"/>

# MoveIt 2 Beta - Demo Setup

The `run_moveit_cpp` package provides a simulated robot setup that shows how to get started with MoveIt 2.
The demo includes examples for:

* Configuring and loading MoveIt using MoveItCpp
* Running a simulated robot
* Visualizing robot and planning scene in RViz
* Planning and execution of robot trajectories

## Running the Demo

This demo launches the RViz GUI and demonstrates planning and execution of a simple collision-free motion plan with the panda robot.
This involves a big range of components: IK solver and collision checking plugins, `PlanningScene`, `RobotModel`, `PlanningPipeline` including adapters, OMPL planner and RViz visualization (`Trajectory` and `PlanningScene` displays).

     ros2 launch run_moveit_cpp run_moveit_cpp.launch.py
