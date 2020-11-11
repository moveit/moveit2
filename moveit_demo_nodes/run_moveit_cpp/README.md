<img src="https://github.com/ros-planning/moveit.ros.org/blob/main/assets/logo/moveit2/moveit_logo-black.png" alt="MoveIt 2 Logo" width="300"/>

# MoveIt 2 Beta - Demo Setup

The `run_moveit_cpp` package provides a simulated robot setup that shows how to get started with MoveIt 2.
The demo includes examples for:

* Configuring and loading MoveIt using MoveItCpp
* Running a simulated robot
* Visualizing robot and planning scene in RViz
* Planning and execution of robot trajectories

The MoveIt 2 Demo provides all necessary features in order to get a simple robot setup running with ROS 2.
There are two options and launch files for simulating the robot controllers (the demo is the same):

1. ros2_control using a fake_joint driver: `run_moveit_cpp.launch.py`
1. MoveIt's fake controller: `run_moveit_cpp_fake_controller.launch.py`
    
## Running the Demo
 
The demo can be run using the launch files `run_moveit_cpp.launch.py` or `run_moveit_cpp_fake_controller.launch.py`:
 
     ros2 launch run_moveit_cpp run_moveit_cpp.launch.py
     
The demo launches the RViz GUI and demonstrates planning and execution of a simple collision-free motion plan with the panda robot.
This involves a big range of functioning components: IK solver and collision checking plugins, `PlanningScene`, `RobotModel`, `PlanningPipeline` including adapters, OMPL planner and RViz visualization (`Trajectory` and `PlanningScene` displays).
