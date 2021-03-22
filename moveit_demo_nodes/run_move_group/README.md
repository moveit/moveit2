<img src="https://github.com/ros-planning/moveit.ros.org/blob/main/assets/logo/moveit2/moveit_logo-black.png" alt="MoveIt 2 Logo" width="300"/>

# MoveIt 2 Beta - MoveGroup Demo

This demo includes a launch configuration for running MoveGroup and a separate MoveGroupInterface demo node.

The MoveGroup setup can be started using the launch file `run_move_group.launch.py`:

     ros2 launch run_move_group run_move_group.launch.py

This allows you to start planning and executing motions using the RViz MotionPlanning display.
The MoveGroupInterface demo can be started by additionally running:

     ros2 launch run_move_group run_move_group_interface.launch.py

This will open an interactive xterm that will prompt for user input for running consecutive planning/execution steps.
