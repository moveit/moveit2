<img src="https://github.com/ros-planning/moveit.ros.org/blob/master/assets/logo/moveit2/moveit_logo-black.png" alt="MoveIt 2 Logo" width="300"/>

# MoveIt 2 Beta - Demo Setup

The `run_moveit_cpp` package provides a simulated robot setup that shows how to get started with MoveIt 2.
The demo includes examples for:

* Configuring and loading MoveIt using MoveItCpp
* Launching a simulated ros2_control driver
* Visualizing robot and planning scene in RViz
* Planning and execution of robot trajectories

Overall, the MoveIt 2 Beta Demo provides all necessary features in order to get a simple robot setup running with ROS 2.

## Prerequesites

Before running the demo, there are additional dependencies that need to be installed:

* [ros2_control](https://github.com/ros-controls/ros2_control)
* [ros2_controllers](https://github.com/ros-controls/ros2_controllers)
* [fake_joint](https://github.com/tork-a/fake_joint)

For that, simply import `run_moveit_cpp.repos` into your workspace and recompile with colcon (assuming the workspace has been compiled and sourced following the [installation instructions](/README.md)):

    cd $COLCON_WS/src
    vcs import < moveit2/moveit_demo_nodes/run_moveit_cpp/run_moveit_cpp.repos
    rosdep install --from-paths . --ignore-src --rosdistro eloquent
    cd ..
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
    
## Running the Demo
 
Now, the demo can be started using the launch file `run_moveit_cpp.launch.py`:
 
     ros2 launch run_moveit_cpp run_moveit_cpp.launch.py
     
The launch file should open the RViz GUI showing the Panda robot in extended position.
The demo starts by computing a simple motion plan which is being visualized via a transparent RobotState display.
This step alone involves a big range of components, like IK, collision checks, planning scene, robot model, OMPL planning plugin and planner adapters.
Right after, the trajectory is being executed on a simulated controller (fake_joint) using the `ros2_control` hardware interface.
As of now, `ros2_control` doesn't support action server interfaces similar to ROS 1, yet.
We are using the available message topic to publish the plan solution to the trajectory controller for execution.
Executing trajectories on real hardware either requires implementing a ros2_control interface for the driver or forwarding the trajectories to a ROS 1 message adapter using [ros1_bridge](https://github.com/ros2/ros1_bridge). 
