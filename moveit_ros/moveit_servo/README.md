# Moveit Servo

## Quick Start Guide for Panda example

### Install
Install and build, roughly following [these instructions](https://moveit.ros.org/install-moveit2/source/).
```bash
source /opt/ros/foxy/setup.bash
export COLCON_WS=~/ws_ros2/
mkdir -p $COLCON_WS/src
cd $COLCON_WS/src
git clone https://github.com/AdamPettinger/moveit2.git -b foxy-compile_servo
vcs import < moveit2/moveit2.repos
vcs import < moveit2/moveit_ros/moveit_servo/moveit_servo.repos
rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y
cd $COLCON_WS
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run C++ Interface Example
In a new terminal, run
```bash
source /opt/ros/foxy/setup.bash
. ~/ws_ros2/install/local_setup.bash
ros2 launch moveit_servo servo_cpp_interface_demo.launch.py
```
RViz should pop up with the panda robot, and start servoing while twisting the end effector. It will approach singularity and stop moving.

### Run Composable Node Example
End the C++ interface demo if running. In two seperate terminals, run:
```bash
source /opt/ros/foxy/setup.bash
. ~/ws_ros2/install/local_setup.bash
ros2 launch moveit_servo servo_server_demo.launch.py
```

and
```bash
source /opt/ros/foxy/setup.bash
. ~/ws_ros2/install/local_setup.bash
ros2 run moveit_servo fake_command_publisher
```
The first will open RViz like the last demo, and start the Servo object as a composable node. The second terminal publishes commands to it. The combined effect should closely resemble the C++ interface demo.