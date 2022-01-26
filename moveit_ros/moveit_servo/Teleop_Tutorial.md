# Tutorial Guide for Teleop Control of Robot ARM
================================================

The purpose of this tutorial is to provide insight on how to use a wireless controller from a list of controller devices available to teleop-control a Robot Arm using `moveit_servo`.

The list of controller devices supported for this task in `Moveit2` are:
1. XBOX 1 controller
2. HTC Vive Controller

The list of robot arms supported for teleop control in `Moveit2` are:
1. Panda Robot Arm (Simulation only)
2. UR5e Robot Arm (6DOF)
3. Kinova Robot Arm (7DOF)

# Teleop Control: XBOX (One and 360)

![xbox controller](https://support.xbox.com/en-US/help/xbox-360/accessories/controllers)
================== ============== =====================================
Button Name & #    Control Mode   Function (details below)
================== ============== =====================================
X, Y, A, B         Joint Jog      Move Distal joints
RIGHT STICK   (1)  End Point Jog  Move End Effector point through
                                  Y (RIGHT_STICK VERTICAL) and Z 
                                  (RIGHT_STICK HORIZONTAL) directions
D-PAD         (2)  Joint Jog      Move the Proximal Joints 
LEFT STICK    (3)  End Point Jog  Move End Effector point through
                                  Yaw (LEFT_STICK VERTICAL) and Pitch
                                  (LEFT_STICK HORIZONTAL) directions
BACK BUTTON   (4)  N/A            N/A
RIGHT BUMPER  (5)  End Point Jog  Move End Effector point through
                                  positive Roll direction
RIGHT TRIGGER (6)  End Point Jog  Move End Effector point through
                                  positive X direction
GUIDE BUTTON  (7)  N/A            N/A
START BUTTON  (8)  N/A            Turn on XBOX Controller
LEFT BUMPER   (9)  End Point Jog  Move End Effector point through
                                  negative Roll direction
LEFT TRIGGER  (10) End Point Jog  Move End Effector point through
                                  negative X direction


## Teleop control of Panda Robot Arm (in simulation):
To use the xbox controller to teleop control the panda arm using `moveit_servo` use the following commands.

Make sure you have paired the controller with your computer via Bluetooth. Then open a new terminal window and use the following launch file to spawn the robot in simulation and control it

```
ros2 launch moveit_servo panda_xbox_teleop.launch.py
```

To run the joystick teleop node independently,

```
ros2 run moveit_servo xbox_teleop_node
```