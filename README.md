<img src="https://moveit.ros.org/assets/logo/moveit2/moveit_logo-black.png" alt="MoveIt 2 Logo" width="200"/>

The MoveIt Motion Planning Framework for **ROS 2**. For ROS 1, see [MoveIt 1](https://github.com/ros-planning/moveit).

*Easy-to-use open source robotics manipulation platform for developing commercial applications, prototyping designs, and benchmarking algorithms.*

## Continuous Integration Status

[![Format](https://github.com/ros-planning/moveit2/actions/workflows/format.yml/badge.svg?branch=main)](https://github.com/ros-planning/moveit2/actions/workflows/format.yml?branch=main) [![BuildAndTest](https://github.com/ros-planning/moveit2/actions/workflows/industrial_ci_action.yml/badge.svg?branch=main)](https://github.com/ros-planning/moveit2/actions/workflows/industrial_ci_action.yml?branch=main) [![codecov](https://codecov.io/gh/ros-planning/moveit2/branch/main/graph/badge.svg?token=W7uHKcY0ly)](https://codecov.io/gh/ros-planning/moveit2)

## General MoveIt Documentation

- [MoveIt Website](http://moveit.ros.org)
- [Tutorials and Documentation](https://ros-planning.github.io/moveit_tutorials/)
- [How to Get Involved](http://moveit.ros.org/about/get_involved/)
- [Future Release Dates](https://moveit.ros.org/#release-versions)

## MoveIt 2 Specific Documentation

- [MoveIt 2 Migration Progress](https://docs.google.com/spreadsheets/d/1aPb3hNP213iPHQIYgcnCYh9cGFUlZmi_06E_9iTSsOI/edit?usp=sharing)
- [MoveIt 2 Migration Guidelines](doc/MIGRATION_GUIDE.md)
- [MoveIt 2 Development Roadmap](https://moveit.ros.org/documentation/contributing/roadmap/)

## Source Build

See [MoveIt 2 Source Build - Linux](https://moveit.ros.org/install-moveit2/source/)

## Getting Started

We've prepared a simple demo setup that you can use for quickly spinning up a simulated robot environment with MoveItCpp.
See the [run_moveit_cpp](moveit_demo_nodes/run_moveit_cpp) demo package for further instructions and information.

The package [run_move_group](moveit_demo_nodes/run_move_group) provides a simple launch file for running a MoveGroup setup.
You can test it using the MotionPlanning display in RViz or by implementing your own MoveGroupInterface application.

## Supporters

This open source project is maintained by supporters from around the world — see [MoveIt maintainers](https://moveit.ros.org/about/). Special thanks to contributor from Intel and Open Robotics.

<a href="https://picknik.ai/">
  <img src="https://picknik.ai/assets/images/logo.jpg" width="168">
</a>

[PickNik Inc.](https://picknik.ai/) is leading and organizing the development of MoveIt 2.
If you would like to support this project, please contact hello@picknik.ai

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a>

The port to ROS 2 is supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 732287.
