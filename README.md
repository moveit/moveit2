<img src="https://github.com/ros-planning/moveit.ros.org/blob/master/assets/logo/moveit2/moveit_logo-black.png" alt="MoveIt 2 Logo" width="300"/>

The MoveIt Motion Planning Framework for **ROS 2**

Currently we support ROS 2 Eloquent.

- [Overview of MoveIt](http://moveit.ros.org)
- [Installation Instructions](http://moveit.ros.org/install/)
- [Documentation](http://moveit.ros.org/documentation/)
- [Get Involved](http://moveit.ros.org/documentation/contributing/)

## ROS 2 Migration Process and Roadmap

See the [MoveIt website](https://moveit.ros.org) for release dates and [this document](https://docs.google.com/spreadsheets/d/1aPb3hNP213iPHQIYgcnCYh9cGFUlZmi_06E_9iTSsOI/edit?usp=sharing) for the current migration progress.

Implementation instructions for the ROS 2 migration process can be found in our [Migration Guidelines](doc/MIGRATION_GUIDE.md).

Plans for future milestones can be found in our [Development Roadmap](https://moveit.ros.org/documentation/contributing/roadmap/).

## Build from Source

> Note: See the [migration progress](https://docs.google.com/spreadsheets/d/1aPb3hNP213iPHQIYgcnCYh9cGFUlZmi_06E_9iTSsOI/edit?usp=sharing) for learning what packages are available.

These instructions assume you are running on Ubuntu 18.04.

1. [Install ROS2 Eloquent](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/) following the installation instructions. Use the desktop installation and don't forget to source the setup script.

1. [Install ROS2 Build Tools](https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/#install-development-tools-and-ros-tools) up until setting up rosdep (we're using slightly different steps for setting up our workspace)

1. Create a colcon workspace:

        export COLCON_WS=~/ws_ros2/
        mkdir -p $COLCON_WS/src
        cd $COLCON_WS/src

1. Download the repository and install any dependencies:

        git clone git@github.com:ros-planning/moveit2.git
        vcs import < moveit2/moveit2.repos
        rosdep install -r --from-paths . --ignore-src --rosdistro eloquent -y

1. Configure and build the workspace:

        cd $COLCON_WS
        colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

1. Source the workspace:

        source $COLCON_WS/install/setup.bash
        
## Getting Started

We've prepared a simple demo setup that you can use for quickly spinning up a simulated robot environment with MoveItCpp.
See the [run_moveit_cpp](moveit_demo_nodes/run_moveit_cpp) demo package for further instructions and information.


## Continuous Integration Status

[![Build Status](https://travis-ci.org/ros-planning/moveit2.svg?branch=master)](https://travis-ci.org/ros-planning/moveit2)
