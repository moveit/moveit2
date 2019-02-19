<img src="http://moveit.ros.org/assets/images/moveit2_logo_black.png" alt="MoveIt! Logo" width="200"/>

The MoveIt! Motion Planning Framework **for ROS 2.0**

- [Milestones](#milestones)
- [Overview of MoveIt!](http://moveit.ros.org)
- [Installation Instructions](http://moveit.ros.org/install/)
- [Documentation](http://moveit.ros.org/documentation/)
- [Get Involved](http://moveit.ros.org/documentation/contributing/)

## Milestones
- [ ] Setup instructions
  - [ ] Install instructions
    - [ ] Ubuntu 18.04
    - [ ] OS X 10.14
- [ ] Update/setup infrastructure for development
  - [x] Delete metapackages
  - [x] Upgrade continuous integration for ROS 2.0
  - [x] Refactor/cleanup folder hierarchy
- [ ] Convert moveit packages to ROS 2.0
  - [x] ~~Include moveit_msgs in HRIM (**@ibaiape**)~~
  - [ ] Convert moveit_core (**in progress @vmayoral**)
    - [x] Convert all headers and link it to HRIM (contributed by @ibaiape)
    - [ ] Dependencies on other packages
      - [x] tf2_kdl https://github.com/ros2/geometry2/pull/90
      - [x] eigen_stl_containers https://github.com/AcutronicRobotics/eigen_stl_containers/tree/ros2
      - [x] geometric_shapes https://github.com/ros-planning/geometric_shapes/pull/96
      - [x] random_numbers https://github.com/ros-planning/random_numbers/pull/12
      - [x] srdfdom (contributed by @anasarrak, @vmayoral and @ahcorde) https://github.com/ros-planning/srdfdom/pull/45
      - [x] urdf_parser_py https://github.com/ros/urdf_parser_py/pull/41
      - [x] Created a ROS 2 version (with package.xml) of urdfdom_headers https://github.com/AcutronicRobotics/urdfdom_headers/tree/ros2
      - [x] octomap https://github.com/AcutronicRobotics/octomap
        - [x]  octomap
        - [ ]  octovis
        - [ ]  dynamicEDT3D
  - [ ] Convert moveit_kinematics
  - [ ] Convert moveit_planners_ompl
  - [ ] Convert moveit_ros_planning
  - [ ] Convert moveit_ros_planning_interface
  - [ ] Convert moveit_ros_benchmarks
  - [ ] Convert moveit_ros_control_interface
  - [ ] Convert moveit_ros_manipulation
  - [ ] Convert moveit_ros_move_group
  - [ ] Convert moveit_ros_perception
  - [ ] Convert moveit_ros_robot_interaction
  - [ ] Convert moveit_ros_visualization
  - [ ] Convert moveit_ros_warehouse
  - [ ] Convert moveit_setup_assistant
  - [ ] Convert moveit_simple_controller_manager
  - [ ] Convert moveit_visual_tools
  - [ ] Convert moveit_task_constructor
  - [ ] Convert moveit_resources
  - [ ] Convert moveit_commander
  - [ ] Convert moveit_fake_controller_manager
- [ ] New features in ROS 2.0
  - [ ] Migrate plugin architecture to ROS2 nodelets
- [ ] Documentation
  - [ ] Create tutorials

## Continuous Integration Status

[![Build Status](https://travis-ci.org/AcutronicRobotics/moveit2.svg?branch=ros2)](https://travis-ci.org/AcutronicRobotics/moveit2)

## Docker Containers

WIP

## ROS Buildfarm

Debian releases of MoveIt2 will not be available during the alpha development stage. Check back May 2019.
