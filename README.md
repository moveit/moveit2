<img src="http://moveit.ros.org/assets/images/moveit2_logo_black.png" alt="MoveIt! Logo" width="200"/>

The MoveIt! Motion Planning Framework **for ROS 2.0**

- [Milestones](#milestones)
- [Overview of MoveIt!](http://moveit.ros.org)
- [Installation Instructions](http://moveit.ros.org/install/)
- [Documentation](http://moveit.ros.org/documentation/)
- [Get Involved](http://moveit.ros.org/documentation/contributing/)

## Milestones
- [ ] Install instructions
  - [ ] Ubuntu 18.04
  - [ ] OS X 10.14
- [x] Upgrade continuous integration for ROS 2.0
- [ ] Convert moveit packages to ROS 2.0
  - [x] Update/setup infrastructure for development
    - [x] Delete metapackages
    - [x] Upgrade continuous integration for ROS 2.0
    - [x] Refactor/cleanup folder hierarchy
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
  - [ ] Convert moveit_core packages to ROS 2.0
    - [x] version
    - [x] macros
    - [x] backtrace
    - [x] exceptions
    - [x] profiler
    - [x] logging
    - [x] background_processing
    - [x] kinematics_base
    - [x] controller_manager
    - [x] sensor_manager
    - [x] robot_model
    - [x] transforms
    - [x] robot_state
    - [x] robot_trajectory
    - [x] collision_detection
    - [x] collision_detection_fcl
    - [x] kinematic_constraints
    - [ ] planning_scene
    - [x] constraint_samplers
    - [x] planning_interface
    - [x] planning_request_adapter
    - [ ] trajectory_processing
    - [x] distance_field
    - [ ] collision_distance_field
    - [ ] kinematics_metrics
    - [ ] dynamics_solver
    - [ ] utils
  - [ ] Necessary for a Minimal Working Example
      - [ ] moveit_ros_planning_interface
     -  [ ] moveit_ros_planning
         -   [ ] moveit_core
         -   [ ] moveit_ros_perception
     -  [ ] moveit_ros_warehouse
       -  [ ] moveit_ros_planning
       -  [ ] warehouse_ros
     -  [ ] moveit_ros_manipulation
         -   [ ] moveit_core
         -   [ ] moveit_ros_planning
         -   [ ] moveit_ros_move_group
             -   [ ] moveit_core
             -   [ ] moveit_ros_planning
  - [ ] Other pending dependencies in moveit2
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
  - [ ] Tutorials for MoveIt2
  - [ ] Create tutorial on using ros1/ros2 bridge to support ros1 hardware drivers
  - [ ] Move install instructions to moveit.ros.org
- [ ] Major refactoring and divergence from moveit2
  - [ ] Run ROS2 C++ and python linters
  - [ ] Delete excesses packages that are left over from rosbuild stacks: moveit_runtime, moveit_plugins, moveit_ros
  - [ ] Rename non-package folders:
    - [ ] rename moveit_planners to planners
    - [ ] rename moveit_plugins to controller_interfaces
  - [ ] Restructure folder layout of moveit repo:
    - [ ] flatten moveit_ros folder to root of repo
    - [ ] rename all moveit_ros folders with moveit_ros prefix
  - [ ] Rename major classes
    - [ ] ControllerManagers become ControllerInterfaces
    - [ ] Rename related packages
  - [ ] Merge repos:
    - [ ] moveit 9.6 MB
    - [ ] moveit_task_constructor
    - [ ] moveit_tutorials  28.6 MB
    - [ ] moveit_msgs
    - [ ] moveit_resources  61 MB
    - [ ] moveit_visual_tools
    - [ ] moveit_advanced?
    - [ ] DELETE: moveit_kinematics_tests
  - [ ] Remove large binaries from moveit repo
  - [ ] Add gitlfs?


## Continuous Integration Status
[![Build Status](https://travis-ci.org/AcutronicRobotics/moveit2.svg?branch=master)](https://travis-ci.org/AcutronicRobotics/moveit2)

## Docker Containers
TODO [Create ROS2 Docker containers for MoveIt!](https://github.com/ros-planning/moveit2/issues/15)

## ROS Buildfarm
Debian releases of MoveIt2 will not be available during the alpha development stage. Check back May 2019.
