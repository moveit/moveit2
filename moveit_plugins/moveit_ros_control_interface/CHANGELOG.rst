^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_control_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.12.3 (2025-04-15)
-------------------
* ROS Parameter for service call timeout for ros_control controllers (`#3419 <https://github.com/ros-planning/moveit2/issues/3419>`_) (`#3433 <https://github.com/ros-planning/moveit2/issues/3433>`_)
* SERVICE_CALL_TIMEOUT = 1 second is harsh 🥵 (`#3382 <https://github.com/ros-planning/moveit2/issues/3382>`_) (`#3407 <https://github.com/ros-planning/moveit2/issues/3407>`_)
* Contributors: Ashwin Sajith Nambiar, Yoan Mollard

2.12.2 (2025-02-15)
-------------------
* Add logic to Ros2ControlManager to match ros2_control (`#3332 <https://github.com/ros-planning/moveit2/issues/3332>`_) (`#3343 <https://github.com/ros-planning/moveit2/issues/3343>`_)
* Fix Ros2ControlManager chained controller logic (`#3301 <https://github.com/ros-planning/moveit2/issues/3301>`_) (`#3307 <https://github.com/ros-planning/moveit2/issues/3307>`_)
* Parallel gripper controller (`#3246 <https://github.com/ros-planning/moveit2/issues/3246>`_) (`#3260 <https://github.com/ros-planning/moveit2/issues/3260>`_)
* Update controller_manager_plugin.cpp (`#3179 <https://github.com/ros-planning/moveit2/issues/3179>`_) (`#3236 <https://github.com/ros-planning/moveit2/issues/3236>`_)
* Contributors: Paul Gesel, Marq Rasmussen, Seohyeon Ryu, mergify[bot]

2.12.1 (2024-12-18)
-------------------

2.12.0 (2024-11-29)
-------------------
* Enhancement/use hpp for headers (`#3113 <https://github.com/ros-planning/moveit2/issues/3113>`_)
* Contributors: Tom Noble

2.11.0 (2024-09-16)
-------------------

2.10.0 (2024-06-13)
-------------------
* Revert "Simplify controller manager namespacing (`#2210 <https://github.com/moveit/moveit2/issues/2210>`_)"
  This reverts commit 55df0bccd5e884649780b4ceeee80891e563b57b.
  The deprecated constructor was being used in the same file
  for the exact use case of enabling namespaces that are not
  specified by the parameter. There is no replacement for
  supporting a dynamic server lookup, however the parameter
  logic could still use simplification.
* Unify log names (`#2720 <https://github.com/moveit/moveit2/issues/2720>`_)
  Co-authored-by: Abishalini Sivaraman <abi.gpuram@gmail.com>
* CMake format and lint in pre-commit (`#2683 <https://github.com/moveit/moveit2/issues/2683>`_)
* Contributors: Henning Kayser, Sebastian Jahr, Tyler Weaver

2.9.0 (2024-01-09)
------------------
* Fix warning and cleanup unneeded placeholders (`#2566 <https://github.com/ros-planning/moveit2/issues/2566>`_)
  * Fix warning and cleanup unneeded placeholders
  * Make clang-tidy happy
  * Remove print statement
* Node logging for the rest of MoveIt (`#2599 <https://github.com/ros-planning/moveit2/issues/2599>`_)
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Contributors: Sebastian Jahr, Tyler Weaver

2.8.0 (2023-09-10)
------------------
* Simplify controller manager namespacing (`#2210 <https://github.com/ros-planning/moveit2/issues/2210>`_)
* Minor cleanup to ros_control_interface and trajectory execution (`#2208 <https://github.com/ros-planning/moveit2/issues/2208>`_)
* Contributors: Stephanie Eng

2.7.4 (2023-05-18)
------------------

2.7.3 (2023-04-24)
------------------
* Replace Variable PROJECT_NAME in CMakeLists.txt with the actual name (`#2020 <https://github.com/ros-planning/moveit2/issues/2020>`_)
* Fix controller_manager_plugin's switch controllers functionality (`#2116 <https://github.com/ros-planning/moveit2/issues/2116>`_)
* Contributors: Jafar, Shobuj Paul

2.7.2 (2023-04-18)
------------------

2.7.1 (2023-03-23)
------------------
* Add Warning Message for Out of Date Controller Information (`#1983 <https://github.com/ros-planning/moveit2/issues/1983>`_)
  Co-authored-by: Joseph Schornak <joe.schornak@gmail.com>
  Co-authored-by: Joseph Schornak <joe.schornak@gmail.com>
* Update SwitchController API usage (`#1996 <https://github.com/ros-planning/moveit2/issues/1996>`_)
  Fixes deprecated and now removed message fields https://github.com/ros-controls/ros2_control/pull/948
* Contributors: Erik Holum, Henning Kayser

2.7.0 (2023-01-29)
------------------
* Fix parameters for ros2_control namespaces (`#1833 <https://github.com/ros-planning/moveit2/issues/1833>`_)
  Co-authored-by: AndyZe <andyz@utexas.edu>
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Contributors: Christian Henkel, Pablo Iñigo Blasco

2.6.0 (2022-11-10)
------------------
* Rename MoveItControllerManager. Add deprecation warning (`#1601 <https://github.com/ros-planning/moveit2/issues/1601>`_)
  * Rename MoveItControllerManager->Ros2ControlManager. Add deprecation warning.
  * Do not rename base class
  * Still allow users to load plugins by the old names
  Co-authored-by: Jafar <jafar.uruc@gmail.com>
* Merge PR `#1553 <https://github.com/ros-planning/moveit2/issues/1553>`_: Improve cmake files
* Use standard exported targets: export\_${PROJECT_NAME} -> ${PROJECT_NAME}Targets
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* Support chained controllers (`#1482 <https://github.com/ros-planning/moveit2/issues/1482>`_)
  * fix controller list if chained controllers exist
  * add comments and clean code
  * added additional comments
  * fix formatting
  * fix white space
  * add const reference and chhnage variable name
  * simplify logic to only  work with one layer chain
  * Don't return false when not finding optional parameter
  * Update moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp
  Co-authored-by: AndyZe <andyz@utexas.edu>
  * add debug information
  * print controller names
  * print controllers with not known type
  * load controller dependencies
  * start chained controllers in switch
  * reverse order of activate controllers
  * prevent stoppping controller twice
  * revert all debug changes
  * add ROS error if a controller chains to more than one
  * use loop to index chained connections
  * update ros_control
  * add empty controller allocator for admittance controller
  * fix plugin xml
  * Update moveit_plugins/moveit_ros_control_interface/src/controller_manager_plugin.cpp
  Co-authored-by: AndyZe <andyz@utexas.edu>
  * Update moveit_plugins/moveit_ros_control_interface/src/controller_manager_plugin.cpp
  Co-authored-by: AndyZe <andyz@utexas.edu>
  * Update moveit_plugins/moveit_ros_control_interface/src/controller_manager_plugin.cpp
  Co-authored-by: AndyZe <andyz@utexas.edu>
  * Update moveit_plugins/moveit_ros_control_interface/src/controller_manager_plugin.cpp
  Co-authored-by: AndyZe <andyz@utexas.edu>
  * fix map indexing
  * add comment
  * Update moveit_plugins/moveit_ros_control_interface/src/controller_manager_plugin.cpp
  Co-authored-by: Tyler Weaver <squirrel428@protonmail.com>
  * Typos
  Co-authored-by: JafarAbdi <cafer.abdi@gmail.com>
  Co-authored-by: Jafar <jafar.uruc@gmail.com>
  Co-authored-by: AndyZe <andyz@utexas.edu>
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
  Co-authored-by: Tyler Weaver <squirrel428@protonmail.com>
  Co-authored-by: AndyZe <zelenak@picknik.ai>
* Contributors: AndyZe, Paul Gesel, Robert Haschke, Sebastian Jahr

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Contributors: Jafar

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Enable cppcheck (`#1224 <https://github.com/ros-planning/moveit2/issues/1224>`_)
  Co-authored-by: jeoseo <jeongwooseo2012@gmail.com>
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* 1.1.9
* 1.1.8
* 1.1.7
* 1.1.6
* Contributors: Jafar, Robert Haschke, jeoseo

2.4.0 (2022-01-20)
------------------
* moveit_build_options()
  Declare common build options like CMAKE_CXX_STANDARD, CMAKE_BUILD_TYPE,
  and compiler options (namely warning flags) once.
  Each package depending on moveit_core can use these via moveit_build_options().
* Contributors: Robert Haschke

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Fix installation of moveit_ros_control_interface header files (`#789 <https://github.com/ros-planning/moveit2/issues/789>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* Update controller_manager_plugin to fix MoveIt-managed controller switching (`#785 <https://github.com/ros-planning/moveit2/issues/785>`_)
* moveit_ros_control_interface: Small comment cleanup (`#754 <https://github.com/ros-planning/moveit2/issues/754>`_)
* Contributors: AndyZe, Dave Coleman, Henning Kayser, Joseph Schornak, Robert Haschke

2.3.0 (2021-10-08)
------------------
* moveit_ros_control_interface: Fix dangling reference (`#710 <https://github.com/ros-planning/moveit2/issues/710>`_)
* Port moveit ros control interface to ROS2 (`#545 <https://github.com/ros-planning/moveit2/issues/545>`_)
  * Port moveit_ros_control_interface to ROS2
  * Multiple fixes to trajectory_execution_manager
* Fix reversed check in switchControllers (`#2726 <https://github.com/ros-planning/moveit2/issues/2726>`_)
* Contributors: Jafar Abdi, Nathan Brooks, Joe Schornak, Henning Kayser

2.2.1 (2021-07-12)
------------------

1.1.1 (2020-10-13)
------------------
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski

1.1.0 (2020-09-04)
------------------
* [feature] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [feature] Remove support for Indigo's ros_control (`#2128 <https://github.com/ros-planning/moveit/issues/2128>`_)
* [feature] Add support for pos_vel_controllers and pos_vel_acc_controllers (`#1806 <https://github.com/ros-planning/moveit/issues/1806>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] clang-tidy-fix `modernize-loop-convert` to entire code base (`#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Dave Coleman, Henning Kayser, Jonathan Binney, Robert Haschke, Sandro Magalhães, Sean Yen, Tyler Weaver, Yu, Yan

1.0.6 (2020-08-19)
------------------

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [maint]   Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint]   Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [feature] Add support for pos_vel_controllers and pos_vel_acc_controllers (`#1806 <https://github.com/ros-planning/moveit/issues/1806>`_)
* Contributors: Robert Haschke, Sandro Magalhães, Sean Yen

1.0.2 (2019-06-28)
------------------

1.0.1 (2019-03-08)
------------------

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* Contributors: Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [maintenance] Code Cleanup (`#1196 <https://github.com/ros-planning/moveit/issues/1196>`_)
* Contributors: Robert Haschke

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------

0.10.2 (2018-10-24)
-------------------

0.10.1 (2018-05-25)
-------------------

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------

0.9.9 (2017-08-06)
------------------
* [improve] add backward compatibility patch for indigo (`#551 <https://github.com/ros-planning/moveit/issues/551>`_)
* Contributors: Michael Görner

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------

0.9.5 (2017-03-08)
------------------

0.9.4 (2017-02-06)
------------------
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Michael Goerner

0.9.3 (2016-11-16)
------------------

0.5.7 (2016-01-30)
------------------
* C++03 conforming nested templates
* fixed typo, added example config
* added brief decription tags
* formatted code to roscpp style
* improved documentation
* introduced getAbsName
* Added missing lock
* pre-allocate handles
* fixed typos
* set version to match the others
* fixed a lot of typos
* Intitial version of moveit_ros_control_interface package
* Contributors: Mathias Luedtke
