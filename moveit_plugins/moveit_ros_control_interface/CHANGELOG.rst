^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_control_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.9 (2025-04-15)
------------------
* SERVICE_CALL_TIMEOUT = 1 second is harsh 🥵 (`#3382 <https://github.com/ros-planning/moveit2/issues/3382>`_) (`#3406 <https://github.com/ros-planning/moveit2/issues/3406>`_)
* Add logic to Ros2ControlManager to match ros2_control (backport `#3332 <https://github.com/ros-planning/moveit2/issues/3332>`_) (`#3342 <https://github.com/ros-planning/moveit2/issues/3342>`_)
* Contributors: Paul Gesel, Sebastian Castro, Yoan Mollard

2.5.8 (2025-02-09)
------------------
* Fix Ros2ControlManager chained controller logic (`#3301 <https://github.com/ros-planning/moveit2/issues/3301>`_) (`#3306 <https://github.com/ros-planning/moveit2/issues/3306>`_)
* Update controller_manager_plugin.cpp (backport `#3179 <https://github.com/ros-planning/moveit2/issues/3179>`_) (`#3235 <https://github.com/ros-planning/moveit2/issues/3235>`_)
  * Fixing the bug where the namespace is not properly applied when using Ros2ControlMultiManager
* Contributors: Paul Gesel, Seohyeon Ryu, mergify[bot]

2.5.7 (2024-12-29)
------------------

2.5.6 (2024-11-17)
------------------
* Use the non-deprecated service fields for switching controllers (`#2927 <https://github.com/ros-planning/moveit2/issues/2927>`_)
* Contributors: Sai Kishor Kothakota

2.5.5 (2023-09-10)
------------------
* Fix parameters for ros2_control namespaces (`#1833 <https://github.com/ros-planning/moveit2/issues/1833>`_) (`#1897 <https://github.com/ros-planning/moveit2/issues/1897>`_)
  Co-authored-by: AndyZe <andyz@utexas.edu>
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  (cherry picked from commit 5838ce890975e3a058cdc9ab699b27941374c3a2)
  Co-authored-by: Pablo Iñigo Blasco <pablo.inigo.blasco@gmail.com>
* Contributors: mergify[bot]

2.5.4 (2022-11-04)
------------------
* Rename MoveItControllerManager. Add deprecation warning (`#1601 <https://github.com/ros-planning/moveit2/issues/1601>`_) (`#1666 <https://github.com/ros-planning/moveit2/issues/1666>`_)
  (cherry picked from commit b050890a2632a723dd95c08542ef6ed57d77fb06)
  Co-authored-by: AndyZe <zelenak@picknik.ai>
* Support chained controllers (backport `#1482 <https://github.com/ros-planning/moveit2/issues/1482>`_) (`#1623 <https://github.com/ros-planning/moveit2/issues/1623>`_)
  (cherry picked from commit 3db960a4b3b1e1d25630867a62ea1182bac2e96a)
  Co-authored-by: Paul Gesel <paulgesel@gmail.com>
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_) (`#1555 <https://github.com/ros-planning/moveit2/issues/1555>`_)
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Contributors: mergify[bot]

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
