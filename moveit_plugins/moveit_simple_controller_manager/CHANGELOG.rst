^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_simple_controller_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.12.3 (2025-04-15)
-------------------

2.12.2 (2025-02-15)
-------------------
* Parallel gripper controller (`#3246 <https://github.com/ros-planning/moveit2/issues/3246>`_) (`#3260 <https://github.com/ros-planning/moveit2/issues/3260>`_)
* Contributors: Marq Rasmussen, mergify[bot]

2.12.1 (2024-12-18)
-------------------

2.12.0 (2024-11-29)
-------------------
* Enhancement/use hpp for headers (`#3113 <https://github.com/ros-planning/moveit2/issues/3113>`_)
* fix parameter namespacing for gripper controller (`#3023 <https://github.com/ros-planning/moveit2/issues/3023>`_)
* Contributors: Michael Ferguson, Tom Noble

2.11.0 (2024-09-16)
-------------------

2.10.0 (2024-06-13)
-------------------
* Migrate ros-planning org to moveit (`#2847 <https://github.com/moveit/moveit2/issues/2847>`_)
  * Rename github.com/ros-planning -> github.com/moveit
  * Rename ros-planning.github.io -> moveit.github.io
  * Rename ros-planning organization in docker and CI workflow files
  - ghcr.io/ros-planning -> ghcr.io/moveit
  - github.repository == 'moveit/*''
* Unify log names (`#2720 <https://github.com/moveit/moveit2/issues/2720>`_)
* CMake format and lint in pre-commit (`#2683 <https://github.com/moveit/moveit2/issues/2683>`_)
* Contributors: Robert Haschke, Sebastian Jahr, Tyler Weaver

2.9.0 (2024-01-09)
------------------
* Node logging for the rest of MoveIt (`#2599 <https://github.com/ros-planning/moveit2/issues/2599>`_)
* Add new clang-tidy style rules (`#2177 <https://github.com/ros-planning/moveit2/issues/2177>`_)
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Contributors: Sebastian Jahr, Tyler Weaver

2.8.0 (2023-09-10)
------------------

2.7.4 (2023-05-18)
------------------

2.7.3 (2023-04-24)
------------------
* Replace Variable PROJECT_NAME in CMakeLists.txt with the actual name (`#2020 <https://github.com/ros-planning/moveit2/issues/2020>`_)
* Contributors: Shobuj Paul

2.7.2 (2023-04-18)
------------------

2.7.1 (2023-03-23)
------------------
* Fix member naming (`#1949 <https://github.com/ros-planning/moveit2/issues/1949>`_)
  * Update clang-tidy rules for readability-identifier-naming
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Contributors: Robert Haschke

2.7.0 (2023-01-29)
------------------
* converted characters from string format to character format (`#1881 <https://github.com/ros-planning/moveit2/issues/1881>`_)
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Add braces around blocks. (`#999 <https://github.com/ros-planning/moveit2/issues/999>`_)
* Use emulated time in action-based controller (`#899 <https://github.com/ros-planning/moveit2/issues/899>`_)
* Fix clang-tidy issues (`#1706 <https://github.com/ros-planning/moveit2/issues/1706>`_)
  * Blindly apply automatic clang-tidy fixes
  * Exemplarily cleanup a few automatic clang-tidy fixes
  * Clang-tidy fixups
  * Missed const-ref fixups
  * Fix unsupported non-const -> const
  * More fixes
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Contributors: Christian Henkel, Cory Crean, Gaël Écorchard, Robert Haschke, Sameer Gupta

2.6.0 (2022-11-10)
------------------
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
* Contributors: Paul Gesel, Robert Haschke, Sebastian Jahr

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Merge pull request `#3106 <https://github.com/ros-planning/moveit2/issues/3106>`_ from v4hn/pr-master-bind-them-all / banish bind()
* banish bind()
* Contributors: Jafar, Michael Görner, v4hn

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Enable cppcheck (`#1224 <https://github.com/ros-planning/moveit2/issues/1224>`_)
  Co-authored-by: jeoseo <jeongwooseo2012@gmail.com>
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* RCLCPP Upgrade Bugfixes (`#1181 <https://github.com/ros-planning/moveit2/issues/1181>`_)
* Merge https://github.com/ros-planning/moveit/commit/a25515b73d682df03ed3eccd839110c296aa79fc
* 1.1.9
* 1.1.8
* 1.1.7
* feat(simple_controller_manager): add `max_effort` parameter to GripperCommand action (`#2984 <https://github.com/ros-planning/moveit2/issues/2984>`_)
  This commit adds the `max_effort` parameter to the GripperCommand
  declaration in the `controller_list` (see issue `#2956 <https://github.com/ros-planning/moveit2/issues/2956>`_). This value is
  only used when effort is set in the requested gripper trajectory.
  Co-authored-by: Jafar Abdi <cafer.abdi@gmail.com>
* Switch to std::bind (`#2967 <https://github.com/ros-planning/moveit2/issues/2967>`_)
  * boost::bind -> std::bind
  grep -rlI --exclude-dir=.git "boost::bind" | xargs sed -i 's/boost::bind/std::bind/g'
  * Convert bind placeholders
  grep -rlI --exclude-dir=.git " _[0-9]" | xargs sed -i 's/ _\([0-9]\)/ std::placeholders::_\1/g'
  * Update bind include header
  grep -rlI --exclude-dir=.git "boost/bind" | xargs sed -i 's#boost/bind.hpp#functional#'
* 1.1.6
* Contributors: Abishalini, David V. Lu!!, Jafar, Jochen Sprickerhof, Rick Staa, Robert Haschke, jeoseo

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
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* Make controller management logic more tolerant of missing or late ros2_control nodes (`#792 <https://github.com/ros-planning/moveit2/issues/792>`_)
* Clang-tidy fixes (`#596 <https://github.com/ros-planning/moveit2/issues/596>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* controller manager: enclose name in quotes (`#2761 <https://github.com/ros-planning/moveit/issues/2761>`_)
* Contributors: Dave Coleman, David V. Lu!!, G.A. vd. Hoorn, Henning Kayser, Joseph Schornak, Robert Haschke, pvanlaar

2.3.0 (2021-10-08)
------------------
* Fix cmake warnings (`#690 <https://github.com/ros-planning/moveit2/issues/690>`_)
  * Fix -Wformat-security
  * Fix -Wunused-variable
  * Fix -Wunused-lambda-capture
  * Fix -Wdeprecated-declarations
  * Fix clang-tidy, readability-identifier-naming in moveit_kinematics
* follow_joint_trajectory_controller_handle: publish new multi_dof_trajectory field (`#492 <https://github.com/ros-planning/moveit2/issues/492>`_)
* Contributors: Henning Kayser, Jafar Abdi, David V. Lu

2.2.1 (2021-07-12)
------------------

2.2.0 (2021-06-30)
------------------
* Enable Rolling and Galactic CI (`#494 <https://github.com/ros-planning/moveit2/issues/494>`_)
* [sync] MoveIt's master branch up-to https://github.com/ros-planning/moveit/commit/0d0a6a171b3fbea97a0c4f284e13433ba66a4ea4
* Contributors: Henning Kayser, JafarAbdi, Tyler Weaver, Vatan Aksoy Tezer

2.1.4 (2021-05-31)
------------------

2.1.3 (2021-05-22)
------------------

2.1.2 (2021-04-20)
------------------

2.1.1 (2021-04-12)
------------------
* Fix EXPORT install in CMake (`#372 <https://github.com/ros-planning/moveit2/issues/372>`_)
* ActionBasedControllerHandle: fix dangling reference in case of timeout
* [fix] Export libs for MoveGroup capabilities and MoveItSimpleControllerManager (`#344 <https://github.com/ros-planning/moveit2/issues/344>`_)
* MTC compatibility fixes (`#323 <https://github.com/ros-planning/moveit2/issues/323>`_)
* Replace workaround for controllerDoneCallback with promise/future
* moveit_simple_controller_manager: Fix waiting for execution
* Fix repo URLs in package.xml files
* Contributors: Boston Cleek, Henning Kayser, Jafar Abdi, Tyler Weaver

2.1.0 (2020-11-23)
------------------

2.0.0 (2020-02-17)
------------------
* [improve] MoveItSimpleControllerManager refactor parameter lookup
* [fix] Fix plugin install of MoveItSimpleControllerManager
* [port] Port moveit_simple_controller_manager to ROS 2 (`#158 <https://github.com/ros-planning/moveit2/issues/158>`_)
* Contributors: Henning Kayser, Jafar Abdi

1.1.1 (2020-10-13)
------------------
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski

1.1.0 (2020-09-04)
------------------
* [feature] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [feature] Allow different controllers for execution `#1832 <https://github.com/ros-planning/moveit/issues/1832>`_)
* [feature] ControllerManager: wait for done-callback (`#1783 <https://github.com/ros-planning/moveit/issues/1783>`_)
* [feature] Use CMAKE_CXX_STANDARD to enforce c++14 for portability (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [fix] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [fix] Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [fix] add missing space to log (`#1477 <https://github.com/ros-planning/moveit/issues/1477>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Dave Coleman, Henning Kayser, Jonathan Binney, Leroy Rügemer, Robert Haschke, Sean Yen, Tyler Weaver, Yu, Yan, llach

1.0.6 (2020-08-19)
------------------
* [maint] Migrate to clang-format-10
* Contributors: Robert Haschke

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [fix]   Handle "default" parameter in MoveitControllerManagers
  MoveIt{Fake|Simple}ControllerManager::getControllerState() now correctly returns current state
* [maint] Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint] Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix]   `ControllerManager`: wait for done-callback (`#1783 <https://github.com/ros-planning/moveit/issues/1783>`_)
* Contributors: Robert Haschke, Sean Yen, Luca Lach

1.0.2 (2019-06-28)
------------------

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Yu, Yan

1.0.0 (2019-02-24)
------------------
* [maintenance] cleanup SimpleControllerManager https://github.com/ros-planning/moveit/pull/1352
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
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* [maintenance] add minimum required pluginlib version (`#927 <https://github.com/ros-planning/moveit/issues/927>`_)
* Contributors: Mikael Arguedas, Mohmmad Ayman, Robert Haschke, mike lautman

0.10.1 (2018-05-25)
-------------------
* switch to ROS_LOGGER from CONSOLE_BRIDGE (`#874 <https://github.com/ros-planning/moveit/issues/874>`_)
* Contributors: Mikael Arguedas, Xiaojian Ma

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [capability][kinetic onward] optionally wait for controllers indefinitely (`#695 <https://github.com/ros-planning/moveit/issues/695>`_)
* Contributors: Bruno Brito, Michael Görner

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------
* [fix] include order (`#529 <https://github.com/ros-planning/moveit/issues/529>`_)
* Contributors: Michael Goerner

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [fix] assertion error when result not returned (`#378 <https://github.com/ros-planning/moveit/issues/378>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Michael Ferguson

0.9.3 (2016-11-16)
------------------

0.5.7 (2016-01-30)
------------------
* expose headers of moveit_simple_controller_manager
* Removed redundant logging information
* More informative warning message about multi-dof trajectories.
* Contributors: Dave Coleman, Dave Hershberger, Mathias Lüdtke

0.5.6 (2014-03-23)
------------------
* Allow simple controller manager to ignore virtual joints without failing
* Contributors: Dave Coleman

0.5.5 (2013-09-30)
------------------
* properly fill in the gripper command effort
* allow trajectories with >1 points, use the last point of any trajectory
* added better error reporting for FollowJointTrajectoryControllers

0.5.4 (2013-09-24)
------------------

0.5.3 (2013-09-23)
------------------
* make things a bit more robust
* make headers and author definitions aligned the same way; white space fixes
* fix `#1 <https://github.com/ros-planning/moveit_plugins/issues/1>`_

0.5.1 (2013-07-30)
------------------
* ns parameter is now action_ns, get rid of defaults

0.5.0 (2013-07-16)
------------------
* white space fixes (tabs are now spaces)

0.4.1 (2013-07-03)
------------------
* minor updates to package.xml

0.4.0 (2013-06-06)
------------------
* debs look good, bump to 0.4.0

0.1.0 (2013-06-05)
------------------
* add metapackage, clean up build in controller manager
* remove the now dead loaded controller stuff
* break out follow/gripper into separate headers
* initial working version
