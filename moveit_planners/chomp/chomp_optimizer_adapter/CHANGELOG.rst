^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_chomp_optimizer_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.7.0 (2023-01-29)
------------------
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Use a stronger source of randomness (`#1721 <https://github.com/ros-planning/moveit2/issues/1721>`_)
  * Remove use of deprecated `std::random_shuffle`
  * Replace random number generators with `rsl::rng`
  * Utilize `rsl::uniform_real`
* Contributors: Chris Thrasher, Christian Henkel

2.6.0 (2022-11-10)
------------------
* Short-circuit planning adapters (`#1694 <https://github.com/ros-planning/moveit2/issues/1694>`_)
  * Revert "Planning request adapters: short-circuit if failure, return code rather than bool (`#1605 <https://github.com/ros-planning/moveit2/issues/1605>`_)"
  This reverts commit 66a64b4a72b6ddef1af2329f20ed8162554d5bcb.
  * Add debug message in call stack of planning_request_adapters
  * Short-circuit planning request adapters
  * Replace if-elseif cascade with switch
  * Cleanup translation of MoveItErrorCode to string
  - Move default code to moveit_core/utils
  - Override defaults in existing getActionResultString()
  - Provide translations for all error codes defined in moveit_msgs
  * Fix comment according to review
  * Add braces
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  * Add braces
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  Co-authored-by: AndyZe <andyz@utexas.edu>
* Planning request adapters: short-circuit if failure, return code rather than bool (`#1605 <https://github.com/ros-planning/moveit2/issues/1605>`_)
  * Return code rather than bool
  * Remove all debug prints
  * Small fixup
  * Minor cleanup of comment and error handling
  * void return from PlannerFn
  * Control reaches end of non-void function
  * Use a MoveItErrorCode cast
  * More efficient callAdapter()
  * More MoveItErrorCode
  * CI fixup attempt
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* Contributors: AndyZe, Robert Haschke, Sebastian Jahr

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Update plugin library paths (`#1304 <https://github.com/ros-planning/moveit2/issues/1304>`_)
* Contributors: Jafar, Sebastian Jahr

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Make TOTG the default time-parameterization algorithm everywhere (`#1218 <https://github.com/ros-planning/moveit2/issues/1218>`_)
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Remove unused parameters. (`#1018 <https://github.com/ros-planning/moveit2/issues/1018>`_)
* Contributors: AndyZe, Cory Crean, Jafar, Robert Haschke, jeoseo

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
* Port CHOMP Motion Planner to ROS 2 (`#809 <https://github.com/ros-planning/moveit2/issues/809>`_)
* Contributors: Henning Kayser, andreas-botbuilt

1.1.1 (2020-10-13)
------------------

1.1.0 (2020-09-04)
------------------
* [feature] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [feature] Allow ROS namespaces for planning request adapters (`#1530 <https://github.com/ros-planning/moveit/issues/1530>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [fix] Fix Chomp planning adapter (`#1525 <https://github.com/ros-planning/moveit/issues/1525>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Dave Coleman, Henning Kayser, Robert Haschke, Sean Yen, Tyler Weaver, Yu, Yan

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
* [maint] Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint] Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* Contributors: Robert Haschke, Sean Yen, Yu, Yan

1.0.2 (2019-06-28)
------------------
* [fix] Chomp planning adapter: Fix spurious error message Fix "Found empty JointState message"
* Contributors: Robert Haschke

1.0.1 (2019-03-08)
------------------
* [fix] segfault in chomp adapter (`#1377 <https://github.com/ros-planning/moveit/issues/1377>`_)
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Robert Haschke, Yu, Yan

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
* [maintenance] Moved CHOMP optimizer into own package (`#1251 <https://github.com/ros-planning/moveit/issues/1251>`_)
* rename default_planner_request_adapters/CHOMPOptimizerAdapter -> chomp/OptimizerAdapter
* Contributors: Robert Haschke
