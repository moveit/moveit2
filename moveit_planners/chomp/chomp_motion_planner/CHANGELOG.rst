^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package chomp_motion_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.7.0 (2023-01-29)
------------------
* Merge https://github.com/ros-planning/moveit/commit/9225971216885490e933ece25390c63ca14f8a58
* Switch to clang-format-14 (`#1877 <https://github.com/ros-planning/moveit2/issues/1877>`_)
  * Switch to clang-format-14
  * Fix clang-format-14
* Change log level of CHOMP runtime output and change kdl print (`#1818 <https://github.com/ros-planning/moveit2/issues/1818>`_)
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Minimize use of `this->` (`#1784 <https://github.com/ros-planning/moveit2/issues/1784>`_)
  It's often unnecessary. MoveIt already avoids this in most cases
  so this PR better cements that existing pattern.
* Add braces around blocks. (`#999 <https://github.com/ros-planning/moveit2/issues/999>`_)
* Used C++ style cast instead of C style cast  (`#1628 <https://github.com/ros-planning/moveit2/issues/1628>`_)
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Use a stronger source of randomness (`#1721 <https://github.com/ros-planning/moveit2/issues/1721>`_)
  * Remove use of deprecated `std::random_shuffle`
  * Replace random number generators with `rsl::rng`
  * Utilize `rsl::uniform_real`
* Fix segfaults in CHOMP (`#3204 <https://github.com/ros-planning/moveit2/issues/3204>`_)
  * due to missing description\_
  * in case of an unspecified input trajectory
* Contributors: Abhijeet Das Gupta, Abishalini, Chris Thrasher, Christian Henkel, Cory Crean, Henning Kayser, Robert Haschke, Sebastian Jahr

2.6.0 (2022-11-10)
------------------
* Replace C array with std::array in std::vector template argument to improve compatibility with clang compiler and libc++ (`#1612 <https://github.com/ros-planning/moveit2/issues/1612>`_)
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
* updated comment formatting for correct doxygen generation (`#1582 <https://github.com/ros-planning/moveit2/issues/1582>`_)
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* Remove __has_include statements (`#1481 <https://github.com/ros-planning/moveit2/issues/1481>`_)
* Contributors: Michael Wrock, Sebastian Jahr, Vatan Aksoy Tezer, light-tech

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Merge remote-tracking branch 'origin/main' into feature/msa
* Removing more boost usage (`#1372 <https://github.com/ros-planning/moveit2/issues/1372>`_)
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Contributors: Henry Moore, Jafar, Vatan Aksoy Tezer

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Merge https://github.com/ros-planning/moveit/commit/424a5b7b8b774424f78346d1e98bf1c9a33f0e78
* MSA: Add STOMP + OMPL-CHOMP configs (`#2955 <https://github.com/ros-planning/moveit2/issues/2955>`_)
* Contributors: Abishalini, Jafar, Rick Staa, Robert Haschke, jeoseo

2.4.0 (2022-01-20)
------------------
* Replace NULL with nullptr (`#961 <https://github.com/ros-planning/moveit2/issues/961>`_)
  * Fixes `#841 <https://github.com/ros-planning/moveit2/issues/841>`_
* moveit_build_options()
  Declare common build options like CMAKE_CXX_STANDARD, CMAKE_BUILD_TYPE,
  and compiler options (namely warning flags) once.
  Each package depending on moveit_core can use these via moveit_build_options().
* Contributors: Robert Haschke, Stephanie Eng

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Replaced C-Style Cast with C++ Style Cast. (`#935 <https://github.com/ros-planning/moveit2/issues/935>`_)
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* Get rid of "std::endl" (`#918 <https://github.com/ros-planning/moveit2/issues/918>`_)
* changed post-increments in loops to preincrements (`#888 <https://github.com/ros-planning/moveit2/issues/888>`_)
* Port CHOMP Motion Planner to ROS 2 (`#809 <https://github.com/ros-planning/moveit2/issues/809>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: David V. Lu!!, Henning Kayser, Kaustubh, Parthasarathy Bana, Robert Haschke, Sencer Yazıcı, andreas-botbuilt, pvanlaar

1.1.1 (2020-10-13)
------------------

1.1.0 (2020-09-04)
------------------
* [feature] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [feature] Unified Collision Environment Integration (`#1584 <https://github.com/ros-planning/moveit/issues/1584>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [fix] Fix calculation of potential (`#1651 <https://github.com/ros-planning/moveit/issues/1651>`_)
* [fix] Fix Chomp planning adapter (`#1525 <https://github.com/ros-planning/moveit/issues/1525>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Ayush Garg, Bence Magyar, Dave Coleman, Jens P, Jonathan Binney, Markus Vieth, Robert Haschke, Sean Yen, Tyler Weaver, Yu, Yan

1.0.6 (2020-08-19)
------------------
* [maint] Migrate to clang-format-10
* [maint] Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* Contributors: Markus Vieth, Robert Haschke

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [maint] Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint] Windows build fixes
  * Fix header inclusion and other MSVC build errors (`#1636 <https://github.com/ros-planning/moveit/issues/1636>`_)
  * Remove GCC extensions (`#1583 <https://github.com/ros-planning/moveit/issues/1583>`_)
  * Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix]   Fix possible division-by-zero (`#1809 <https://github.com/ros-planning/moveit/issues/1809>`_)
* Contributors: Max Krichenbauer, Robert Haschke, Sean Yen, Yu, Yan

1.0.2 (2019-06-28)
------------------
* [fix] Fix chomp planner (`#1512 <https://github.com/ros-planning/moveit/issues/1512>`_)
  * Fix start-state handling
  * remove time parameterization from planning code
* Contributors: Robert Haschke

1.0.1 (2019-03-08)
------------------
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
* [fix] memory leak (ChompOptimizer)
* [fix] Set last_state for active joints only (`#1222 <https://github.com/ros-planning/moveit/issues/1222>`_)
* [maintenance] Rearranged CHOMP-related modules within moveit_planners/chomp (`#1251 <https://github.com/ros-planning/moveit/issues/1251>`_)
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Remove ChompParameters::getNonConstParams() (`#1196 <https://github.com/ros-planning/moveit/issues/1196>`_)
* [maintenance] Code Cleanup (`#1179 <https://github.com/ros-planning/moveit/issues/1179>`_)
* Contributors: Dave Coleman, Michael Görner, Robert Haschke, Shingo Kitagawa

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------

0.10.2 (2018-10-24)
-------------------
* [fix] Eigen alignment issuses due to missing aligned allocation (`#1039 <https://github.com/ros-planning/moveit/issues/1039>`_)
* [fix] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* [fix] changelogs: migration from tf -> tf2 only accidentally became part of 0.9.12's changelog
* [capability] Addition of CHOMP planning adapter for optimizing result of other planners (`#1012 <https://github.com/ros-planning/moveit/issues/1012>`_)
* [capability] Failure recovery options for CHOMP by tweaking parameters (`#987 <https://github.com/ros-planning/moveit/issues/987>`_)
* [capability] cleanup of unused parameters and code + addition of trajectory initialization methods (linear, cubic, quintic-spline) (`#960 <https://github.com/ros-planning/moveit/issues/960>`_)
* Contributors: Adrian Zwiener, Raghavender Sahdev, Robert Haschke

0.10.1 (2018-05-25)
-------------------
* [fix] for chomp fixed base joint bug (`#870 <https://github.com/ros-planning/moveit/issues/870>`_)
* [maintenance] MoveIt tf2 migration (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* [maintenance] switch to ROS_LOGGER from CONSOLE_BRIDGE (`#874 <https://github.com/ros-planning/moveit/issues/874>`_)
* Contributors: Bence Magyar, Dave Coleman, Ian McMahon, Mike Lautman, Xiaojian Ma

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------

0.9.9 (2017-08-06)
------------------
* [improve] Chomp use PlanningScene (`#546 <https://github.com/ros-planning/moveit/issues/546>`_) to partially address `#305 <https://github.com/ros-planning/moveit/issues/305>`_
* Contributors: Simon Schmeisser

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

0.9.3 (2016-11-16)
------------------

0.9.2 (2016-11-05)
------------------
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman, Isaac I.Y. Saito

0.9.0 (2016-10-19)
------------------
* Use shared_ptr typedefs in collision_distance_field and chomp.
* Fix CHOMP planner and CollisionDistanceField (`#155 <https://github.com/ros-planning/moveit/issues/155>`_)
  * Copy collision_distance_field package
  * Resurrect chomp
  * remove some old Makefiles and manifests
  * Correct various errors
  * Code formatting, author, description, version, etc
  * Add definitions for c++11. Nested templates problem.
  * Add name to planner plugin.
  * Change getJointModels to getActiveJointModels.
  * Call robot_state::RobotState::update in setRobotStateFromPoint.
  * Create README.md
  * Improve package.xml, CMake config and other changes suggested by jrgnicho.
  * Remove some commented code, add scaling factors to computeTimeStampes
  * Add install targets in moveit_experimental and chomp
  * Add install target for headers in chomp pkgs.
  * Remove unnecessary debugging ROS_INFO.
  * Port collision_distance_field test to indigo.
  * Remove one assertion that makes collision_distance_field test to fail.
* Contributors: Chittaranjan Srinivas Swaminathan, Maarten de Vries

0.8.3 (2016-08-21)
------------------
