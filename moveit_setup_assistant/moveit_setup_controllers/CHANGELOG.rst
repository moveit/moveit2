^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_setup_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.11.0 (2024-09-16)
-------------------
* Fix key duplication in moveit_setup_assistant for FollowJointTrajectory (`#2959 <https://github.com/moveit/moveit2/issues/2959>`_)
* Replace deprecated load_yaml with xacro.load_yaml in ros2_control.xacro template (`#2934 <https://github.com/moveit/moveit2/issues/2934>`_)
* Remove additional word 'hardware' in Moveit Controllers section of MoveIt Setup Assistant (`#2936 <https://github.com/moveit/moveit2/issues/2936>`_)
* Contributors: Chris Schindlbeck

2.10.0 (2024-06-13)
-------------------
* Add allow_nonzero_velocity_at_trajectory_end parameter to exported ros2_controllers config file (`#2751 <https://github.com/moveit/moveit2/issues/2751>`_)
* CMake format and lint in pre-commit (`#2683 <https://github.com/moveit/moveit2/issues/2683>`_)
* Contributors: Sebastian Castro, Tyler Weaver

2.9.0 (2024-01-09)
------------------
* Node logging for the rest of MoveIt (`#2599 <https://github.com/ros-planning/moveit2/issues/2599>`_)
* Don't assume gripper controller for single joint control in MoveIt Setup Assistant (`#2555 <https://github.com/ros-planning/moveit2/issues/2555>`_)
  * For single joint controllers which are not gripper controllers, still output joints list
  * Use OR
  * Only check for GripperActionController
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
  ---------
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Contributors: Forrest Rogers-Marcovitz, Sebastian Jahr, Tyler Weaver

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
* Update pre-commit (`#2094 <https://github.com/ros-planning/moveit2/issues/2094>`_)
* Contributors: Shobuj Paul

2.7.1 (2023-03-23)
------------------
* add missing dependencies on config utils (`#1962 <https://github.com/ros-planning/moveit2/issues/1962>`_)
  when installing ros-humble-moveit-setup-assistant from debs,
  the package cannot currently run due to this missing depend
* Contributors: Michael Ferguson

2.7.0 (2023-01-29)
------------------
* Merge PR `#1712 <https://github.com/ros-planning/moveit2/issues/1712>`_: fix clang compiler warnings + stricter CI
* Add default constructors
  ... as they are not implicitly declared anymore
* Add default copy/move constructors/assignment operators
  As a user-declared destructor deletes any implicitly-defined move constructor/assignment operator,
  we need to declared them manually. This in turn requires to declare the copy constructor/assignment as well.
* Fix -Wdelete-non-abstract-non-virtual-dtor
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Minimize use of `this->` (`#1784 <https://github.com/ros-planning/moveit2/issues/1784>`_)
  It's often unnecessary. MoveIt already avoids this in most cases
  so this PR better cements that existing pattern.
* Add braces around blocks. (`#999 <https://github.com/ros-planning/moveit2/issues/999>`_)
* Fix clang-tidy issues (`#1706 <https://github.com/ros-planning/moveit2/issues/1706>`_)
  * Blindly apply automatic clang-tidy fixes
  * Exemplarily cleanup a few automatic clang-tidy fixes
  * Clang-tidy fixups
  * Missed const-ref fixups
  * Fix unsupported non-const -> const
  * More fixes
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Contributors: Chris Thrasher, Christian Henkel, Cory Crean, Robert Haschke

2.6.0 (2022-11-10)
------------------
* Merge PR `#1553 <https://github.com/ros-planning/moveit2/issues/1553>`_: Improve cmake files
* Cleanup cmake files
  - Replace ament_export_libraries() -> ament_export_targets(HAS_LIBRARY_TARGET)
  - Replace ament_export_include_directories() -> INCLUDES DESTINATION include
  See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#building-a-library
* Use standard exported targets: export\_${PROJECT_NAME} -> ${PROJECT_NAME}Targets
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* MSA: write the default controller namespace (`#1515 <https://github.com/ros-planning/moveit2/issues/1515>`_)
  * Write the default controller namespace
  * Check for a sane controller type
  * Revert the check for FollowJointTrajectory type
* Contributors: AndyZe, Robert Haschke, Sebastian Jahr

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Rename cartesian_limits.yaml (`#1422 <https://github.com/ros-planning/moveit2/issues/1422>`_)
  Co-authored-by: Tyler Weaver <maybe@tylerjw.dev>
* [MSA] Testing Framework for MoveItSetupAssistant (`#1383 <https://github.com/ros-planning/moveit2/issues/1383>`_)
  Adds a nice testing harness for easy checking of generated files.
  New Features:
  * Updated sensor configuration format
  * Keep trajectory execution parameters around
  Tests
  * Ported the ROS 1 tests for perception and controllers
  * Added some SRDF Planning Groups tests
* fix duplicated ros_control joints (`#1406 <https://github.com/ros-planning/moveit2/issues/1406>`_)
  when more than one controller can control a joint,
  that joint would appear multiple times
* [MSA] Add additional parameters to Controller definitions (`#1408 <https://github.com/ros-planning/moveit2/issues/1408>`_)
* Merge pull request `#1254 <https://github.com/ros-planning/moveit2/issues/1254>`_ from ros-planning/feature/msa
  MoveIt Setup Assistant - Merge the Feature branch
* [MSA] Clean up extra parentheses (`#1366 <https://github.com/ros-planning/moveit2/issues/1366>`_)
* [MSA] ros2_control Integration (`#1299 <https://github.com/ros-planning/moveit2/issues/1299>`_)
* [MSA] Workaround to launch files without controllers (`#1275 <https://github.com/ros-planning/moveit2/issues/1275>`_)
* [MSA] PR Feedback
* [MSA] Migration Cleanup (`#1253 <https://github.com/ros-planning/moveit2/issues/1253>`_)
* [MSA] Initial Controllers and Simulation Steps Port (`#1252 <https://github.com/ros-planning/moveit2/issues/1252>`_)
* Contributors: AndyZe, David V. Lu!!, Michael Ferguson, Vatan Aksoy Tezer

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------

2.3.0 (2021-10-08)
------------------

2.2.1 (2021-07-12)
------------------

2.2.0 (2021-06-30)
------------------

2.1.4 (2021-05-31)
------------------

2.1.3 (2021-05-22)
------------------

2.1.2 (2021-04-22)
------------------

2.1.1 (2021-04-13)
------------------

2.1.0 (2020-11-24)
------------------

2.0.0 (2020-05-13)
------------------
