^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_setup_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.9 (2025-04-15)
------------------

2.5.8 (2025-02-09)
------------------
* Fix key duplication in moveit_setup_assistant for FollowJointTrajectory (`#2959 <https://github.com/ros-planning/moveit2/issues/2959>`_) (`#3316 <https://github.com/ros-planning/moveit2/issues/3316>`_)
* Contributors: Chris Schindlbeck, mergify[bot]

2.5.7 (2024-12-29)
------------------

2.5.6 (2024-11-17)
------------------
* Don't assume gripper controller for single joint control in MoveIt Setup Assistant (backport `#2555 <https://github.com/ros-planning/moveit2/issues/2555>`_) (`#2559 <https://github.com/ros-planning/moveit2/issues/2559>`_)
  * For single joint controllers which are not gripper controllers, still output joints list
  * Use OR
  * Only check for GripperActionController
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
  ---------
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
  (cherry picked from commit 81094a63898ace7829687d2d6aa3ccb3cdd81b58)
  Co-authored-by: Forrest Rogers-Marcovitz <39061824+forrest-rm@users.noreply.github.com>
* Contributors: Forrest Rogers-Marcovitz, mergify[bot]

2.5.5 (2023-09-10)
------------------
* add missing dependencies on config utils (backport `#1962 <https://github.com/ros-planning/moveit2/issues/1962>`_) (`#2206 <https://github.com/ros-planning/moveit2/issues/2206>`_)
  when installing ros-humble-moveit-setup-assistant from debs,
  the package cannot currently run due to this missing depend
  (cherry picked from commit cc635471aadfb9446398ece319ae31c6b72bec86)
  Co-authored-by: Michael Ferguson <mfergs7@gmail.com>
* Fix clang compiler warnings (backport of `#1712 <https://github.com/ros-planning/moveit2/issues/1712>`_) (`#1896 <https://github.com/ros-planning/moveit2/issues/1896>`_)
  - Fix warning: definition of implicit copy assignment operator is deprecated
  - Fix warning: expression with side effects will be evaluated
  - Fix warning: passing by value
  - Enable -Werror
  - Fix -Wdelete-non-abstract-non-virtual-dtor
  - Fix more clang warnings
  - Modernize gtest: TYPED_TEST_CASE -> TYPED_TEST_SUITE
  - Fix GoogleTestVerification.UninstantiatedTypeParameterizedTestSuite
  - Add default copy/move constructors/assignment operators
  As a user-declared destructor deletes any implicitly-defined move constructor/assignment operator,
  we need to declared them manually. This in turn requires to declare the copy constructor/assignment as well.
  - Explicitly declare overrides
  - Add default constructors as they are not implicitly declared anymore
  - Declare selected classes as final
  - Add noexcept specifier to constructors
  - Fixup gmock/gtest warnings
* Re-enable clang-tidy check `performance-unnecessary-value-param` (backport `#1703 <https://github.com/ros-planning/moveit2/issues/1703>`_)
  * Re-enable clang-tidy check performance-unnecessary-value-param (`#1703 <https://github.com/ros-planning/moveit2/issues/1703>`_)
  * Fix clang-tidy issues (`#1706 <https://github.com/ros-planning/moveit2/issues/1706>`_)
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  Co-authored-by: Robert Haschke <rhaschke@users.noreply.github.com>
* Contributors: Robert Haschke, mergify[bot]

2.5.4 (2022-11-04)
------------------
* MSA: write the default controller namespace (`#1515 <https://github.com/ros-planning/moveit2/issues/1515>`_) (`#1651 <https://github.com/ros-planning/moveit2/issues/1651>`_)
  (cherry picked from commit 066e8621166858d3f556726197b07643a1b685f9)
  Co-authored-by: AndyZe <zelenak@picknik.ai>
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_) (`#1555 <https://github.com/ros-planning/moveit2/issues/1555>`_)
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Contributors: mergify[bot]

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
