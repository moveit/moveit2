^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_setup_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
