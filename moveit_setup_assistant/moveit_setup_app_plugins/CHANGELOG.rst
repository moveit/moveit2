^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_setup_app_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.7.1 (2023-03-23)
------------------
* add missing dependencies on config utils (`#1962 <https://github.com/ros-planning/moveit2/issues/1962>`_)
  when installing ros-humble-moveit-setup-assistant from debs,
  the package cannot currently run due to this missing depend
* Contributors: Michael Ferguson

2.7.0 (2023-01-29)
------------------
* Merge PR `#1712 <https://github.com/ros-planning/moveit2/issues/1712>`_: fix clang compiler warnings + stricter CI
* Declare selected classes as final
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Minimize use of `this->` (`#1784 <https://github.com/ros-planning/moveit2/issues/1784>`_)
  It's often unnecessary. MoveIt already avoids this in most cases
  so this PR better cements that existing pattern.
* Contributors: Chris Thrasher, Christian Henkel, Robert Haschke

2.6.0 (2022-11-10)
------------------
* Merge PR `#1553 <https://github.com/ros-planning/moveit2/issues/1553>`_: Improve cmake files
* Cleanup cmake files
  - Replace ament_export_libraries() -> ament_export_targets(HAS_LIBRARY_TARGET)
  - Replace ament_export_include_directories() -> INCLUDES DESTINATION include
  See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#building-a-library
* Use standard exported targets: export\_${PROJECT_NAME} -> ${PROJECT_NAME}Targets
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* Contributors: Robert Haschke, Sebastian Jahr

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* [MSA] Testing Framework for MoveItSetupAssistant (`#1383 <https://github.com/ros-planning/moveit2/issues/1383>`_)
* MSA: Fix bug in sensors_3d reexport (`#1405 <https://github.com/ros-planning/moveit2/issues/1405>`_)
* update format of msa sensors_3d config (`#1398 <https://github.com/ros-planning/moveit2/issues/1398>`_)
* Merge pull request `#1254 <https://github.com/ros-planning/moveit2/issues/1254>`_ from ros-planning/feature/msa
  MoveIt Setup Assistant - Merge the Feature branch
* [MSA] Clean up extra parentheses (`#1366 <https://github.com/ros-planning/moveit2/issues/1366>`_)
* PR Feedback
* [MSA] Initial Controllers and Simulation Steps Port (`#1252 <https://github.com/ros-planning/moveit2/issues/1252>`_)
* [MSA] Merge main into feature/msa (Part III) (`#1249 <https://github.com/ros-planning/moveit2/issues/1249>`_)
* [MSA] Launch Updates (`#1247 <https://github.com/ros-planning/moveit2/issues/1247>`_)
* [MSA] Merge main into feature/msa (Part II) (`#1240 <https://github.com/ros-planning/moveit2/issues/1240>`_)
* [MSA] Generate More New Launch Files (`#1213 <https://github.com/ros-planning/moveit2/issues/1213>`_)
* [MSA] Add Setup Step for Generating Launch Files (`#1129 <https://github.com/ros-planning/moveit2/issues/1129>`_)
* [MSA] Merge Upstream into `feature/msa` (`#1119 <https://github.com/ros-planning/moveit2/issues/1119>`_)
* [MSA] SRDF Setup (`#1057 <https://github.com/ros-planning/moveit2/issues/1057>`_)
* [MSA] Initial Refactor
* Move Files Around (split into multiple packages, change to hpp extension)
* Contributors: David V. Lu, David V. Lu!!, Michael Ferguson, Vatan Aksoy Tezer

* [MSA] Testing Framework for MoveItSetupAssistant (`#1383 <https://github.com/ros-planning/moveit2/issues/1383>`_)
* MSA: Fix bug in sensors_3d reexport (`#1405 <https://github.com/ros-planning/moveit2/issues/1405>`_)
* update format of msa sensors_3d config (`#1398 <https://github.com/ros-planning/moveit2/issues/1398>`_)
  This mostly handles `#1388 <https://github.com/ros-planning/moveit2/issues/1388>`_ - it is still possible to enter an integer where a float is required (or to leave a field blank and end up with a string), but it's a big improvement over what we currently have
* Merge pull request `#1254 <https://github.com/ros-planning/moveit2/issues/1254>`_ from ros-planning/feature/msa
  MoveIt Setup Assistant - Merge the Feature branch
* [MSA] Clean up extra parentheses (`#1366 <https://github.com/ros-planning/moveit2/issues/1366>`_)
* PR Feedback
* [MSA] Initial Controllers and Simulation Steps Port (`#1252 <https://github.com/ros-planning/moveit2/issues/1252>`_)
* [MSA] Merge main into feature/msa (Part III) (`#1249 <https://github.com/ros-planning/moveit2/issues/1249>`_)
* [MSA] Launch Updates (`#1247 <https://github.com/ros-planning/moveit2/issues/1247>`_)
* [MSA] Merge main into feature/msa (Part II) (`#1240 <https://github.com/ros-planning/moveit2/issues/1240>`_)
* [MSA] Generate More New Launch Files (`#1213 <https://github.com/ros-planning/moveit2/issues/1213>`_)
* [MSA] Add Setup Step for Generating Launch Files (`#1129 <https://github.com/ros-planning/moveit2/issues/1129>`_)
* [MSA] Merge Upstream into `feature/msa` (`#1119 <https://github.com/ros-planning/moveit2/issues/1119>`_)
* [MSA] SRDF Setup (`#1057 <https://github.com/ros-planning/moveit2/issues/1057>`_)
* [MSA] Initial Refactor
* [MSA] Move Files Around (split into multiple packages, change to hpp extension)
* Contributors: David V. Lu!!, Michael Ferguson, Vatan Aksoy Tezer

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
