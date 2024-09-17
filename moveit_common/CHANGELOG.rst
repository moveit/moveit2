^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* CMake format and lint in pre-commit (`#2683 <https://github.com/moveit/moveit2/issues/2683>`_)
* Contributors: Robert Haschke, Tyler Weaver

2.9.0 (2024-01-09)
------------------
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Contributors: Sebastian Jahr

2.8.0 (2023-09-10)
------------------

2.7.4 (2023-05-18)
------------------

2.7.3 (2023-04-24)
------------------
* Replace check for the ROS_DISTRO env variable with a check for the rclcpp version (`#2135 <https://github.com/ros-planning/moveit2/issues/2135>`_)
* Replace Variable PROJECT_NAME in CMakeLists.txt with the actual name (`#2020 <https://github.com/ros-planning/moveit2/issues/2020>`_)
* Contributors: Jafar, Shobuj Paul

2.7.2 (2023-04-18)
------------------
* Switch from qos_event.hpp to event_handler.hpp (`#2111 <https://github.com/ros-planning/moveit2/issues/2111>`_)
  * Switch from qos_event.hpp to event_handler.hpp
  * moveit_common: Add a cmake interface library to keep humble support on main
  * Include qos_event.hpp or event_handler.hpp depending on the ROS 2 version
  * Fix ament_lint_cmake
  * Fix clang-tidy
  * PRIVATE linking in some cases
  * Update moveit_common/cmake/moveit_package.cmake
  Co-authored-by: Chris Thrasher <chrisjthrasher@gmail.com>
  * Fix servo and cleanup excessive CMake variable usage
  * Cleanup & make compiling
  * Small variable naming and const cleanup
  * Restore OpenCV linking
  * Public/private linking fixup
  * Revert "Restore OpenCV linking"
  This reverts commit 57a9efa806e59223e35a1f7e998d7b52f930c263.
  ---------
  Co-authored-by: JafarAbdi <jafar.uruc@gmail.com>
  Co-authored-by: Jafar <cafer.abdi@gmail.com>
  Co-authored-by: AndyZe <andyz@utexas.edu>
  Co-authored-by: Chris Thrasher <chrisjthrasher@gmail.com>
* enabled -wformat (`#2065 <https://github.com/ros-planning/moveit2/issues/2065>`_)
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Contributors: Heramb Modugula, Sebastian Jahr

2.7.1 (2023-03-23)
------------------

2.7.0 (2023-01-29)
------------------
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Enable `-Wold-style-cast` (`#1770 <https://github.com/ros-planning/moveit2/issues/1770>`_)
* Add `-Wunused-parameter` (`#1756 <https://github.com/ros-planning/moveit2/issues/1756>`_)
* Add `-Wunused-function` (`#1754 <https://github.com/ros-planning/moveit2/issues/1754>`_)
* Contributors: Chris Thrasher, Christian Henkel

2.6.0 (2022-11-10)
------------------
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* Contributors: Sebastian Jahr

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------

2.4.0 (2022-01-20)
------------------
* Merge https://github.com/ros-planning/moveit/commit/f3ac6070497da90da33551fc1dc3a68938340413
* Contributors: Abishalini

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* Add backward_ros to moveit_common (`#794 <https://github.com/ros-planning/moveit2/issues/794>`_)
* Contributors: Dave Coleman, Henning Kayser, Tyler Weaver

2.3.0 (2021-10-08)
------------------
* Upgrade to C++17
* Contributors: Henning Kayser, Tyler Weaver

2.2.1 (2021-07-12)
------------------
* Set project VERSION in moveit_common, fix sonames (`#532 <https://github.com/ros-planning/moveit2/issues/532>`_)
* Contributors: Henning Kayser

2.2.0 (2021-06-30)
------------------

2.1.4 (2021-05-31)
------------------
* Fix ament_cmake 'buildtool_depend' in moveit_common (`#480 <https://github.com/ros-planning/moveit2/issues/480>`_)
* Contributors: Henning Kayser

2.1.3 (2021-05-22)
------------------

2.1.2 (2021-04-20)
------------------
* Unify PickNik name in copyrights (`#419 <https://github.com/ros-planning/moveit2/issues/419>`_)
* Contributors: Tyler Weaver

2.1.1 (2021-04-12)
------------------
* Enable linting for moveit_common package (`#379 <https://github.com/ros-planning/moveit2/issues/379>`_)
* Enable ament_lint tests (`#340 <https://github.com/ros-planning/moveit2/issues/340>`_)
* Fix repo URLs in package.xml files
* Contributors: Henning Kayser, Tyler Weaver

2.1.0 (2020-11-23)
------------------
* [maint] Wrap common cmake code in 'moveit_package()' macro (`#285 <https://github.com/ros-planning/moveit2/issues/285>`_)
  * New moveit_package() macro for compile flags, Windows support etc
  * Add package 'moveit_common' as build dependency for moveit_package()
  * Added -Wno-overloaded-virtual compiler flag for moveit_ros_planners_ompl
* Contributors: Lior Lustgarten

1.1.1 (2020-10-13)
------------------

1.1.0 (2020-09-07)
------------------

1.0.1 (2019-03-08)
------------------

1.0.0 (2019-02-24)
------------------

0.10.8 (2018-12-24)
-------------------

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29 19:44)
-------------------------

0.10.3 (2018-10-29 04:12)
-------------------------

0.10.2 (2018-10-24)
-------------------

0.10.1 (2018-05-25)
-------------------

0.10.0 (2018-05-22)
-------------------

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------

0.9.9 (2017-08-06)
------------------

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

0.9.1 (2016-10-21)
------------------
