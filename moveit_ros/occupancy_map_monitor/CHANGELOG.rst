^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_occupancy_map_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.13.2 (2025-04-16)
-------------------

2.13.1 (2025-04-15)
-------------------

2.13.0 (2025-02-15)
-------------------

2.12.0 (2024-11-29)
-------------------
* Enhancement/use hpp for headers (`#3113 <https://github.com/ros-planning/moveit2/issues/3113>`_)
* Contributors: Tom Noble

2.11.0 (2024-09-16)
-------------------
* Switch to system version of octomap (`#2881 <https://github.com/moveit/moveit2/issues/2881>`_)
* Contributors: Chris Lalancette

2.10.0 (2024-06-13)
-------------------
* Enforce liboctomap-dev by using a cmake version range
* Migrate ros-planning org to moveit (`#2847 <https://github.com/moveit/moveit2/issues/2847>`_)
  * Rename github.com/ros-planning -> github.com/moveit
  * Rename ros-planning.github.io -> moveit.github.io
  * Rename ros-planning organization in docker and CI workflow files
  - ghcr.io/ros-planning -> ghcr.io/moveit
  - github.repository == 'moveit/*''
* Unify log names (`#2720 <https://github.com/moveit/moveit2/issues/2720>`_)
  Co-authored-by: Abishalini Sivaraman <abi.gpuram@gmail.com>
* CMake format and lint in pre-commit (`#2683 <https://github.com/moveit/moveit2/issues/2683>`_)
* Contributors: Henning Kayser, Robert Haschke, Sebastian Jahr, Tyler Weaver

2.9.0 (2024-01-09)
------------------
* Node logging in moveit_core (`#2503 <https://github.com/ros-planning/moveit2/issues/2503>`_)
* Use node logging in moveit_ros (`#2482 <https://github.com/ros-planning/moveit2/issues/2482>`_)
* Add new clang-tidy style rules (`#2177 <https://github.com/ros-planning/moveit2/issues/2177>`_)
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Contributors: Sebastian Jahr, Tyler Weaver

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
* Contributors: Sebastian Jahr

2.7.1 (2023-03-23)
------------------

2.7.0 (2023-01-29)
------------------
* converted characters from string format to character format (`#1881 <https://github.com/ros-planning/moveit2/issues/1881>`_)
* Switch to clang-format-14 (`#1877 <https://github.com/ros-planning/moveit2/issues/1877>`_)
  * Switch to clang-format-14
  * Fix clang-format-14
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Enable `-Wold-style-cast` (`#1770 <https://github.com/ros-planning/moveit2/issues/1770>`_)
* Remove `MOVEIT_LIB_NAME` (`#1751 <https://github.com/ros-planning/moveit2/issues/1751>`_)
  It's more readable and searchable if we just spell out the target
  name.
* Add braces around blocks. (`#999 <https://github.com/ros-planning/moveit2/issues/999>`_)
* Used C++ style cast instead of C style cast  (`#1628 <https://github.com/ros-planning/moveit2/issues/1628>`_)
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Fix clang-tidy issues (`#1706 <https://github.com/ros-planning/moveit2/issues/1706>`_)
  * Blindly apply automatic clang-tidy fixes
  * Exemplarily cleanup a few automatic clang-tidy fixes
  * Clang-tidy fixups
  * Missed const-ref fixups
  * Fix unsupported non-const -> const
  * More fixes
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Contributors: Abhijeet Das Gupta, Chris Thrasher, Christian Henkel, Cory Crean, Henning Kayser, Robert Haschke, Sameer Gupta

2.6.0 (2022-11-10)
------------------
* Merge PR `#1553 <https://github.com/ros-planning/moveit2/issues/1553>`_: Improve cmake files
* Use standard exported targets: export\_${PROJECT_NAME} -> ${PROJECT_NAME}Targets
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* Contributors: Robert Haschke, Sebastian Jahr

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Remove no longer needed no-pedantic option in moveit_ros_occupancy_map_monitor package (`#1397 <https://github.com/ros-planning/moveit2/issues/1397>`_)
* Merge remote-tracking branch 'upstream/main' into feature/msa
* Removing some boost usage (`#1331 <https://github.com/ros-planning/moveit2/issues/1331>`_)
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Merge pull request `#3106 <https://github.com/ros-planning/moveit2/issues/3106>`_ from v4hn/pr-master-bind-them-all / banish bind()
* banish bind()
* various: prefer objects and references over pointers
* Contributors: David V. Lu, Henry Moore, Jafar, Michael Görner, Nathan Brooks, v4hn

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Enable cppcheck (`#1224 <https://github.com/ros-planning/moveit2/issues/1224>`_)
  Co-authored-by: jeoseo <jeongwooseo2012@gmail.com>
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Remove unused includes for boost::bind (`#1220 <https://github.com/ros-planning/moveit2/issues/1220>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
  Adaption of https://github.com/ros-planning/moveit/pull/3106
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* various: prefer object and references over pointers
  source: https://github.com/ros-planning/moveit/pull/3106/commits/1a8e5715e3142a92977ac585031b9dc1871f8718; this commit contains minor changes when compared to the source commit which it is based on, these changes are limited to ensuring compatibility with ROS2.
* Disable separate TransformListener thread in OccupancyMapServer (`#1130 <https://github.com/ros-planning/moveit2/issues/1130>`_)
* Merge https://github.com/ros-planning/moveit/commit/ab42a1d7017b27eb6c353fb29331b2da08ab0039
* 1.1.9
* Misc fixes for time and transforms (`#768 <https://github.com/ros-planning/moveit2/issues/768>`_)
  * Fix setting shape_transform_cache_lookup_wait_time from seconds
  * Fix setting last_update_time from seconds
  * Check the return value of canTransform
* 1.1.8
* Remove unused parameters. (`#1018 <https://github.com/ros-planning/moveit2/issues/1018>`_)
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* 1.1.7
* Switch to std::bind (`#2967 <https://github.com/ros-planning/moveit2/issues/2967>`_)
  * boost::bind -> std::bind
  grep -rlI --exclude-dir=.git "boost::bind" | xargs sed -i 's/boost::bind/std::bind/g'
  * Convert bind placeholders
  grep -rlI --exclude-dir=.git " _[0-9]" | xargs sed -i 's/ _\([0-9]\)/ std::placeholders::_\1/g'
  * Update bind include header
  grep -rlI --exclude-dir=.git "boost/bind" | xargs sed -i 's#boost/bind.hpp#functional#'
* 1.1.6
* Contributors: Abishalini, Cory Crean, Henning Kayser, Jafar, Jafar Abdi, Jochen Sprickerhof, Robert Haschke, Stephanie Eng, jeoseo, v4hn

2.4.0 (2022-01-20)
------------------
* Fix boost linking errors for Windows (`#957 <https://github.com/ros-planning/moveit2/issues/957>`_)
* Replace NULL with nullptr (`#961 <https://github.com/ros-planning/moveit2/issues/961>`_)
  * Fixes `#841 <https://github.com/ros-planning/moveit2/issues/841>`_
* moveit_build_options()
  Declare common build options like CMAKE_CXX_STANDARD, CMAKE_BUILD_TYPE,
  and compiler options (namely warning flags) once.
  Each package depending on moveit_core can use these via moveit_build_options().
* Contributors: Akash, Robert Haschke, Stephanie Eng

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Replaced C-Style Cast with C++ Style Cast. (`#935 <https://github.com/ros-planning/moveit2/issues/935>`_)
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* Get rid of "std::endl" (`#918 <https://github.com/ros-planning/moveit2/issues/918>`_)
* Fix boost linking errors (`#900 <https://github.com/ros-planning/moveit2/issues/900>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* Prefer std::make_shared over new operator (`#2756 <https://github.com/ros-planning/moveit/issues/2756>`_)
* Contributors: Dave Coleman, David V. Lu!!, Henning Kayser, Kaustubh, Michael Görner, Parthasarathy Bana, Robert Haschke, Vatan Aksoy Tezer

2.3.0 (2021-10-08)
------------------
* Error if Octomap 'map_frame' is not provided (`#667 <https://github.com/ros-planning/moveit2/issues/667>`_)
* OccupancyMapMonitor tests using Dependency Injection (`#569 <https://github.com/ros-planning/moveit2/issues/569>`_)
* Refactors for OccMapTree in PlanningScene (`#2684 <https://github.com/ros-planning/moveit2/issues/2684>`_)
* Move OccMapTree to moveit_core/collision_detection
* Contributors: AndyZe, Henning Kayser, Simon Schmeisser, Tyler Weaver, Jafar Abdi

2.2.1 (2021-07-12)
------------------

2.2.0 (2021-06-30)
------------------
* [sync] with MoveIt's master branch up-to https://github.com/ros-planning/moveit/commit/0d0a6a171b3fbea97a0c4f284e13433ba66a4ea4
  * Add missing OCTOMAP_INCLUDE_DIRS (`#2671 <https://github.com/ros-planning/moveit/issues/2671>`_)
  * document solution in ROS_ERROR on failed self-filtering (`#2627 <https://github.com/ros-planning/moveit/issues/2627>`_)
  * It's not an error not to define a plugin (`#2521 <https://github.com/ros-planning/moveit/issues/2521>`_)
* Contributors: 0Nel, JafarAbdi, Michael Görner, Robert Haschke, Simon Schmeisser, Tyler Weaver, v4hn

2.1.4 (2021-05-31)
------------------

2.1.3 (2021-05-22)
------------------

2.1.2 (2021-04-20)
------------------

2.1.1 (2021-04-12)
------------------
* Fix EXPORT install in CMake (`#372 <https://github.com/ros-planning/moveit2/issues/372>`_)
* Porting moveit_ros_perception/pointcloud_octomap_updater (`#307 <https://github.com/ros-planning/moveit2/issues/307>`_)
* Fix repo URLs in package.xml files
* Contributors: Henning Kayser, Tyler Weaver, Yu Yan

2.1.0 (2020-11-23)
------------------
* [maint] Wrap common cmake code in 'moveit_package()' macro (`#285 <https://github.com/ros-planning/moveit2/issues/285>`_)
  * New moveit_package() macro for compile flags, Windows support etc
  * Add package 'moveit_common' as build dependency for moveit_package()
  * Added -Wno-overloaded-virtual compiler flag for moveit_ros_planners_ompl
* Contributors: Lior Lustgarten

2.0.0 (2020-02-17)
------------------
* [port] Port occupancy_map_monitor to ROS2 (`#148 <https://github.com/ros-planning/moveit2/issues/148>`_)
* [fix] replace rosunit -> ament_cmake_gtest
* [port] Port rdf_loader to ROS2 (`#104 <https://github.com/ros-planning/moveit2/issues/104>`_)
* Contributors: Henning Kayser, Jafar Abdi, Yu Yan

1.1.1 (2020-10-13)
------------------
* [fix] clang-tidy issues on Travis (`#2337 <https://github.com/ros-planning/moveit/issues/2337>`_)
  * Replace typedefs with using declarations
  * Move default destructor definitions to headers
  * Silent spurious clang-tidy warning
  * Move variable definitions to their usage location
* [maint] Cleanup MSA includes (`#2351 <https://github.com/ros-planning/moveit/issues/2351>`_)
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski, Robert Haschke

1.1.0 (2020-09-04)
------------------

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
* [fix]   Add error message on failure to initialize occupancy map monitor (`#1873 <https://github.com/ros-planning/moveit/issues/1873>`_)
* [fix]   Update occupancy grid when loaded from file (`#1594 <https://github.com/ros-planning/moveit/issues/1594>`_)
* [maint] Apply clang-tidy fix (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint] Windows build fixes
  * Fix header inclusion and other MSVC build errors (`#1636 <https://github.com/ros-planning/moveit/issues/1636>`_)
  * Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 for portability (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [maint] move occupancy_map_monitor into its own package (`#1533 <https://github.com/ros-planning/moveit/issues/1533>`_)
* Contributors: Bjar Ne, Dale Koenig, Raphael Druon, Robert Haschke, Sean Yen, Simon Schmeisser, Yu, Yan, jschleicher
