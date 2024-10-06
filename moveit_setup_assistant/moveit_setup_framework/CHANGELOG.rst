^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_setup_framework
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.11.0 (2024-09-16)
-------------------
* Cast of "max_velocity" and "max_acceleration" values to double (`#2803 <https://github.com/moveit/moveit2/issues/2803>`_)
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Contributors: Jorge Pérez Ramos

2.10.0 (2024-06-13)
-------------------
* Apply clang-tidy fixes
* Migrate ros-planning org to moveit (`#2847 <https://github.com/moveit/moveit2/issues/2847>`_)
  * Rename github.com/ros-planning -> github.com/moveit
  * Rename ros-planning.github.io -> moveit.github.io
  * Rename ros-planning organization in docker and CI workflow files
  - ghcr.io/ros-planning -> ghcr.io/moveit
  - github.repository == 'moveit/*''
* Unify log names (`#2720 <https://github.com/moveit/moveit2/issues/2720>`_)
  Co-authored-by: Abishalini Sivaraman <abi.gpuram@gmail.com>
* CMake format and lint in pre-commit (`#2683 <https://github.com/moveit/moveit2/issues/2683>`_)
* Contributors: Robert Haschke, Sebastian Jahr, Tyler Weaver

2.9.0 (2024-01-09)
------------------
* Node logging for the rest of MoveIt (`#2599 <https://github.com/ros-planning/moveit2/issues/2599>`_)
* Update pre-commit and add to .codespell_words (`#2465 <https://github.com/ros-planning/moveit2/issues/2465>`_)
* Fix `#1971 <https://github.com/ros-planning/moveit2/issues/1971>`_ (`#2428 <https://github.com/ros-planning/moveit2/issues/2428>`_)
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Fix collisions_updater CLI if no package is used (`#2344 <https://github.com/ros-planning/moveit2/issues/2344>`_)
  * Fix collisions_updater CLI if no package is used
  The exceptions introduced with https://github.com/ros-planning/moveit2/pull/2032
  prevented from running the collisions updater CLI without a ROS package.
  This fix makes ROS packages optional again, allowing to use the CLI with absolute
  paths only.
  * Improve warn message wording
* Contributors: David V. Lu!!, Henning Kayser, Sebastian Jahr, Shobuj Paul, Tyler Weaver

2.8.0 (2023-09-10)
------------------
* Replaced boost::algorithm::join with fmt::join (`#2273 <https://github.com/ros-planning/moveit2/issues/2273>`_)
  * Replaced boost::algorithm::join with fmt::join
  * Made changes in CMakeLists.txt to accomodate fmt
  * Updated package.xml files
  * removed redundant boost dependencies
  * Rename variables -> variable
  ---------
  Co-authored-by: Sebastian Castro <4603398+sea-bass@users.noreply.github.com>
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Contributors: Shobuj Paul

2.7.4 (2023-05-18)
------------------

2.7.3 (2023-04-24)
------------------
* Replace Variable PROJECT_NAME in CMakeLists.txt with the actual name (`#2020 <https://github.com/ros-planning/moveit2/issues/2020>`_)
* Contributors: Shobuj Paul

2.7.2 (2023-04-18)
------------------
* Add URDF Loader Exceptions and Fix Infinite While-Loop when URDF file isn't in a ROS Package (`#2032 <https://github.com/ros-planning/moveit2/issues/2032>`_)
  * Fixed infinite while loop in utilities.cpp and added some exception handling to start screen widget
  * Fix trailing whitespace, fix getSharePath exception catch on empty request
  * Fix clang tidy suggestion and error message updates based on pr comments
* Contributors: Chance Cardona

2.7.1 (2023-03-23)
------------------
* Fix member naming (`#1949 <https://github.com/ros-planning/moveit2/issues/1949>`_)
  * Update clang-tidy rules for readability-identifier-naming
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Contributors: Robert Haschke

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
* Use <> for non-local headers (`#1734 <https://github.com/ros-planning/moveit2/issues/1734>`_)
  Unless a header lives in the same or a child directory of the file
  including it, it's recommended to use <> for the #include statement.
  For more information, see the C++ Core Guidelines item SF.12
  https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#sf12-prefer-the-quoted-form-of-include-for-files-relative-to-the-including-file-and-the-angle-bracket-form-everywhere-else
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
* Contributors: Robert Haschke, Sebastian Jahr

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Rename cartesian_limits.yaml (`#1422 <https://github.com/ros-planning/moveit2/issues/1422>`_)
  Co-authored-by: Tyler Weaver <maybe@tylerjw.dev>
* [MSA] Testing Framework for MoveItSetupAssistant (`#1383 <https://github.com/ros-planning/moveit2/issues/1383>`_)
* Merge pull request `#1254 <https://github.com/ros-planning/moveit2/issues/1254>`_ from ros-planning/feature/msa
  MoveIt Setup Assistant - Merge the Feature branch
* [MSA] One XML Parser (`#1382 <https://github.com/ros-planning/moveit2/issues/1382>`_)
* [MSA] Fix SRDF Initialization Bug / Copy Paste Error (`#1381 <https://github.com/ros-planning/moveit2/issues/1381>`_)
* Merge remote-tracking branch 'upstream/main' into feature/msa
* [MSA] Clean up extra parentheses (`#1366 <https://github.com/ros-planning/moveit2/issues/1366>`_)
* [MSA] ros2_control Integration (`#1299 <https://github.com/ros-planning/moveit2/issues/1299>`_)
* [MSA] Workaround to launch files without controllers (`#1275 <https://github.com/ros-planning/moveit2/issues/1275>`_)
* PR Feedback
* [MSA] Migration Cleanup (`#1253 <https://github.com/ros-planning/moveit2/issues/1253>`_)
* [MSA] Initial Controllers and Simulation Steps Port (`#1252 <https://github.com/ros-planning/moveit2/issues/1252>`_)
* [MSA] Merge main into feature/msa (Part III) (`#1249 <https://github.com/ros-planning/moveit2/issues/1249>`_)
* [MSA] Fix loading from previous config (`#1246 <https://github.com/ros-planning/moveit2/issues/1246>`_)
* [MSA] Generate joint_limits.yaml and cartesian_limits.yaml (`#1245 <https://github.com/ros-planning/moveit2/issues/1245>`_)
* [MSA] Merge main into feature/msa (Part II) (`#1240 <https://github.com/ros-planning/moveit2/issues/1240>`_)
* [MSA] Generate More New Launch Files (`#1213 <https://github.com/ros-planning/moveit2/issues/1213>`_)
* [MSA] Existing Package Loading Tweaks (`#1212 <https://github.com/ros-planning/moveit2/issues/1212>`_)
* [MSA] Fix for long TLDs (`#1214 <https://github.com/ros-planning/moveit2/issues/1214>`_)
* [MSA] Add Setup Step for Generating Launch Files (`#1129 <https://github.com/ros-planning/moveit2/issues/1129>`_)
* [MSA] Simplify loading of new SRDF (`#1102 <https://github.com/ros-planning/moveit2/issues/1102>`_)
* [MSA] Merge Upstream into `feature/msa` (`#1119 <https://github.com/ros-planning/moveit2/issues/1119>`_)
* [MSA] Upgrade templates to ROS 2 (`#1101 <https://github.com/ros-planning/moveit2/issues/1101>`_)
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* [MSA] SRDF Setup (`#1057 <https://github.com/ros-planning/moveit2/issues/1057>`_)
* [MSA] Initial Refactor
* Move Files Around (split into multiple packages, change to hpp extension)
* Contributors: AndyZe, David V. Lu!!, Vatan Aksoy Tezer

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
