^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_setup_framework
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.9 (2025-04-15)
------------------

2.5.8 (2025-02-09)
------------------
* Explicit convert from std::filesystem::path to std::string for Windows compatibility (backport `#3249 <https://github.com/ros-planning/moveit2/issues/3249>`_) (`#3253 <https://github.com/ros-planning/moveit2/issues/3253>`_)
* Fix: Conditionally install launch directory in generated MoveIt config package (backport `#3191 <https://github.com/ros-planning/moveit2/issues/3191>`_) (`#3194 <https://github.com/ros-planning/moveit2/issues/3194>`_)
* Contributors: Filippo Bosi, Silvio Traversaro, mergify[bot]

2.5.7 (2024-12-29)
------------------

2.5.6 (2024-11-17)
------------------
* Cast of "max_velocity" and "max_acceleration" values to double (`#2803 <https://github.com/ros-planning/moveit2/issues/2803>`_) (`#3038 <https://github.com/ros-planning/moveit2/issues/3038>`_)
* Fix `#1971 <https://github.com/ros-planning/moveit2/issues/1971>`_ (`#2428 <https://github.com/ros-planning/moveit2/issues/2428>`_) (`#2430 <https://github.com/ros-planning/moveit2/issues/2430>`_)
* Contributors: Michael Ferguson, Jorge Pérez Ramos, David V. Lu!!, mergify[bot]

2.5.5 (2023-09-10)
------------------
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
* Use <> for non-local headers (`#1765 <https://github.com/ros-planning/moveit2/issues/1765>`_)
  Unless a header lives in the same or a child directory of the file
  including it, it's recommended to use <> for the #include statement.
  For more information, see the C++ Core Guidelines item SF.12
  https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#sf12-prefer-the-quoted-form-of-include-for-files-relative-to-the-including-file-and-the-angle-bracket-form-everywhere-else
  (cherry picked from commit 7a1f2a101f9aeb8557e8a31656bbe1a6d53b430e)
* Re-enable clang-tidy check `performance-unnecessary-value-param` (backport `#1703 <https://github.com/ros-planning/moveit2/issues/1703>`_)
  * Re-enable clang-tidy check performance-unnecessary-value-param (`#1703 <https://github.com/ros-planning/moveit2/issues/1703>`_)
  * Fix clang-tidy issues (`#1706 <https://github.com/ros-planning/moveit2/issues/1706>`_)
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  Co-authored-by: Robert Haschke <rhaschke@users.noreply.github.com>
* Contributors: Chris Thrasher, Robert Haschke, mergify[bot]

2.5.4 (2022-11-04)
------------------
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
