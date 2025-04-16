^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_setup_app_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.9 (2025-04-15)
------------------

2.5.8 (2025-02-09)
------------------

2.5.7 (2024-12-29)
------------------

2.5.6 (2024-11-17)
------------------

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
* Contributors: Robert Haschke, mergify[bot]

2.5.4 (2022-11-04)
------------------
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_) (`#1555 <https://github.com/ros-planning/moveit2/issues/1555>`_)
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Contributors: mergify[bot]

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
