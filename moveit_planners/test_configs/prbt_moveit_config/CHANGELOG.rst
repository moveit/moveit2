^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_resources_prbt_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Update ros2_control usage (`#2620 <https://github.com/ros-planning/moveit2/issues/2620>`_)
  * Update ros2_control usage
  * Update xacro file
* [Planning Pipeline Refactoring] `#2 <https://github.com/ros-planning/moveit2/issues/2>`_ Enable chaining planners (`#2457 <https://github.com/ros-planning/moveit2/issues/2457>`_)
  * Enable chaining multiple planners
* [Planning Pipeline Refactoring] `#1 <https://github.com/ros-planning/moveit2/issues/1>`_ Simplify Adapter - Planner chain (`#2429 <https://github.com/ros-planning/moveit2/issues/2429>`_)
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Contributors: Sebastian Jahr

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

2.7.0 (2023-01-29)
------------------
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Contributors: Christian Henkel

2.6.0 (2022-11-10)
------------------
* Use generate_parameter_library to load kinematics parameters (`#1568 <https://github.com/ros-planning/moveit2/issues/1568>`_)
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* Contributors: Abishalini Sivaraman, Sebastian Jahr

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Rename cartesian_limits.yaml (`#1422 <https://github.com/ros-planning/moveit2/issues/1422>`_)
  Co-authored-by: Tyler Weaver <maybe@tylerjw.dev>
* Delete the fake_controller_manager from ROS1 (`#1413 <https://github.com/ros-planning/moveit2/issues/1413>`_)
* Merge remote-tracking branch 'origin/main' into feature/msa
* Launch file cleanup (`#1380 <https://github.com/ros-planning/moveit2/issues/1380>`_)
* Contributors: AndyZe, Vatan Aksoy Tezer

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Enable rolling / jammy CI (again) (`#1134 <https://github.com/ros-planning/moveit2/issues/1134>`_)
  * Use ros2_control binaries
  * Use output screen instead of explicitly stating stderr
* Merge https://github.com/ros-planning/moveit/commit/25a63b920adf46f0a747aad92ada70d8afedb3ec
* Contributors: Abishalini, Vatan Aksoy Tezer

2.4.0 (2022-01-20)
------------------

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Bump new packages to 2.3.0
* Add PRBT test dependencies for PILZ planner (`#909 <https://github.com/ros-planning/moveit2/issues/909>`_)
  * Adding PRBT config
  * Port prbt packages to ROS 2
  * Move PRBT into test_configs directory
  * Fix pre-commit for pilz test_config
  * Revert "Docker - Temporarily move moveit_resources under target workspace due to `#885 <https://github.com/ros-planning/moveit2/issues/885>`_ (`#915 <https://github.com/ros-planning/moveit2/issues/915>`_)"
  * Reset repos file entry for moveit_resources
  * prbt_support: drop all test code
  Co-authored-by: Christian Henkel <post@henkelchristian.de>
  Co-authored-by: Michael Görner <me@v4hn.de>
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Contributors: Henning Kayser, Tyler Weaver

* initial commit from upstream PilzDE/prbt_movit_config version 0.5.18
