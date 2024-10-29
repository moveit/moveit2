^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_resources_prbt_pg70_support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Replace Variable PROJECT_NAME in CMakeLists.txt with the actual name (`#2020 <https://github.com/ros-planning/moveit2/issues/2020>`_)
* Contributors: Shobuj Paul

2.7.2 (2023-04-18)
------------------

2.7.1 (2023-03-23)
------------------

2.7.0 (2023-01-29)
------------------

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

* initial commit from upstream PilzDE/prbt_grippers version 0.0.4
