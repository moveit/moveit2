^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_resources_prbt_ikfast_manipulator_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.4 (2022-11-04)
------------------
* Use pragma once as header include guard (`#1525 <https://github.com/ros-planning/moveit2/issues/1525>`_) (`#1652 <https://github.com/ros-planning/moveit2/issues/1652>`_)
  (cherry picked from commit 7d758de1b2f2904b8c85520129fa8d48aad93713)
  Co-authored-by: J. Javan <J-Javan@users.noreply.github.com>
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_) (`#1555 <https://github.com/ros-planning/moveit2/issues/1555>`_)
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Remove __has_include statements (`#1481 <https://github.com/ros-planning/moveit2/issues/1481>`_) (`#1483 <https://github.com/ros-planning/moveit2/issues/1483>`_)
* Contributors: mergify[bot]

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Merge remote-tracking branch 'upstream/main' into feature/msa
* Removing some boost usage (`#1331 <https://github.com/ros-planning/moveit2/issues/1331>`_)
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Contributors: David V. Lu, Henry Moore, Jafar

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Fix prbt_ikfast win compilation (`#1161 <https://github.com/ros-planning/moveit2/issues/1161>`_)
* Remove new operators (`#1135 <https://github.com/ros-planning/moveit2/issues/1135>`_)
  replace new operator with make_shared
* Remove unused parameters. (`#1018 <https://github.com/ros-planning/moveit2/issues/1018>`_)
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
* Contributors: Cory Crean, Sencer Yazıcı, Tobias Fischer

2.4.0 (2022-01-20)
------------------
* Replace NULL with nullptr (`#961 <https://github.com/ros-planning/moveit2/issues/961>`_)
  * Fixes `#841 <https://github.com/ros-planning/moveit2/issues/841>`_
* Contributors: Stephanie Eng

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

* initial commit from upstream PilzDE/pilz_robots version 0.5.19 (2020-09-07)
