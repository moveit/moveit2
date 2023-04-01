^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_resources_prbt_ikfast_manipulator_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.7.1 (2023-03-23)
------------------
* Fix include install destination (`#2008 <https://github.com/ros-planning/moveit2/issues/2008>`_)
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  Co-authored-by: Tyler Weaver <maybe@tylerjw.dev>
* Contributors: Abhijeet Dasgupta

2.7.0 (2023-01-29)
------------------
* converted characters from string format to character format (`#1881 <https://github.com/ros-planning/moveit2/issues/1881>`_)
* Used C++ style casting for int type (`#1627 <https://github.com/ros-planning/moveit2/issues/1627>`_)
* Add `-Wunused-parameter` (`#1756 <https://github.com/ros-planning/moveit2/issues/1756>`_)
* Add braces around blocks. (`#999 <https://github.com/ros-planning/moveit2/issues/999>`_)
* Used C++ style cast instead of C style cast  (`#1628 <https://github.com/ros-planning/moveit2/issues/1628>`_)
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Contributors: Abhijeet Das Gupta, Chris Thrasher, Cory Crean, Sameer Gupta

2.6.0 (2022-11-10)
------------------
* Add missing depend (`#1684 <https://github.com/ros-planning/moveit2/issues/1684>`_)
* Use generate_parameter_library to load prbt ikfast kinematics parameters (`#1680 <https://github.com/ros-planning/moveit2/issues/1680>`_)
* Merge PR `#1553 <https://github.com/ros-planning/moveit2/issues/1553>`_: Improve cmake files
* Use standard exported targets: export\_${PROJECT_NAME} -> ${PROJECT_NAME}Targets
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* Use pragma once as header include guard (`#1525 <https://github.com/ros-planning/moveit2/issues/1525>`_)
* Remove __has_include statements (`#1481 <https://github.com/ros-planning/moveit2/issues/1481>`_)
* Contributors: Abishalini Sivaraman, J. Javan, Robert Haschke, Sebastian Jahr, Vatan Aksoy Tezer

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
