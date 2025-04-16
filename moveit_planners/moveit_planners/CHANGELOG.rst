^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_planners
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.12.3 (2025-04-15)
-------------------

2.12.2 (2025-02-15)
-------------------
* fix chomp inclued, closes `#3228 <https://github.com/ros-planning/moveit2/issues/3228>`_ (`#3229 <https://github.com/ros-planning/moveit2/issues/3229>`_) (`#3231 <https://github.com/ros-planning/moveit2/issues/3231>`_)
* Contributors: Michael Ferguson, mergify[bot]

2.12.1 (2024-12-18)
-------------------

2.12.0 (2024-11-29)
-------------------
* have moveit_planners depend on chomp (`#3015 <https://github.com/ros-planning/moveit2/issues/3015>`_)
* Contributors: Michael Ferguson

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
* Migrate STOMP from ros-planning/stomp_moveit (`#2158 <https://github.com/ros-planning/moveit2/issues/2158>`_)
* Migrate stomp_moveit into moveit_planners
  * Move package into moveit_planners subdirectory
  * Rename stomp_moveit package to moveit_planners_stomp
  * List moveit_planners_stomp as package dependency
* Contributors: Henning Kayser

2.7.3 (2023-04-24)
------------------

2.7.2 (2023-04-18)
------------------

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
* Merge pr `#3000 <https://github.com/ros-planning/moveit2/issues/3000>`_: Pilz planner: improve reporting of invalid start joints
* add pilz planner to moveit_planners dependency
* Contributors: Robert Haschke, v4hn

2.4.0 (2022-01-20)
------------------

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* PILZ: Migrate and Restructure test directory
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* Contributors: Dave Coleman, Henning Kayser, Robert Haschke, Sebastian Jahr

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

2.1.2 (2021-04-20)
------------------

2.1.1 (2021-04-12)
------------------
* Enable ament_lint tests (`#340 <https://github.com/ros-planning/moveit2/issues/340>`_)
* Fix repo URLs in package.xml files
* Compile metapackages
* Contributors: Henning Kayser, Tyler Weaver

2.1.0 (2020-11-23)
------------------

2.0.0 (2020-02-17)
------------------

1.1.1 (2020-10-13)
------------------

1.1.0 (2020-09-04)
------------------

1.0.6 (2020-08-19)
------------------

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------

1.0.2 (2019-06-28)
------------------

1.0.1 (2019-03-08)
------------------

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* Contributors: Dave Coleman, Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------

0.10.2 (2018-10-24)
-------------------

0.10.1 (2018-05-25)
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
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------

0.7.0 (2016-01-30)
------------------

0.6.7 (2014-10-28)
------------------

0.6.6 (2014-07-06)
------------------

0.5.5 (2014-03-22)
------------------

0.5.4 (2014-02-06)
------------------

0.5.3 (2013-10-11)
------------------

0.5.2 (2013-09-23)
------------------

0.5.1 (2013-08-13)
------------------

0.5.0 (2013-07-15)
------------------

0.4.2 (2013-07-12)
------------------

0.4.1 (2013-07-04)
------------------

0.4.0 (2013-05-27)
------------------
