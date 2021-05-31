^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package run_move_group
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.4 (2021-05-31)
------------------

2.1.3 (2021-05-22)
------------------
* Load panda_hand_controller (`#460 <https://github.com/ros-planning/moveit2/issues/460>`_)
* Contributors: Jafar Abdi

2.1.2 (2021-04-20)
------------------
* Remove move_group namespace from MotionPlanning display (`#420 <https://github.com/ros-planning/moveit2/issues/420>`_)
* Fix node install directory in run_move_group (`#418 <https://github.com/ros-planning/moveit2/issues/418>`_)
* Contributors: Vatan Aksoy Tezer

2.1.1 (2021-04-12)
------------------
* Update launch files to use ros2 control spawner (`#405 <https://github.com/ros-planning/moveit2/issues/405>`_)
* Use fake_components::GenericSystem from ros2_control (`#361 <https://github.com/ros-planning/moveit2/issues/361>`_)
* Fix EXPORT install in CMake (`#372 <https://github.com/ros-planning/moveit2/issues/372>`_)
* MTC compatibility fixes (`#323 <https://github.com/ros-planning/moveit2/issues/323>`_)
* Fix node remapping
* Contributors: Henning Kayser, Jafar Abdi, Tyler Weaver

2.1.0 (2020-11-23)
------------------
* [maint] Wrap common cmake code in 'moveit_package()' macro (`#285 <https://github.com/ros-planning/moveit2/issues/285>`_)
  * New moveit_package() macro for compile flags, Windows support etc
  * Add package 'moveit_common' as build dependency for moveit_package()
  * Added -Wno-overloaded-virtual compiler flag for moveit_ros_planners_ompl
* [maint] Update MoveIt logo URLs in the demos' README (`#289 <https://github.com/ros-planning/moveit2/issues/289>`_)
* [ros2-migration] Port moveit_ros_warehouse to ROS 2 (`#273 <https://github.com/ros-planning/moveit2/issues/273>`_)
  * Port moveit_ros_warehouse package.xml, CMakeLists.txt
  * Apply ROS 2 migration steps
  * Enable warehouse in motion_planning_frame
  * Run mongo db in run_move_group.launch
* [ros2-migration] Port MoveGroupInterface and MotionPlanning display (`#272 <https://github.com/ros-planning/moveit2/issues/272>`_)
* Contributors: Henning Kayser, Jafar Abdi, Lior Lustgarten, Yu Yan

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
