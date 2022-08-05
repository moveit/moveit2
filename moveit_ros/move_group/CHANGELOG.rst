^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_move_group
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Merge pull request `#1402 <https://github.com/ros-planning/moveit2/issues/1402>`_ from Abishalini/pr-sync-a436a97
  Sync with MoveIt
* Merge https://github.com/ros-planning/moveit/commit/a436a9771f7445c162cc3090c4c7c57bdb5bf194
* Merge remote-tracking branch 'origin/main' into feature/msa
* Remove manipulation from moveit_ros (`#1177 <https://github.com/ros-planning/moveit2/issues/1177>`_)
* Merge pull request `#3137 <https://github.com/ros-planning/moveit2/issues/3137>`_ from TAMS-Group/pr-master-monitor-dynamics
  move_group can optionally monitor dynamics
* optionally enable dynamics monitoring in move_group node
* convert move_group to LOGNAME
* Merge pull request `#3106 <https://github.com/ros-planning/moveit/issues/3106>`_ from v4hn/pr-master-bind-them-all / banish bind()
* banish bind()
* Contributors: Abishalini, Michael Görner, Stephanie Eng, Tyler Weaver, Vatan Aksoy Tezer, v4hn

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Merge https://github.com/ros-planning/moveit/commit/72d919299796bffc21f5eb752d66177841dc3442
* Enable cppcheck (`#1224 <https://github.com/ros-planning/moveit2/issues/1224>`_)
  Co-authored-by: jeoseo <jeongwooseo2012@gmail.com>
* Make TOTG the default time-parameterization algorithm everywhere (`#1218 <https://github.com/ros-planning/moveit2/issues/1218>`_)
  Co-authored-by: Jafar <cafer.abdi@gmail.com>
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
  Adaption of https://github.com/ros-planning/moveit/pull/3106
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* Merge https://github.com/ros-planning/moveit/commit/ab42a1d7017b27eb6c353fb29331b2da08ab0039
* 1.1.9
* Fix missing include (`#3051 <https://github.com/ros-planning/moveit2/issues/3051>`_)
* 1.1.8
* Improve loading of planning pipelines (`#3036 <https://github.com/ros-planning/moveit2/issues/3036>`_)
  Ensure that planning pipelines considered in `~/planning_pipelines/*` actually have a `planning_plugin` parameter defined.
  Otherwise, issue an error message.
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
* Contributors: Abishalini, AndyZe, Cory Crean, Henning Kayser, Jafar, Jochen Sprickerhof, Robert Haschke, Tobias Fischer, jeoseo, v4hn

2.4.0 (2022-01-20)
------------------
* moveit_build_options()
  Declare common build options like CMAKE_CXX_STANDARD, CMAKE_BUILD_TYPE,
  and compiler options (namely warning flags) once.
  Each package depending on moveit_core can use these via moveit_build_options().
* Contributors: Robert Haschke

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* Get rid of "std::endl" (`#918 <https://github.com/ros-planning/moveit2/issues/918>`_)
* Consider simulated time (`#883 <https://github.com/ros-planning/moveit2/issues/883>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: Dave Coleman, David V. Lu!!, Felix von Drigalski, Gaël Écorchard, Henning Kayser, Parthasarathy Bana, Robert Haschke, pvanlaar

2.3.0 (2021-10-08)
------------------
* Fix warnings in Galactic and Rolling (`#598 <https://github.com/ros-planning/moveit2/issues/598>`_)
  * Use __has_includes preprocessor directive for deprecated headers
  * Fix parameter template types
  * Proper initialization of smart pointers, rclcpp::Duration
* Contributors: Vatan Aksoy Tezer

2.2.1 (2021-07-12)
------------------

2.2.0 (2021-06-30)
------------------
* Fix deprecated planner namespace in MoveGroup (`#524 <https://github.com/ros-planning/moveit2/issues/524>`_)
* [sync] MoveIt's master branch up-to https://github.com/ros-planning/moveit/commit/0d0a6a171b3fbea97a0c4f284e13433ba66a4ea4
  * Support multiple planning pipelines with MoveGroup via MoveItCpp (`#2127 <https://github.com/ros-planning/moveit/issues/2127>`_)
  * Fix missing isEmpty check in compute_ik service (`#2544 <https://github.com/ros-planning/moveit/issues/2544>`_)
* Contributors: Henning Kayser, Jafar Abdi, JafarAbdi, Michael Görner, Robert Haschke, Tyler Weaver

2.1.4 (2021-05-31)
------------------

2.1.3 (2021-05-22)
------------------

2.1.2 (2021-04-20)
------------------

2.1.1 (2021-04-12)
------------------
* Fix EXPORT install in CMake (`#372 <https://github.com/ros-planning/moveit2/issues/372>`_)
* TfPublisher: tf frame name can't start with '/'
* [fix] Export libs for MoveGroup capabilities and MoveItSimpleControllerManager (`#344 <https://github.com/ros-planning/moveit2/issues/344>`_)
* Fix repo URLs in package.xml files
* Contributors: Boston Cleek, Henning Kayser, Jafar Abdi, Tyler Weaver

2.1.0 (2020-11-23)
------------------
* [fix] Interactive markers not visible in motion planning plugin (`#299 <https://github.com/ros-planning/moveit2/issues/299>`_)
* [fix] Rosdep dependencies ros_testing, OpenMP (`#309 <https://github.com/ros-planning/moveit2/issues/309>`_)
* [maint] Remove deprecated namespaces robot_model, robot_state  (`#276 <https://github.com/ros-planning/moveit2/issues/276>`_)
* [maint] Wrap common cmake code in 'moveit_package()' macro (`#285 <https://github.com/ros-planning/moveit2/issues/285>`_)
  * New moveit_package() macro for compile flags, Windows support etc
  * Add package 'moveit_common' as build dependency for moveit_package()
  * Added -Wno-overloaded-virtual compiler flag for moveit_ros_planners_ompl
* [maint] Compilation fixes for macOS (`#271 <https://github.com/ros-planning/moveit2/issues/271>`_)
* [ros2-migration] Enable warehouse in moveit_ros_benchmarks (`#301 <https://github.com/ros-planning/moveit2/issues/301>`_)
* [ros2-migration] Port moveit_ros_warehouse to ROS 2 (`#273 <https://github.com/ros-planning/moveit2/issues/273>`_)
* [ros2-migration] Port moveit_ros_benchmarks to ROS 2 (`#225 <https://github.com/ros-planning/moveit2/issues/225>`_)
* [ros2-migration] Port moveit_group to ROS 2 (`#217 <https://github.com/ros-planning/moveit2/issues/217>`_)
* Contributors: Abdullah Alzaidy, Edwin Fan, Henning Kayser, Jafar Abdi, Lior Lustgarten, Yu Yan

1.1.1 (2020-10-13)
------------------
* [fix] Let the max number of contacts be the amount of world objects + link models with geometry (`#2355 <https://github.com/ros-planning/moveit/issues/2355>`_)
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski, Loy van Beek, Michael Görner, v4hn

1.1.0 (2020-09-04)
------------------
* [feature] Start new joint_state_publisher_gui on param use_gui (`#2257 <https://github.com/ros-planning/moveit/issues/2257>`_)
* [feature] TfPublisher: fixup and add attached collsion objects (`#1792 <https://github.com/ros-planning/moveit/issues/1792>`_)
* [feature] move_group capability for publishing planning scene frames to the tf system (`#1761 <https://github.com/ros-planning/moveit/issues/1761>`_)
* [feature] get_planning_scene_service: return full scene when nothing was requested (`#1424 <https://github.com/ros-planning/moveit/issues/1424>`_)
* [feature] Separate source file for CartesianInterpolator (`#1149 <https://github.com/ros-planning/moveit/issues/1149>`_)
* [fix]   Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix]   Fix TfPublisher subframe publishing (`#2002 <https://github.com/ros-planning/moveit/issues/2002>`_)
* [maint] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_) (`#2004 <https://github.com/ros-planning/moveit/issues/2004>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Move isEmpty() test functions to moveit_core/utils (`#1627 <https://github.com/ros-planning/moveit/issues/1627>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Dave Coleman, Mike Lautman, Robert Haschke, Felix von Drigalski, Jens P, Jonathan Binney, JonasTietz, Michael Görner, Tyler Weaver, Yoan Mollard, Yu, Yan, v4hn

1.0.6 (2020-08-19)
------------------
* [maint] Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint] Migrate to clang-format-10
* Contributors: Robert Haschke, Michael Görner

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [maint] Move `get_planning_scene` service into `PlanningSceneMonitor` for reusability (`#1854 <https://github.com/ros-planning/moveit/issues/1854>`_)
* [maint] Cleanup move_group capabilities (`#1515 <https://github.com/ros-planning/moveit/issues/1515>`_)
* [maint] Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint] Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint] Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* Contributors: Robert Haschke, Sean Yen, Yu, Yan

1.0.2 (2019-06-28)
------------------

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* [improve] Remove (redundant) random seeding and #attempts from RobotState::setFromIK() as the IK solver perform random seeding themselves. `#1288 <https://github.com/ros-planning/moveit/issues/1288>`_
* Contributors: Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [fix] Fixed various memory leaks (`#1104 <https://github.com/ros-planning/moveit/issues/1104>`_)
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Code Cleanup
  * `#1180 <https://github.com/ros-planning/moveit/issues/1180>`_
  * `#1196 <https://github.com/ros-planning/moveit/issues/1196>`_
* Contributors: Dave Coleman, Robert Haschke

0.10.2 (2018-10-24)
-------------------
* [fix] PlanningSceneMonitor lock `#1033 <https://github.com/ros-planning/moveit/issues/1033>`_: Fix `#868 <https://github.com/ros-planning/moveit/issues/868>`_ (`#1057 <https://github.com/ros-planning/moveit/issues/1057>`_)
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* [maintenance] add minimum required pluginlib version (`#927 <https://github.com/ros-planning/moveit/issues/927>`_)
* Contributors: Mikael Arguedas, Mohmmad Ayman, Robert Haschke, mike lautman

0.10.1 (2018-05-25)
-------------------
* Remove deprecated ExecuteTrajectoryServiceCapability (`#833 <https://github.com/ros-planning/moveit/issues/833>`_)
* migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* Add namespace capabilities to moveit_commander (`#835 <https://github.com/ros-planning/moveit/issues/835>`_)
* [fix] MoveAction capability can drop cancel request if it is sent shortly after goal is sent (`#756 <https://github.com/ros-planning/moveit/issues/756>`_)
* Contributors: Dave Coleman, Ian McMahon, Mikael Arguedas, Robert Haschke, Will Baker

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] always return true in MoveGroupPlanService callback `#674 <https://github.com/ros-planning/moveit/pull/674>`_
* [improve] adding swp's to gitignore and removing redundant capabilites from capability_names.h (`#704 <https://github.com/ros-planning/moveit/issues/704>`_)
* Contributors: Mike Lautman, Shingo Kitagawa

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
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [fix] race conditions when updating PlanningScene (`#350 <https://github.com/ros-planning/moveit/issues/350>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Robert Haschke

0.9.3 (2016-11-16)
------------------
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------

0.6.6 (2016-06-08)
------------------
* added missing validity check
  iterator found with `configs.find()` needs to be validated before use...
* Removed trailing whitespace from entire repository
* moved planner params services into existing capability QueryPlannerInterfaces
* capability plugin MoveGroupPlannerParamsService to get/set planner params
* Fixed bug(?) in move_group::MoveGroupKinematicsService::computeIK link name selection.
* Contributors: Dave Coleman, Mihai Pomarlan, Robert Haschke

0.6.5 (2015-01-24)
------------------
* update maintainers
* Contributors: Michael Ferguson

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------

0.6.2 (2014-10-31)
------------------
* Merge pull request `#522 <https://github.com/ros-planning/moveit_ros/issues/522>`_ from mikeferguson/indigo-devel
  add dependency on std_srvs (part of octomap clearing service)
* Contributors: Ioan A Sucan, Michael Ferguson

0.6.1 (2014-10-31)
------------------

0.6.0 (2014-10-27)
------------------
* Added move_group capability for clearing octomap.
* Contributors: Dave Hershberger, Sachin Chitta

0.5.19 (2014-06-23)
-------------------
* Address [cppcheck: duplicateIf] error.
  The same condition was being checked twice, and the same action was being taken.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update maintainer e-mail
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------
* empty state should be a diff state, otherwise attached objects are deleted
* Contributors: sachinc

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* Re-ordered movegroup's initialization, so capabilities start after monitors.
* correcting maintainer email
* Added planning feedback to gui, refactored states tab

0.5.8 (2013-10-11)
------------------

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* porting to new RobotState API
* more console output

0.5.4 (2013-08-14)
------------------

* make headers and author definitions aligned the same way; white space fixes
* Dependency for move_group_capabilities_base fixed.

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)
* port to new base class for planning_interface (using planning contexts)

0.4.5 (2013-07-03)
------------------
* Fixed for moveit_msgs/JointLimits.h no such file or directory

0.4.4 (2013-06-26)
------------------
* fix `#259 <https://github.com/ros-planning/moveit_ros/issues/259>`_ and `#260 <https://github.com/ros-planning/moveit_ros/issues/260>`_.
