^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_robot_interaction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Merge https://github.com/ros-planning/moveit/commit/c88f6fb64e9057a4b9a8f6fafc01060e8c48a216
* Merge remote-tracking branch 'origin/main' into feature/msa
* Removing more boost usage (`#1372 <https://github.com/ros-planning/moveit2/issues/1372>`_)
* Merge remote-tracking branch 'upstream/main' into feature/msa
* Removing some boost usage (`#1331 <https://github.com/ros-planning/moveit2/issues/1331>`_)
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Find end-effectors for empty parent_group (`#3108 <https://github.com/ros-planning/moveit2/issues/3108>`_)
  The parent_group attribute is optional in MSA. If it is not set, end-effector markers should be created in any case.
  Fixup for 9271e6a2edbeed291b7c713f55000bbc59d37b9e
* Merge pull request `#3106 <https://github.com/ros-planning/moveit2/issues/3106>`_ from v4hn/pr-master-bind-them-all / banish bind()
* banish bind()
* various: prefer objects and references over pointers
* Contributors: Abishalini, David V. Lu, Henry Moore, Michael Görner, Vatan Aksoy Tezer, v4hn

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Merge https://github.com/ros-planning/moveit/commit/72d919299796bffc21f5eb752d66177841dc3442
* Enable cppcheck (`#1224 <https://github.com/ros-planning/moveit2/issues/1224>`_)
  Co-authored-by: jeoseo <jeongwooseo2012@gmail.com>
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
  Adaption of https://github.com/ros-planning/moveit/pull/3106
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* various: prefer object and references over pointers
  source: https://github.com/ros-planning/moveit/pull/3106/commits/1a8e5715e3142a92977ac585031b9dc1871f8718; this commit contains minor changes when compared to the source commit which it is based on, these changes are limited to ensuring compatibility with ROS2.
* Merge https://github.com/ros-planning/moveit/commit/424a5b7b8b774424f78346d1e98bf1c9a33f0e78
* Remove new operators (`#1135 <https://github.com/ros-planning/moveit2/issues/1135>`_)
  replace new operator with make_shared
* Consider eef's parent group when creating eef markers (`#3095 <https://github.com/ros-planning/moveit2/issues/3095>`_)
  If we have end-effector(s) defined, a corresponding rviz marker for IK
  should be created only if the eef's group matches the considered JMG.
* Merge https://github.com/ros-planning/moveit/commit/a25515b73d682df03ed3eccd839110c296aa79fc
* Merge https://github.com/ros-planning/moveit/commit/ab42a1d7017b27eb6c353fb29331b2da08ab0039
* 1.1.9
* 1.1.8
* 1.1.7
* Disable slow robot_interaction tests in DEBUG mode (`#3014 <https://github.com/ros-planning/moveit2/issues/3014>`_)
  These tests are known to run for a very long time in debug builds. So let's disable them in this case.
  If you still insist to run them, you can do so via `locked_robot_state_test --gtest_also_run_disabled_tests`
* Add marker for subgroups even if no endeffector is defined for them (`#2977 <https://github.com/ros-planning/moveit2/issues/2977>`_)
  For single groups, the old logic fell back to add a marker
  for the last link if IK is supported for it and no endeffector is defined.
  That (quite reasonable) fallback did not yet work for subgroups though.
* Switch to std::bind (`#2967 <https://github.com/ros-planning/moveit2/issues/2967>`_)
  * boost::bind -> std::bind
  grep -rlI --exclude-dir=.git "boost::bind" | xargs sed -i 's/boost::bind/std::bind/g'
  * Convert bind placeholders
  grep -rlI --exclude-dir=.git " _[0-9]" | xargs sed -i 's/ _\([0-9]\)/ std::placeholders::_\1/g'
  * Update bind include header
  grep -rlI --exclude-dir=.git "boost/bind" | xargs sed -i 's#boost/bind.hpp#functional#'
* 1.1.6
* Contributors: Abishalini, Henning Kayser, Jafar, Jochen Sprickerhof, Michael Görner, Robert Haschke, Sencer Yazıcı, jeoseo, v4hn

2.4.0 (2022-01-20)
------------------
* Replace NULL with nullptr (`#961 <https://github.com/ros-planning/moveit2/issues/961>`_)
  * Fixes `#841 <https://github.com/ros-planning/moveit2/issues/841>`_
* moveit_build_options()
  Declare common build options like CMAKE_CXX_STANDARD, CMAKE_BUILD_TYPE,
  and compiler options (namely warning flags) once.
  Each package depending on moveit_core can use these via moveit_build_options().
* Contributors: Robert Haschke, Stephanie Eng

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* changed post-increments in loops to preincrements (`#888 <https://github.com/ros-planning/moveit2/issues/888>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: Dave Coleman, David V. Lu!!, Henning Kayser, Robert Haschke, Sencer Yazıcı, pvanlaar

2.3.0 (2021-10-08)
------------------
* Fix warnings in Galactic and Rolling (`#598 <https://github.com/ros-planning/moveit2/issues/598>`_)
  * Use __has_includes preprocessor directive for deprecated headers
  * Fix parameter template types
  * Proper initialization of smart pointers, rclcpp::Duration
* Fixes for Windows (`#530 <https://github.com/ros-planning/moveit2/issues/530>`_)
* Contributors: Akash, Nisala Kalupahana, Jorge Nicho, Henning Kayser, Vatan Aksoy Tezer, Tyler Weaver, Lior Lustgarten

2.2.1 (2021-07-12)
------------------

2.2.0 (2021-06-30)
------------------
* Enable Rolling and Galactic CI (`#494 <https://github.com/ros-planning/moveit2/issues/494>`_)
* [sync] MoveIt's master branch up-to https://github.com/ros-planning/moveit/commit/0d0a6a171b3fbea97a0c4f284e13433ba66a4ea4
* Contributors: JafarAbdi, Tyler Weaver, Vatan Aksoy Tezer

2.1.4 (2021-05-31)
------------------

2.1.3 (2021-05-22)
------------------

2.1.2 (2021-04-20)
------------------

2.1.1 (2021-04-12)
------------------
* Enable ament_lint tests (`#340 <https://github.com/ros-planning/moveit2/issues/340>`_)
* Fix EXPORT install in CMake (`#372 <https://github.com/ros-planning/moveit2/issues/372>`_)
* Sync main branch with MoveIt 1 from previous head https://github.com/ros-planning/moveit/commit/0247ed0027ca9d7f1a7f066e62c80c9ce5dbbb5e up to https://github.com/ros-planning/moveit/commit/74b3e30db2e8683ac17b339cc124675ae52a5114
* [fix] export cmake library install (`#339 <https://github.com/ros-planning/moveit2/issues/339>`_)
* Fix repo URLs in package.xml files
* Contributors: Henning Kayser, Jafar Abdi, Robert Haschke, Tyler Weaver

2.1.0 (2020-11-23)
------------------
* [maint] Wrap common cmake code in 'moveit_package()' macro (`#285 <https://github.com/ros-planning/moveit2/issues/285>`_)
  * New moveit_package() macro for compile flags, Windows support etc
  * Add package 'moveit_common' as build dependency for moveit_package()
  * Added -Wno-overloaded-virtual compiler flag for moveit_ros_planners_ompl
* [ros2-migration] Port robot_interaction to ROS 2 (`#211 <https://github.com/ros-planning/moveit2/issues/211>`_)
* Contributors: Jafar Abdi, Lior Lustgarten, Edwin Fan

1.1.1 (2020-10-13)
------------------
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski

1.1.0 (2020-09-04)
------------------
* [feature] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [fix] Various fixes for upcoming Noetic release `#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Ayush Garg, Dave Coleman, Henning Kayser, Jonathan Binney, Markus Vieth, Robert Haschke, Sean Yen, Tyler Weaver, Yu, Yan

1.0.6 (2020-08-19)
------------------
* [maint] Migrate to clang-format-10
* [maint] Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* Contributors: Markus Vieth, Robert Haschke

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
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
* [improve] cleanup RobotInteraction (`#1287 <https://github.com/ros-planning/moveit/issues/1287>`_)
* [improve] limit IK timeout to 0.1s for a responsive interaction behaviour (`#1291 <https://github.com/ros-planning/moveit/issues/1291>`_)
* Contributors: Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Use C++14 (`#1146 <https://github.com/ros-planning/moveit/issues/1146>`_)
* [maintenance] Cleanup Robot Interaction (`#1194 <https://github.com/ros-planning/moveit/issues/1194>`_)
  * Remove deprecated handling of own KinematicsOptionsMap
  * Use normalized quaternions
* [maintenance] Code Cleanup
  * `#1179 <https://github.com/ros-planning/moveit/issues/1179>`_
  * `#1196 <https://github.com/ros-planning/moveit/issues/1196>`_
* Contributors: Alex Moriarty, Dave Coleman, Michael Görner, Robert Haschke

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------
* [fix] compiler warnings (`#1089 <https://github.com/ros-planning/moveit/issues/1089>`_)
* Contributors: Robert Haschke

0.10.2 (2018-10-24)
-------------------
* [fix] Text refrences to MoveIt (`#1020 <https://github.com/ros-planning/moveit/issues/1020>`_)
* Contributors: Mohmmad Ayman, Robert Haschke, mike lautman

0.10.1 (2018-05-25)
-------------------
* [fix] interaction with planar joints (`#767 <https://github.com/ros-planning/moveit/issues/767>`_)
* [maintenance] boost::shared_ptr -> std::shared_ptr
* [maintenance] migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* [enhance] association of IK solvers to groups `#769 <https://github.com/ros-planning/moveit/issues/769>`_
* Contributors: Bence Magyar, Ian McMahon, Michael Görner, Robert Haschke

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [package.xml] Add a release-maintainer. Cleanup `#649 <https://github.com/ros-planning/moveit/pull/649>`_

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [fix] `catkin_make -DCMAKE_ENABLE_TESTING=0` failure (`#478 <https://github.com/ros-planning/moveit/issues/478>`_)
* Contributors: Michael Goerner

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman

0.9.3 (2016-11-16)
------------------
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman

0.6.6 (2016-06-08)
------------------
* use getModelFrame() as reference frame for markers
* publish markers relative to robot's root frame
  In addition to `#669 <https://github.com/ros-planning/moveit_ros/issues/669>`_, interactive markers need to be place relative to the
  robot's root frame. If nothing is specified (as before), rviz' fixed frame
  is used, leading to offsets when both frames are not identical.
* merge indigo-devel changes (PR `#633 <https://github.com/ros-planning/moveit_ros/issues/633>`_ trailing whitespace) into jade-devel
* Removed trailing whitespace from entire repository
* further adapted marker size computation
  - drop largest extension dimension (-> use cross-section size of elongated link)
  - for an end-effector group, consider the sizes of individual links
  instead of the overall size of all links (which becomes huge very fast)
  - enlarge marker size by factor of 1.5 when there is only a single eef marker
* reworked computeLinkMarkerSize()
  compute size such that the marker sphere will cover
  - a spherical link geometry -> AABB.maxCoeff
  - a cubical link geometry -> AABB.norm
  -> use average of both values
  Virtual links (without any shape) will have a size of AABB of zero dims.
  In this case use the dimensions of the closest parent link instead.
* improved computation of interactive marker size
  - use parent_link if group == parent_group
  - scale smaller than 5cm is clipped to 5cm instead of using default
  - clarified size computation, using diameter of AABB
* fixing error caused by BOOST_STATIC_ASSERT
* Fixed compile error caused by BOOST_STATIC_ASSERT in kinematic_options.cpp
  Added kinematics::DiscretizationMethods::DiscretizationMethod to QO_FIELDS in kinematic_options.cpp.
  At pull request `#581 <https://github.com/ros-planning/moveit_ros/issues/581>`_, type of discretization_method was set to int. Changed it to proper type.
* reinstated changes related to the updates in the  moveit_core::KinematicsBase interface
* Revert "  Kinematics Base changes in moveit_core"
* adds the 'returns_approximate_solution' entry so that it is compatible with the changes in kinematics::KinematicsBase class in the moveit_core repo
* Contributors: Daichi Yoshikawa, Dave Coleman, Robert Haschke, Sachin Chitta, jrgnicho

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

0.6.1 (2014-10-31)
------------------

0.6.0 (2014-10-27)
------------------
* Fix coding style according to the moveit style
* update joystick documentation according to the latest implementation
* add checkbox to toggle if moveit rviz plugin subscribes
  the topics to be used for communication to the external ros nodes.
  update moveit_joy.py to parse srdf to know planning_groups and the
  names of the end effectors and support multi-endeffector planning groups.
* adding PoseStamped topic to move the interactive marker from other ros nodes
  such as joystick programs.
* Contributors: Ryohei Ueda, Sachin Chitta

0.5.19 (2014-06-23)
-------------------
* Fix [-Wreorder] warning.
* Allow planning groups to have more than one tip
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update maintainer e-mail
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------
* fix test
  This was testing functionality that got removed.  Removed that part of the
  test.
* robot_interaction: add comments
  Comment cryptic public function behavior.
* robot_interaction: fix formatting
  remove tabs and whitespace at the end of lines.
* robot_interaction: fix comment formatting
  Limit lines to 120 chars max (80 preferred in headers).
* robot_interaction: fix setStateFromIK prototypes
  use references instead of pointers.
* robot_interaction: fix header problems
  fix getRobotModel() bug
  make internal functions private.
* remove extraneous code
* add missing headers
* robot_interaction: Fix issues raised by Ioan
* robot_interaction: use LockedRobotState
  Fix a number of thread safety violations.
* robot_interaction: add LockedRobotState and tests
* robot_interaction: use KinematicOptionsMap
  Fixes threading issues.
  Separate the handling of kinematics options into a separate object which
  enforces thread safe access.
* robot_interaction: add KinematicOptions
  KinematicOptions contains the parameters needed to call RobotState::setFromIK.
  KinematicOptionsMap contains a map of string->KinematicOptions a default KinematicOptions.
  These are useful in RobotInteraction with the group name as the key.
* pull RobotInteraction structures out of class
  The Generic, EndEffector, and Joint structures complicate the core of
  RobotInteraction.  Pull them out to simplify the code.  This will also
  help with future plans to make the core of RobotInteraction more
  generic and flexible.
* fix include guards to match moveit conventions
* robot_interaction: include interaction_handler.h from robot_interaction.h
  This is for backwards compatibility with code that only includes
  robot_interaction.h
* robot_interaction: split handler into own file
* robot_interaction: split InteractionHandler into its own file
* robot_interaction: make lock-protected members private
  Since the lock is needed to access these and the lock is private it makes no
  sense for them to be protected.
* robot_interaction: add locking comments
* robot_interaction: simplify code
* robot_interaction: fix comments
* Contributors: Acorn Pooley

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------

0.5.12 (2014-01-03)
-------------------
* Fixed trailing underscores in CHANGELOGs.
* Contributors: Dave Hershberger

0.5.11 (2014-01-03)
-------------------

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* adds KDL link directories to robot_interaction/CMakeLists.txt (fixes `#376 <https://github.com/ros-planning/moveit_ros/issues/376>`_)
* fixed computation of dimension\_.
* fixes for mimic joints and redundant joints

0.5.8 (2013-10-11)
------------------

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* porting to new RobotState API

0.5.4 (2013-08-14)
------------------

* make headers and author definitions aligned the same way; white space fixes
* fix `#283 <https://github.com/ros-planning/moveit_ros/issues/283>`_

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* fix `#275 <https://github.com/ros-planning/moveit_ros/issues/275>`_
* white space fixes (tabs are now spaces)
* adding options struct to kinematics base

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
* bugfixes
* robot_interaction: include sphere markers by default
* use improved MOVE_ROTATE_3D marker
