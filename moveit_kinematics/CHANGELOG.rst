^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Merge remote-tracking branch 'origin/main' into feature/msa
* Removing more boost usage (`#1372 <https://github.com/ros-planning/moveit2/issues/1372>`_)
* Merge remote-tracking branch 'upstream/main' into feature/msa
* Removing some boost usage (`#1331 <https://github.com/ros-planning/moveit2/issues/1331>`_)
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Update plugin library paths (`#1304 <https://github.com/ros-planning/moveit2/issues/1304>`_)
* Switch to hpp headers of pluginlib
* Merge pull request `#3106 <https://github.com/ros-planning/moveit2/issues/3106>`_ from v4hn/pr-master-bind-them-all / banish bind()
* Fix clang-tidy
* kinematics test: remove unused argument
* Contributors: David V. Lu, Henry Moore, Jochen Sprickerhof, Michael Görner, Robert Haschke, Sebastian Jahr, Vatan Aksoy Tezer, v4hn

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Fix reading joint weights in KDLKinematicsPlugin (`#1216 <https://github.com/ros-planning/moveit2/issues/1216>`_)
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
  Adaption of https://github.com/ros-planning/moveit/pull/3106
* remove unused arguments from kinematics test
  source https://github.com/ros-planning/moveit/pull/3106/commits/ddb68b6178ecfde267b5c7c9734aa47f6c4c4a5f; I also had to amend moveit_msgs to moveit_msgs::msg in this commit, otherwise everything remains the same as source commit. When I ran the kinematics plugin test locally it threw an error both before and after this change. Hopefully we can revisit this point as part of the code review, the error related to the robot description.
* Use orocos_kdl_vendor package (`#1207 <https://github.com/ros-planning/moveit2/issues/1207>`_)
* Use a steady clock for timeout for IK (`#795 <https://github.com/ros-planning/moveit2/issues/795>`_)
* Merge https://github.com/ros-planning/moveit/commit/a25515b73d682df03ed3eccd839110c296aa79fc
* Merge https://github.com/ros-planning/moveit/commit/ab42a1d7017b27eb6c353fb29331b2da08ab0039
* Compilation fixes for Jammy and bring back Rolling CI (`#1095 <https://github.com/ros-planning/moveit2/issues/1095>`_)
* Add moveit_configs_utils package to simplify loading paramters (`#591 <https://github.com/ros-planning/moveit2/issues/591>`_)
* round_collada_numbers.py: python 2/3 compatibility (`#2983 <https://github.com/ros-planning/moveit2/issues/2983>`_)
  Python3 requires the files to be opened in binary mode read a bytes object instead of a string, which is needed in turn by etree.parse().
  Co-authored-by: Robert Haschke <rhaschke@techfak.uni-bielefeld.de>
* Switch to std::bind (`#2967 <https://github.com/ros-planning/moveit2/issues/2967>`_)
* Contributors: Abishalini, Gaël Écorchard, Henning Kayser, Jafar, Jafar Abdi, Jochen Sprickerhof, Robert Haschke, Tomislav Bazina, Vatan Aksoy Tezer, jeoseo, v4hn

2.4.0 (2022-01-20)
------------------
* Fix IKFast test dependency (`#993 <https://github.com/ros-planning/moveit2/issues/993>`_)
* Replace NULL with nullptr (`#961 <https://github.com/ros-planning/moveit2/issues/961>`_)
  * Fixes `#841 <https://github.com/ros-planning/moveit2/issues/841>`_
* moveit_build_options()
  Declare common build options like CMAKE_CXX_STANDARD, CMAKE_BUILD_TYPE,
  and compiler options (namely warning flags) once.
  Each package depending on moveit_core can use these via moveit_build_options().
* Contributors: Henning Kayser, Robert Haschke, Stephanie Eng

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Convert to modern include guard `#882 <https://github.com/ros-planning/moveit2/issues/882>`_ (`#891 <https://github.com/ros-planning/moveit2/issues/891>`_)
* Replaced C-Style Cast with C++ Style Cast. (`#935 <https://github.com/ros-planning/moveit2/issues/935>`_)
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* Get rid of "std::endl" (`#918 <https://github.com/ros-planning/moveit2/issues/918>`_)
* changed post-increments in loops to preincrements (`#888 <https://github.com/ros-planning/moveit2/issues/888>`_)
* Update README (`#812 <https://github.com/ros-planning/moveit2/issues/812>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: Dave Coleman, David V. Lu!!, Henning Kayser, Kaustubh, Parthasarathy Bana, Robert Haschke, Sencer Yazıcı, Stephanie Eng, predystopic-dev, pvanlaar

2.3.0 (2021-10-08)
------------------
* Fix cmake warnings (`#690 <https://github.com/ros-planning/moveit2/issues/690>`_)
  * Fix -Wformat-security
  * Fix -Wunused-variable
  * Fix -Wunused-lambda-capture
  * Fix -Wdeprecated-declarations
  * Fix clang-tidy, readability-identifier-naming in moveit_kinematics
* Fix warnings in Galactic and Rolling (`#598 <https://github.com/ros-planning/moveit2/issues/598>`_)
  * Use __has_includes preprocessor directive for deprecated headers
  * Fix parameter template types
  * Proper initialization of smart pointers, rclcpp::Duration
* Fix loading joint_limits.yaml in demo and test launch files (`#544 <https://github.com/ros-planning/moveit2/issues/544>`_)
* Contributors: David V. Lu, Henning Kayser, Jafar Abdi, Vatan Aksoy Tezer

2.2.1 (2021-07-12)
------------------
* Pluginlib Deprecation Fix (`#542 <https://github.com/ros-planning/moveit2/issues/542>`_)
* Contributors: David V. Lu!!

2.2.0 (2021-06-30)
------------------
* [sync] MoveIt's master branch up-to https://github.com/ros-planning/moveit/commit/0d0a6a171b3fbea97a0c4f284e13433ba66a4ea4
  * Improve ikfast QUIET handling (`#2685 <https://github.com/ros-planning/moveit/issues/2685>`_)
  * ikfast script: install sympy 0.7.1 from git (`#2650 <https://github.com/ros-planning/moveit/issues/2650>`_)
  * Replaced eigen+kdl conversions with tf2_eigen + tf2_kdl (`#2472 <https://github.com/ros-planning/moveit/issues/2472>`_)
* Contributors: JafarAbdi, Robert Haschke, Tyler Weaver, ags-dy, petkovich

2.1.4 (2021-05-31)
------------------
* Enable LMA and KDL kinematic launch tests (`#435 <https://github.com/ros-planning/moveit2/issues/435>`_)
* Contributors: Vatan Aksoy Tezer

2.1.3 (2021-05-22)
------------------
* Replace last ament_export_libraries macro calls with ament_export_targets (`#448 <https://github.com/ros-planning/moveit2/issues/448>`_)
* Contributors: Sebastian Jahr

2.1.2 (2021-04-20)
------------------
* Unify PickNik name in copyrights (`#419 <https://github.com/ros-planning/moveit2/issues/419>`_)
* Contributors: Tyler Weaver

2.1.1 (2021-04-12)
------------------
* Fix EXPORT install in CMake (`#372 <https://github.com/ros-planning/moveit2/issues/372>`_)
* Sync main branch with MoveIt 1 from previous head https://github.com/ros-planning/moveit/commit/0247ed0027ca9d7f1a7f066e62c80c9ce5dbbb5e up to https://github.com/ros-planning/moveit/commit/74b3e30db2e8683ac17b339cc124675ae52a5114
* [fix] export cmake library install (`#339 <https://github.com/ros-planning/moveit2/issues/339>`_)
* Python3 compatibility for ikfast's round_collada_numbers.py (`#2473 <https://github.com/ros-planning/moveit2/issues/2473>`_)
* Fix repo URLs in package.xml files
* Contributors: Henning Kayser, Jafar Abdi, Tobias Fischer, Tyler Weaver

2.1.0 (2020-11-23)
------------------
* [maint] Wrap common cmake code in 'moveit_package()' macro (`#285 <https://github.com/ros-planning/moveit2/issues/285>`_)
  * New moveit_package() macro for compile flags, Windows support etc
  * Add package 'moveit_common' as build dependency for moveit_package()
  * Added -Wno-overloaded-virtual compiler flag for moveit_ros_planners_ompl
* [maint] small compilation fixes for macOS (`#271 <https://github.com/ros-planning/moveit2/issues/271>`_)
* [maint] kinematics_base: remove deprecated initialize function (`#232 <https://github.com/ros-planning/moveit2/issues/232>`_)
* [maint] Enable clang-tidy-fix and ament_lint_cmake (`#210 <https://github.com/ros-planning/moveit2/issues/210>`_)
* [maint] Simplify kdl now that kinetic support is dropped (`#237 <https://github.com/ros-planning/moveit2/issues/237>`_)
* [ros2-migration] Migrate to ROS 2 Foxy (`#227 <https://github.com/ros-planning/moveit2/issues/227>`_)
* [ros2-migration] Port Ikfast kinematics solver (`#205 <https://github.com/ros-planning/moveit2/issues/205>`_)
* [ros2-migration] Port CachedIKKinematicsPlugin to ROS2 (`#207 <https://github.com/ros-planning/moveit2/issues/207>`_)
* Contributors: Henning Kayser, Jafar Abdi, Lior Lustgarten, Mark Moll, Mohmmad Ayman, Nathan Brooks, Ruffin

2.0.0 (2020-02-17)
------------------
* [port] Port moveit kinematics to ROS2 (`#128 <https://github.com/ros-planning/moveit2/issues/128>`_)
* Contributors: Henning Kayser, Jafar Abdi

1.1.1 (2020-10-13)
------------------
* [fix] various issues with Noetic build (`#2327 <https://github.com/ros-planning/moveit/issues/2327>`_)
* [fix] python3 issues (`#2323 <https://github.com/ros-planning/moveit/issues/2323>`_)
* Contributors: G.A. vd. Hoorn, Michael Görner, Robert Haschke

1.1.0 (2020-09-04)
------------------
* [feature] Implementation of parameter TranslationXY2D IKFast (`#1949 <https://github.com/ros-planning/moveit/issues/1949>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Delete IKCache copy constructor (`#1750 <https://github.com/ros-planning/moveit/issues/1750>`_)
* [maint] Move NOLINT instructions to intended positions (`#2058 <https://github.com/ros-planning/moveit/issues/2058>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_) (`#2004 <https://github.com/ros-planning/moveit/issues/2004>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Fix various build issues on Windows (`#1880 <https://github.com/ros-planning/moveit/issues/1880>`_)
* [maint] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 for portability (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* [maint] Relax dependencies of moveit_kinematics (`#1529 <https://github.com/ros-planning/moveit/issues/1529>`_)
* Contributors: Ayush Garg, Christian Henkel, Dave Coleman, Henning Kayser, Immanuel Martini, Jonathan Binney, Markus Vieth, Martin Günther, Michael Ferguson, Michael Görner, Robert Haschke, Sean Yen, Tyler Weaver, Yu, Yan, edetleon, jschleicher, v4hn

1.0.6 (2020-08-19)
------------------
* [maint] Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint] Migrate to clang-format-10
* [maint] Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* Contributors: Markus Vieth, Robert Haschke, Michael Görner

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------
* Fix broken IKFast generator (`#2116 <https://github.com/ros-planning/moveit/issues/2116>`_)
* Contributors: Robert Haschke

1.0.3 (2020-04-26)
------------------
* [feature] KDL IK: constrain wiggled joints to limits (`#1953 <https://github.com/ros-planning/moveit/issues/1953>`_)
* [feature] IKFast: optional prefix for link names (`#1599 <https://github.com/ros-planning/moveit/issues/1599>`_)
  If you pass a `link_prefix` parameter in your `kinematics.yaml`, this string is prepended to the base and tip links.
  It allows multi-robot setups (e.g. dual-arm) and still instantiate the same solver for both manipulators.
* [feature] IKFast: increase verbosity of generated script (`#1434 <https://github.com/ros-planning/moveit/issues/1434>`_)
* [maint]   Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint]   Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint]   Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [feature] IKFast: implement `Translation*AxisAngle4D` IK type (`#1823 <https://github.com/ros-planning/moveit/issues/1823>`_)
* [fix]     Fix possible division-by-zero (`#1809 <https://github.com/ros-planning/moveit/issues/1809>`_)
* Contributors: Christian Henkel, Martin Günther, Max Krichenbauer, Michael Görner, Robert Haschke, Sean Yen, Yu, Yan, jschleicher

1.0.2 (2019-06-28)
------------------
* [fix] KDL IK solver: fix handling of mimic joints (`#1490 <https://github.com/ros-planning/moveit/issues/1490>`_)
* [fix] Fix ROS apt-key in OpenRAVE docker image (`#1503 <https://github.com/ros-planning/moveit/issues/1503>`_)
* [fix] Fix ikfast plugin-generator script (`#1492 <https://github.com/ros-planning/moveit/issues/1492>`_, `#1449 <https://github.com/ros-planning/moveit/issues/1449>`_)
* Contributors: Immanuel Martini, Michael Görner, Robert Haschke

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* [capability] Adapt ikfast plugin to new KinematicsBase API. `#1320 <https://github.com/ros-planning/moveit/issues/1320>`_
* [improve] cleanup LMA kinematics solver `#1318 <https://github.com/ros-planning/moveit/issues/1318>`_
* [improve] KDL IK solver improvements (`#1321 <https://github.com/ros-planning/moveit/issues/1321>`_)
* [improve] Kinematics tests, kdl cleanup `#1272 <https://github.com/ros-planning/moveit/issues/1272>`_, `#1294 <https://github.com/ros-planning/moveit/issues/1294>`_
* Contributors: Dave Coleman, Jorge Nicho, Mike Lautman, Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [enhancement] Pass RobotModel to IK, avoiding multiple loading (`#1166 <https://github.com/ros-planning/moveit/issues/1166>`_)
  See `MIGRATION notes <https://github.com/ros-planning/moveit/blob/melodic-devel/MIGRATION.md>`_ for API changes in IK plugins,
  kdl, srv, or cached_ik for examples.
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Use C++14 (`#1146 <https://github.com/ros-planning/moveit/issues/1146>`_)
* Contributors: Alex Moriarty, Michael Görner, Robert Haschke

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------

0.10.2 (2018-10-24)
-------------------
* [capability] add IKP_Translation{X,Y,Z}AxisAngle4D to the cpp template, see https://github.com/ros-planning/moveit/issues/548#issuecomment-316298918
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* Contributors: Kei Okada, Mikael Arguedas, Mohmmad Ayman, Robert Haschke, mike lautman, v4hn

0.10.1 (2018-05-25)
-------------------
* migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* switch to ROS_LOGGER from CONSOLE_BRIDGE (`#874 <https://github.com/ros-planning/moveit/issues/874>`_)
* fixes to ikfast kinematics plugin (`#808 <https://github.com/ros-planning/moveit/issues/808>`_)
* Cached ik kinematics plugin (`#612 <https://github.com/ros-planning/moveit/issues/612>`_)
  add caching wrapper for IK solvers
* Contributors: Ian McMahon, Mark Moll, Mikael Arguedas, Robert Haschke, Xiaojian Ma

0.9.11 (2017-12-25)
-------------------
* Merge pull request `#714 <https://github.com/ros-planning/moveit/issues/714>`_ from henhenhen/kinetic-devel_lookup-param
  Use lookupParam() in kinematics plugins
* Replace param() with lookupParam() in srv_kinematics_plugin
* Replace param() with lookupParam() in lma_kinematics_plugin
* Replace param() with lookupParam() in kdl_kinematics_plugin
* Replace param() with lookupParam() in ikfast_kinematics_plugin
* Remove redundant parameter query
* Contributors: Henning Kayser, Isaac I.Y. Saito

0.9.10 (2017-12-09)
-------------------
* [fix][kinetic onward] Fix create_ikfast_moveit_plugin to comply with format 2 of the package.xml. Remove collada_urdf dependency `#666 <https://github.com/ros-planning/moveit/pull/666>`_
* [fix] create_ikfast_moveit_plugin: fixed directory variable for templates that were moved to ikfast_kinematics_plugin `#620 <https://github.com/ros-planning/moveit/issues/620>`_
* [improve] IKFastTemplate: Expand solutions to full joint range in searchPositionIK `#598 <https://github.com/ros-planning/moveit/issues/598>`_
* [improve] IKFastTemplate: searchPositionIK now returns collision-free solution which is nearest to seed state. (`#585 <https://github.com/ros-planning/moveit/issues/585>`_)
* Contributors: Dennis Hartmann, G.A. vd. Hoorn, Michael Görner, fsuarez6

0.9.9 (2017-08-06)
------------------
* [improve] Modify ikfast_template for getPositionIK single solution results (`#537 <https://github.com/ros-planning/moveit/issues/537>`_)
* Contributors: nsnitish

0.9.8 (2017-06-21)
------------------
* [build] ikfast_kinematics_plugin: Write XML files as UTF-8 (`#514 <https://github.com/ros-planning/moveit/issues/514>`_)
* [build] adjust cmake_minimum_required for add_compile_options (`#521 <https://github.com/ros-planning/moveit/issues/521>`_)
* [build] ikfast_kinematics_plugin: Add c++11 compile option. This is required for Kinetic.
* Contributors: Martin Guenther, Michael Goerner

0.9.7 (2017-06-05)
------------------
* [fix][Kinetic+] ikfast_kinematics_plugin: Add c++11 compile option `#515 <https://github.com/ros-planning/moveit/pull/515>`_
* [fix][Indigo] moveit_kinematics Eigen3 dependency (`#470 <https://github.com/ros-planning/moveit/issues/470>`_)
* Contributors: Martin Guenther, YuehChuan

0.9.6 (2017-04-12)
------------------

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* Contributors: Dave Coleman

0.9.4 (2017-02-06)
------------------
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman

0.9.3 (2016-11-16)
------------------
* [fix] Replace unused service dependency with msg dep (`#361 <https://github.com/ros-planning/moveit/issues/361>`_)
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman

0.9.0 (2016-10-19)
------------------
* Add dependency on new moveit_kinematics package
* Move moveit_ikfast into moveit_kinematics
* Moved kinematics plugins to new pkg moveit_kinematics
* Contributors: Dave Coleman

0.8.3 (2016-08-21)
------------------
