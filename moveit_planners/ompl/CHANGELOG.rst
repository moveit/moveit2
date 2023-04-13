^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_planners_ompl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.7.1 (2023-03-23)
------------------
* Fix include install destination (`#2008 <https://github.com/ros-planning/moveit2/issues/2008>`_)
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  Co-authored-by: Tyler Weaver <maybe@tylerjw.dev>
* Temporarily disable TestPathConstraints with the Panda robot (`#2016 <https://github.com/ros-planning/moveit2/issues/2016>`_)
  This test has become flaky since it was modified to use the OMPL constrained state space (https://github.com/ros-planning/moveit2/issues/2015).
* Increase priority for constrained planning state space (`#1300 <https://github.com/ros-planning/moveit2/issues/1300>`_)
  * Change priority for the constrained planning state space
  * Fix constrained planning tests
  * Use PRM instead of RRTConnect
  ---------
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* remove underscore from public member in MotionPlanResponse (`#1939 <https://github.com/ros-planning/moveit2/issues/1939>`_)
  * remove underscore from private members
  * fix more uses of the suffix notation
* Contributors: Abhijeet Dasgupta, AlexWebb, Stephanie Eng

2.7.0 (2023-01-29)
------------------
* converted characters from string format to character format (`#1881 <https://github.com/ros-planning/moveit2/issues/1881>`_)
* Cleanup msg includes: Use C++ instead of C header (`#1844 <https://github.com/ros-planning/moveit2/issues/1844>`_)
* Remove ancient OMPL version directives (`#1825 <https://github.com/ros-planning/moveit2/issues/1825>`_)
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Minimize use of `this->` (`#1784 <https://github.com/ros-planning/moveit2/issues/1784>`_)
  It's often unnecessary. MoveIt already avoids this in most cases
  so this PR better cements that existing pattern.
* Enable `-Wold-style-cast` (`#1770 <https://github.com/ros-planning/moveit2/issues/1770>`_)
* Remove `MOVEIT_LIB_NAME` (`#1751 <https://github.com/ros-planning/moveit2/issues/1751>`_)
  It's more readable and searchable if we just spell out the target
  name.
* Add braces around blocks. (`#999 <https://github.com/ros-planning/moveit2/issues/999>`_)
* Use <> for non-local headers (`#1734 <https://github.com/ros-planning/moveit2/issues/1734>`_)
  Unless a header lives in the same or a child directory of the file
  including it, it's recommended to use <> for the #include statement.
  For more information, see the C++ Core Guidelines item SF.12
  https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#sf12-prefer-the-quoted-form-of-include-for-files-relative-to-the-including-file-and-the-angle-bracket-form-everywhere-else
* Used C++ style cast instead of C style cast  (`#1628 <https://github.com/ros-planning/moveit2/issues/1628>`_)
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Fix clang-tidy issues (`#1706 <https://github.com/ros-planning/moveit2/issues/1706>`_)
  * Blindly apply automatic clang-tidy fixes
  * Exemplarily cleanup a few automatic clang-tidy fixes
  * Clang-tidy fixups
  * Missed const-ref fixups
  * Fix unsupported non-const -> const
  * More fixes
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Contributors: Abhijeet Das Gupta, Chris Thrasher, Christian Henkel, Cory Crean, Henning Kayser, Robert Haschke, Sameer Gupta

2.6.0 (2022-11-10)
------------------
* Fix logic with enforcing constrained planning state space in OMPL (`#1589 <https://github.com/ros-planning/moveit2/issues/1589>`_)
* Convert OMPL status to MoveItErrorCode in the OMPL interface (`#1606 <https://github.com/ros-planning/moveit2/issues/1606>`_)
* Factor of 2 in OMPL orientation constraints, to match kinematic_constraints (`#1592 <https://github.com/ros-planning/moveit2/issues/1592>`_)
* Merge PR `#1553 <https://github.com/ros-planning/moveit2/issues/1553>`_: Improve cmake files
* Use standard exported targets: export\_${PROJECT_NAME} -> ${PROJECT_NAME}Targets
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* size_t bijection index type (`#1544 <https://github.com/ros-planning/moveit2/issues/1544>`_)
* Fixes for using generate_state_database (`#1412 <https://github.com/ros-planning/moveit2/issues/1412>`_)
* simplify_solution per planning context (`#1437 <https://github.com/ros-planning/moveit2/issues/1437>`_)
  * Allowing to dynamically change the parameter simplify_solutions
  * Delete this configuration because it overrides the configuration loaded
  The parameters simplify_solutions is passed to the context trough the configuration of each planner but this function overrides it and seems to be contradictory to rest of the implementation. simplify_solutions shouldn't be considered as the rest of the other parameters, like interpolate or hybridize ?
  * Remove simplify_solutions\_ from OMPL interface and all its setter/getter
  * Clean-up code without ConfigureContext and unneeded code related to simplify_solution
* correctly initialize rmw_serialized_message_t
* automatically declare parameters from overrides
* Remove __has_include statements (`#1481 <https://github.com/ros-planning/moveit2/issues/1481>`_)
* Merge https://github.com/ros-planning/moveit/commit/a63580edd05b01d9480c333645036e5b2b222da9
* Remove ConstraintSampler::project() (`#3170 <https://github.com/ros-planning/moveit2/issues/3170>`_)
  * Remove unused ompl_interface::ValidConstrainedSampler
  Last usage was removed in f2f6097ab7e272568d6ab258a53be3c7ca67cf3b.
  * Remove ConstraintSampler::project()
  sample() and project() only differ in whether they perform random sampling
  of the reference joint pose or not. Both of them are sampling.
  This was highly confusing, as from project() one wouldn't expect sampling.
* Contributors: Alaa, AndyZe, Antoine Duplex, Henning Kayser, Robert Haschke, Sebastian Jahr, Stephanie Eng, Tyler Weaver, Vatan Aksoy Tezer, abishalini

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Merge remote-tracking branch 'origin/main' into feature/msa
* Removing more boost usage (`#1372 <https://github.com/ros-planning/moveit2/issues/1372>`_)
* Merge remote-tracking branch 'upstream/main' into feature/msa
* Removing some boost usage (`#1331 <https://github.com/ros-planning/moveit2/issues/1331>`_)
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Add support for mixed constraints with constrained planner (`#1319 <https://github.com/ros-planning/moveit2/issues/1319>`_)
  Co-authored-by: AndyZe <zelenak@picknik.ai>
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Print OMPL setup info at the DEBUG level (`#1330 <https://github.com/ros-planning/moveit2/issues/1330>`_)
* Port OMPL orientation constraints to MoveIt2 (`#1273 <https://github.com/ros-planning/moveit2/issues/1273>`_)
  Co-authored-by: AndyZe <andyz@utexas.edu>
  Co-authored-by: JeroenDM <jeroendemaeyer@live.be>
* Merge pull request `#3106 <https://github.com/ros-planning/moveit2/issues/3106>`_ from v4hn/pr-master-bind-them-all / banish bind()
* Cleanup OMPL's PlanningContextManager's protected API
* banish bind()
* planning_context_manager: rename protected methods
* Contributors: AndyZe, David V. Lu, Henry Moore, Jafar, Jeroen De Maeyer, Michael Görner, Robert Haschke, Stephanie Eng, Vatan Aksoy Tezer

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* planning_context_manager: rename protected methods
  sources: https://github.com/ros-planning/moveit/pull/3106/commits/a183bc16f0b5490b1b40789ad2709d1cdbba7453, https://github.com/ros-planning/moveit/pull/3106/commits/c07be63b6cd5cfcea51e91e613bea9be68950754;
* Revert OMPL parameter loading
* [ompl] Small code refactor (`#1138 <https://github.com/ros-planning/moveit2/issues/1138>`_)
* Remove new operators (`#1135 <https://github.com/ros-planning/moveit2/issues/1135>`_)
* Merge https://github.com/ros-planning/moveit/commit/a25515b73d682df03ed3eccd839110c296aa79fc
* Delete profiler (`#998 <https://github.com/ros-planning/moveit2/issues/998>`_)
* Use termination condition for simplification step (`#2981 <https://github.com/ros-planning/moveit2/issues/2981>`_)
  ... to allow canceling the simplification step
* Switch to std::bind (`#2967 <https://github.com/ros-planning/moveit2/issues/2967>`_)
* Contributors: Abishalini, Gaël Écorchard, Henning Kayser, Jafar, Jochen Sprickerhof, Robert Haschke, Sencer Yazıcı, Simon Schmeisser, Tyler Weaver, Vatan Aksoy Tezer, jeoseo, rhaschke, v4hn

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
* changed post-increments in loops to preincrements (`#888 <https://github.com/ros-planning/moveit2/issues/888>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* Reduce log verbosity, improved info message (`#714 <https://github.com/ros-planning/moveit2/issues/714>`_)
* Fix `#2811 <https://github.com/ros-planning/moveit/issues/2811>`_ (`#2872 <https://github.com/ros-planning/moveit/issues/2872>`_)
  This is a PR for `#2811 <https://github.com/ros-planning/moveit/issues/2811>`_
* Add missing dependencies to generated dynamic_reconfigure headers
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: Dave Coleman, David V. Lu!!, Henning Kayser, Mathias Lüdtke, Parthasarathy Bana, Robert Haschke, Sencer Yazıcı, pvanlaar, v4hn, werner291

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
* Fix linking issues for ODE on macOS (`#549 <https://github.com/ros-planning/moveit2/issues/549>`_)
* Contributors: Henning Kayser, Nisala Kalupahana, Vatan Aksoy Tezer, David V. Lu, Jafar Abdi

2.2.1 (2021-07-12)
------------------
* Fix test dependencies (`#539 <https://github.com/ros-planning/moveit2/issues/539>`_)
* Add persistent planner support back (`#537 <https://github.com/ros-planning/moveit2/issues/537>`_)
* Contributors: Jochen Sprickerhof, Michael Görner

2.2.0 (2021-06-30)
------------------
* Enable Rolling and Galactic CI (`#494 <https://github.com/ros-planning/moveit2/issues/494>`_)
* Temporarily disable flaky OMPL test (`#495 <https://github.com/ros-planning/moveit2/issues/495>`_)
* [sync] MoveIt's master branch up-to https://github.com/ros-planning/moveit/commit/0d0a6a171b3fbea97a0c4f284e13433ba66a4ea4
  * CI: Use compiler flag --pedantic (`#2691 <https://github.com/ros-planning/moveit/issues/2691>`_)
  * Replaced eigen+kdl conversions with tf2_eigen + tf2_kdl (`#2472 <https://github.com/ros-planning/moveit/issues/2472>`_)
* Contributors: JafarAbdi, Michael Görner, Robert Haschke, Tyler Weaver, Vatan Aksoy Tezer, petkovich

2.1.4 (2021-05-31)
------------------

2.1.3 (2021-05-22)
------------------
* Fix incomplete start states in OMPL ThreadSafeStateStorage (`#455 <https://github.com/ros-planning/moveit2/issues/455>`_)
* ompl_interface: Fix loading group's specific parameters (`#461 <https://github.com/ros-planning/moveit2/issues/461>`_)
* Contributors: Jafar Abdi, Pradeep Rajendran

2.1.2 (2021-04-20)
------------------

2.1.1 (2021-04-12)
------------------
* Add differential drive joint model (`#390 <https://github.com/ros-planning/moveit2/issues/390>`_)
* Fix EXPORT install in CMake (`#372 <https://github.com/ros-planning/moveit2/issues/372>`_)
* OMPL constrained planning (`#347 <https://github.com/ros-planning/moveit2/issues/347>`_)
  Co-authored-by: JeroenDM <jeroendemaeyer@live.be>
* [fix] export cmake library install (`#339 <https://github.com/ros-planning/moveit2/issues/339>`_)
* Fix repo URLs in package.xml files
* Contributors: Boston Cleek, David V. Lu!!, Henning Kayser, Tyler Weaver

2.1.0 (2020-11-23)
------------------
* [fix] Rosdep dependencies for ros_testing, OpenMP (`#309 <https://github.com/ros-planning/moveit2/issues/309>`_)
* [fix] OMPL parameter loading (`#178 <https://github.com/ros-planning/moveit2/issues/178>`_)
* [maint] Wrap common cmake code in 'moveit_package()' macro (`#285 <https://github.com/ros-planning/moveit2/issues/285>`_)
  * New moveit_package() macro for compile flags, Windows support etc
  * Add package 'moveit_common' as build dependency for moveit_package()
  * Added -Wno-overloaded-virtual compiler flag for moveit_ros_planners_ompl
* [maint] Update to new moveit_resources layout (`#247 <https://github.com/ros-planning/moveit2/issues/247>`_)
* [maint] Enable clang-tidy-fix and ament_lint_cmake (`#210 <https://github.com/ros-planning/moveit2/issues/210>`_)
* [ros2-migration] Port move_group to ROS2 (`#217 <https://github.com/ros-planning/moveit2/issues/217>`_)
  * switch OMPL to use pluginlib
* Contributors: Edwin Fan, Henning Kayser, Jonathan Chapple, Lior Lustgarten

2.0.0 (2020-02-17)
------------------
* [fix] Fix OMPL logging macros
* [fix] Fix OMPL planner plugin install
* [improve] Load planner parameters from subnamespace
* [port] Port moveit_planners_ompl to ROS 2 (`#142 <https://github.com/ros-planning/moveit2/issues/142>`_)
* [improve] Load OMPL planner config parameters
* [sys] replace rosunit -> ament_cmake_gtest
* Contributors: Henning Kayser

1.1.1 (2020-10-13)
------------------
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski

1.1.0 (2020-09-04)
------------------

1.0.6 (2020-08-19)
------------------
* [maint] Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint] Migrate to clang-format-10
* [maint] Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* [fix]   Fix memcpy bug in copyJointToOMPLState in ompl interface (`#2239 <https://github.com/ros-planning/moveit/issues/2239>`_)
* Contributors: Jeroen, Markus Vieth, Robert Haschke, Michael Görner

1.0.5 (2020-07-08)
------------------
* [feature] Added support for hybridize/interpolate flags in ModelBasedPlanningContext via ompl_planning.yaml (`#2171 <https://github.com/ros-planning/moveit/issues/2171>`_, `#2172 <https://github.com/ros-planning/moveit/issues/2172>`_)
* Contributors: Constantinos, Mark Moll

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [maint] Cleanup OMPL dynamic reconfigure config (`#1649 <https://github.com/ros-planning/moveit/issues/1649>`_)
  * Reduce minimum number of waypoints in solution to 2
* [maint] Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint] Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint] Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* Contributors: Michael Görner, Robert Haschke, Sean Yen, Yu, Yan

1.0.2 (2019-06-28)
------------------

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Robert Haschke, Yu, Yan

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
* [fix] Fixed memory leak in OMPL planner (`#1104 <https://github.com/ros-planning/moveit/issues/1104>`_)
  * Resolve circular reference to ompl::geometric::SimpleSetupPtr
* [maintenance] Use C++14 (`#1146 <https://github.com/ros-planning/moveit/issues/1146>`_)
* [maintenance] Code Cleanup
  * `#1179 <https://github.com/ros-planning/moveit/issues/1179>`_
  * `#1196 <https://github.com/ros-planning/moveit/issues/1196>`_
* Contributors: Alex Moriarty, Dave Coleman, Robert Haschke

0.10.5 (2018-11-01)
-------------------
* [fix] Build regression (`#1174 <https://github.com/ros-planning/moveit/issues/1174>`_)
* Contributors: Chris Lalancette

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------
* [maintenance] Use locale independent conversion from double to string (`#1099 <https://github.com/ros-planning/moveit/issues/1099>`_)
* Contributors: Simon Schmeisser

0.10.2 (2018-10-24)
-------------------
* [capability] adaptions for OMPL 1.4 (`#903 <https://github.com/ros-planning/moveit/issues/903>`_)
* Contributors: Dave Coleman, Michael Görner, Mikael Arguedas, Mohmmad Ayman, Robert Haschke, mike lautman

0.10.1 (2018-05-25)
-------------------
* migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* switch to ROS_LOGGER from CONSOLE_BRIDGE (`#874 <https://github.com/ros-planning/moveit/issues/874>`_)
* Make trajectory interpolation in MoveIt consistent to OMPL (`#869 <https://github.com/ros-planning/moveit/issues/869>`_)
* Contributors: Bryce Willey, Ian McMahon, Mikael Arguedas, Robert Haschke, Xiaojian Ma

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [maintenance][kinetic onward] Remove OutputHandlerROS from ompl_interface (`#609 <https://github.com/ros-planning/moveit/issues/609>`_)
* Contributors: Bence Magyar

0.9.9 (2017-08-06)
------------------
* [improve][moveit_planners_ompl] Optional forced use of JointModelStateSpaceFactory (`#541 <https://github.com/ros-planning/moveit/issues/541>`_)
* Contributors: henhenhen

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* Always update initial robot state to prevent dirty robot state error.
* Contributors: Henning Kayser

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* Contributors: Dave Coleman

0.9.4 (2017-02-06)
------------------
* [enhancement] ompl_interface: uniform & simplified handling of the default planner (`#371 <https://github.com/ros-planning/moveit/issues/371>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Michael Goerner

0.9.3 (2016-11-16)
------------------
* [capability] Exposed planners from latest ompl release. (`#338 <https://github.com/ros-planning/moveit/issues/338>`_)
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon, Ruben Burger

0.9.2 (2016-11-05)
------------------

0.7.0 (2016-01-30)
------------------
* Removed trailing whitespace from entire repository
* Fixed include directory order to make ros package shadowing work.
* fixing internal storing of config settings
* Make sure an overlayed OMPL is used instead of the ROS one.
* fix simplifySolutions(bool) setter
  The method simplifySolutions(bool) always set the simplify_solutions member to true and the input variable "flag" was ignored.
  The method is fixed by setting the simplify_solutions member to the value of the input variable "flag".
* changed location of getDefaultPlanner
* Contributors: Bastian Gaspers, Christian Dornhege, Dave Coleman, Dave Hershberger, Sachin Chitta

0.6.7 (2014-10-28)
------------------
* Changed OMPL SimpleSetup member variable to shared pointer, passed MotionPlanningRequest to child function
* Simplified number of solve() entry points in moveit_planners_ompl
* Fixed uninitialized ``ptc_`` pointer causing a crash.
* renamed newGoal to new_goal for keeping with formatting
* setting GroupStateValidityCallbackFn member for constraint_sampler member and implementing callbacks for state validity checking
* added functions to check validit of state, and also to act as callback for constraint sampler
* Added copy function from MoveIt robot_state joint values to ompl state
* fix for demo constraints database linking error
* Namespaced less useful debug output to allow to be easily silenced using ros console
* Contributors: Dave Coleman, Dave Hershberger, Sachin Chitta, arjungm

0.6.6 (2014-07-06)
------------------
* indigo version of moveit planners
* fix compile error on Indigo
* Fix for getMeasure() virtual function OMPL change
* Move OMPL paths before catkin to avoid compilation against ROS OMPL package when specifying a different OMPL installation
* Fixed bug which limited the number of plans considered to the number of threads.
* Contributors: Alexander Stumpf, Chris Lewis, Dave Coleman, Ryan Luna, Sachin Chitta

0.5.5 (2014-03-22)
------------------
* update build system for ROS indigo
* Removed duplicate call to setPlanningScene(), added various comments
* Contributors: Dave Coleman, Ioan Sucan

0.5.4 (2014-02-06)
------------------
* fix segfault when multiple goals are passed to move_group

0.5.3 (2013-10-11)
------------------
* update to new API

0.5.2 (2013-09-23)
------------------
* porting to new robot state

0.5.1 (2013-08-13)
------------------
* make headers and author definitions aligned the same way; white space fixes
* namespace change for profiler

0.5.0 (2013-07-15)
------------------

0.4.2 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)
* port ompl plugin to new base class for planning_interface (using planning contexts)

0.4.1 (2013-07-04)
------------------
* use new location of RRTstar, add PRMstar
* Added new cost function that takes into account robot joint movements
* Added ability for parameter sweeping by allowing parameters to be changed in planning contexts
* Added ability to alter configs in a cache

0.4.0 (2013-05-27)
------------------
* propagating changes from moveit_core

0.3.11 (2013-05-02)
-------------------
* remove some debug output and add some fixes
* some fixes for planning with constraint approximations
* more refactoring; what used to work (including using constraint approximations) works fine. explicitly storing motions is not yet done
* refactor constraints storage stuff
* display random motions in a slightly more robust way
* remove follow constraints API
* combine ompl_interface and ompl_interface_ros
* don't print status
* remove option for ordering constraint approximations (and fix `#12 <https://github.com/ros-planning/moveit_planners/issues/12>`_)
* add test for jumping configs
* use project() instead of sample() for producing goals
* minor fixes and add demo database construction code
* switch to using the profiler in moveit and add one more debug tool

0.3.10 (2013-04-17)
-------------------
* Merge branch 'groovy-devel' of github.com:ros-planning/moveit_planners into groovy-devel
* remove incorrect dep
* add dynamic reconfigure options for `#2 <https://github.com/ros-planning/moveit_planners/issues/2>`_

0.3.9 (2013-04-16 13:39)
------------------------
* disable old style benchmarking

0.3.8 (2013-04-16 11:23)
------------------------
* fix `#8 <https://github.com/ros-planning/moveit_planners/issues/8>`_
* use namespace option in ompl plugin
* remove unused functions
* add buildtool depends
* Fixed state deserialization: now update var transform too
* collapse OMPL plugin to one package
* robustness fix
* Fixed github url name

0.3.7 (2013-03-09)
------------------
* Remove configure from PlanningScene
* add multi-collision to PlanningScene
* renaming kinematic_model to robot_model

0.3.6 (2013-02-02)
------------------
* complete renaming process
* rename KinematicState to RobotState, KinematicTrajectory to RobotTrajectory
* propagating fixes from moveit_core
* use new robot_trajectory lib

0.3.5 (2013-01-28)
------------------
* fix reporting of goal collisions
* add some verbose output for failing goals
* port to new DisplayTrajectory message
* propagate API changes from planning_interface
* minor fix
* use the project() method to improve constraint following algorithm
* change default build flags

0.3.4 (2012-12-20 23:59)
------------------------
* dynamic_reconfigure workaroung

0.3.3 (2012-12-20 21:51)
------------------------
* update dyn reconfig call

0.3.2 (2012-12-20 13:45)
------------------------
* fix call to obsolete function

0.3.1 (2012-12-19)
------------------
* using the constraint sampler loading library
* make sure sampled goals are valid
* fix buildtool tag

0.3.0 (2012-12-10)
------------------
* add a debug msg
* re-enable heuristic
* first working version of follow planner
* most of the follow alg, but not 100% complete yet
* pass valid state samplers into the follow algorithm
* add constrained valid state sampler
* minor fixes
* fixes some catkin CMakeLists issues
* add code to allow execution of follow()
* port test to groovy
* placeholder for to-be-added algorithm
* minor touch-ups; no real functional changes other than a bias for state samplers wrt dimension of the space (when sampling in a ball of dimension D, focus the sampling towards the surface of the ball)
* minor & incomplete fix

0.2.5 (2012-11-26)
------------------
* update to new message API

0.2.4 (2012-11-23)
------------------
* improve error message
* stricter error checking
* update include path

0.2.3 (2012-11-21 22:47)
------------------------
* use generalized version of getMaximumExtent()

0.2.2 (2012-11-21 22:41)
------------------------
* more fixes to planners
* removed bad include dir
* fixed some plugin issues
* fixed include dirs in ompl ros interface
* added gitignore for ompl/ros

0.2.1 (2012-11-06)
------------------
* update install location of include/

0.2.0 (2012-11-05)
------------------
* udpate install targets

0.1.2 (2012-11-01)
------------------
* bump version
* install the plugin lib as well
* add TRRT to the list of options

0.1.1 (2012-10-29)
------------------
* fixes for build against groovy

0.1.0 (2012-10-28)
------------------
* port to groovy
* added some groovy build system files
* more moving around of packages
