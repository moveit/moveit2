^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_planning_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Merge https://github.com/ros-planning/moveit/commit/c88f6fb64e9057a4b9a8f6fafc01060e8c48a216
* Merge remote-tracking branch 'origin/main' into feature/msa
* Launch file cleanup (`#1380 <https://github.com/ros-planning/moveit2/issues/1380>`_)
  * Launch file cleanup
  * Delete deprecated launch files
* Removing more boost usage (`#1372 <https://github.com/ros-planning/moveit2/issues/1372>`_)
* Remove manipulation from moveit_ros (`#1177 <https://github.com/ros-planning/moveit2/issues/1177>`_)
* Merge remote-tracking branch 'upstream/main' into feature/msa
* Removing some boost usage (`#1331 <https://github.com/ros-planning/moveit2/issues/1331>`_)
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Merge pull request `#3106 <https://github.com/ros-planning/moveit2/issues/3106>`_ from v4hn/pr-master-bind-them-all / banish bind()
* banish bind()
* Formatting (`#3105 <https://github.com/ros-planning/moveit2/issues/3105>`_)
* Replace obsolete distutils.core with setuptools (`#3103 <https://github.com/ros-planning/moveit2/issues/3103>`_)
  http://wiki.ros.org/noetic/Migration#Setuptools_instead_of_Distutils
* Contributors: Abishalini, AndyZe, David V. Lu, Henry Moore, Jafar, Michael Görner, Stephanie Eng, Vatan Aksoy Tezer, v4hn

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* move_group_interface: No need to spin after publishing (`#1250 <https://github.com/ros-planning/moveit2/issues/1250>`_)
* Enable cppcheck (`#1224 <https://github.com/ros-planning/moveit2/issues/1224>`_)
  Co-authored-by: jeoseo <jeongwooseo2012@gmail.com>
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Fix deprecated namespace (`#1228 <https://github.com/ros-planning/moveit2/issues/1228>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
  Adaption of https://github.com/ros-planning/moveit/pull/3106
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* RCLCPP Upgrade Bugfixes (`#1181 <https://github.com/ros-planning/moveit2/issues/1181>`_)
* Rename panda controllers
* Merge https://github.com/ros-planning/moveit/commit/424a5b7b8b774424f78346d1e98bf1c9a33f0e78
* Enable rolling / jammy CI (again) (`#1134 <https://github.com/ros-planning/moveit2/issues/1134>`_)
  * Use ros2_control binaries
  * Use output screen instead of explicitly stating stderr
* Update black version, formatting changes (`#1148 <https://github.com/ros-planning/moveit2/issues/1148>`_)
* Merge https://github.com/ros-planning/moveit/commit/a25515b73d682df03ed3eccd839110c296aa79fc
* Merge https://github.com/ros-planning/moveit/commit/ab42a1d7017b27eb6c353fb29331b2da08ab0039
* Temporarily add galactic CI (`#1107 <https://github.com/ros-planning/moveit2/issues/1107>`_)
  * Add galactic CI
  * Comment out rolling
  * panda_ros_controllers -> panda_ros2_controllers
  * Ignore flake8 tests
* 1.1.9
* 1.1.8
* Add moveit_configs_utils package to simplify loading paramters (`#591 <https://github.com/ros-planning/moveit2/issues/591>`_)
  Co-authored-by: AndyZe <zelenak@picknik.ai>
  Co-authored-by: Stephanie Eng <stephanie-eng@users.noreply.github.com>
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
* 1.1.7
* Move MoveItErrorCode class to moveit_core (`#3009 <https://github.com/ros-planning/moveit2/issues/3009>`_)
  ... reducing code duplication and facilitating re-use
* Fix MoveGroupInterface uninitialized RobotState (`#3008 <https://github.com/ros-planning/moveit2/issues/3008>`_)
* Switch to std::bind (`#2967 <https://github.com/ros-planning/moveit2/issues/2967>`_)
  * boost::bind -> std::bind
  grep -rlI --exclude-dir=.git "boost::bind" | xargs sed -i 's/boost::bind/std::bind/g'
  * Convert bind placeholders
  grep -rlI --exclude-dir=.git " _[0-9]" | xargs sed -i 's/ _\([0-9]\)/ std::placeholders::_\1/g'
  * Update bind include header
  grep -rlI --exclude-dir=.git "boost/bind" | xargs sed -i 's#boost/bind.hpp#functional#'
* 1.1.6
* Merge PRs `#2948 <https://github.com/ros-planning/moveit2/issues/2948>`_ (improve CI) and `#2949 <https://github.com/ros-planning/moveit2/issues/2949>`_ (simplify ROS .test files)
* Use test_environment.launch in unittests
* Contributors: Abishalini, Captain Yoshi, David V. Lu!!, Henning Kayser, Jafar, Jafar Abdi, Jochen Sprickerhof, Robert Haschke, Stephanie Eng, Tyler Weaver, Vatan Aksoy Tezer, jeoseo, v4hn

2.4.0 (2022-01-20)
------------------
* Fix boost linking errors for Windows (`#957 <https://github.com/ros-planning/moveit2/issues/957>`_)
* Replace NULL with nullptr (`#961 <https://github.com/ros-planning/moveit2/issues/961>`_)
  * Fixes `#841 <https://github.com/ros-planning/moveit2/issues/841>`_
* Merge https://github.com/ros-planning/moveit/commit/a0ee2020c4a40d03a48044d71753ed23853a665d
* moveit_build_options()
  Declare common build options like CMAKE_CXX_STANDARD, CMAKE_BUILD_TYPE,
  and compiler options (namely warning flags) once.
  Each package depending on moveit_core can use these via moveit_build_options().
* MGI: add missing replan/look options to interface (`#2892 <https://github.com/ros-planning/moveit2/issues/2892>`_)
  - reordered methods because looking requires replanning
  - there's no sense in wrapping methods in methods.
  just use pimpl-friend paradigm instead. Someone could
  rework all the other methods in the future.
* PSI: get object.pose from new msg field (`#2877 <https://github.com/ros-planning/moveit2/issues/2877>`_)
* Contributors: Abishalini, Akash, Gauthier Hentz, Michael Görner, Robert Haschke, Stephanie Eng

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* Consider simulated time (`#883 <https://github.com/ros-planning/moveit2/issues/883>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* Find/replace deprecated spawner.py (`#737 <https://github.com/ros-planning/moveit2/issues/737>`_)
* common_objects: getSharedRobotModelLoader fix deadlock (`#734 <https://github.com/ros-planning/moveit2/issues/734>`_)
* fix trajectory constraints for moveit commander (`#2429 <https://github.com/ros-planning/moveit/issues/2429>`_)
* MGI::setStartState: Only fetch current state when new state is diff (`#2775 <https://github.com/ros-planning/moveit/issues/2775>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: AndyZe, Dave Coleman, David V. Lu!!, Felix von Drigalski, Gaël Écorchard, Henning Kayser, Jafar Abdi, Kevin Chang, Robert Haschke, pvanlaar

2.3.0 (2021-10-08)
------------------
* Support passing MoveGroup's namespace to MoveGroupInterface (`#533 <https://github.com/ros-planning/moveit2/issues/533>`_)
* Add getSharedRobotModelLoader to fix race condition when having multiple displays for the same node (`#525 <https://github.com/ros-planning/moveit2/issues/525>`_)
* Make TF buffer & listener in PSM private (`#654 <https://github.com/ros-planning/moveit2/issues/654>`_)
  * Add private buffer & tf listener to PSM
  * Remove coupled deleter
  * Decouple PSM from CSM
  * Deprecate old constructors
* getInterfaceDescription: Fix rclcpp API breakage (`#686 <https://github.com/ros-planning/moveit2/issues/686>`_)
* [main] Migrate to joint_state_broadcaster (`#657 <https://github.com/ros-planning/moveit2/issues/657>`_)
* Fix warnings in Galactic and Rolling (`#598 <https://github.com/ros-planning/moveit2/issues/598>`_)
  * Use __has_includes preprocessor directive for deprecated headers
  * Fix parameter template types
  * Proper initialization of smart pointers, rclcpp::Duration
* Create a transform subscribers to enable virtual joints (`#310 <https://github.com/ros-planning/moveit2/issues/310>`_)
* Fix loading joint_limits.yaml in demo and test launch files (`#544 <https://github.com/ros-planning/moveit2/issues/544>`_)
* Fixes for Windows (`#530 <https://github.com/ros-planning/moveit2/issues/530>`_)
* Contributors: Akash, Jafar Abdi, Nisala Kalupahana, Jorge Nicho, Henning Kayser, Vatan Aksoy Tezer, Tyler Weaver, Lior Lustgarten

2.2.1 (2021-07-12)
------------------
* Fix test dependencies (`#539 <https://github.com/ros-planning/moveit2/issues/539>`_)
* Contributors: Jochen Sprickerhof

2.2.0 (2021-06-30)
------------------
* Enable Rolling and Galactic CI (`#494 <https://github.com/ros-planning/moveit2/issues/494>`_)
* [sync] with MoveIt's master branch up-to https://github.com/ros-planning/moveit/commit/0d0a6a171b3fbea97a0c4f284e13433ba66a4ea4
  * Allow selecting planning pipeline in MotionSequenceAction (`#2657 <https://github.com/ros-planning/moveit/issues/2657>`_)
  * planning_interface: synchronize async interfaces in test (`#2640 <https://github.com/ros-planning/moveit/issues/2640>`_)
  * Add planning_pipeline_id setting to Python MGI (`#2622 <https://github.com/ros-planning/moveit/issues/2622>`_)
  * fix docstring in MGI API (`#2626 <https://github.com/ros-planning/moveit/issues/2626>`_)
  * Support multiple planning pipelines with MoveGroup via MoveItCpp (`#2127 <https://github.com/ros-planning/moveit/issues/2127>`_)
    * Deprecate namespace moveit::planning_interface in favor of moveit_cpp
  * add get_active_joint_names (`#2533 <https://github.com/ros-planning/moveit/issues/2533>`_)
  * Add debugging log statement for a common error (`#2509 <https://github.com/ros-planning/moveit/issues/2509>`_)
  * Replaced eigen+kdl conversions with tf2_eigen + tf2_kdl (`#2472 <https://github.com/ros-planning/moveit/issues/2472>`_)
* Contributors: Felix von Drigalski, Henning Kayser, JafarAbdi, Michael Görner, Peter Mitrano, Robert Haschke, Tyler Weaver, Vatan Aksoy Tezer, petkovich

2.1.4 (2021-05-31)
------------------
* Disable flaky test (`#482 <https://github.com/ros-planning/moveit2/issues/482>`_)
* Delete MoveIt fake_controller_manager (`#471 <https://github.com/ros-planning/moveit2/issues/471>`_)
* Contributors: AndyZe, Vatan Aksoy Tezer

2.1.3 (2021-05-22)
------------------
* Configure OMPL projection_evaluator in move_group_launch_test_common.py (`#470 <https://github.com/ros-planning/moveit2/issues/470>`_)
* Contributors: Jafar Abdi

2.1.2 (2021-04-20)
------------------
* Re-enable test_servo_pose_tracking integration test (`#423 <https://github.com/ros-planning/moveit2/issues/423>`_)
  Co-authored-by: AndyZe <zelenak@picknik.ai>
* Re-enable moveit_ros_warehouse for moveit_ros_planning_interface (`#424 <https://github.com/ros-planning/moveit2/issues/424>`_)
  * Remove warehouse_ros_mongo from moveit_ros_planning_interface test depends
* Unify PickNik name in copyrights (`#419 <https://github.com/ros-planning/moveit2/issues/419>`_)
* Contributors: Jafar Abdi, Tyler Weaver, Vatan Aksoy Tezer

2.1.1 (2021-04-12)
------------------
* Update launch files to use ros2 control spawner (`#405 <https://github.com/ros-planning/moveit2/issues/405>`_)
* Use fake_components::GenericSystem from ros2_control (`#361 <https://github.com/ros-planning/moveit2/issues/361>`_)
* Solved small issue with a message not being logged due to an early return statement (`#368 <https://github.com/ros-planning/moveit2/issues/368>`_)
* OMPL constrained planning (`#347 <https://github.com/ros-planning/moveit2/issues/347>`_)
* Sync main branch with MoveIt 1 from previous head https://github.com/ros-planning/moveit/commit/0247ed0027ca9d7f1a7f066e62c80c9ce5dbbb5e up to https://github.com/ros-planning/moveit/commit/74b3e30db2e8683ac17b339cc124675ae52a5114
* [fix] MGI server timeout, infinite duration by default (`#349 <https://github.com/ros-planning/moveit2/issues/349>`_)
  By setting the default server timeout duration to -1, the MoveGroupInterface is ensured to be ready to use after construction.
* [fix] export cmake library install (`#339 <https://github.com/ros-planning/moveit2/issues/339>`_)
* Fix scaling factor parameter names (`#2452 <https://github.com/ros-planning/moveit2/issues/2452>`_)
* MTC compatibility fixes (`#323 <https://github.com/ros-planning/moveit2/issues/323>`_)
* Fix node remapping
* Make sure planning scene interface have a unique name for the internal node
* planning_scene_interface: Fix node name being empty
* Fix repo URLs in package.xml files
* Contributors: Boston Cleek, FlorisE, Henning Kayser, Jafar Abdi, Shota Aoki, Tyler Weaver

2.1.0 (2020-11-23)
------------------
* [maint] Wrap common cmake code in 'moveit_package()' macro (`#285 <https://github.com/ros-planning/moveit2/issues/285>`_)
  * New moveit_package() macro for compile flags, Windows support etc
  * Add package 'moveit_common' as build dependency for moveit_package()
  * Added -Wno-overloaded-virtual compiler flag for moveit_ros_planners_ompl
* [maint] Compilation fixes for macOS (`#271 <https://github.com/ros-planning/moveit2/issues/271>`_)
* [ros2-migration] Port MoveGroupInterface and MotionPlanning display (`#272 <https://github.com/ros-planning/moveit2/issues/272>`_)
* Contributors: Henning Kayser, Jafar Abdi, Lior Lustgarten, Mark Moll, Yu Yan

2.0.0 (2020-02-17)
------------------
* [fix] Install moveit_cpp as SHARED library
* [fix] Fix parameter lookup and runtime in MoveItCpp
* [fix] Fix moveit_ros_visualization (`#167 <https://github.com/ros-planning/moveit2/issues/167>`_)
* [fix] moveit_ros_planning_interface: Fix libraries export
* [port] Port moveit_cpp to ROS 2 (`#163 <https://github.com/ros-planning/moveit2/issues/163>`_)
* [port] Port common_planning_interface_objects to ROS 2 (`#159 <https://github.com/ros-planning/moveit2/issues/159>`_)
* [port] Port rdf_loader to ROS2 (`#104 <https://github.com/ros-planning/moveit2/issues/104>`_)
* Contributors: Henning Kayser, Jafar Abdi

1.1.1 (2020-10-13)
------------------
* [feature] moveit_cpp: more informative error message, cover another potential failure condition. (`ros-planning:moveit#2336 <https://github.com/ros-planning/moveit/issues/2336>`_)
* [fix] Make GILReleaser exception-safe (`ros-planning:moveit#2363 <https://github.com/ros-planning/moveit/issues/2363>`_)
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`ros-planning:moveit#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* [maint] Replace panda_moveit_config -> moveit_resources_panda_moveit_config (`ros-planning:moveit#2300 <https://github.com/ros-planning/moveit/issues/2300>`_)
* Contributors: AndyZe, Bjar Ne, Felix von Drigalski, Robert Haschke

1.1.0 (2020-09-04)
------------------
* [feature] Use Eigen::Transform::linear() instead of rotation() (`ros-planning:moveit#1964 <https://github.com/ros-planning/moveit/issues/1964>`_)
* [feature] move_group pick place test (`ros-planning:moveit#2031 <https://github.com/ros-planning/moveit/issues/2031>`_)
* [feature] Check for grasp service - general cleanup MGI (`ros-planning:moveit#2077 <https://github.com/ros-planning/moveit/issues/2077>`_)
* [feature] Integration test to defend subframe tutorial (`ros-planning:moveit#1757 <https://github.com/ros-planning/moveit/issues/1757>`_)
* [feature] Release Python GIL for C++ calls (`ros-planning:moveit#1947 <https://github.com/ros-planning/moveit/issues/1947>`_)
* [feature] Add default velocity/acceleration scaling factors (`ros-planning:moveit#1890 <https://github.com/ros-planning/moveit/issues/1890>`_)
* [feature] Improve move_group_interface's const correctness (`ros-planning:moveit#1715 <https://github.com/ros-planning/moveit/issues/1715>`_)
* [feature] Add get_jacobian_matrix to moveit_commander (`ros-planning:moveit#1501 <https://github.com/ros-planning/moveit/issues/1501>`_)
* [feature] Add named frames to CollisionObjects (`ros-planning:moveit#1439 <https://github.com/ros-planning/moveit/issues/1439>`_)
* [feature] Added GILRelease to pick and place (`ros-planning:moveit#2272 <https://github.com/ros-planning/moveit/issues/2272>`_)
* [feature] Add missing variants of place from list of PlaceLocations and Poses in the python interface (`ros-planning:moveit#2231 <https://github.com/ros-planning/moveit/issues/2231>`_)
* [fix] Various fixes for upcoming Noetic release (`ros-planning:moveit#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Resolve PSI lock-up in RViz display (`ros-planning:moveit#1951 <https://github.com/ros-planning/moveit/issues/1951>`_)
* [fix] Fix flaky moveit_cpp test (`ros-planning:moveit#1781 <https://github.com/ros-planning/moveit/issues/1781>`_)
* [fix] Fix compiler warnings (`ros-planning:moveit#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] Fix a parameter mix-up in moveit_cpp loading (`ros-planning:moveit#2187 <https://github.com/ros-planning/moveit/issues/2187>`_)
* [maint] Optional cpp version setting (`ros-planning:moveit#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [maint] update dependencies for python3 in noetic (`ros-planning:moveit#2131 <https://github.com/ros-planning/moveit/issues/2131>`_)
* [maint] clang-tidy fixes (`ros-planning:moveit#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `ros-planning:moveit#1586 <https://github.com/ros-planning/moveit/issues/1586>`_, `ros-planning:moveit#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Fix some clang tidy issues (`ros-planning:moveit#2004 <https://github.com/ros-planning/moveit/issues/2004>`_)
* [maint] export  moveit_py_bindings_tools library (`ros-planning:moveit#1970 <https://github.com/ros-planning/moveit/issues/1970>`_)
* [maint] Fix usage of panda_moveit_config (`ros-planning:moveit#1904 <https://github.com/ros-planning/moveit/issues/1904>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`ros-planning:moveit#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Fix typo in cmake file (`ros-planning:moveit#1857 <https://github.com/ros-planning/moveit/issues/1857>`_)
* [maint] Reduce console output warnings (`ros-planning:moveit#1845 <https://github.com/ros-planning/moveit/issues/1845>`_)
* [maint] Switch from include guards to pragma once (`ros-planning:moveit#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`ros-planning:moveit#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* [maint] improve [get|set]JointValueTarget in python wrapper (`ros-planning:moveit#858 <https://github.com/ros-planning/moveit/issues/858>`_)
* [maint] moveit_commander.MoveGroupInterface.plan() to better align with C++ MoveGroup::plan() (`ros-planning:moveit#790 <https://github.com/ros-planning/moveit/issues/790>`_)
* Contributors: AndyZe, Ayush Garg, Bence Magyar, Bjar Ne, Dave Coleman, Felix von Drigalski, Gerard Canal, Guilhem Saurel, Henning Kayser, Jafar Abdi, JafarAbdi, Jere Liukkonen, Jonathan Binney, Kunal Tyagi, Luca Rinelli, Mahmoud Ahmed Selim, Markus Vieth, Martin Pecka, Masaki Murooka, Michael Ferguson, Michael Görner, Niklas Fiedler, Robert Haschke, Ryosuke Tajima, Sean Yen, Tyler Weaver, Yeshwanth, Yu, Yan, mvieth, v4hn

1.0.6 (2020-08-19)
------------------
* [maint]   Adapt repository for splitted moveit_resources layout (`ros-planning:moveit#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint]   Migrate to clang-format-10, Fix warnings
* [maint]   Optimize includes (`ros-planning:moveit#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* [feature] Exposed parameter wait_for_servers and getPlannerId() API in MoveGroup's Python API (`ros-planning:moveit#2201 <https://github.com/ros-planning/moveit/issues/2201>`_)
* Contributors: Gerard Canal, Markus Vieth, Robert Haschke, Michael Görner

1.0.5 (2020-07-08)
------------------
* [maint]   Remove dependency on panda_moveit_config (ros-planning:moveit#2194 <https://github.com/ros-planning/moveit/issues/2194>`_, ros-planning:moveit#2197 <https://github.com/ros-planning/moveit/issues/2197>`_)
* [maint]   Adapt linking to eigenpy (`ros-planning:moveit#2118 <https://github.com/ros-planning/moveit/issues/2118>`_)
* [maint]   Replace robot_model and robot_state namespaces with moveit::core (`ros-planning:moveit#2135 <https://github.com/ros-planning/moveit/issues/2135>`_)
* [feature] PlanningComponent: Load plan_request_params (`ros-planning:moveit#2033 <https://github.com/ros-planning/moveit/issues/2033>`_)
* [feature] MoveItCpp: a high-level C++ planning API (`ros-planning:moveit#1656 <https://github.com/ros-planning/moveit/issues/1656>`_)
* [fix]     Validate action client pointer before access
* [fix]     Wait and check for the grasp service
* [maint]   Add tests for move_group interface (`ros-planning:moveit#1995 <https://github.com/ros-planning/moveit/issues/1995>`_)
* Contributors: AndyZe, Henning Kayser, Jafar Abdi, Michael Görner, Robert Haschke, Tyler Weaver, Yeshwanth

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [feature] `MoveGroupInterface`: Add execution methods for moveit_msgs::RobotTrajectory (`ros-planning:moveit#1955 <https://github.com/ros-planning/moveit/issues/1955>`_)
* [feature] Allow to instantiate a `PlanningSceneInterface` w/ and w/o a running `move_group` node
* [fix]     Release Python `GIL` for C++ calls (`ros-planning:moveit#1947 <https://github.com/ros-planning/moveit/issues/1947>`_)
* [feature] Expose reference_point_position parameter in getJacobian() (`ros-planning:moveit#1595 <https://github.com/ros-planning/moveit/issues/1595>`_)
* [feature] `MoveGroupInterface`: Expose `constructPickGoal` and `constructPlaceGoal` (`ros-planning:moveit#1498 <https://github.com/ros-planning/moveit/issues/1498>`_)
* [feature] `python MoveGroupInterface`: Added custom time limit for `wait_for_servers()` (`ros-planning:moveit#1444 <https://github.com/ros-planning/moveit/issues/1444>`_)
* [maint]   Apply clang-tidy fix to entire code base (`ros-planning:moveit#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint]   Fix errors: catkin_lint 1.6.7 (`ros-planning:moveit#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint]   Improve Python 3 compatibility (`ros-planning:moveit#1870 <https://github.com/ros-planning/moveit/issues/1870>`_)
  * Replaced StringIO with BytesIO for python msg serialization
  * Use py_bindings_tools::ByteString as byte-based serialization buffer on C++ side
* [feature] Export moveit_py_bindings_tools library
* [maint]   Fix various build issues on Windows
  * Use `.pyd` as the output suffix for Python module on Windows. (`ros-planning:moveit#1637 <https://github.com/ros-planning/moveit/issues/1637>`_)
  * Favor ros::Duration.sleep over sleep. (`ros-planning:moveit#1634 <https://github.com/ros-planning/moveit/issues/1634>`_)
  * Fix binary artifact install locations. (`ros-planning:moveit#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`ros-planning:moveit#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [maint]   Updated deprecation method: MOVEIT_DEPRECATED -> [[deprecated]] (`ros-planning:moveit#1748 <https://github.com/ros-planning/moveit/issues/1748>`_)
* [maint]   `eigenpy`: switched to system package (`ros-planning:moveit#1737 <https://github.com/ros-planning/moveit/issues/1737>`_)
* [featue]  `PlanningSceneInterface`: wait for its two services
* [feature] Select time parametrization algorithm in retime_trajectory (`ros-planning:moveit#1508 <https://github.com/ros-planning/moveit/issues/1508>`_)
* Contributors: Bjar Ne, Felix von Drigalski, Kunal Tyagi, Luca Rinelli, Masaki Murooka, Michael Görner, Niklas Fiedler, Robert Haschke, Sean Yen, Yu, Yan, mvieth, v4hn

1.0.2 (2019-06-28)
------------------
* [maintenance] Removed unnecessary null pointer checks on deletion (`ros-planning:moveit#1410 <https://github.com/ros-planning/moveit/issues/1410>`_)
* Contributors: Mahmoud Ahmed Selim

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`ros-planning:moveit#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`ros-planning:moveit#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* [improve] Remove (redundant) random seeding and ros-planning:moveit#attempts from RobotState::setFromIK() as the IK solver perform random seeding themselves. `ros-planning:moveit#1288 <https://github.com/ros-planning/moveit/issues/1288>`_
* Contributors: Dave Coleman, Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------
* [fix] Fixed destruction order of shared tf2::Buffer / tf2::TransformListener (`ros-planning:moveit#1261 <https://github.com/ros-planning/moveit/pull/1261>`_)
* Contributors: Robert Haschke

0.10.6 (2018-12-09)
-------------------
* [fix] Fixed various memory leaks (`ros-planning:moveit#1104 <https://github.com/ros-planning/moveit/issues/1104>`_)
  * SharedStorage: Use weak_ptrs for caching
* [enhancement] Add getMoveGroupClient() to move_group_interface (`ros-planning:moveit#1215 <https://github.com/ros-planning/moveit/issues/1215>`_)
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`ros-planning:moveit#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Remove deprecated MoveGroup class (`ros-planning:moveit#1211 <https://github.com/ros-planning/moveit/issues/1211>`_)
* [maintenance] Use C++14 (`ros-planning:moveit#1146 <https://github.com/ros-planning/moveit/issues/1146>`_)
* [maintenance] Code Cleanup
  * `ros-planning:moveit#1179 <https://github.com/ros-planning/moveit/issues/1179>`_
  * `ros-planning:moveit#1196 <https://github.com/ros-planning/moveit/issues/1196>`_
* Contributors: Alex Moriarty, Dave Coleman, Martin Günther, Michael Görner, Robert Haschke

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------
* [capability] Get available planning group names from MoveGroup C++ (`ros-planning:moveit#1159 <https://github.com/ros-planning/moveit/issues/1159>`_)
* Contributors: Dave Coleman

0.10.2 (2018-10-24)
-------------------
* [capability] Added plan_only flags to pick and place (`ros-planning:moveit#862 <https://github.com/ros-planning/moveit/issues/862>`_)
* [maintenance] Python3 support (`ros-planning:moveit#1103 <https://github.com/ros-planning/moveit/issues/1103>`_, `ros-planning:moveit#1054 <https://github.com/ros-planning/moveit/issues/1054>`_)
* [fix] optional namespace args (`ros-planning:moveit#929 <https://github.com/ros-planning/moveit/issues/929>`_)
* Contributors: David Watkins, Michael Görner, Mohmmad Ayman, Robert Haschke, mike lautman

0.10.1 (2018-05-25)
-------------------
* [maintenance] Remove deprecated ExecuteTrajectoryServiceCapability (`ros-planning:moveit#833 <https://github.com/ros-planning/moveit/issues/833>`_)
* [maintenance] migration from tf to tf2 API (`ros-planning:moveit#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* [maintenance] switch to ROS_LOGGER from CONSOLE_BRIDGE (`ros-planning:moveit#874 <https://github.com/ros-planning/moveit/issues/874>`_)
* [capability] namespace to moveit_commander (`ros-planning:moveit#835 <https://github.com/ros-planning/moveit/issues/835>`_)
* Constrained Cartesian planning using moveit commander (`ros-planning:moveit#805 <https://github.com/ros-planning/moveit/issues/805>`_)
* Simplify adding CollisionObjects with colors (`ros-planning:moveit#810 <https://github.com/ros-planning/moveit/issues/810>`_)
* support TrajectoryConstraints in MoveGroupInterface + MoveitCommander (`ros-planning:moveit#793 <https://github.com/ros-planning/moveit/issues/793>`_)
* Add API to get planner_id (`ros-planning:moveit#788 <https://github.com/ros-planning/moveit/issues/788>`_)
* Allow wait time to be specified for getCurrentState() (`ros-planning:moveit#685 <https://github.com/ros-planning/moveit/issues/685>`_)
* Contributors: 2scholz, Akiyoshi Ochiai, Bence Magyar, Dave Coleman, Ian McMahon, Robert Haschke, Will Baker, Xiaojian Ma, srsidd

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] MoveGroupInterface: Fixed computeCartesianPath to use selected end-effector. (`ros-planning:moveit#580 <https://github.com/ros-planning/moveit/issues/580>`_)
* [capability][kinetic onward] Adapt pick pipeline to function without object (`ros-planning:moveit#599 <https://github.com/ros-planning/moveit/issues/599>`_)
* [improve] Disabled copy constructors and added a move constructor to MoveGroupInterface (`ros-planning:moveit#664 <https://github.com/ros-planning/moveit/issues/664>`_)
* Contributors: 2scholz, Dennis Hartmann, Jonathan Meyer, Simon Schmeisser

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [improve] MoveGroupInterface: add public interface to construct the MotionPlanRequest (`ros-planning:moveit#461 <https://github.com/ros-planning/moveit/issues/461>`_)
* Contributors: Michael Goerner

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `ros-planning:moveit#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`ros-planning:moveit#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* [enhancement][MoveGroup] Add getLinkNames function (`ros-planning:moveit#440 <https://github.com/ros-planning/moveit/issues/440>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [fix] move_group.cpp: seg fault bug (`ros-planning:moveit#426 <https://github.com/ros-planning/moveit/issues/426>`_)
* [fix] mgi: show correct include path in doxygen (`ros-planning:moveit#419 <https://github.com/ros-planning/moveit/issues/419>`_)
* [fix] fix race conditions when updating PlanningScene (`ros-planning:moveit#350 <https://github.com/ros-planning/moveit/issues/350>`_)
* [fix] issue `ros-planning:moveit#373 <https://github.com/ros-planning/moveit/issues/373>`_ for Kinetic (`ros-planning:moveit#377 <https://github.com/ros-planning/moveit/issues/377>`_) (`ros-planning:moveit#385 <https://github.com/ros-planning/moveit/issues/385>`_)
* [capability] PSI: add apply* functions that use ApplyPlanningScene.srv (`ros-planning:moveit#381 <https://github.com/ros-planning/moveit/issues/381>`_)
* [maintenance] Fix test file issues (`ros-planning:moveit#415 <https://github.com/ros-planning/moveit/pull/415>`_, `ros-planning:moveit#412 <https://github.com/ros-planning/moveit/issues/412>`_)
* [maintenance] clang-format upgraded to 3.8 (`ros-planning:moveit#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Bastian Gaspers, Dave Coleman, Isaac I.Y. Saito, Jorge Santos Simon, Michael Goerner, Robert Haschke

0.9.3 (2016-11-16)
------------------

0.6.6 (2016-06-08)
------------------
* replaced cmake_modules dependency with eigen
* [jade] eigen3 adjustment
* merge indigo-devel changes (PR `ros-planning:moveit-ros#633 <https://github.com/ros-planning/moveit_ros/issues/633>`_ trailing whitespace) into jade-devel
* Removed trailing whitespace from entire repository
* planning_interface::MoveGroup::get/setPlannerParams
* new method MoveGroup::getDefaultPlannerId(const std::string &group)
  ... to retrieve default planner config from param server
  moved corresponding code from rviz plugin to MoveGroup interface
  to facilitate re-use
* fixing conflicts, renaming variable
* Merge pull request `ros-planning:moveit-ros#589 <https://github.com/ros-planning/moveit_ros/issues/589>`_ from MichaelStevens/set_num_planning_attempts
  adding set_num_planning_attempts to python interface
* comments addressed
* Added python wrapper for setMaxVelocityScalingFactor
* saves robot name to db from moveit. also robot name accessible through robot interface python wrapper
* adding set_num_planning_attempts to python interface
* Merge pull request `ros-planning:moveit-ros#571 <https://github.com/ros-planning/moveit_ros/issues/571>`_ from ymollard/indigo-devel
  Added python wrapper for MoveGroup.asyncExecute()
* Added python wrapper for MoveGroup.asyncExecute()
* Add retime_trajectory to moveit python wrapper
* add getHandle to move_group_interface
* Updated documentation on move() to inform the user that an asynchronus spinner is required. Commonly new users don't do this and move() blocks permanently
* Contributors: Dave Coleman, Dave Hershberger, Isaac I.Y. Saito, Kei Okada, Michael Stevens, Robert Haschke, Sachin Chitta, Scott, Yoan Mollard, dg, ferherranz

0.6.5 (2015-01-24)
------------------
* update maintainers
* Add time factor support for iterative_time_parametrization
* Contributors: Michael Ferguson, kohlbrecher

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------
* include correct ``boost::*_ptr`` class for boost 1.57.
* Contributors: v4hn

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------

0.6.0 (2014-10-27)
------------------
* Add missing variants of place (PlaceLocation, place anywhere) for python interface
* Python wrapper for getEndEffectorTips()
* Contributors: Dave Coleman, Sachin Chitta, corot

0.5.19 (2014-06-23)
-------------------
* Add check for planning scene monitor connection, with 5 sec delay
* Contributors: Dave Coleman

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* added move_group python interface bindings to move group interface
  function:
  void setPathConstraints(const moveit_msgs::Constraint &constraint)
  in order to be able to set path constraints from python scripts
  directly and no need to use the DB.
* Use member NodeHandle in action clients.
  Currently services and topics are already using the member NodeHandle instance,
  but not the action clients.
  This is relevant for two reasons:
  - Consistency in the resulting ROS API namespace (everything in the same namespace).
  - Consistency in the spinning policy. All services, topics and actions will be spinned
  by the same NodeHandle, and whatever custom (or not) spinners and callback queues it
  has associated.
* adding error code returns to relevant functions
* Contributors: Adolfo Rodriguez Tsouroukdissian, Emili Boronat, Ioan A Sucan, Sachin Chitta

0.5.16 (2014-02-27)
-------------------
* adding node handle to options in move_group_interface
* adding get for active joints
* Contributors: Sachin Chitta

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------
* add API for setting the number of motion plans to be evaluated via the MoveGroupInterface
* move_group_interface: improve documentation
* Contributors: Acorn Pooley, Ioan Sucan

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------
* Fixed bug in computeCartesianPathPython.
* Adding collision object interface to planning_scene interface.
* Contributors: Acorn Pooley, Sachin Chitta

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* Fixed doxygen function-grouping.
* Added planning feedback to gui, refactored states tab

0.5.8 (2013-10-11)
------------------
* add function to start state monitor in move_group_interface::MoveGroup

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------
* update planning options

0.5.5 (2013-09-23)
------------------
* add support for setting joint targets from approximate IK
* specifies python version 2.7 for linking (fixes `ros-planning:moveit-ros#302 <https://github.com/ros-planning/moveit_ros/issues/302>`_)
* use new messages for pick & place
* expand functionality of MoveGroupInterface
* porting to new RobotState API

0.5.4 (2013-08-14)
------------------

* make pick more general
* use message serialization for python bindings
* remove CollisionMap, expose topic names in PlanningSceneMonitor, implement detach / attach operations as requested by `ros-planning:moveit-ros#280 <https://github.com/ros-planning/moveit_ros/issues/280>`_
* make headers and author definitions aligned the same way; white space fixes

0.5.2 (2013-07-15)
------------------
* move msgs to common_msgs

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
* some refactoring
