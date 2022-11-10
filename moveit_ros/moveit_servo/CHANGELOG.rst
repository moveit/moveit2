^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_servo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.0 (2022-11-10)
------------------
* Fix dead tutorial link (`#1701 <https://github.com/ros-planning/moveit2/issues/1701>`_)
  When we refactored the tutorials site it looks like we killed some links. Do we not have a CI job to catch dead links?
* [Servo] CI simplification (`#1556 <https://github.com/ros-planning/moveit2/issues/1556>`_)
  This reverts commit 3322f19056d10d5e5c95c0276e383b048a840573.
* [Servo] Remove the option for "stop distance"-based collision checking (`#1574 <https://github.com/ros-planning/moveit2/issues/1574>`_)
* Merge PR `#1553 <https://github.com/ros-planning/moveit2/issues/1553>`_: Improve cmake files
* Use standard exported targets: export\_${PROJECT_NAME} -> ${PROJECT_NAME}Targets
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* [Servo] Use a WallRate so the clock is monotonically increasing (`#1543 <https://github.com/ros-planning/moveit2/issues/1543>`_)
  * [Servo] Use a WallRate so the clock is monotonically increasing
  * Re-enable a commented integration test
* Disable flaky test_servo_singularity + test_rdf_integration (`#1530 <https://github.com/ros-planning/moveit2/issues/1530>`_)
* Enforce singularity threshold when moving away from a singularity (`#620 <https://github.com/ros-planning/moveit2/issues/620>`_)
  * Enforce singularity threshold behavior even when moving away from a singularity
  - Prevent uncontrolled behavior when servo starts close to a singularity and then servos away from it
  - Scale velocity at a different rate when approaching/leaving singularity
  - Add status code to distinguish between velocity scaling when moving towards/away from the singularity
  * Work on expanding servo singularity tests
  * Pre-commit
  * removed duplicate input checking
  * added 2 other tests
  * undid changes to singularity test
  * Update moveit_ros/moveit_servo/src/servo_calcs.cpp with Nathan's suggestion
  Co-authored-by: Nathan Brooks <nbbrooks@gmail.com>
  * readability changes and additional servo parameter check
  * updating to newest design
  * added warning message
  * added missing semicolon
  * made optional parameter nicer
  * Remove outdated warning
  Co-authored-by: AndyZe <andyz@utexas.edu>
  * Removing inaccurate comment
  Co-authored-by: AndyZe <andyz@utexas.edu>
  * making Andy's suggested changes, added some comments and defaults, moved code block next to relevant singularity code
  * removed part of comment that does not apply any more
  * Mention "deprecation" in the warning
  Co-authored-by: Henry Moore <henrygerardmoore@gmail.com>
  Co-authored-by: Henry Moore <44307180+henrygerardmoore@users.noreply.github.com>
  Co-authored-by: AndyZe <zelenak@picknik.ai>
  Co-authored-by: AndyZe <andyz@utexas.edu>
* Remove __has_include statements (`#1481 <https://github.com/ros-planning/moveit2/issues/1481>`_)
* Servo: check for and enable a realtime kernel (`#1464 <https://github.com/ros-planning/moveit2/issues/1464>`_)
  * Check for and enable a realtime kernel
  * Set thread priority to 40. Link against controller_mgr.
  * Do it from the right thread
* Contributors: AndyZe, Nathan Brooks, Robert Haschke, Sebastian Jahr, Vatan Aksoy Tezer

2.5.3 (2022-07-28)
------------------
* Use kinematics plugin instead of inverse Jacobian for servo IK (`#1434 <https://github.com/ros-planning/moveit2/issues/1434>`_)
* Contributors: Wyatt Rees

2.5.2 (2022-07-18)
------------------
* Merge remote-tracking branch 'origin/main' into feature/msa
* Removing more boost usage (`#1372 <https://github.com/ros-planning/moveit2/issues/1372>`_)
* Merge remote-tracking branch 'upstream/main' into feature/msa
* Removing some boost usage (`#1331 <https://github.com/ros-planning/moveit2/issues/1331>`_)
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Update Servo integration tests (`#1336 <https://github.com/ros-planning/moveit2/issues/1336>`_)
* Minor cleanup of Servo CMakeLists (`#1345 <https://github.com/ros-planning/moveit2/issues/1345>`_)
* Contributors: AndyZe, David V. Lu, Henry Moore, Jafar, Vatan Aksoy Tezer

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Enable cppcheck (`#1224 <https://github.com/ros-planning/moveit2/issues/1224>`_)
  Co-authored-by: jeoseo <jeongwooseo2012@gmail.com>
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
  Adaption of https://github.com/ros-planning/moveit/pull/3106
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* Delete an unused variable and a redundant log message (`#1179 <https://github.com/ros-planning/moveit2/issues/1179>`_)
* [Servo] Add override parameter to set constant velocity scaling in Servo (`#1169 <https://github.com/ros-planning/moveit2/issues/1169>`_)
* Rename panda controllers
* Enable rolling / jammy CI (again) (`#1134 <https://github.com/ros-planning/moveit2/issues/1134>`_)
  * Use ros2_control binaries
  * Use output screen instead of explicitly stating stderr
* Temporarily add galactic CI (`#1107 <https://github.com/ros-planning/moveit2/issues/1107>`_)
  * Add galactic CI
  * Comment out rolling
  * panda_ros_controllers -> panda_ros2_controllers
  * Ignore flake8 tests
* 1.1.9
* Compilation fixes for Jammy and bring back Rolling CI (`#1095 <https://github.com/ros-planning/moveit2/issues/1095>`_)
  * Use jammy dockers and clang-format-12
  * Fix unused depend, and move to python3-lxml
  * add ompl to repos, fix versions and ogre
  * Remove ogre keys
  * Fix boolean node operator
  * Stop building dockers on branch and fix servo null pointer
  * update pre-commit to clang-format-12 and pre-commit fixes
  * clang-format workaround and more pre-commit fixes
* Explicitly set is_primary_planning_scene_monitor in Servo example config (`#1060 <https://github.com/ros-planning/moveit2/issues/1060>`_)
* 1.1.8
* [hybrid planning] Add action abortion and test; improve the existing test (`#980 <https://github.com/ros-planning/moveit2/issues/980>`_)
  * Add action abortion and test; improve the existing test
  * Add controller run-dependency
  * Fix the clearing of robot trajectory when a collision would occur
  * Fix replanning if local planner is stuck
  * Lambda function everything
  * Thread safety for stop_hybrid_planning\_
  * Thread-safe state\_
  * Clang tidy
  * Update the planning scene properly
  * Update Servo test initial_positions.yaml
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
* Remove unused parameters. (`#1018 <https://github.com/ros-planning/moveit2/issues/1018>`_)
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
* Add moveit_configs_utils package to simplify loading paramters (`#591 <https://github.com/ros-planning/moveit2/issues/591>`_)
  Co-authored-by: AndyZe <zelenak@picknik.ai>
  Co-authored-by: Stephanie Eng <stephanie-eng@users.noreply.github.com>
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
* 1.1.7
* 1.1.6
* Servo: sync position limit enforcement with MoveIt2 (`#2898 <https://github.com/ros-planning/moveit2/issues/2898>`_)
  * fix enforce position bug
  * remove unnecessary variable
  * make clang tidy happy
  * Update my comment
  * implement same logic as in the moveit2! repo
  * fix copy-pase error
  Co-authored-by: Michael Wiznitzer <michael.wiznitzer@resquared.com>
  Co-authored-by: AndyZe <andyz@utexas.edu>
* Contributors: AndyZe, Cory Crean, Henning Kayser, Jafar, Jafar Abdi, Joseph Schornak, Marq Rasmussen, Michael Wiznitzer, Robert Haschke, Vatan Aksoy Tezer, jeoseo, v4hn

2.4.0 (2022-01-20)
------------------
* Remove 'using namespace' from header files. (`#994 <https://github.com/ros-planning/moveit2/issues/994>`_)
* Servo: re-order velocity limit check & minor cleanup (`#956 <https://github.com/ros-planning/moveit2/issues/956>`_)
* moveit_build_options()
  Declare common build options like CMAKE_CXX_STANDARD, CMAKE_BUILD_TYPE,
  and compiler options (namely warning flags) once.
  Each package depending on moveit_core can use these via moveit_build_options().
* Contributors: AndyZe, Cory Crean, Robert Haschke

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Servo: fix -Wunused-private-field (`#937 <https://github.com/ros-planning/moveit2/issues/937>`_)
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* Add descriptions and default values to servo parameters (`#799 <https://github.com/ros-planning/moveit2/issues/799>`_)
* Update README (`#812 <https://github.com/ros-planning/moveit2/issues/812>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* moveit_servo: Fix ACM for collision checking & PSM's scene monitor topic (`#673 <https://github.com/ros-planning/moveit2/issues/673>`_)
* Fix initialization of PSM publisher in servo (`#771 <https://github.com/ros-planning/moveit2/issues/771>`_)
* Move initialization of ServoNode into constructor (`#761 <https://github.com/ros-planning/moveit2/issues/761>`_)
* Fix missing test depend in servo (`#759 <https://github.com/ros-planning/moveit2/issues/759>`_)
* Find/replace deprecated spawner.py (`#737 <https://github.com/ros-planning/moveit2/issues/737>`_)
* Fix the servo executable name (`#746 <https://github.com/ros-planning/moveit2/issues/746>`_)
* Use rclcpp::SystemDefaultsQoS in Servo (`#721 <https://github.com/ros-planning/moveit2/issues/721>`_)
* Use multi-threaded component container, do not use intraprocess comms in Servo (`#723 <https://github.com/ros-planning/moveit2/issues/723>`_)
* Disable use_intra_process_comms in servo launch files (`#722 <https://github.com/ros-planning/moveit2/issues/722>`_)
* Servo: minor fixups (`#2759 <https://github.com/ros-planning/moveit/issues/2759>`_)
* Contributors: AndyZe, Dave Coleman, David V. Lu!!, Henning Kayser, Jafar Abdi, Robert Haschke, Stephanie Eng, Tyler Weaver, toru-kuga

2.3.0 (2021-10-08)
------------------
* Make TF buffer & listener in PSM private (`#654 <https://github.com/ros-planning/moveit2/issues/654>`_)
* Rename ServoServer to ServerNode (`#649 <https://github.com/ros-planning/moveit2/issues/649>`_)
* Fix std::placeholders namespace conflict (`#713 <https://github.com/ros-planning/moveit2/issues/713>`_)
* Publish singularity condition to ~/servo_server/condition (`#695 <https://github.com/ros-planning/moveit2/issues/695>`_)
* Skip publishing to Servo topics if input commands are stale (`#707 <https://github.com/ros-planning/moveit2/issues/707>`_)
* Delete duplicate entry in Servo launch file (`#684 <https://github.com/ros-planning/moveit2/issues/684>`_)
* Fix cmake warnings (`#690 <https://github.com/ros-planning/moveit2/issues/690>`_)
  * Fix -Wformat-security
  * Fix -Wunused-variable
  * Fix -Wunused-lambda-capture
  * Fix -Wdeprecated-declarations
  * Fix clang-tidy, readability-identifier-naming in moveit_kinematics
* Add standalone executable for Servo node, and example launch file (`#621 <https://github.com/ros-planning/moveit2/issues/621>`_)
* Validate return of getJointModelGroup in ServoCalcs (`#648 <https://github.com/ros-planning/moveit2/issues/648>`_)
* Migrate to joint_state_broadcaster (`#657 <https://github.com/ros-planning/moveit2/issues/657>`_)
* Add gripper and traj control packages as run dependencies (`#636 <https://github.com/ros-planning/moveit2/issues/636>`_)
* Fix warnings in Galactic and Rolling (`#598 <https://github.com/ros-planning/moveit2/issues/598>`_)
  * Use __has_includes preprocessor directive for deprecated headers
  * Fix parameter template types
  * Proper initialization of smart pointers, rclcpp::Duration
* Remove stray semicolon (`#613 <https://github.com/ros-planning/moveit2/issues/613>`_)
* Re-Enable Servo Tests (`#603 <https://github.com/ros-planning/moveit2/issues/603>`_)
* Fix missing include in servo example (`#604 <https://github.com/ros-planning/moveit2/issues/604>`_)
* Document the difference between Servo pause/unpause and start/stop (`#605 <https://github.com/ros-planning/moveit2/issues/605>`_)
* Wait for complete state duration fix (`#590 <https://github.com/ros-planning/moveit2/issues/590>`_)
* Delete "stop distance"-based collision checking (`#564 <https://github.com/ros-planning/moveit2/issues/564>`_)
* Fix loading joint_limits.yaml in demo and test launch files (`#544 <https://github.com/ros-planning/moveit2/issues/544>`_)
* Fixes for Windows (`#530 <https://github.com/ros-planning/moveit2/issues/530>`_)
* Refactor out velocity limit enforcement with test (`#540 <https://github.com/ros-planning/moveit2/issues/540>`_)
* Refactor moveit_servo::LowPassFilter to be assignable (`#572 <https://github.com/ros-planning/moveit2/issues/572>`_)
* Fix MoveIt Servo compilation on macOS (`#555 <https://github.com/ros-planning/moveit2/issues/555>`_)
* Fix segfault if servo collision checking is disabled (`#568 <https://github.com/ros-planning/moveit2/issues/568>`_)
* Remove gtest include from non-testing source (`#2747 <https://github.com/ros-planning/moveit2/issues/2747>`_)
* Fix an off-by-one error in servo_calcs.cpp (`#2740 <https://github.com/ros-planning/moveit2/issues/2740>`_)
* Contributors: AdamPettinger, Akash, AndyZe, Griswald Brooks, Henning Kayser, Jafar Abdi, Joseph Schornak, Michael Görner, Nathan Brooks, Nisala Kalupahana, Tyler Weaver, Vatan Aksoy Tezer, luisrayas3, Lior Lustgarten

2.2.1 (2021-07-12)
------------------
* moveit_servo: Add a parameter to halt only joints that violate position limits  (`#515 <https://github.com/ros-planning/moveit2/issues/515>`_)
  Add halt_all_joints_in_joint_mode & halt_all_joints_in_cartesian_mode parameters to decide whether to halt all joints or some of them in case of joint limit violation
* Contributors: Jafar Abdi

2.2.0 (2021-06-30)
------------------
* Allow a negative joint margin (`#501 <https://github.com/ros-planning/moveit2/issues/501>`_)
* Move servo doc and examples to moveit2_tutorials (`#486 <https://github.com/ros-planning/moveit2/issues/486>`_)
* Remove faulty gtest include (`#526 <https://github.com/ros-planning/moveit2/issues/526>`_)
* Fix segfault when publish_joint_velocities set to false and a joint is close to position limit (`#497 <https://github.com/ros-planning/moveit2/issues/497>`_)
* Enable Rolling and Galactic CI (`#494 <https://github.com/ros-planning/moveit2/issues/494>`_)
* [sync] MoveIt's master branch up-to https://github.com/ros-planning/moveit/commit/0d0a6a171b3fbea97a0c4f284e13433ba66a4ea4
  * Misspelled MoveIt (`#2692 <https://github.com/ros-planning/moveit/issues/2692>`_)
  * Avoid joint jump when SuddenHalt() is called in velocity mode (`#2594 <https://github.com/ros-planning/moveit/issues/2594>`_)
  * Halt Servo command on Pose Tracking stop (`#2501 <https://github.com/ros-planning/moveit/issues/2501>`_)
  * stop_requested\_ flag clearing fix (`#2537 <https://github.com/ros-planning/moveit/issues/2537>`_)
  * add missing include (`#2519 <https://github.com/ros-planning/moveit/issues/2519>`_)
  * Refactor velocity bounds enforcement (`#2471 <https://github.com/ros-planning/moveit/issues/2471>`_)
* Contributors: AdamPettinger, AndyZe, Henning Kayser, Jafar Abdi, JafarAbdi, Jere Liukkonen, Michael Görner, Nathan Brooks, Robert Haschke, Tyler Weaver, Vatan Aksoy Tezer, parunapu

2.1.4 (2021-05-31)
------------------
* Delete MoveIt fake_controller_manager (`#471 <https://github.com/ros-planning/moveit2/issues/471>`_)
* Contributors: AndyZe

2.1.3 (2021-05-22)
------------------
* Refactor Servo velocity bounds enforcement. Disable flaky unit tests. (`#428 <https://github.com/ros-planning/moveit2/issues/428>`_)
* Fix joint limit handling when velocities aren't included in robot state (`#451 <https://github.com/ros-planning/moveit2/issues/451>`_)
* Fix Servo logging frequency (`#457 <https://github.com/ros-planning/moveit2/issues/457>`_)
* Replace last ament_export_libraries macro calls with ament_export_targets (`#448 <https://github.com/ros-planning/moveit2/issues/448>`_)
* Contributors: AndyZe, Sebastian Jahr, Vatan Aksoy Tezer

2.1.2 (2021-04-20)
------------------
* Re-enable test_servo_pose_tracking integration test (`#423 <https://github.com/ros-planning/moveit2/issues/423>`_)
  Co-authored-by: AndyZe <zelenak@picknik.ai>
* Unify PickNik name in copyrights (`#419 <https://github.com/ros-planning/moveit2/issues/419>`_)
* Contributors: Tyler Weaver, Vatan Aksoy Tezer

2.1.1 (2021-04-12)
------------------
* Do not output positions at all if they are set to false (`#410 <https://github.com/ros-planning/moveit2/issues/410>`_)
* Update launch files to use ros2 control spawner (`#405 <https://github.com/ros-planning/moveit2/issues/405>`_)
* Include boost optional in pose_tracking (`#406 <https://github.com/ros-planning/moveit2/issues/406>`_)
* Use fake_components::GenericSystem from ros2_control (`#361 <https://github.com/ros-planning/moveit2/issues/361>`_)
* Fix EXPORT install in CMake (`#372 <https://github.com/ros-planning/moveit2/issues/372>`_)
* moveit servo: fix constructing duration from double & fix bug in insertRedundantPointsIntoTrajectory function (`#374 <https://github.com/ros-planning/moveit2/issues/374>`_)
* port pose tracking (`#320 <https://github.com/ros-planning/moveit2/issues/320>`_)
* Fix 'start_servo' service topic in demo
* Sync main branch with MoveIt 1 from previous head https://github.com/ros-planning/moveit/commit/0247ed0027ca9d7f1a7f066e62c80c9ce5dbbb5e up to https://github.com/ros-planning/moveit/commit/74b3e30db2e8683ac17b339cc124675ae52a5114
* Protect paused\_ flag, for thread safety (`#2494 <https://github.com/ros-planning/moveit2/issues/2494>`_)
* Do not break out of loop -- need to update low pass filters (`#2496 <https://github.com/ros-planning/moveit2/issues/2496>`_)
* [Servo] Fix initial angle error is always 0 (`#2464 <https://github.com/ros-planning/moveit2/issues/2464>`_)
* Add an important sleep in Servo pose tracking (`#2463 <https://github.com/ros-planning/moveit2/issues/2463>`_)
* Prevent moveit_servo transforms between fixed frames from causing timeout (`#2418 <https://github.com/ros-planning/moveit2/issues/2418>`_)
* [feature] Low latency mode (`#2401 <https://github.com/ros-planning/moveit2/issues/2401>`_)
* Move timer initialization down to fix potential race condition
* Contributors: Abishalini Sivaraman, AdamPettinger, AndyZe, Boston Cleek, Henning Kayser, Jafar Abdi, Nathan Brooks, Tyler Weaver

2.1.0 (2020-11-23)
------------------
* [maint] Wrap common cmake code in 'moveit_package()' macro (`#285 <https://github.com/ros-planning/moveit2/issues/285>`_)
  * New moveit_package() macro for compile flags, Windows support etc
  * Add package 'moveit_common' as build dependency for moveit_package()
  * Added -Wno-overloaded-virtual compiler flag for moveit_ros_planners_ompl
* [fix] Servo runtime issues (`#257 <https://github.com/ros-planning/moveit2/issues/257>`_, `#265 <https://github.com/ros-planning/moveit2/issues/265>`_, `#294 <https://github.com/ros-planning/moveit2/issues/294>`_)
* [ros2-migration] Port moveit_servo to ROS 2 (`#248 <https://github.com/ros-planning/moveit2/issues/248>`_)
  * Ports the source from MoveIt
  * Adds examples (C++ interface, composable node interface, teleoperation demo for gamepad)
  * Adds integration and unit tests
* Contributors: Adam Pettinger, Henning Kayser, Lior Lustgarten, Tyler Weaver

1.1.1 (2020-10-13)
------------------
* [feature] A library for servoing toward a moving pose (`#2203 <https://github.com/ros-planning/moveit/issues/2203>`_)
* [feature] Refactor velocity limit enforcement and add a unit test (`#2260 <https://github.com/ros-planning/moveit/issues/2260>`_)
* [fix] Servo thread interruption (`#2314 <https://github.com/ros-planning/moveit/issues/2314>`_)
* [fix] Servo heap-buffer-overflow bug (`#2307 <https://github.com/ros-planning/moveit/issues/2307>`_)
* [maint] Cleanup MSA includes (`#2351 <https://github.com/ros-planning/moveit/issues/2351>`_)
* Contributors: AndyZe, Robert Haschke, Tyler Weaver

1.1.0 (2020-09-04)
------------------
* [feature] Update last_sent_command\_ at ServoCalcs start (`#2249 <https://github.com/ros-planning/moveit/issues/2249>`_)
* [feature] Add a utility to print collision pairs (`#2275 <https://github.com/ros-planning/moveit/issues/2275>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [maint] add soname version to moveit_servo (`#2266 <https://github.com/ros-planning/moveit/issues/2266>`_)
* [maint] delete python integration tests (`#2186 <https://github.com/ros-planning/moveit/issues/2186>`_)
* Contributors: AdamPettinger, AndyZe, Robert Haschke, Ruofan Xu, Tyler Weaver, v4hn

1.0.6 (2020-08-19)
------------------
* [feature] A ROS service to reset the Servo status (`#2246 <https://github.com/ros-planning/moveit/issues/2246>`_)
* [feature] Check collisions during joint motions, too (`#2204 <https://github.com/ros-planning/moveit/issues/2204>`_)
* [fix]     Correctly set velocities to zero when stale (`#2255 <https://github.com/ros-planning/moveit/issues/2255>`_)
* [maint]   Remove unused yaml param (`#2232 <https://github.com/ros-planning/moveit/issues/2232>`_)
* [maint]   Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint]   Migrate to clang-format-10
* Contributors: AndyZe, Robert Haschke, Ruofan Xu, Michael Görner

1.0.5 (2020-07-08)
------------------
* [maint]   Minor moveit_servo header cleanup (`#2173 <https://github.com/ros-planning/moveit/issues/2173>`_)
* [maint]   Move and rename to moveit_ros/moveit_servo (`#2165 <https://github.com/ros-planning/moveit/issues/2165>`_)
* [maint]   Changes before porting to ROS2 (`#2151 <https://github.com/ros-planning/moveit/issues/2151>`_)
  * throttle warning logs
  * ROS1 Basic improvements and changes
  * Fixes to drift dimensions, singularity velocity scaling
  * tf name changes, const fixes, slight logic changes
  * Move ROS_LOG_THROTTLE_PERIOD to cpp files
  * Track staleness of joint and twist seperately
  * Ensure joint_trajectory output is always populated with something, even when no jog
  * Fix joint trajectory redundant points for gazebo pub
  * Fix crazy joint jog from bad Eigen init
  * Fix variable type in addJointIncrements()
  * Initialize last sent command in constructor
  * More explicit joint_jog_cmd\ and twist_stamped_cmd\ names
  * Add comment clarying transform calculation / use
* [fix]     Fix access past end of array bug (`#2155 <https://github.com/ros-planning/moveit/issues/2155>`_)
* [maint]   Remove duplicate line (`#2154 <https://github.com/ros-planning/moveit/issues/2154>`_)
* [maint]   pragma once in jog_arm.h (`#2152 <https://github.com/ros-planning/moveit/issues/2152>`_)
* [feature] Simplify communication between threads (`#2103 <https://github.com/ros-planning/moveit/issues/2103>`_)
  * get latest joint state c++ api
  * throttle warning logs
  * publish from jog calcs timer, removing redundant timer and internal messaging to main timer
  * outgoing message as pool allocated shared pointer for zero copy
  * replace jog_arm shared variables with ros pub/sub
  * use built in zero copy message passing instead of spsc_queues
  * use ros timers instead of threads in jog_arm
* [feature] Added throttle to jogarm accel limit warning (`#2141 <https://github.com/ros-planning/moveit/issues/2141>`_)
* [feature] Time-based collision avoidance (`#2100 <https://github.com/ros-planning/moveit/issues/2100>`_)
* [fix]     Fix crash on empty jog msgs (`#2094 <https://github.com/ros-planning/moveit/issues/2094>`_)
* [feature] Jog arm dimensions (`#1724 <https://github.com/ros-planning/moveit/issues/1724>`_)
* [maint]   Clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_)
* [feature] Keep updating joints, even while waiting for a valid command (`#2027 <https://github.com/ros-planning/moveit/issues/2027>`_)
* [fix]     Fix param logic bug for self- and scene-collision proximity thresholds (`#2022 <https://github.com/ros-planning/moveit/issues/2022>`_)
* [feature] Split collision proximity threshold (`#2008 <https://github.com/ros-planning/moveit/issues/2008>`_)
  * separate proximity threshold values for self-collisions and scene collisions
  * increase default value of scene collision proximity threshold
  * deprecate old parameters
* [fix]     Fix valid command flags (`#2013 <https://github.com/ros-planning/moveit/issues/2013>`_)
  * Rename the 'zero command flag' variables for readability
  * Reset flags when incoming commands timeout
  * Remove debug line, clang format
* [maint]   Use default move constructor + assignment operators for MoveItCpp. (`#2004 <https://github.com/ros-planning/moveit/issues/2004>`_)
* [fix]     Fix low-pass filter initialization (`#1982 <https://github.com/ros-planning/moveit/issues/1982>`_)
  * Pause/stop JogArm threads using shared atomic bool variables
  * Add pause/unpause flags for jog thread
  * Verify valid joints by filtering for active joint models only
  * Remove redundant joint state increments
  * Wait for initial jog commands in main loop
* [fix]     Remove duplicate collision check in JogArm (`#1986 <https://github.com/ros-planning/moveit/issues/1986>`_)
* [feature] Add a binary collision check (`#1978 <https://github.com/ros-planning/moveit/issues/1978>`_)
* [feature] Publish more detailed warnings (`#1915 <https://github.com/ros-planning/moveit/issues/1915>`_)
* [feature] Use wait_for_service() to fix flaky tests (`#1946 <https://github.com/ros-planning/moveit/issues/1946>`_)
* [maint]   Fix versioning (`#1948 <https://github.com/ros-planning/moveit/issues/1948>`_)
* [feature] SRDF velocity and acceleration limit enforcement (`#1863 <https://github.com/ros-planning/moveit/issues/1863>`_)
* [maint]   Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [fix]     JogArm C++ API fixes (`#1911 <https://github.com/ros-planning/moveit/issues/1911>`_)
* [feature] A ROS service to enable task redundancy (`#1855 <https://github.com/ros-planning/moveit/issues/1855>`_)
* [fix]     Fix segfault with uninitialized JogArm thread (`#1882 <https://github.com/ros-planning/moveit/issues/1882>`_)
* [feature] Add warnings to moveit_jog_arm low pass filter (`#1872 <https://github.com/ros-planning/moveit/issues/1872>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 for portability (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix]     Fix initial end effector transform jump (`#1871 <https://github.com/ros-planning/moveit/issues/1871>`_)
* [feature] Rework the halt msg functionality (`#1868 <https://github.com/ros-planning/moveit/issues/1868>`_)
* [fix]     Various small fixes (`#1859 <https://github.com/ros-planning/moveit/issues/1859>`_)
* [maint]   Improve formatting in comments
* [fix]     Prevent a crash at velocity limit (`#1837 <https://github.com/ros-planning/moveit/issues/1837>`_)
* [feature] Remove scale/joint parameter (`#1838 <https://github.com/ros-planning/moveit/issues/1838>`_)
* [feature] Pass planning scene monitor into cpp interface (`#1849 <https://github.com/ros-planning/moveit/issues/1849>`_)
* [maint]   Move attribution below license file, standardize with MoveIt (`#1847 <https://github.com/ros-planning/moveit/issues/1847>`_)
* [maint]   Reduce console output warnings (`#1845 <https://github.com/ros-planning/moveit/issues/1845>`_)
* [fix]     Fix command frame transform computation (`#1842 <https://github.com/ros-planning/moveit/issues/1842>`_)
* [maint]   Fix dependencies + catkin_lint issues
* [feature] Update link transforms before calling checkCollision on robot state in jog_arm (`#1825 <https://github.com/ros-planning/moveit/issues/1825>`_)
* [feature] Add atomic bool flags for terminating JogArm threads gracefully (`#1816 <https://github.com/ros-planning/moveit/issues/1816>`_)
* [feature] Get transforms from RobotState instead of TF (`#1803 <https://github.com/ros-planning/moveit/issues/1803>`_)
* [feature] Add a C++ API (`#1763 <https://github.com/ros-planning/moveit/issues/1763>`_)
* [maint]   Fix unused parameter warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint]   Update license formatting (`#1764 <https://github.com/ros-planning/moveit/issues/1764>`_)
* [maint]   Unify jog_arm package to be C++14 (`#1762 <https://github.com/ros-planning/moveit/issues/1762>`_)
* [fix]     Fix jog_arm segfault (`#1692 <https://github.com/ros-planning/moveit/issues/1692>`_)
* [fix]     Fix double mutex unlock (`#1672 <https://github.com/ros-planning/moveit/issues/1672>`_)
* [maint]   Rename jog_arm->moveit_jog_arm (`#1663 <https://github.com/ros-planning/moveit/issues/1663>`_)
* [feature] Do not wait for command msg to start spinning (`#1603 <https://github.com/ros-planning/moveit/issues/1603>`_)
* [maint]   Update jog_arm README with rviz config (`#1614 <https://github.com/ros-planning/moveit/issues/1614>`_)
* [maint]   Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint]   Separate moveit_experimental packages (`#1606 <https://github.com/ros-planning/moveit/issues/1606>`_)
* [feature] Use UR5 example (`#1605 <https://github.com/ros-planning/moveit/issues/1605>`_)
* [feature] Sudden stop for critical issues, filtered deceleration otherwise (`#1468 <https://github.com/ros-planning/moveit/issues/1468>`_)
* [feature] Change 2nd order Butterworth low pass filter to 1st order (`#1483 <https://github.com/ros-planning/moveit/issues/1483>`_)
* [maint]   Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* [feature] JogArm: Remove dependency on move_group node (`#1569 <https://github.com/ros-planning/moveit/issues/1569>`_)
* [fix]     Fix jog arm CI integration test (`#1466 <https://github.com/ros-planning/moveit/issues/1466>`_)
* [feature] A jogging PR for Melodic. (`#1360 <https://github.com/ros-planning/moveit/issues/1360>`_)
  * Allow for joints in the msg that are not part of the MoveGroup.
  * Switching to the Panda robot model for tests.
  * Blacklist the test as I can't get it to pass Travis (fine locally).
  * Throttling all warnings. Fix build warning re. unit vs int comparison.
  * Continue to publish commands even if stationary
  * Scale for 'unitless' commands is not tied to publish_period.
  * New function name for checkIfJointsWithinBounds()
  * Configure the number of msgs to publish when stationary.
  * Run jog_calcs at the same rate as the publishing thread.
  * Better comments in config file, add spacenav_node dependency
  * Add spacenav_node to CMakeLists.
* Contributors: AdamPettinger, AndyZe, Ayush Garg, Dale Koenig, Dave Coleman, Jonathan Binney, Paul Verhoeckx, Henning Kayser, Jafar Abdi, John Stechschulte, Mike Lautman, Robert Haschke, SansoneG, jschleicher, Tyler Weaver, rfeistenauer

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
