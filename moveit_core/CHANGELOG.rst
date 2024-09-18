^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.11.0 (2024-09-16)
-------------------
* Fix RobotState::getRigidlyConnectedParentLinkModel() (`#2985 <https://github.com/moveit/moveit2/issues/2985>`_)
* Implement realtime Ruckig jerk-limited smoothing (`#2956 <https://github.com/moveit/moveit2/issues/2956>`_)
* New implementation for computeCartesianPath() (`#2916 <https://github.com/moveit/moveit2/issues/2916>`_)
* Don't set reset observer callback & set CB after world\_ is initialized (`#2950 <https://github.com/moveit/moveit2/issues/2950>`_)
* Deduplicate joint trajectory points in Pilz Move Group Sequence capability (`#2943 <https://github.com/moveit/moveit2/issues/2943>`_)
* Optimize MOVE_SHAPE operations for FCL (`#3601 <https://github.com/moveit/moveit2/issues/3601>`_)
* Allow moving of all shapes of an object in one go (`#3599 <https://github.com/moveit/moveit2/issues/3599>`_)
* Silent "empty quaternion" warning from poseMsgToEigen() (`#3435 <https://github.com/moveit/moveit2/issues/3435>`_)
* Propagate "clear octomap" actions to monitoring planning scenes (`#3134 <https://github.com/moveit/moveit2/issues/3134>`_)
* Copy planning scene predicates in the copy constructor (`#2858 <https://github.com/moveit/moveit2/issues/2858>`_)
* PSM: Correctly handle full planning scene message (`#3610 <https://github.com/moveit/moveit2/issues/3610>`_) (`#2876 <https://github.com/moveit/moveit2/issues/2876>`_), fixes `#3538 <https://github.com/moveit/moveit2/issues/3538>`_/`#3609 <https://github.com/moveit/moveit2/issues/3609>`_
* Switch to system version of octomap (`#2881 <https://github.com/moveit/moveit2/issues/2881>`_)
* Contributors: AndyZe, Captain Yoshi, Chris Lalancette, Chris Schindlbeck, FSund, Gaël Écorchard, Robert Haschke, Sebastian Castro, Sebastian Jahr

2.10.0 (2024-06-13)
-------------------
* Enforce liboctomap-dev by using a cmake version range
* Add utility functions to get limits and trajectory message (`#2861 <https://github.com/moveit/moveit2/issues/2861>`_)
* Migrate ros-planning org to moveit (`#2847 <https://github.com/moveit/moveit2/issues/2847>`_)
  * Rename github.com/ros-planning -> github.com/moveit
  * Rename ros-planning.github.io -> moveit.github.io
  * Rename ros-planning organization in docker and CI workflow files
  - ghcr.io/ros-planning -> ghcr.io/moveit
  - github.repository == 'moveit/*''
* Use std::optional instead of nullptr checking (`#2454 <https://github.com/moveit/moveit2/issues/2454>`_)
* Enable mdof trajectory execution (`#2740 <https://github.com/moveit/moveit2/issues/2740>`_)
  * Add RobotTrajectory conversion from MDOF to joints
  * Convert MDOF trajectories to joint trajectories in planning interfaces
  * Treat mdof joint variables as common joints in
  TrajectoryExecutionManager
  * Convert multi-DOF trajectories to joints in TEM
  * Revert "Convert MDOF trajectories to joint trajectories in planning interfaces"
  This reverts commit 885ee2718594859555b73dc341311a859d31216e.
  * Handle multi-DOF variables in TEM's bound checking
  * Add parameter to optionally enable multi-dof conversion
  * Improve error message about unknown controllers
  * Fix name ordering in JointTrajectory conversion
  * Improve DEBUG output in TEM
  * Comment RobotTrajectory test
  * add acceleration to avoid out of bounds read
* Fix doc reference to non-existent function (`#2765 <https://github.com/moveit/moveit2/issues/2765>`_)
* (core) Remove unused python docs folder (`#2746 <https://github.com/moveit/moveit2/issues/2746>`_)
* Unify log names (`#2720 <https://github.com/moveit/moveit2/issues/2720>`_)
* (core) Install collision_detector_fcl_plugin (`#2699 <https://github.com/moveit/moveit2/issues/2699>`_)
  FCL version of acda563
* Simplify Isometry multiplication benchmarks (`#2628 <https://github.com/moveit/moveit2/issues/2628>`_)
  With the benchmark library, there is no need to specify an iteration count.
  Interestingly, 4x4 matrix multiplication is faster than affine*matrix
* CMake format and lint in pre-commit (`#2683 <https://github.com/moveit/moveit2/issues/2683>`_)
* Merge pull request `#2660 <https://github.com/moveit/moveit2/issues/2660>`_ from MarqRazz/pr-fix_model_with_collision
  Fix getLinkModelNamesWithCollisionGeometry to include the base link
* validate link has parent
* pre-commit
* Fix getLinkModelNamesWithCollisionGeometry to include the base link of the planning group
* Acceleration Limited Smoothing Plugin for Servo (`#2651 <https://github.com/moveit/moveit2/issues/2651>`_)
* Contributors: Henning Kayser, Marq Rasmussen, Matthijs van der Burgh, Paul Gesel, Robert Haschke, Sebastian Jahr, Shobuj Paul, Tyler Weaver, marqrazz

2.9.0 (2024-01-09)
------------------
* (core) Remove all references to python wrapper from the core pkg (`#2623 <https://github.com/ros-planning/moveit2/issues/2623>`_)
  * (core) remove commented python wrapper code
  * (core) remove dep on pybind11_vendor
* Add missing pluginlib dependency (`#2641 <https://github.com/ros-planning/moveit2/issues/2641>`_)
* make time_optimal_trajectory_generation harder to misuse (`#2624 <https://github.com/ros-planning/moveit2/issues/2624>`_)
  * make time_optimal_trajectory_generation harder to misuse
  * style fixes
  * more style fixes
  * more style fixes
  * address PR comments
* Add collision_detection dependency on pluginlib (`#2638 <https://github.com/ros-planning/moveit2/issues/2638>`_)
  It is included by src/collision_plugin_cache.cpp, so it should be set as a dependency.
* reset accelerations on setToDefaultValues (`#2618 <https://github.com/ros-planning/moveit2/issues/2618>`_)
* fix out-of-bounds memory access with zero-variable joints (`#2617 <https://github.com/ros-planning/moveit2/issues/2617>`_)
* Node logging for the rest of MoveIt (`#2599 <https://github.com/ros-planning/moveit2/issues/2599>`_)
* Sync Ruckig with MoveIt1 (`#2596 <https://github.com/ros-planning/moveit2/issues/2596>`_)
  * Debug Ruckig tests (MoveIt1 3300)
  * Add a test, termination condition bugfix (MoveIt1 3348)
  * Mitigate Ruckig overshoot (MoveIt1 3376)
  * Small variable fixup
* Add missing header (`#2592 <https://github.com/ros-planning/moveit2/issues/2592>`_)
* Depend on `rsl::rsl` as a non-Ament dependency (`#2578 <https://github.com/ros-planning/moveit2/issues/2578>`_)
* [Planning Pipeline Refactoring] `#2 <https://github.com/ros-planning/moveit2/issues/2>`_ Enable chaining planners (`#2457 <https://github.com/ros-planning/moveit2/issues/2457>`_)
  * Enable chaining multiple planners
* Pass more distance information out from FCL collision check (`#2572 <https://github.com/ros-planning/moveit2/issues/2572>`_)
  * Pass more distance information out from FCL collision check
  * Update moveit_core/collision_detection/include/moveit/collision_detection/collision_common.h
  Co-authored-by: Sebastian Jahr <sebastian.jahr@tuta.io>
  * Update moveit_core/collision_detection/include/moveit/collision_detection/collision_common.h
  ---------
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Node logging in moveit_core (`#2503 <https://github.com/ros-planning/moveit2/issues/2503>`_)
* Benchmark robot state (`#2546 <https://github.com/ros-planning/moveit2/issues/2546>`_)
  * simplify memory management in RobotState
  * further changes
  * avoid pointer arithmetic where possible
  * fix memory access issue on root joint with 0 variables
  * fix vector size
  * remove unused header
* Remember original color of objects in planning scene (`#2549 <https://github.com/ros-planning/moveit2/issues/2549>`_)
* Allow editing allowed collision matrix in python + fix get_entry function (`#2551 <https://github.com/ros-planning/moveit2/issues/2551>`_)
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* [Planning Pipeline Refactoring] `#1 <https://github.com/ros-planning/moveit2/issues/1>`_ Simplify Adapter - Planner chain (`#2429 <https://github.com/ros-planning/moveit2/issues/2429>`_)
* Fix angular distance calculation in floating joint model (`#2538 <https://github.com/ros-planning/moveit2/issues/2538>`_)
* Add a benchmark for RobotTrajectory creation and timing. (`#2530 <https://github.com/ros-planning/moveit2/issues/2530>`_)
* Consolidate RobotState benchmarks in single file (`#2528 <https://github.com/ros-planning/moveit2/issues/2528>`_)
  * Consolidate RobotState benchmarks in single file
  * some cosmetics
  * style fixes
  * additional comments
* add rsl depend to moveit_core (`#2532 <https://github.com/ros-planning/moveit2/issues/2532>`_)
  - This should fix `#2516 <https://github.com/ros-planning/moveit2/issues/2516>`_
  - Several moveit2 packages already depend on rsl
  - PR `#2482 <https://github.com/ros-planning/moveit2/issues/2482>`_ added a depend in moveit_core
  This is only broken when building all of moveit2 deps in one colcon workspace
  And not using rosdep because colcon uses the package.xml and rsl might not have been built
* Avoid calling static node's destructor. (`#2513 <https://github.com/ros-planning/moveit2/issues/2513>`_)
* Factor out path joint-space jump check (`#2506 <https://github.com/ros-planning/moveit2/issues/2506>`_)
* Use node logging in moveit_ros (`#2482 <https://github.com/ros-planning/moveit2/issues/2482>`_)
* Add new clang-tidy style rules (`#2177 <https://github.com/ros-planning/moveit2/issues/2177>`_)
* Use default initializers in collision_common.h (`#2475 <https://github.com/ros-planning/moveit2/issues/2475>`_)
* Node logger through singleton (warehouse) (`#2445 <https://github.com/ros-planning/moveit2/issues/2445>`_)
  Co-authored-by: Abishalini Sivaraman <abi.gpuram@gmail.com>
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Smoothing plugin API update and bug fix (`#2470 <https://github.com/ros-planning/moveit2/issues/2470>`_)
  * Use Eigen::vector in smoothing plugin
  * Fix dependencies
  * Make args to reset const
  * Make KinematicState use Eigen::Vector
  * Mark params as unused
  * Fix type issues
  * Variable optimization
  Co-authored-by: AndyZe <andyz@utexas.edu>
  * Link against Eigen, not tf2_eigen
  * Don't resize every time
  * Don't reset the smoother\_ every time
  * Initialize the kinematic state of the smoother
  * Cleanup
  ---------
  Co-authored-by: ibrahiminfinite <ibrahimjkd@gmail.com>
  Co-authored-by: V Mohammed Ibrahim <12377945+ibrahiminfinite@users.noreply.github.com>
* Do not pass and return simple types by const ref (`#2453 <https://github.com/ros-planning/moveit2/issues/2453>`_)
  Co-authored-by: Nils <nilsmailiseke@gmail.com>
* Fix typo in bullet function name (`#2472 <https://github.com/ros-planning/moveit2/issues/2472>`_)
* Update pre-commit and add to .codespell_words (`#2465 <https://github.com/ros-planning/moveit2/issues/2465>`_)
* Port `#3464 <https://github.com/ros-planning/moveit2/issues/3464>`_ from MoveIt1 (`#2456 <https://github.com/ros-planning/moveit2/issues/2456>`_)
  * Port unit test
  cherry-pick of https://github.com/ros-planning/moveit/pull/3464
  * Increment added_path_index in callAdapter
  Doesn't work because previous=0 for all recursively called functions.
  * Pass individual add_path_index vectors to callAdapter
  ---------
  Co-authored-by: Hugal31 <hla@lescompanions.com>
* [Python] Add RetimeTrajectory to RobotTrajectory (`#2411 <https://github.com/ros-planning/moveit2/issues/2411>`_)
  * [Python] Add RetimeTrajectory to RobotTrajectory
  * Split retime trajecotry in multiple functions
  Moved logic to trajectory_tools
  Added Docstrings
  * Removed retime function from python binding
  ---------
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Use find_package for fcl (`#2399 <https://github.com/ros-planning/moveit2/issues/2399>`_)
* Remove old deprecated functions (`#2384 <https://github.com/ros-planning/moveit2/issues/2384>`_)
* Make getJacobian simpler and faster (`#2389 <https://github.com/ros-planning/moveit2/issues/2389>`_)
  * Make getJacobian simpler and faster
  * readability and const-correctness
  * fix issue when joint group is not at URDF origin
  * Update moveit_core/robot_state/src/robot_state.cpp
* Add RobotState::getJacobian() tests (`#2375 <https://github.com/ros-planning/moveit2/issues/2375>`_)
* Compare MoveIt! Jacobian against KDL (`#2377 <https://github.com/ros-planning/moveit2/issues/2377>`_)
* Update clang-format-14 with QualifierAlignment (`#2362 <https://github.com/ros-planning/moveit2/issues/2362>`_)
  * Set qualifier order in .clang-format
  * Ran pre-commit to update according to new style guide
* Converts float to double (`#2343 <https://github.com/ros-planning/moveit2/issues/2343>`_)
  * Limiting the scope of variables `#874 <https://github.com/ros-planning/moveit2/issues/874>`_
  Limited the scope of variables in moveit_core/collision_detection
  * Update moveit_core/collision_detection/src/collision_octomap_filter.cpp
  Co-authored-by: AndyZe <andyz@utexas.edu>
  * Update moveit_core/collision_detection/src/collision_octomap_filter.cpp
  Co-authored-by: AndyZe <andyz@utexas.edu>
  * Update moveit_core/collision_detection/src/collision_octomap_filter.cpp
  Co-authored-by: AndyZe <andyz@utexas.edu>
  * convert float to double
  * change double to float
  * Feedback fixes
  * Introduced variables removed from previous merge commit
  * Updated GL_Renderer function definitions with double instead of float
  * Changed update() function arguments to float since it is a derived virtual function and needs to be overriden
  * Fixed all override errors in visualization
  * *Fixed override errors in perception
  *Changed reinterpret_cast to double* from float*
  * change variable types to fit function definition
  * Fixed clang-tidy warnings
  * Fixed scope of reusable variables
  ---------
  Co-authored-by: Salah Soliman <salahsoliman96@gmail.com>
  Co-authored-by: AndyZe <andyz@utexas.edu>
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Contributors: Alex Moriarty, AndyZe, Chris Lalancette, Chris Thrasher, Jens Vanhooydonck, Mario Prats, Marq Rasmussen, Matthijs van der Burgh, Nacho Mellado, Rayene Messaoud, Robert Haschke, Sebastian Castro, Sebastian Jahr, Shobuj Paul, Stephanie Eng, Tyler Weaver

2.8.0 (2023-09-10)
------------------
* Add a benchmark for 'getJacobian' (`#2326 <https://github.com/ros-planning/moveit2/issues/2326>`_)
* [TOTG] Exit loop when position can't change (`#2307 <https://github.com/ros-planning/moveit2/issues/2307>`_)
* Remove added path index from planner adapter function signature (`#2285 <https://github.com/ros-planning/moveit2/issues/2285>`_)
* Fix typo in error message (`#2286 <https://github.com/ros-planning/moveit2/issues/2286>`_)
* Fix comment formatting (`#2276 <https://github.com/ros-planning/moveit2/issues/2276>`_)
* Cleanup planning request adapter interface (`#2266 <https://github.com/ros-planning/moveit2/issues/2266>`_)
  * Use default arguments instead of additional functions
  * Use generate param lib for default plan request adapters
  * Small cleanup of ResolveConstraintFrames
  * Remove dublicate yaml file entry
  * Move list_planning_adapter_plugins into own directory
  * Apply suggestions from code review
  Co-authored-by: Sebastian Castro <4603398+sea-bass@users.noreply.github.com>
  * Fix copy& paste error
  * Update parameter descriptions
  Co-authored-by: Sebastian Castro <4603398+sea-bass@users.noreply.github.com>
  * Apply suggestions from code review
  Co-authored-by: Kyle Cesare <kcesare@gmail.com>
  * EMPTY_PATH_INDEX_VECTOR -> empty_path_index_vector
  * Update parameter yaml
  * Make param listener unique
  * Fix build error
  * Use gt_eq instead of deprecated lower_bounds
  ---------
  Co-authored-by: Sebastian Castro <4603398+sea-bass@users.noreply.github.com>
  Co-authored-by: Kyle Cesare <kcesare@gmail.com>
* fix for kinematic constraints parsing (`#2267 <https://github.com/ros-planning/moveit2/issues/2267>`_)
* Contributors: Jorge Nicho, Mario Prats, Nacho Mellado, Sebastian Jahr, Stephanie Eng

2.7.4 (2023-05-18)
------------------
* Add documentation and cleanups for PlanningRequestAdapter and PlanningRequestAdapterChain classes (`#2142 <https://github.com/ros-planning/moveit2/issues/2142>`_)
  * Cleanups
  * Add documentation and more cleanups
  * Revert size_t change
* Fix collision checking in VisibilityConstraint (`#1986 <https://github.com/ros-planning/moveit2/issues/1986>`_)
* Alphabetize, smart pointer not needed (`#2148 <https://github.com/ros-planning/moveit2/issues/2148>`_)
  * Alphabetize, smart pointer not needed
  * Readability
* Fix getting variable bounds in mimic joints for TOTG (`#2030 <https://github.com/ros-planning/moveit2/issues/2030>`_)
  * Fix getting variable bounds in mimic joints for TOTG
  * Formatting
  * Remove unnecessary code
  * Do not include mimic joints in timing calculations
  * Change joint variable bounds at mimic creation time
  * Braces take you places
  * Fix other single-line if-else without braces in file for clang_tidy
  * Remove mimic bounds modification
  * Variable renaming and a comment
  * Fix index naming
  ---------
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
  Co-authored-by: Jafar <cafer.abdi@gmail.com>
  Co-authored-by: AndyZe <andyz@utexas.edu>
* Contributors: AndyZe, Joseph Schornak, Sebastian Castro, Sebastian Jahr

2.7.3 (2023-04-24)
------------------

2.7.2 (2023-04-18)
------------------
* Add JointModel::satisfiesAccelerationBounds() (`#2092 <https://github.com/ros-planning/moveit2/issues/2092>`_)
  * Add JointModel::satisfiesAccelerationBounds()
  * Check Jerk bounds too
  * Check if bounds exist
  ---------
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* A ROS param for Servo filter coefficient (`#2091 <https://github.com/ros-planning/moveit2/issues/2091>`_)
  * Add generate_parameter_library as dependency
  * Generate and export parameter target
  * Update butterworth filter to use params
  * Move param listener declaration to header
  * Formatting
  * Remove unnecessary rclcpp include
  * Fix alphabetical order
  * Make param listener local
  * Fix target exporting in cmake
  * Add moveit\_ prefix to parameter library target
  * Remove obsolete comment
  * Member variable naming
  * Alphabetize
  ---------
  Co-authored-by: AndyZe <andyz@utexas.edu>
* Merge pull request `#1900 <https://github.com/ros-planning/moveit2/issues/1900>`_ from Abishalini/pr-sync-1245f15
  Sync with MoveIt1
* Readd comment and assign error code
* Merge https://github.com/ros-planning/moveit/commit/1245f151393fe09023efec3e1faead2d26737227
* Add test and debug issue where TOTG returns accels > limit (`#2084 <https://github.com/ros-planning/moveit2/issues/2084>`_)
* Move stateless PlanningScene helper functions out of the class (`#2025 <https://github.com/ros-planning/moveit2/issues/2025>`_)
* Document how collision checking includes descendent links (`#2058 <https://github.com/ros-planning/moveit2/issues/2058>`_)
* Optionally mitigate Ruckig overshoot (`#2051 <https://github.com/ros-planning/moveit2/issues/2051>`_)
  * Optionally mitigate Ruckig overshoot
  * Cleanup
* Delete the Ruckig "batches" option, deprecated by `#1990 <https://github.com/ros-planning/moveit2/issues/1990>`_ (`#2028 <https://github.com/ros-planning/moveit2/issues/2028>`_)
* Merge PR `#3197 <https://github.com/ros-planning/moveit2/issues/3197>`_: Improve computeCartesianPath()
* Gracefully handle gtest 1.8 (Melodic)
  gtest 1.8 doesn't provide SetUpTestSuite().
  Thus, we cannot share the RobotModel across tests.
* Add unit tests for computeCartesianPath()
* Add utils to simplify (approximate) equality checks for Eigen entities
* robot_model_test_utils: Add loadIKPluginForGroup()
* Simplify test_cartesian_interpolator.cpp
* Generalize computeCartesianPath() to consider a link_offset
  This allows performing a circular motion about a non-link origin.
* Cleanup CartesianInterpolator
  - Fixup doc comments
  - Add API providing the translation vector = direction * distance
  - Simplify implementation
* Contributors: Abishalini, Abishalini Sivaraman, AndyZe, Robert Haschke, V Mohammed Ibrahim

2.7.1 (2023-03-23)
------------------
* Ruckig-smoothing : reduce number of  duration extensions (`#1990 <https://github.com/ros-planning/moveit2/issues/1990>`_)
  * extend duration only for failed segment
  * update comment
  * Remove trajectory reset before extension
  * readability improvement
  * Remove call to RobotState update
  ---------
  Co-authored-by: ibrahiminfinite <ibrahimjkd@@gmail.com>
  Co-authored-by: AndyZe <andyz@utexas.edu>
* Fix mimic joints with TOTG (`#1989 <https://github.com/ros-planning/moveit2/issues/1989>`_)
* changed C style cast to C++ style cast for void type (`#2010 <https://github.com/ros-planning/moveit2/issues/2010>`_)
  (void) -> static_cast<void>
* Fix member naming (`#1949 <https://github.com/ros-planning/moveit2/issues/1949>`_)
  * Update clang-tidy rules for readability-identifier-naming
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Fix Ruckig termination condition (`#1963 <https://github.com/ros-planning/moveit2/issues/1963>`_)
* Fix ci-testing build issues (`#1998 <https://github.com/ros-planning/moveit2/issues/1998>`_)
* Fix invalid case style for private member in RobotTrajectory
* Fix unreachable child logger instance
* Document the Butterworth filter better (`#1961 <https://github.com/ros-planning/moveit2/issues/1961>`_)
* Merge pull request `#1546 <https://github.com/ros-planning/moveit2/issues/1546>`_ from peterdavidfagan/moveit_py
  Python Bindings - moveit_py
* remove old python bindings
* remove underscore from public member in MotionPlanResponse (`#1939 <https://github.com/ros-planning/moveit2/issues/1939>`_)
  * remove underscore from private members
  * fix more uses of the suffix notation
* Contributors: AlexWebb, AndyZe, Henning Kayser, Jafar, Robert Haschke, Sebastian Castro, Shobuj Paul, V Mohammed Ibrahim, peterdavidfagan

2.7.0 (2023-01-29)
------------------
* Merge PR `#1712 <https://github.com/ros-planning/moveit2/issues/1712>`_: fix clang compiler warnings + stricter CI
* Don't use ament_export_targets from package sub folder (`#1889 <https://github.com/ros-planning/moveit2/issues/1889>`_)
* kinematic_constraints: update header frames (`#1890 <https://github.com/ros-planning/moveit2/issues/1890>`_)
* Install collision_detector_bullet_plugin from moveit_core
* Sort exports from moveit_core
* Clean up kinematic_constraints/utils, add update functions (`#1875 <https://github.com/ros-planning/moveit2/issues/1875>`_)
* Merge https://github.com/ros-planning/moveit/commit/9225971216885490e933ece25390c63ca14f8a58
* converted characters from string format to character format (`#1881 <https://github.com/ros-planning/moveit2/issues/1881>`_)
* Switch to clang-format-14 (`#1877 <https://github.com/ros-planning/moveit2/issues/1877>`_)
  * Switch to clang-format-14
  * Fix clang-format-14
* Add velocity and acceleration scaling when using custom limits in Time Parameterization (`#1832 <https://github.com/ros-planning/moveit2/issues/1832>`_)
  * add velocity and accelerations scaling when using custom limits for time parameterization
  * add scaling when passing in vecotor of joint-limits
  * add function descriptions
  * add verifyScalingFactor helper function
  * make map const
  * address feedback
  * add comment
  Co-authored-by: Michael Wiznitzer <michael.wiznitzer@resquared.com>
* Add default constructors
  ... as they are not implicitly declared anymore
* Add default copy/move constructors/assignment operators
  As a user-declared destructor deletes any implicitly-defined move constructor/assignment operator,
  we need to declared them manually. This in turn requires to declare the copy constructor/assignment as well.
* Fix GoogleTestVerification.UninstantiatedTypeParameterizedTestSuite
* Modernize gtest: TYPED_TEST_CASE -> TYPED_TEST_SUITE
* Fix warning: expression with side effects will be evaluated
* Fix warning: definition of implicit copy assignment operator is deprecated
* Cleanup msg includes: Use C++ instead of C header (`#1844 <https://github.com/ros-planning/moveit2/issues/1844>`_)
* Fix trajectory unwind bug (`#1772 <https://github.com/ros-planning/moveit2/issues/1772>`_)
  * ensure trajectory starting point's position is enforced
  * fix angle jump bug
  * handle bounds enforcement edge case
  * clang tidy
  * Minor renaming, better comment, use .at() over []
  * First shot at a unit test
  * fix other unwind bugs
  * test should succeed now
  * unwind test needs a model with a continuous joint
  * clang tidy
  * add test for unwinding from wound up robot state
  * clang tidy
  * tweak test for special case to show that it will fail without these changes
  Co-authored-by: Michael Wiznitzer <michael.wiznitzer@resquared.com>
  Co-authored-by: AndyZe <zelenak@picknik.ai>
* Require velocity and acceleration limits in TOTG (`#1794 <https://github.com/ros-planning/moveit2/issues/1794>`_)
  * Require vel/accel limits for TOTG
  * Comment improvements
  Co-authored-by: Sebastian Jahr <sebastian.jahr@tuta.io>
  Co-authored-by: Sebastian Jahr <sebastian.jahr@tuta.io>
* Use adjustable waypoint batch sizes for Ruckig (`#1719 <https://github.com/ros-planning/moveit2/issues/1719>`_)
  * Use adjustable waypoint batch sizes for Ruckig
  * Use std::optional for return value
  * Cleanup
  * Add comment about parameterizing
  * Fix potential segfault
  * Batch size argument
  * Use append()
  * Revert "Use append()"
  This reverts commit 96b86a6c783b05ba57e5a6a20bf901cd92ab74d7.
* Fix moveit_core dependency on tf2_kdl (`#1817 <https://github.com/ros-planning/moveit2/issues/1817>`_)
  This is a proper dependency, and not only a test dependency. It is still
  needed when building moveit_core with -DBUILD_TESTING=OFF.
* Bug fix: RobotTrajectory append() (`#1813 <https://github.com/ros-planning/moveit2/issues/1813>`_)
  * Add a test for append()
  * Don't add to the timestep, rather overwrite it
* Print a warning from TOTG if the robot model mixes revolute/prismatic joints (`#1799 <https://github.com/ros-planning/moveit2/issues/1799>`_)
* Tiny optimizations in enforcePositionBounds() for RevoluteJointModel (`#1803 <https://github.com/ros-planning/moveit2/issues/1803>`_)
* Better TOTG comments (`#1779 <https://github.com/ros-planning/moveit2/issues/1779>`_)
  * Increase understanding of TOTG path_tolerance\_
  Tiny readability optimization - makes it a little easier for people to figure out what `path_tolerance\_` does
  * Update the units of path_tolerance\_
  * Comment all 3 versions of computeTimeStamps
  * Add \param for num_waypoints
  * More clarity on units
  Co-authored-by: AndyZe <zelenak@picknik.ai>
  Co-authored-by: Nathan Brooks <nathan.brooks@picknik.ai>
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Remove unnecessary CMake variables and lists (`#1790 <https://github.com/ros-planning/moveit2/issues/1790>`_)
* Stopping calling MoveIt an alpha-stage project (`#1789 <https://github.com/ros-planning/moveit2/issues/1789>`_)
* Ensure all headers get installed within moveit_core directory (`#1786 <https://github.com/ros-planning/moveit2/issues/1786>`_)
* Set the resample_dt\_ member of TOTG back to const (`#1776 <https://github.com/ros-planning/moveit2/issues/1776>`_)
  * Set the resample_dt\_ member of TOTG back to const
  * Remove unused TOTG instance in test
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  * Add "totg" to function name
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Remove Iterative Spline and Iterative Parabola time-param algorithms (v2) (`#1780 <https://github.com/ros-planning/moveit2/issues/1780>`_)
  * Iterative parabolic parameterization fails for nonzero initial/final conditions
  * Iterative spline parameterization fails, too
  * Delete Iterative Spline & Iterative Parabola algorithms
* Use `target_include_directories` (`#1785 <https://github.com/ros-planning/moveit2/issues/1785>`_)
* Minimize use of `this->` (`#1784 <https://github.com/ros-planning/moveit2/issues/1784>`_)
  It's often unnecessary. MoveIt already avoids this in most cases
  so this PR better cements that existing pattern.
* Enable `-Wold-style-cast` (`#1770 <https://github.com/ros-planning/moveit2/issues/1770>`_)
* Add a version of TOTG computeTimeStamps() for a fixed num waypoints (`#1771 <https://github.com/ros-planning/moveit2/issues/1771>`_)
  * Add a version of computeTimeStamps() to yield a fixed num. waypoints
  * Add unit test
  * Prevent an ambiguous function signature
  * Remove debugging stuff
  * Can't have fewer than 2 waypoints
  * Warning about sparse waypoint spacing
  * Doxygen comments
  * Clarify about changing the shape of the path
  * Better comment
  Co-authored-by: Sebastian Jahr <sebastian.jahr@tuta.io>
  Co-authored-by: Sebastian Jahr <sebastian.jahr@tuta.io>
* Add `-Wunused-function` (`#1754 <https://github.com/ros-planning/moveit2/issues/1754>`_)
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
* Cleanup lookup of planning pipelines in MoveItCpp (`#1710 <https://github.com/ros-planning/moveit2/issues/1710>`_)
  * Revert "Add planner configurations to CHOMP and PILZ (`#1522 <https://github.com/ros-planning/moveit2/issues/1522>`_)"
  * Cleanup lookup of planning pipelines
  Remove MoveItCpp::getPlanningPipelineNames(), which was obviously intended initially to provide a planning-group-based filter for all available planning pipelines: A pipeline was discarded for a group, if there were no `planner_configs` defined for that group on the parameter server.
  As pointed out in `#1522 <https://github.com/ros-planning/moveit2/issues/1522>`_, only OMPL actually explicitly declares planner_configs on the parameter server.
  To enable all other pipelines as well (and thus circumventing the original filter mechanism), `#1522 <https://github.com/ros-planning/moveit2/issues/1522>`_ introduced empty dummy planner_configs for all other planners as well (CHOMP + Pilz).
  This, obviously, renders the whole filter mechanism useless. Thus, here we just remove the function getPlanningPipelineNames() and the corresponding member groups_pipelines_map\_.
* Small optimization in constructGoalConstraints() (`#1707 <https://github.com/ros-planning/moveit2/issues/1707>`_)
  * Small optimization in constructGoalConstraints()
  * Quat defaults to unity
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Fix clang-tidy issues (`#1706 <https://github.com/ros-planning/moveit2/issues/1706>`_)
  * Blindly apply automatic clang-tidy fixes
  * Exemplarily cleanup a few automatic clang-tidy fixes
  * Clang-tidy fixups
  * Missed const-ref fixups
  * Fix unsupported non-const -> const
  * More fixes
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Generate version.h with git branch and commit hash (`#2793 <https://github.com/ros-planning/moveit2/issues/2793>`_)
  * Generate version.h on every build and include git hash and branch/tag name
  * Don't generate "alpha" postfix on buildfarm
  * Show git version via moveit_version
  * Change version postfix: alpha -> devel
  Co-authored-by: Robert Haschke <rhaschke@techfak.uni-bielefeld.de>
* Contributors: Abhijeet Das Gupta, Abishalini, AndyZe, Captain Yoshi, Chris Thrasher, Christian Henkel, Cory Crean, Henning Kayser, Michael Wiznitzer, Nathan Brooks, Robert Haschke, Sameer Gupta, Scott K Logan, Tyler Weaver

2.6.0 (2022-11-10)
------------------
* Short-circuit planning adapters (`#1694 <https://github.com/ros-planning/moveit2/issues/1694>`_)
  * Revert "Planning request adapters: short-circuit if failure, return code rather than bool (`#1605 <https://github.com/ros-planning/moveit2/issues/1605>`_)"
  This reverts commit 66a64b4a72b6ddef1af2329f20ed8162554d5bcb.
  * Add debug message in call stack of planning_request_adapters
  * Short-circuit planning request adapters
  * Replace if-elseif cascade with switch
  * Cleanup translation of MoveItErrorCode to string
  - Move default code to moveit_core/utils
  - Override defaults in existing getActionResultString()
  - Provide translations for all error codes defined in moveit_msgs
  * Fix comment according to review
  * Add braces
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  * Add braces
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  Co-authored-by: AndyZe <andyz@utexas.edu>
* Parallel planning pipelines (`#1420 <https://github.com/ros-planning/moveit2/issues/1420>`_)
  * Add setTrajectoryConstraints() to PlanningComponent
  * Add planning time to PlanningComponent::PlanSolution
  * Replace PlanSolution with MotionPlanResponse
  * Address review
  * Add MultiPipelinePlanRequestParameters
  Add plan(const MultiPipelinePlanRequestParameters& parameters)
  Add mutex to avoid segfaults
  Add optional stop_criterion_callback and solution_selection_callback
  Remove stop_criterion_callback
  Make default solution_selection_callback = nullptr
  Remove parameter handling copy&paste code in favor of a template
  Add TODO to refactor pushBack() method into insert()
  Fix selection criteria and add RCLCPP_INFO output
  Changes due to rebase and formatting
  Fix race condition and segfault when no solution is found
  Satisfy clang tidy
  Remove mutex and thread safety TODOs
  Add stopping functionality to parallel planning
  Remove unnecessary TODOs
  * Fix unused plan solution with failure
  * Add sanity check for number of parallel planning problems
  * Check stopping criterion when new solution is generated + make thread safe
  * Add terminatePlanningPipeline() to MoveItCpp interface
  * Format!
  * Bug fixes
  * Move getShortestSolution callback into own function
  * No east const
  * Remove PlanSolutions and make planner_id accessible
  * Make solution executable
  * Rename update_last_solution to store_solution
  * Alphabetize includes and include plan_solutions.hpp instead of .h
  * Address review
  * Add missing header
  * Apply suggestions from code review
  Co-authored-by: AndyZe <andyz@utexas.edu>
  Co-authored-by: AndyZe <andyz@utexas.edu>
* Deprecate lookupParam function (`#1681 <https://github.com/ros-planning/moveit2/issues/1681>`_)
* Add new error types (moveit_msgs `#146 <https://github.com/ros-planning/moveit2/issues/146>`_) (`#1683 <https://github.com/ros-planning/moveit2/issues/1683>`_)
  * Add new error types (moveit_msgs `#146 <https://github.com/ros-planning/moveit2/issues/146>`_)
  * Add default case
  * Small change to the default case
  Co-authored-by: Tyler Weaver <maybe@tylerjw.dev>
  Co-authored-by: Tyler Weaver <maybe@tylerjw.dev>
* Planning request adapters: short-circuit if failure, return code rather than bool (`#1605 <https://github.com/ros-planning/moveit2/issues/1605>`_)
  * Return code rather than bool
  * Remove all debug prints
  * Small fixup
  * Minor cleanup of comment and error handling
  * void return from PlannerFn
  * Control reaches end of non-void function
  * Use a MoveItErrorCode cast
  * More efficient callAdapter()
  * More MoveItErrorCode
  * CI fixup attempt
* Improve Cartesian interpolation (`#1547 <https://github.com/ros-planning/moveit2/issues/1547>`_)
  * Generalize computeCartesianPath() to consider a link_offset
  which allows performing a circular motion about a non-link origin.
  * Augment reference to argument global_reference_frame
  Co-authored-by: AndyZe <andyz@utexas.edu>
* Remove unused clock from RobotTrajectory (`#1639 <https://github.com/ros-planning/moveit2/issues/1639>`_)
* Added brace intialization in moveit_core/collision_detection_fcl & moveit_core/collision_detection_field (`#1622 <https://github.com/ros-planning/moveit2/issues/1622>`_)
* added brace intialization (`#1615 <https://github.com/ros-planning/moveit2/issues/1615>`_)
* Merge PR `#1553 <https://github.com/ros-planning/moveit2/issues/1553>`_: Improve cmake files
* Use standard exported targets: export\_${PROJECT_NAME} -> ${PROJECT_NAME}Targets
* moveit_core/collision_detection: fix include order
  moveit_planning_scene's include directories have to be appended
  to the include directories found by ament_target_dependencies().
* Add missing srdfdom dependency
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* size_t bijection index type (`#1544 <https://github.com/ros-planning/moveit2/issues/1544>`_)
* Free functions for calculating properties of trajectories (`#1503 <https://github.com/ros-planning/moveit2/issues/1503>`_)
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Const ptr to jmg arg for cost function (`#1537 <https://github.com/ros-planning/moveit2/issues/1537>`_)
* Add planner configurations to CHOMP and PILZ (`#1522 <https://github.com/ros-planning/moveit2/issues/1522>`_)
* Add error_code_to_string function (`#1523 <https://github.com/ros-planning/moveit2/issues/1523>`_)
* Use pragma once as header include guard (`#1525 <https://github.com/ros-planning/moveit2/issues/1525>`_)
* Unified code comment style (`#1053 <https://github.com/ros-planning/moveit2/issues/1053>`_)
  * Changes the comment style from /**/ to //
  Co-authored-by: JafarAbdi <cafer.abdi@gmail.com>
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Remove sensor manager (`#1172 <https://github.com/ros-planning/moveit2/issues/1172>`_)
* Fixed fabs() use in quaternion interpolation (`#1479 <https://github.com/ros-planning/moveit2/issues/1479>`_)
  * Interpolate using Eigen::Quaternion::slerp() to (hopefully) save us further headaches and take advantage of Eigen probably having a better implementation than us.
  * Created a test case that fails for the old version, but passes for the new.
  Co-authored-by: AndyZe <zelenak@picknik.ai>
* Fixes for using generate_state_database (`#1412 <https://github.com/ros-planning/moveit2/issues/1412>`_)
* fix path to constraints parameters
* Remove __has_include statements (`#1481 <https://github.com/ros-planning/moveit2/issues/1481>`_)
* Merge https://github.com/ros-planning/moveit/commit/a63580edd05b01d9480c333645036e5b2b222da9
* Remove ConstraintSampler::project() (`#3170 <https://github.com/ros-planning/moveit2/issues/3170>`_)
  * Remove unused ompl_interface::ValidConstrainedSampler
  Last usage was removed in f2f6097ab7e272568d6ab258a53be3c7ca67cf3b.
  * Remove ConstraintSampler::project()
  sample() and project() only differ in whether they perform random sampling
  of the reference joint pose or not. Both of them are sampling.
  This was highly confusing, as from project() one wouldn't expect sampling.
* Add and fix dual arm test (`#3119 <https://github.com/ros-planning/moveit2/issues/3119>`_)
  * Add dual arm test
  * Fix and simplify UnionConstraintSampler: update joint transforms
  Co-authored-by: Cristian Beltran <cristianbehe@gmail.com>
  Co-authored-by: Robert Haschke <rhaschke@techfak.uni-bielefeld.de>
* Contributors: Abhijeet Das Gupta, Abishalini Sivaraman, Alaa, AndyZe, Henning Kayser, J. Javan, Michael Marron, Robert Haschke, Sebastian Jahr, Tyler Weaver, Vatan Aksoy Tezer, abishalini, cambel, werner291

2.5.3 (2022-07-28)
------------------
* Constraint samplers seed (`#1411 <https://github.com/ros-planning/moveit2/issues/1411>`_)
* Contributors: Henry Moore

2.5.2 (2022-07-18)
------------------
* Added const to moveit_core/collision_detection per issue 879 (`#1416 <https://github.com/ros-planning/moveit2/issues/1416>`_)
* Add generic cost function to KinematicsBase, CartesianInterpolator, and RobotState (`#1386 <https://github.com/ros-planning/moveit2/issues/1386>`_)
* Merge pull request `#1402 <https://github.com/ros-planning/moveit2/issues/1402>`_ from Abishalini/pr-sync-a436a97
  Sync with MoveIt
* Merge https://github.com/ros-planning/moveit/commit/a436a9771f7445c162cc3090c4c7c57bdb5bf194
* Merge https://github.com/ros-planning/moveit/commit/c88f6fb64e9057a4b9a8f6fafc01060e8c48a216
* Merge remote-tracking branch 'origin/main' into feature/msa
* Removing more boost usage (`#1372 <https://github.com/ros-planning/moveit2/issues/1372>`_)
* Fix PlanarJointModel::satisfiesPositionBounds (`#1353 <https://github.com/ros-planning/moveit2/issues/1353>`_)
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Type safety for CartesianInterpolator (`#1325 <https://github.com/ros-planning/moveit2/issues/1325>`_)
* Merge remote-tracking branch 'upstream/main' into feature/msa
* Removing some boost usage (`#1331 <https://github.com/ros-planning/moveit2/issues/1331>`_)
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Fix PlanarJointModel::satisfiesPositionBounds (`#3160 <https://github.com/ros-planning/moveit/issues/3160>`_)
* Port OMPL orientation constraints to MoveIt2 (`#1273 <https://github.com/ros-planning/moveit2/issues/1273>`_)
  Co-authored-by: JeroenDM <jeroendemaeyer@live.be>
  Co-authored-by: AndyZe <andyz@utexas.edu>
* Switch to hpp headers of pluginlib
* Adds another test case to `#3124 <https://github.com/ros-planning/moveit/issues/3124>`_ and adds some further minor improvements to the original PR (`#3142 <https://github.com/ros-planning/moveit/issues/3142>`_)
* Fix bug in applying planning scene diffs that have attached collision objects (`#3124 <https://github.com/ros-planning/moveit/issues/3124>`_)
  Co-authored-by: AndyZe <andyz@utexas.edu>
* Fix flaky constraint sampler test (`#3135 <https://github.com/ros-planning/moveit/issues/3135>`_)
* Constraint samplers with seed (`#3112 <https://github.com/ros-planning/moveit/issues/3112>`_)
  Co-authored-by: Robert Haschke <rhaschke@techfak.uni-bielefeld.de>
* Fix clang-tidy warning (`#3129 <https://github.com/ros-planning/moveit/issues/3129>`_)
* Merge pull request `#3106 <https://github.com/ros-planning/moveit/issues/3106>`_ from v4hn/pr-master-bind-them-all / banish bind()
* Fix clang-tidy
* using namespace collision_detection
* banish bind()
* various: prefer objects and references over pointers
* Migrate PRA internals to lambdas
* drop unused arguments not needed for lambda binding
* simplify distance field method binding
* Fix null pointer access to CollisionEnvObject in PlanningScene (`#3104 <https://github.com/ros-planning/moveit2/issues/3104>`_)
* Contributors: Abishalini, Bilal Gill, David V. Lu, Henry Moore, Jafar, Jochen Sprickerhof, Michael Görner, Robert Haschke, Rufus Wong, Stephanie Eng, Tahsincan Köse, Tyler Weaver, Vatan Aksoy Tezer, Wyatt Rees, v4hn

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Fix a bug when checking a pose is empty and TOTG corner case (`#1274 <https://github.com/ros-planning/moveit2/issues/1274>`_)
  * Fix having empty object pose would use the shape pose as the object pose
  * TOTG: Fix parameterizing a trajectory would produce a different last waypoint than the input last waypoint
* Add missing dependencies to cmake (`#1258 <https://github.com/ros-planning/moveit2/issues/1258>`_)
* Fix bug in applying planning scene diffs that have attached collision objects (`#3124 <https://github.com/ros-planning/moveit2/issues/3124>`_) (`#1251 <https://github.com/ros-planning/moveit2/issues/1251>`_)
* Merge https://github.com/ros-planning/moveit/commit/72d919299796bffc21f5eb752d66177841dc3442
* Allow custom velocity/accel/jerk limits for Ruckig smoothing (`#1221 <https://github.com/ros-planning/moveit2/issues/1221>`_)
* Allow custom velocity/acceleration limits for TOTG time-parameterization algorithm (`#1195 <https://github.com/ros-planning/moveit2/issues/1195>`_)
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Remove unused includes for boost::bind (`#1220 <https://github.com/ros-planning/moveit2/issues/1220>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
* Fix clang-tidy warning (`#1208 <https://github.com/ros-planning/moveit2/issues/1208>`_)
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* various: prefer object and references over pointers
  source: https://github.com/ros-planning/moveit/pull/3106/commits/1a8e5715e3142a92977ac585031b9dc1871f8718; this commit contains minor changes when compared to the source commit which it is based on, these changes are limited to ensuring compatibility with ROS2.
* migrate PRA internals to lambdas
  source: https://github.com/ros-planning/moveit/pull/3106/commits/6436597d5113a02dcfc976c85a2710fe7cd4c69e; in addition to the original commit I updated logging to support ros2 logging standards.
* drop unused arguments not needed for lambda binding
  source: https://github.com/ros-planning/moveit/pull/3106/commits/6805b7edc248a1e4557977f45722997bbbef5b22 ; I have also had to update how moveit_msgs is referenced (movit_msgs:: -> moveit_msgs::msg:: ) and I  added the changes to this commit that correspond to tests for the constraint samplers package.
* simplify distance field method binding
  source: https://github.com/ros-planning/moveit/pull/3106/commits/0322d63242d9990a9f93debd72085ede94efe0e9
* Use orocos_kdl_vendor package (`#1207 <https://github.com/ros-planning/moveit2/issues/1207>`_)
* Clamp inputs to Ruckig. Use current waypoint as input for next iteration (`#1202 <https://github.com/ros-planning/moveit2/issues/1202>`_)
  * Clamp inputs to Ruckig. Use the current waypoint as input for next iteration.
  * Fix the usage of std::clamp()
* Add a warning for TOTG if vel/accel limits aren't specified. (`#1186 <https://github.com/ros-planning/moveit2/issues/1186>`_)
* RCLCPP Upgrade Bugfixes (`#1181 <https://github.com/ros-planning/moveit2/issues/1181>`_)
* Ruckig smoothing cleanup (`#1111 <https://github.com/ros-planning/moveit2/issues/1111>`_)
* Replace num_dof and idx variables with JointGroup API (`#1152 <https://github.com/ros-planning/moveit2/issues/1152>`_)
* Merge https://github.com/ros-planning/moveit/commit/424a5b7b8b774424f78346d1e98bf1c9a33f0e78
* Remove new operators (`#1135 <https://github.com/ros-planning/moveit2/issues/1135>`_)
  replace new operator with make_shared
* ACM: Consider default entries when packing a ROS message (`#3096 <https://github.com/ros-planning/moveit2/issues/3096>`_)
  Previously, getAllEntryNames() just returned names occurring in the collision pair list.
  Now, also consider names in `default_entries\_`.
* Merge https://github.com/ros-planning/moveit/commit/a25515b73d682df03ed3eccd839110c296aa79fc
* Off by one in getAverageSegmentDuration (`#1079 <https://github.com/ros-planning/moveit2/issues/1079>`_)
* Fix missing boost::ref -> std::ref
* Merge https://github.com/ros-planning/moveit/commit/ab42a1d7017b27eb6c353fb29331b2da08ab0039
* Add special case for sphere bodies in sphere decomposition (`#3056 <https://github.com/ros-planning/moveit2/issues/3056>`_)
* Add Ptr definitions for TimeParameterization classes (`#3078 <https://github.com/ros-planning/moveit2/issues/3078>`_)
  Follow up on `#3021 <https://github.com/ros-planning/moveit2/issues/3021>`_.
* Fix Python versioned dependency (`#3063 <https://github.com/ros-planning/moveit2/issues/3063>`_)
* Merge https://github.com/ros-planning/moveit/commit/25a63b920adf46f0a747aad92ada70d8afedb3ec
* Merge https://github.com/ros-planning/moveit/commit/0d7462f140e03b4c319fa8cce04a47fe3f650c60
* Avoid downgrading default C++ standard (`#3043 <https://github.com/ros-planning/moveit2/issues/3043>`_)
* Delete profiler (`#998 <https://github.com/ros-planning/moveit2/issues/998>`_)
* Initalize RobotState in Ruckig test (`#1032 <https://github.com/ros-planning/moveit2/issues/1032>`_)
* Remove unused parameters. (`#1018 <https://github.com/ros-planning/moveit2/issues/1018>`_)
* Merge PR `#2938 <https://github.com/ros-planning/moveit2/issues/2938>`_: Rework ACM
  Implement ACM defaults as a fallback instead of an override.
  Based on `ros-planning/srdfdom#97 <https://github.com/ros-planning/srdfdom/issues/97>`_, this allows disabling collisions for specific links/objects by default and re-enabling individual pairs if necessary.
* Make TimeParameterization classes polymorphic (`#3021 <https://github.com/ros-planning/moveit2/issues/3021>`_)
* Fix wrong transform in distance fields' determineCollisionSpheres() (`#3022 <https://github.com/ros-planning/moveit2/issues/3022>`_)
* collision_distance_field: Fix undefined behavior vector insertion (`#3017 <https://github.com/ros-planning/moveit2/issues/3017>`_)
  Co-authored-by: andreas-botbuilt <94128674+andreas-botbuilt@users.noreply.github.com>
* Unify initialization of ACM from SRDF
* Adapt to API changes in srdfdom
  @v4hn requested splitting of collision_pairs into (re)enabled and disabled.
* ACM:print(): show default value
* Adapt message passing of AllowedCollisionMatrix
  - Serialize full current state (previously pairs with a default, but no entry were skipped)
  - Only initialize matrix entries that deviate from the default.
* Optimization: Check for most common case first
* Add comment to prefer setDefaultEntry() over setEntry()
  ... because the former will consider future collision entries as well.
* ACM: specific pair entries take precedence over defaults
  Reverts c72a8570d420a23a9fe4715705ed617f18836634
* Improve formatting of comments
* Don't fill all ACM entries by default
* Adapt to API changes in srdfdom
* Move MoveItErrorCode class to moveit_core (`#3009 <https://github.com/ros-planning/moveit2/issues/3009>`_)
  ... reducing code duplication and facilitating re-use
* Disable (flaky) timing tests in DEBUG mode (`#3012 <https://github.com/ros-planning/moveit2/issues/3012>`_)
* RobotState::attachBody: Migrate to unique_ptr argument (`#3011 <https://github.com/ros-planning/moveit2/issues/3011>`_)
  ... to indicate transfer of ownership and simplify pointer handling
* Add API stress tests for TOTG, fix undefined behavior (`#2957 <https://github.com/ros-planning/moveit2/issues/2957>`_)
* TOTG: catch division by 0
  This bug is already in the original implementation:
  https://github.com/tobiaskunz/trajectories/blob/master/Path.cpp
  In case the dot product between the two vectors is close to +/-1,
  angle becomes +/-PI and cos/tan of 0.5 * PI in the lines below will
  produce a division by 0.
  This happens easily if a optimal trajectory is processed by TOTG, i.e.,
  if a trajectory is processed by TOTG twice.
* Add API stress tests for TOTG
* Do not assert on printTransform with non-isometry (`#3005 <https://github.com/ros-planning/moveit2/issues/3005>`_)
  instead print a tag and the matrix
  building a Quaternion from non-isometries is undefined behavior in Eigen, thus the split.
* Provide MOVEIT_VERSION_CHECK macro (`#2997 <https://github.com/ros-planning/moveit2/issues/2997>`_)
  - Rename MOVEIT_VERSION -> MOVEIT_VERSION_STR
  - MOVEIT_VERSION becomes a numeric identifier
  - Use like: #if MOVEIT_VERSION >= MOVEIT_VERSION_CHECK(1, 0, 0)
* quietly use backward_cpp/ros if available (`#2988 <https://github.com/ros-planning/moveit2/issues/2988>`_)
  This is simply convenient and you always need it when you did not explicitly add it.
  Follow @tylerjw's initiative to add it to MoveIt2:
  https://github.com/ros-planning/moveit2/pull/794
* Allow restricting collision pairs to a group (`#2987 <https://github.com/ros-planning/moveit2/issues/2987>`_)
* Add backwards compatibility for old scene serialization format (`#2986 <https://github.com/ros-planning/moveit2/issues/2986>`_)
  * [moveit_core] test_planning_scene: Add failing unit test for old scene format
  The serialization format for the .scene files changed in
  https://github.com/ros-planning/moveit/pull/2037. This commits a
  testcase using the old scene format. It will fail and a subsequent
  commit to introduce backwards compatibility to the scene-file parsing
  will make it pass.
  * [moveit_core] PlanningScene: Add backwards compatibility for old scene version format
  This commit adds a mechanism for detecting the version of the scene file
  format to enable the loadGeometryFromStream method to read old version
  scene files without having to migrate them. To detect the version of the
  scene format, we use the content of the line following the start of an
  object: In the old version of the format, this specified the number of
  shapes in the object (a single int). In the new version of the format,
  it is the translational part of the pose of the object (i.e. three
  double values separated by spaces). To detect the format, we check for
  the number of spaces after trimming the string.
  * Simplify code: Avoid reading full stream
  Co-authored-by: Robert Haschke <rhaschke@techfak.uni-bielefeld.de>
* Switch to std::bind (`#2967 <https://github.com/ros-planning/moveit2/issues/2967>`_)
  * boost::bind -> std::bind
  grep -rlI --exclude-dir=.git "boost::bind" | xargs sed -i 's/boost::bind/std::bind/g'
  * Convert bind placeholders
  grep -rlI --exclude-dir=.git " _[0-9]" | xargs sed -i 's/ _\([0-9]\)/ std::placeholders::_\1/g'
  * Update bind include header
  grep -rlI --exclude-dir=.git "boost/bind" | xargs sed -i 's#boost/bind.hpp#functional#'
* Add waypoint duration to the trajectory deep copy unit test (`#2961 <https://github.com/ros-planning/moveit2/issues/2961>`_)
  * Add waypoint duration to the trajectory deep copy test
  * Slightly more accurate comments
* 1.1.6
* Silent warning about virtual_joint in Gazebo setups
  Gazebo requires a fixed joint from world to the first robot link.
  This resembles the virtual_joint of SRDF.
  However, the RobotModel parser issues the following warning:
  Skipping virtual joint 'xxx' because its child frame 'xxx' does not match the URDF frame 'world'
* Drop the minimum velocity/acceleration limits for TOTG (`#2937 <https://github.com/ros-planning/moveit2/issues/2937>`_)
  Just complain about negative / zero values.
* Fix Debug build: re-add seemingly unused arguments
* Merge `#2918 <https://github.com/ros-planning/moveit2/issues/2918>`_ (add RobotState::getRigidlyAttachedParentLink)
  Merge branch 'pr-master-state-rigidly-attached-parent'
* add RS::getRigidlyConnectedParentLinkModel
  to resolve links for attached objects as well
* consistent parameter names for AttachedBody constructor
  "attach_posture" is plain wrong.
  I don't see why clang-tidy did not find this before.
* Contributors: Abishalini, AndyZe, Burak Payzun, Cory Crean, David V. Lu!!, Henning Kayser, Jafar, Jafar Abdi, Jochen Sprickerhof, Jonathan Grebe, Martin Oehler, Michael Görner, Robert Haschke, Sencer Yazıcı, Simon Schmeisser, Stephanie Eng, Tyler Weaver, Wolfgang Merkt, jeoseo, pvanlaar, v4hn

2.4.0 (2022-01-20)
------------------
* Move background_processing (`#997 <https://github.com/ros-planning/moveit2/issues/997>`_)
* Fix boost linking errors for Windows (`#957 <https://github.com/ros-planning/moveit2/issues/957>`_)
* Delete backtrace hack (`#995 <https://github.com/ros-planning/moveit2/issues/995>`_)
* Use size_t for index variables (`#946 <https://github.com/ros-planning/moveit2/issues/946>`_)
* Remove moveit_build_options
* Merge https://github.com/ros-planning/moveit/commit/f3ac6070497da90da33551fc1dc3a68938340413
* Replace NULL with nullptr (`#961 <https://github.com/ros-planning/moveit2/issues/961>`_)
  * Fixes `#841 <https://github.com/ros-planning/moveit2/issues/841>`_
* Merge https://github.com/ros-planning/moveit/commit/a0ee2020c4a40d03a48044d71753ed23853a665d
* Add jerk to the robot model (`#683 <https://github.com/ros-planning/moveit2/issues/683>`_)
  * Add jerk to the robot model
  * Add joint limit parsing to a unit test
  * Add jerk to computeVariableBoundsMsg and <<, too
* collision_distance_field: Fix undefined behavior vector insertion (`#942 <https://github.com/ros-planning/moveit2/issues/942>`_)
* Normalize incoming transforms (`#2920 <https://github.com/ros-planning/moveit2/issues/2920>`_)
  * Normalize incoming transforms
  * fixup: adapt comment according to review suggestion
  Co-authored-by: Michael Görner <me@v4hn.de>
* Completely silent -Wmaybe-uninitialized
* Don't fail on -Wmaybe-uninitialized. Needs more analysis!
* Fix unused-variable warning
* Silent unused-function warnings
* Remove unused arguments from global_adjustment_factor()
  Looks like, dt and x were passed originally to call fit_cubic_spline()
  inside that function. However, later it was assumed that fit_cubic_spline()
  was already called, rendering these parameters superfluous.
* Simplify API: Remove obviously unused arguments
* clang-tidy: fix unused parameter (critical cases)
  This warnings should be considered in more detail (TODO).
  Not using these arguments might be an actual bug.
* clang-tidy: fix unused parameter (uncritical cases)
  These parameters aren't used for an obvious reason.
* moveit_build_options()
  Declare common build options like CMAKE_CXX_STANDARD, CMAKE_BUILD_TYPE,
  and compiler options (namely warning flags) once.
  Each package depending on moveit_core can use these via moveit_build_options().
* RobotState: write to correct array (`#2909 <https://github.com/ros-planning/moveit2/issues/2909>`_)
  Not an actual bug because both arrays share the same memory.
  As mentioned in https://github.com/ros-planning/moveit2/pull/683#pullrequestreview-780447848
* fix uninitialized orientation in default shape pose (`#2896 <https://github.com/ros-planning/moveit2/issues/2896>`_)
* Readability and consistency improvements in TOTG (`#2882 <https://github.com/ros-planning/moveit2/issues/2882>`_)
  * Use std::fabs() everywhere
  * Better comments
* Contributors: Abishalini, Akash, AndyZe, Michael Görner, Robert Haschke, Stephanie Eng, Tyler Weaver, andreas-botbuilt

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Convert to modern include guard `#882 <https://github.com/ros-planning/moveit2/issues/882>`_ (`#891 <https://github.com/ros-planning/moveit2/issues/891>`_)
* Replaced C-Style Cast with C++ Style Cast. (`#935 <https://github.com/ros-planning/moveit2/issues/935>`_)
* Fix CHOMP motion planner build on Windows (`#890 <https://github.com/ros-planning/moveit2/issues/890>`_)
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* Get rid of "std::endl" (`#918 <https://github.com/ros-planning/moveit2/issues/918>`_)
* changed post-increments in loops to preincrements (`#888 <https://github.com/ros-planning/moveit2/issues/888>`_)
* Fix boost linking errors (`#900 <https://github.com/ros-planning/moveit2/issues/900>`_)
* Remove unused dependency from cmake (`#839 <https://github.com/ros-planning/moveit2/issues/839>`_)
* Revert debug warning (`#884 <https://github.com/ros-planning/moveit2/issues/884>`_)
* tf2_eigen header fix for galactic
* Clang-tidy fixes (`#596 <https://github.com/ros-planning/moveit2/issues/596>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* RobotTrajectory as standard container (`#720 <https://github.com/ros-planning/moveit2/issues/720>`_)
  * Based on initial size/iterator implementations from https://github.com/ros-planning/moveit/pull/1162
* Ruckig trajectory smoothing improvements (`#712 <https://github.com/ros-planning/moveit2/issues/712>`_)
* Fixed Bullet collision checker not taking defaults into account. (`#2871 <https://github.com/ros-planning/moveit/issues/2871>`_)
* PlanningScene::getPlanningSceneDiffMsg(): Do not list an object as destroyed when it got attached (`#2864 <https://github.com/ros-planning/moveit/issues/2864>`_)
* Fix bullet-collision constructor not updating world objects (`#2830 <https://github.com/ros-planning/moveit/issues/2830>`_)
  Ensure getting notified about any objects in the world.
* Split CollisionPluginLoader (`#2834 <https://github.com/ros-planning/moveit/issues/2834>`_)
* Use default copy constructor to clone attached objects (`#2855 <https://github.com/ros-planning/moveit/issues/2855>`_)
* Remove unnecessary copy of global sub-frames map (`#2850 <https://github.com/ros-planning/moveit/issues/2850>`_)
* update comments to current parameter name (`#2853 <https://github.com/ros-planning/moveit/issues/2853>`_)
* Fix pose-not-set-bug (`#2852 <https://github.com/ros-planning/moveit/issues/2852>`_)
* add API for passing RNG to setToRandomPositionsNearBy (`#2799 <https://github.com/ros-planning/moveit/issues/2799>`_)
* PS: backwards compatibility for specifying poses for a single collision shape (`#2816 <https://github.com/ros-planning/moveit/issues/2816>`_)
* Fix Bullet collision returning wrong contact type (`#2829 <https://github.com/ros-planning/moveit/issues/2829>`_)
* Add RobotState::setToDefaultValues from group string (`#2828 <https://github.com/ros-planning/moveit/issues/2828>`_)
* Fix issue `#2809 <https://github.com/ros-planning/moveit/issues/2809>`_ (broken test with clang) (`#2820 <https://github.com/ros-planning/moveit/issues/2820>`_)
  Because std::make_pair uses the decayed type (std::string), the strings were actually copied into a temporary, which was subsequently referenced by the elements of std::pair, leading to a stack-use-after-scope error.
  Explicitly passing const references into std::make_pair via std::cref() resolves the issue mentioned in `#2809 <https://github.com/ros-planning/moveit/issues/2809>`_.
* [moveit_core] Fix export of FCL dependency (`#2819 <https://github.com/ros-planning/moveit/issues/2819>`_)
  Regression of 93c3f63
  Closes: `#2804 <https://github.com/ros-planning/moveit/issues/2804>`_
* code fix on wrong substitution (`#2815 <https://github.com/ros-planning/moveit/issues/2815>`_)
* Preserve metadata when detaching objects (`#2814 <https://github.com/ros-planning/moveit/issues/2814>`_)
* [fix] RobotState constructor segfault (`#2790 <https://github.com/ros-planning/moveit/issues/2790>`_)
* Fix compiler selecting the wrong function overload
* more fixes for the clang-tidy job (`#2813 <https://github.com/ros-planning/moveit/issues/2813>`_)
* fix clang-tidy CI job (`#2792 <https://github.com/ros-planning/moveit/issues/2792>`_)
* Fix bullet plugin library path name (`#2783 <https://github.com/ros-planning/moveit/issues/2783>`_)
* Trajectory: Improve docstrings (`#2781 <https://github.com/ros-planning/moveit/issues/2781>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Fix Windows CI (`#2776 <https://github.com/ros-planning/moveit/issues/2776>`_)
* Fixup devel-space build after `#2604 <https://github.com/ros-planning/moveit/issues/2604>`_
* Cleanup CollisionDetectorAllocatorTemplate::getName()
* RobotTrajectory: add convenience constructor
* Fix windows compilation failures
* CMakeLists.txt and package.xml fixes for cross-platform CI
* Contributors: Abishalini, Akash, AndyZe, Captain Yoshi, Dave Coleman, David V. Lu!!, Felix von Drigalski, Henning Kayser, Jafar Abdi, Jochen Sprickerhof, Kaustubh, Michael Görner, Michael Wiznitzer, Parthasarathy Bana, Peter Mitrano, Robert Haschke, Sencer Yazıcı, Silvio Traversaro, Simon Schmeisser, Tobias Fischer, Tyler Weaver, Vatan Aksoy Tezer, Wolf Vollprecht, Yuri Rocha, predystopic-dev, pvanlaar, toru-kuga, v4hn, werner291

2.3.0 (2021-10-08)
------------------
* Add debug print function to RobotTrajectory (`#715 <https://github.com/ros-planning/moveit2/issues/715>`_)
* Small matrix calc speedup in collision_distance_field_types (`#666 <https://github.com/ros-planning/moveit2/issues/666>`_)
  * Use transpose of rotation matrix in collision_distance_field_types
  * Add comment
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Fix cmake install in collision_detection_bullet (`#685 <https://github.com/ros-planning/moveit2/issues/685>`_)
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Fix cmake warnings (`#690 <https://github.com/ros-planning/moveit2/issues/690>`_)
  * Fix -Wformat-security
  * Fix -Wunused-variable
  * Fix -Wunused-lambda-capture
  * Fix -Wdeprecated-declarations
  * Fix clang-tidy, readability-identifier-naming in moveit_kinematics
* Add Ruckig trajectory_processing plugin (jerk-limited) (`#571 <https://github.com/ros-planning/moveit2/issues/571>`_)
* New orientation constraint parameterization (`#550 <https://github.com/ros-planning/moveit2/issues/550>`_)
* Pulled in changes from the ROS MoveIt PR 'New orientation constraint parameterization `#2402 <https://github.com/ros-planning/moveit2/issues/2402>`_'.
* Fix constraint tolerance assignment (`#622 <https://github.com/ros-planning/moveit2/issues/622>`_)
* Fix warnings in Galactic and Rolling (`#598 <https://github.com/ros-planning/moveit2/issues/598>`_)
  * Use __has_includes preprocessor directive for deprecated headers
  * Fix parameter template types
  * Proper initialization of smart pointers, rclcpp::Duration
* Check for nullptr on getGlobalLinkTransform (`#611 <https://github.com/ros-planning/moveit2/issues/611>`_)
* Minor documentation and cleanup of TOTG plugin (`#584 <https://github.com/ros-planning/moveit2/issues/584>`_)
* Fixed message when parameter was found (`#595 <https://github.com/ros-planning/moveit2/issues/595>`_)
* Fix some format strings (`#587 <https://github.com/ros-planning/moveit2/issues/587>`_)
* Fixes for Windows (`#530 <https://github.com/ros-planning/moveit2/issues/530>`_)
* Tests for CurrentStateMonitor using dependency injection (`#562 <https://github.com/ros-planning/moveit2/issues/562>`_)
* Refactors for OccMapTree in PlanningScene (`#2684 <https://github.com/ros-planning/moveit2/issues/2684>`_)
* Add new orientation constraint parameterization (`#2402 <https://github.com/ros-planning/moveit2/issues/2402>`_)
* Avoid push_back within getAttachedBodyObjects (`#2732 <https://github.com/ros-planning/moveit2/issues/2732>`_)
* Port `#2721 <https://github.com/ros-planning/moveit2/issues/2721>`_ (fixed padding collision attached objects) to Master (`#2731 <https://github.com/ros-planning/moveit2/issues/2731>`_)
* New RobotState interpolation test (`#2665 <https://github.com/ros-planning/moveit2/issues/2665>`_)
  * started interpolation test
  * more tests
  * test interpolation bounds checking
* use lockable octomap for MotionPlanningDisplay
* Implement checkCollision with default ACM as wrapper
* Move OccMapTree to moveit_core/collision_detection
* Contributors: AdamPettinger, Akash, AndyZe, Bjar Ne, David V. Lu!!, George Stavrinos, Henning Kayser, Jafar Abdi, Jeroen, John Stechschulte, Michael J. Park, Nathan Brooks, Robert Haschke, Simon Schmeisser, Tyler Weaver, Vatan Aksoy Tezer, Jack, Wyatt Rees, Nisala Kalupahana, Jorge Nicho, Lior Lustgarten

2.2.1 (2021-07-12)
------------------
* Pluginlib Deprecation Fix (`#542 <https://github.com/ros-planning/moveit2/issues/542>`_)
* Set project VERSION in moveit_common, fix sonames (`#532 <https://github.com/ros-planning/moveit2/issues/532>`_)
* Contributors: David V. Lu!!, Henning Kayser

2.2.0 (2021-06-30)
------------------
* Enable Bullet and fix plugin configuration (`#489 <https://github.com/ros-planning/moveit2/issues/489>`_)
* Fix typo in joint_model_group.h (`#510 <https://github.com/ros-planning/moveit2/issues/510>`_)
* Enable Rolling and Galactic CI (`#494 <https://github.com/ros-planning/moveit2/issues/494>`_)
* Add pluginlib dependency (`#485 <https://github.com/ros-planning/moveit2/issues/485>`_)
* [sync] MoveIt's master branch up-to https://github.com/ros-planning/moveit/commit/0d0a6a171b3fbea97a0c4f284e13433ba66a4ea4
  * Use thread_local var's in FCL distanceCallback() (`#2698 <https://github.com/ros-planning/moveit/issues/2698>`_)
  * Remove octomap from catkin_packages LIBRARIES entries (`#2700 <https://github.com/ros-planning/moveit/issues/2700>`_)
  * CI: Use compiler flag --pedantic (`#2691 <https://github.com/ros-planning/moveit/issues/2691>`_)
  * Remove deprecated header deprecation.h (`#2693 <https://github.com/ros-planning/moveit/issues/2693>`_)
  * collision_detection_fcl: Report link_names in correct order (`#2682 <https://github.com/ros-planning/moveit/issues/2682>`_)
  * RobotState interpolation: warn if interpolation parameter is out of range [0, 1] (`#2664 <https://github.com/ros-planning/moveit/issues/2664>`_)
  * Add sphinx-rtd-theme for python docs as a dependency (`#2645 <https://github.com/ros-planning/moveit/issues/2645>`_)
  * Set rotation value of cartesian MaxEEFStep by default (`#2614 <https://github.com/ros-planning/moveit/issues/2614>`_)
  * Lock the Bullet collision environment, for thread safety (`#2598 <https://github.com/ros-planning/moveit/issues/2598>`_)
  * Make setToIKSolverFrame accessible again (`#2580 <https://github.com/ros-planning/moveit/issues/2580>`_)
  * Python bindings for moveit_core (`#2547 <https://github.com/ros-planning/moveit/issues/2547>`_)
  * Add get_active_joint_names (`#2533 <https://github.com/ros-planning/moveit/issues/2533>`_)
  * Update doxygen comments for distance() and interpolate() (`#2528 <https://github.com/ros-planning/moveit/issues/2528>`_)
  * Replaced eigen+kdl conversions with tf2_eigen + tf2_kdl (`#2472 <https://github.com/ros-planning/moveit/issues/2472>`_)
  * Fix logic, improve function comment for clearDiffs() (`#2497 <https://github.com/ros-planning/moveit/issues/2497>`_)
* Contributors: 0Nel, AndyZe, David V. Lu!!, Felix von Drigalski, JafarAbdi, Jochen Sprickerhof, John Stechschulte, Jorge Nicho, Max Schwarz, Michael Görner, Peter Mitrano, Robert Haschke, Simon Schmeisser, Tyler Weaver, Vatan Aksoy Tezer, petkovich

2.1.4 (2021-05-31)
------------------
* PlanningRequestAdapter helper method getParam()  (`#468 <https://github.com/ros-planning/moveit2/issues/468>`_)
  * Implement parameters for adapter plugins
* Contributors: David V. Lu!!

2.1.3 (2021-05-22)
------------------
* Delete exclusive arg for collision detector creation (`#466 <https://github.com/ros-planning/moveit2/issues/466>`_)
  * Delete exclusive arg for collision detector creation
  * Rename setActiveCollisionDetector->allocateCollisionDetector everywhere
* Cleanup collision_distance_field test dependencies (`#465 <https://github.com/ros-planning/moveit2/issues/465>`_)
* Fix PlanningScene CollisionDetector diff handling (`#464 <https://github.com/ros-planning/moveit2/issues/464>`_)
* Fix joint limit handling when velocities aren't included in robot state (`#451 <https://github.com/ros-planning/moveit2/issues/451>`_)
* Contributors: AndyZe, Henning Kayser

2.1.2 (2021-04-20)
------------------
* Fix robot_model & moveit_ros_visualization dependencies (`#421 <https://github.com/ros-planning/moveit2/issues/421>`_)
* Unify PickNik name in copyrights (`#419 <https://github.com/ros-planning/moveit2/issues/419>`_)
* Contributors: Jafar Abdi, Tyler Weaver

2.1.1 (2021-04-12)
------------------
* Update doxygen comments for distance() and interpolate() (`#401 <https://github.com/ros-planning/moveit2/issues/401>`_)
* Add differential drive joint model (`#390 <https://github.com/ros-planning/moveit2/issues/390>`_)
  * RobotModelBuilder: Add new function addJointProperty to add a property for a joint
  * Add angular_distance_weight joint property
  * Add motion_model joint property
  * Add min_translational_distance joint property
* Add initialize function for moveit_sensor_manager plugin (`#386 <https://github.com/ros-planning/moveit2/issues/386>`_)
* Eliminate ability to keep multiple collision detectors updated (`#364 <https://github.com/ros-planning/moveit2/issues/364>`_)
  * Fix seg faults in setCollisionDetectorType()
  * Add unit test for switching collision detector types
* Port of Bullet collision to ROS2 (`#322 <https://github.com/ros-planning/moveit2/issues/322>`_)
* Fix EXPORT install in CMake (`#372 <https://github.com/ros-planning/moveit2/issues/372>`_)
* Bug fixes in main branch (`#362 <https://github.com/ros-planning/moveit2/issues/362>`_)
  * robot_trajectory: Fix bugs in getRobotTrajectoryMsg function
  * controller_manager: Use Duration(-1) as infinite timeout
  * ActionBasedControllerHandle: fix dangling reference in case of timeout
  * TfPublisher: tf frame name can't start with '/'
* Sync main branch with MoveIt 1 from previous head https://github.com/ros-planning/moveit/commit/0247ed0027ca9d7f1a7f066e62c80c9ce5dbbb5e up to https://github.com/ros-planning/moveit/commit/74b3e30db2e8683ac17b339cc124675ae52a5114
* [fix] export cmake library install (`#339 <https://github.com/ros-planning/moveit2/issues/339>`_)
* Clean up collision-related log statements (`#2480 <https://github.com/ros-planning/moveit2/issues/2480>`_)
* Fix RobotState::dropAccelerations/dropEffort to not drop velocities (`#2478 <https://github.com/ros-planning/moveit2/issues/2478>`_)
* Provide a function to set the position of active joints in a JointModelGroup (`#2456 <https://github.com/ros-planning/moveit2/issues/2456>`_)
  * RobotState::setJointGroupPositions: assert correct size of  vector
  * setJointGroupActivePositions sets only the positions of active joints
  * implement JointModelGroup::getActiveVariableCount
* Fix doxygen documentation for setToIKSolverFrame (`#2461 <https://github.com/ros-planning/moveit2/issues/2461>`_)
  * Fix doxygen documentation for setToIKSolverFrame
  * "Convert" -> "Transform"
  * Make function private. Update comments.
  * Make inline and private
  * Longer function should not be inline
* Fix validation of orientation constraints (`#2434 <https://github.com/ros-planning/moveit2/issues/2434>`_)
* RobotModelBuilder: Add parameter to specify the joint rotation axis
* RobotModelBuilder: Allow adding end effectors (`#2454 <https://github.com/ros-planning/moveit2/issues/2454>`_)
* Delete CollisionRequest min_cost_density
* Fix OrientationConstraint::decide (`#2414 <https://github.com/ros-planning/moveit2/issues/2414>`_)
* Changed processing_thread\_ spin to use std::make_unique instead of new (`#2412 <https://github.com/ros-planning/moveit2/issues/2412>`_)
* Update collision-related comments (`#2382 <https://github.com/ros-planning/moveit2/issues/2382>`_) (`#2388 <https://github.com/ros-planning/moveit2/issues/2388>`_)
* Contributors: AndyZe, David V. Lu!!, Henning Kayser, Jafar Abdi, Jorge Nicho, Robert Haschke, Simon Schmeisser, Stuart Anderson, Thomas G, Tyler Weaver, sevangelatos

2.1.0 (2020-11-23)
------------------
* [fix] Clang-tidy fixes (`#264 <https://github.com/ros-planning/moveit2/issues/264>`_, `#210 <https://github.com/ros-planning/moveit2/issues/210>`_)
  * Suppress false-positive clang-tidy fix in DistanceResultsData
  * Fix Eigen alignment in DistanceResultsData
  * Fix readability-identifier-naming, performance-for-range-copy, readability-named-parameter
* [fix] Fixup moveit_resources usage in moveit_core test (`#259 <https://github.com/ros-planning/moveit2/issues/259>`_)
* [maint] Remove deprecated namespaces robot_model, robot_state  (`#276 <https://github.com/ros-planning/moveit2/issues/276>`_)
* [maint] Wrap common cmake code in 'moveit_package()' macro (`#285 <https://github.com/ros-planning/moveit2/issues/285>`_)
  * New moveit_package() macro for compile flags, Windows support etc
  * Add package 'moveit_common' as build dependency for moveit_package()
  * Added -Wno-overloaded-virtual compiler flag for moveit_ros_planners_ompl
* [maint] Compilation fixes for macOS (`#271 <https://github.com/ros-planning/moveit2/issues/271>`_)
* [maint] kinematics_base: remove deprecated initialize function (`#232 <https://github.com/ros-planning/moveit2/issues/232>`_)
* [maint] Update to new moveit_resources layout (`#247 <https://github.com/ros-planning/moveit2/issues/247>`_)
* [maint] Enable "test_time_optimal_trajectory_generation" unit test (`#241 <https://github.com/ros-planning/moveit2/issues/241>`_)
* [maint] CMakeLists dependency cleanup and fixes (`#226 <https://github.com/ros-planning/moveit2/issues/226>`_, `#228 <https://github.com/ros-planning/moveit2/issues/228>`_)
* [ros2-migration] Migrate to ROS 2 Foxy (`#227 <https://github.com/ros-planning/moveit2/issues/227>`_)
* Contributors: Abdullah Alzaidy, Dave Coleman, Henning Kayser, Jafar Abdi, Lior Lustgarten, Mark Moll, Mohmmad Ayman, Robert Haschke, Yu Yan, Tyler Weaver, Sebastian Jahr

2.0.0 (2020-02-17)
------------------
* [improve] Load OMPL planner config parameters
* [fix] Fix double node executor exceptions
  * Load parameters from node instead of SyncParameterClient
* [fix] Load planning request adapter parameters from subnamespace
* [fix] KinematicsBase: fix default value in parameter lookup (`#154 <https://github.com/ros-planning/moveit2/issues/154>`_)
* [sys] Upgrade to ROS 2 Eloquent (`#152 <https://github.com/ros-planning/moveit2/issues/152>`_)
* [sys] Fix CMakeLists.txt files for Eloquent
* [sys] replace rosunit -> ament_cmake_gtest
* [maintenance] Remove redundant build dependency to 'angles'
* [ros2-migration] Build moveit_core with colcon (`#117 <https://github.com/ros-planning/moveit2/issues/117>`_, `#125 <https://github.com/ros-planning/moveit2/issues/125>`_, `#164 <https://github.com/ros-planning/moveit2/issues/164>`_)
* [ros2-migration] Increase CMake version to 3.10.2 per REP 2000 (`#27 <https://github.com/ros-planning/moveit2/issues/27>`_)
* [ros2-migration] Port moveit ros visualization to ROS 2 (`#160 <https://github.com/ros-planning/moveit2/issues/160>`_)
* [ros2-migration] Port moveit_simple_controller_manager to ROS 2 (`#158 <https://github.com/ros-planning/moveit2/issues/158>`_)
* [ros2-migration] Port planning_request_adapter_plugins to ROS 2 (`#62 <https://github.com/ros-planning/moveit2/issues/62>`_, `#87 <https://github.com/ros-planning/moveit2/issues/87>`_, `#114 <https://github.com/ros-planning/moveit2/issues/114>`_)
* [ros2-migration] Port kinematic_constraints to ROS2 (`#42 <https://github.com/ros-planning/moveit2/issues/42>`_)
* [ros2-migration] Port collision_distance_field to ROS 2 (`#65 <https://github.com/ros-planning/moveit2/issues/65>`_)
* [ros2-migration] Port constraint_samplers to ROS 2 (`#60 <https://github.com/ros-planning/moveit2/issues/60>`_)
* [ros2-migration] Port kinematics_base to ROS 2 (`#8 <https://github.com/ros-planning/moveit2/issues/8>`_, `#83 <https://github.com/ros-planning/moveit2/issues/83>`_, `#145 <https://github.com/ros-planning/moveit2/issues/145>`_)
* [ros2-migration] Port collision_detection_fcl to ROS 2 (`#41 <https://github.com/ros-planning/moveit2/issues/41>`_)
* [ros2-migration] Port planning_scene to ROS2 (`#43 <https://github.com/ros-planning/moveit2/issues/43>`_)
* [ros2-migration] Port trajectory_processing to ROS 2 (`#63 <https://github.com/ros-planning/moveit2/issues/63>`_)
* [ros2-migration] Port collision_detection to ROS 2 (`#40 <https://github.com/ros-planning/moveit2/issues/40>`_)
* [ros2-migration] Port distance_field to ROS 2 (`#64 <https://github.com/ros-planning/moveit2/issues/64>`_)
* [ros2-migration] Port background_processing to ROS 2  (`#55 <https://github.com/ros-planning/moveit2/issues/55>`_, `#82 <https://github.com/ros-planning/moveit2/issues/82>`_)
* [ros2-migration] Port controller_manager to ROS 2 (`#84 <https://github.com/ros-planning/moveit2/issues/84>`_)
* [ros2-migration] Port moveit_core_utils to ROS 2 (`#68 <https://github.com/ros-planning/moveit2/issues/68>`_)
* [ros2-migration] Port robot_state to ROS 2 (`#80 <https://github.com/ros-planning/moveit2/issues/80>`_)
* [ros2-migration] Port robot_trajectory to ROS 2 (`#39 <https://github.com/ros-planning/moveit2/issues/39>`_)
* [ros2-migration] Port kinematics_metrics to ROS 2 (`#66 <https://github.com/ros-planning/moveit2/issues/66>`_, `#88 <https://github.com/ros-planning/moveit2/issues/88>`_)
* [ros2-migration] Port planning_interface to ROS 2 (`#61 <https://github.com/ros-planning/moveit2/issues/61>`_, `#86 <https://github.com/ros-planning/moveit2/issues/86>`_)
* [ros2-migration] Port dynamics_solver to ROS 2 (`#67 <https://github.com/ros-planning/moveit2/issues/67>`_, `#89 <https://github.com/ros-planning/moveit2/issues/89>`_)
* [ros2-migration] Port robot_model to ROS 2 (`#10 <https://github.com/ros-planning/moveit2/issues/10>`_)
* [ros2-migration] Port profiler to ROS 2 (`#9 <https://github.com/ros-planning/moveit2/issues/9>`_)
* [ros2-migration] Port transforms to ROS 2 (`#12 <https://github.com/ros-planning/moveit2/issues/12>`_)
* [ros2-migration] Port exceptions to ROS 2 (`#7 <https://github.com/ros-planning/moveit2/issues/7>`_, `#81 <https://github.com/ros-planning/moveit2/issues/81>`_)
* [ros2-migration] Port controller_manager submodule of moveit_core to ROS 2 (`#6 <https://github.com/ros-planning/moveit2/issues/6>`_)
* [ros2-migration] Port version submodule of moveit_core (`#4 <https://github.com/ros-planning/moveit2/issues/4>`_)
* [ros2-migration] Port backtrace to ROS 2 (`#5 <https://github.com/ros-planning/moveit2/issues/5>`_)
* [ros2-migration] Port sensor_manager ROS 2 (`#11 <https://github.com/ros-planning/moveit2/issues/11>`_)
* [ros2-migration] Port macros to ROS 2 (`#3 <https://github.com/ros-planning/moveit2/issues/3>`_)
* Contributors: Abdullah Alzaidy, Alejandro Hernández Cordero, Anas Mchichou El Harrak, Dave Coleman, Henning Kayser, Jafar Abdi, Mark Moll, Michael Görner, Mike Lautman, Mohmmad Ayman, Robert Haschke, Tyler Weaver, Víctor Mayoral Vilches, Yu Yan

1.1.1 (2020-10-13)
------------------
* [feature] Handle multiple link libraries for FCL (`ros-planning:moveit#2325 <https://github.com/ros-planning/moveit/issues/2325>`_)
* [feature] Adapt to API changes in geometric_shapes (`ros-planning:moveit#2324 <https://github.com/ros-planning/moveit/issues/2324>`_)
* [fix] clang-tidy issues (`ros-planning:moveit#2337 <https://github.com/ros-planning/moveit/issues/2337>`_)
* [fix] various issues with Noetic build (`ros-planning:moveit#2327 <https://github.com/ros-planning/moveit/issues/2327>`_)
* [maint] Depend on ros-noetic-fcl (0.6) in Noetic (`ros-planning:moveit#2359 <https://github.com/ros-planning/moveit/issues/2359>`_)
* [maint] Cleanup MSA includes (`ros-planning:moveit#2351 <https://github.com/ros-planning/moveit/issues/2351>`_)
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`ros-planning:moveit#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski, G.A. vd. Hoorn, Robert Haschke

1.1.0 (2020-09-04)
------------------
* [feature] Add a utility to print collision pairs (`ros-planning:moveit#2275 <https://github.com/ros-planning/moveit/issues/2275>`_)
* [feature] Fix subframes disappearing when object is detached/scaled/renamed (`ros-planning:moveit#1866 <https://github.com/ros-planning/moveit/issues/1866>`_)
* [feature] Use Eigen::Transform::linear() instead of rotation() (`ros-planning:moveit#1964 <https://github.com/ros-planning/moveit/issues/1964>`_)
* [feature] Utilize new geometric_shapes functions to improve performance (`ros-planning:moveit#2038 <https://github.com/ros-planning/moveit/issues/2038>`_)
* [feature] move_group pick place test (`ros-planning:moveit#2031 <https://github.com/ros-planning/moveit/issues/2031>`_)
* [feature] Split collision proximity threshold (`ros-planning:moveit#2008 <https://github.com/ros-planning/moveit/issues/2008>`_)
* [feature] Integration test to defend subframe tutorial (`ros-planning:moveit#1757 <https://github.com/ros-planning/moveit/issues/1757>`_)
* [feature] List missing joints in group states (`ros-planning:moveit#1935 <https://github.com/ros-planning/moveit/issues/1935>`_)
* [feature] Improve documentation for setJointPositions() (`ros-planning:moveit#1921 <https://github.com/ros-planning/moveit/issues/1921>`_)
* [feature] Installs an empty plugin description xml file if bullet is not found (`ros-planning:moveit#1898 <https://github.com/ros-planning/moveit/issues/1898>`_)
* [feature] Bullet collision detection (`ros-planning:moveit#1839 <https://github.com/ros-planning/moveit/issues/1839>`_)
* [feature] Improve RobotState documentation (`ros-planning:moveit#1846 <https://github.com/ros-planning/moveit/issues/1846>`_)
* [feature] Adapt cmake for Bullet (`ros-planning:moveit#1744 <https://github.com/ros-planning/moveit/issues/1744>`_)
* [feature] Unified Collision Environment Bullet (`ros-planning:moveit#1572 <https://github.com/ros-planning/moveit/issues/1572>`_)
* [feature] Adding continuous collision detection to Bullet (`ros-planning:moveit#1551 <https://github.com/ros-planning/moveit/issues/1551>`_)
* [feature] Bullet Collision Detection (`ros-planning:moveit#1504 <https://github.com/ros-planning/moveit/issues/1504>`_)
* [feature] Generic collision detection test suite (`ros-planning:moveit#1543 <https://github.com/ros-planning/moveit/issues/1543>`_)
* [feature] Empty collision checker template for usage with tesseract and bullet (`ros-planning:moveit#1499 <https://github.com/ros-planning/moveit/issues/1499>`_)
* [feature] Add deepcopy option for RobotTrajectory's copy constructor (`ros-planning:moveit#1760 <https://github.com/ros-planning/moveit/issues/1760>`_)
* [feature] Enable code-coverage test (`ros-planning:moveit#1776 <https://github.com/ros-planning/moveit/issues/1776>`_)
* [feature] Provide UniquePtr macros (`ros-planning:moveit#1771 <https://github.com/ros-planning/moveit/issues/1771>`_)
* [feature] Improve variable name in RobotModel (`ros-planning:moveit#1752 <https://github.com/ros-planning/moveit/issues/1752>`_)
* [feature] Adding documentation to collision detection (`ros-planning:moveit#1645 <https://github.com/ros-planning/moveit/issues/1645>`_)
* [feature] Unified Collision Environment Integration (`ros-planning:moveit#1584 <https://github.com/ros-planning/moveit/issues/1584>`_)
* [feature] Document discretization behavior in KinematicsBase (`ros-planning:moveit#1602 <https://github.com/ros-planning/moveit/issues/1602>`_)
* [feature] Rename lm to link_model (`ros-planning:moveit#1592 <https://github.com/ros-planning/moveit/issues/1592>`_)
* [feature] Allow ROS namespaces for planning request adapters (`ros-planning:moveit#1530 <https://github.com/ros-planning/moveit/issues/1530>`_)
* [feature] Add named frames to CollisionObjects (`ros-planning:moveit#1439 <https://github.com/ros-planning/moveit/issues/1439>`_)
* [feature] More verbose "id" argument in PlanningScene, RobotState & CollisionWorld functions (`ros-planning:moveit#1450 <https://github.com/ros-planning/moveit/issues/1450>`_)
* [feature] Separate source file for CartesianInterpolator (`ros-planning:moveit#1149 <https://github.com/ros-planning/moveit/issues/1149>`_)
* [fix] Various fixes for upcoming Noetic release (`ros-planning:moveit#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Change FloatingJointModel::getStateSpaceDimension return value to 7
* [fix] collision world: check for empty shapes vector before access (`ros-planning:moveit#2026 <https://github.com/ros-planning/moveit/issues/2026>`_)
* [fix] Fix Condition for Adding current DistanceResultData to DistanceMap for DistanceRequestType::SINGLE (`ros-planning:moveit#1963 <https://github.com/ros-planning/moveit/issues/1963>`_)
* [fix] Do not override empty URDF link collision geometry (`ros-planning:moveit#1952 <https://github.com/ros-planning/moveit/issues/1952>`_)
* [fix] Fix issue in unpadded collision checking (`ros-planning:moveit#1899 <https://github.com/ros-planning/moveit/issues/1899>`_)
* [fix] Remove object from collision world only once (`ros-planning:moveit#1900 <https://github.com/ros-planning/moveit/issues/1900>`_)
* [fix] Initialize zero dynamics in CurrentStateMonitor (`ros-planning:moveit#1883 <https://github.com/ros-planning/moveit/issues/1883>`_)
* [fix] getFrameInfo(): Avoid double search for link name (`ros-planning:moveit#1853 <https://github.com/ros-planning/moveit/issues/1853>`_)
* [fix] Fix RobotTrajectory's copy constructor (`ros-planning:moveit#1834 <https://github.com/ros-planning/moveit/issues/1834>`_)
* [fix] Fix flaky moveit_cpp test (`ros-planning:moveit#1781 <https://github.com/ros-planning/moveit/issues/1781>`_)
* [fix] Fix doc string OrientationConstraint (`ros-planning:moveit#1793 <https://github.com/ros-planning/moveit/issues/1793>`_)
* [fix] Move ASSERT() into test setup (`ros-planning:moveit#1657 <https://github.com/ros-planning/moveit/issues/1657>`_)
* [fix] Add missing dependencies to library (`ros-planning:moveit#1746 <https://github.com/ros-planning/moveit/issues/1746>`_)
* [fix] Fix clang-tidy for unified collision environment (`ros-planning:moveit#1638 <https://github.com/ros-planning/moveit/issues/1638>`_)
* [fix] PlanningRequestAdapter::initialize() = 0 (`ros-planning:moveit#1621 <https://github.com/ros-planning/moveit/issues/1621>`_)
* [fix] Fix World::getTransform (`ros-planning:moveit#1553 <https://github.com/ros-planning/moveit/issues/1553>`_)
* [fix] Link moveit_robot_model from moveit_test_utils (`ros-planning:moveit#1534 <https://github.com/ros-planning/moveit/issues/1534>`_)
* [maint] Move constraint representation dox to moveit_tutorials (`ros-planning:moveit#2147 <https://github.com/ros-planning/moveit/issues/2147>`_)
* [maint] Update dependencies for python3 in noetic (`ros-planning:moveit#2131 <https://github.com/ros-planning/moveit/issues/2131>`_)
* [maint] clang-tidy fixes (`ros-planning:moveit#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `ros-planning:moveit#2004 <https://github.com/ros-planning/moveit/issues/2004>`_, `ros-planning:moveit#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`ros-planning:moveit#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Rename PR2-related collision test files (`ros-planning:moveit#1856 <https://github.com/ros-planning/moveit/issues/1856>`_)
* [maint] Fix compiler warnings (`ros-planning:moveit#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] Add missing licenses (`ros-planning:moveit#1716 <https://github.com/ros-planning/moveit/issues/1716>`_) (`ros-planning:moveit#1720 <https://github.com/ros-planning/moveit/issues/1720>`_)
* [maint] Move isEmpty() test functions to moveit_core/utils (`ros-planning:moveit#1627 <https://github.com/ros-planning/moveit/issues/1627>`_)
* [maint] Switch from include guards to pragma once (`ros-planning:moveit#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`ros-planning:moveit#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: AndyZe, Aris Synodinos, Ayush Garg, Bryce Willey, Dale Koenig, Dave Coleman, Felix von Drigalski, Henning Kayser, Jafar Abdi, Jens P, Jere Liukkonen, Jeroen, John Stechschulte, Jonas Wittmann, Jonathan Binney, Markus Vieth, Martin Pecka, Michael Ferguson, Michael Görner, Mike Lautman, Niklas Fiedler, Patrick Beeson, Robert Haschke, Sean Yen, Shivang Patel, Tyler Weaver, Wolfgang Merkt, Yu, Yan, tsijs, v4hn

1.0.6 (2020-08-19)
------------------
* [maint] Adapt repository for splitted moveit_resources layout (`ros-planning:moveit#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint] Migrate to clang-format-10, Fix warnings
* [maint] Optimize includes (`ros-planning:moveit#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* [maint] Fix docs in robot_state.h (`ros-planning:moveit#2215 <https://github.com/ros-planning/moveit/issues/2215>`_)
* Contributors: Jeroen, Markus Vieth, Michael Görner, Robert Haschke

1.0.5 (2020-07-08)
------------------
* [fix]     Fix memory leaks related to geometric shapes usage (`ros-planning:moveit#2138 <https://github.com/ros-planning/moveit/issues/2138>`_)
* [fix]     Prevent collision checking segfault if octomap has NULL root pointer (`ros-planning:moveit#2104 <https://github.com/ros-planning/moveit/issues/2104>`_)
* [feature] Allow to parameterize input trajectory density of Time Optimal trajectory generation (`ros-planning:moveit#2185 <https://github.com/ros-planning/moveit/issues/2185>`_)
* [maint]   Optional C++ version setting (`ros-planning:moveit#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [maint]   Added missing boost::regex dependency (`ros-planning:moveit#2163 <https://github.com/ros-planning/moveit/issues/2163>`_)
* [maint]   PropagationDistanceField: Replace eucDistSq with squaredNorm (`ros-planning:moveit#2101 <https://github.com/ros-planning/moveit/issues/2101>`_)
* [fix]     Fix getTransform() (`ros-planning:moveit#2113 <https://github.com/ros-planning/moveit/issues/2113>`_)
  - PlanningScene::getTransforms().getTransform() -> PlanningScene::getFrameTransform()
  - PlanningScene::getTransforms().canTransform() -> PlanningScene::knowsFrameTransform()
* [fix]     Change FloatingJointModel::getStateSpaceDimension return value to 7 (`ros-planning:moveit#2106 <https://github.com/ros-planning/moveit/issues/2106>`_)
* [fix]     Check for empty quaternion message (`ros-planning:moveit#2089 <https://github.com/ros-planning/moveit/issues/2089>`_)
* [fix]     TOTG: Fix parameterization for single-waypoint trajectories (`ros-planning:moveit#2054 <https://github.com/ros-planning/moveit/issues/2054>`_)
  - RobotState: Added interfaces to zero and remove dynamics
* [maint]   Remove unused angles.h includes (`ros-planning:moveit#1985 <https://github.com/ros-planning/moveit/issues/1985>`_)
* Contributors: Felix von Drigalski, Henning Kayser, Michael Görner, Jere Liukkonen, John Stechschulte, Patrick Beeson, Robert Haschke, Tyler Weaver, Wolfgang Merkt

1.0.4 (2020-05-30)
------------------
* Fix broken IKFast generator (`ros-planning:moveit#2116 <https://github.com/ros-planning/moveit/issues/2116>`_)
* Contributors: Robert Haschke

1.0.3 (2020-04-26)
------------------
* [feature] Allow to filter for joint when creating a RobotTrajectory message (`ros-planning:moveit#1927 <https://github.com/ros-planning/moveit/issues/1927>`_)
* [fix]     Fix RobotState::copyFrom()
* [fix]     Fix segfault in totg (`ros-planning:moveit#1861 <https://github.com/ros-planning/moveit/issues/1861>`_)
* [fix]     Handle incomplete group states
* [fix]     Fix issue in totg giving invalid accelerations (`ros-planning:moveit#1729 <https://github.com/ros-planning/moveit/issues/1729>`_)
* [feature] New isValidVelocityMove() for checking time between two waypoints given velocity (`ros-planning:moveit#684 <https://github.com/ros-planning/moveit/issues/684>`_)
* [maint]   Apply clang-tidy fix to entire code base (`ros-planning:moveit#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [fix]     Fix Condition for adding current DistanceResultData to DistanceMap (`ros-planning:moveit#1968 <https://github.com/ros-planning/moveit/issues/1968>`_)
* [maint]   Fix various build issues on Windows (`ros-planning:moveit#1880 <https://github.com/ros-planning/moveit/issues/1880>`_)
  * remove GCC extensions (`ros-planning:moveit#1583 <https://github.com/ros-planning/moveit/issues/1583>`_)
  * Fix binary artifact install locations. (`ros-planning:moveit#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`ros-planning:moveit#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix]     Delete attached body before adding a new one with same id (`ros-planning:moveit#1821 <https://github.com/ros-planning/moveit/issues/1821>`_)
* [maint]   Provide UniquePtr macros (`ros-planning:moveit#1771 <https://github.com/ros-planning/moveit/issues/1771>`_)
* [maint]   Updated deprecation method: MOVEIT_DEPRECATED -> [[deprecated]] (`ros-planning:moveit#1748 <https://github.com/ros-planning/moveit/issues/1748>`_)
* [feature] Add RobotTrajectory::getDuration() (`ros-planning:moveit#1554 <https://github.com/ros-planning/moveit/issues/1554>`_)
* Contributors: Ayush Garg, Dale Koenig, Dave Coleman, Felix von Drigalski, Jafar Abdi, Jeroen, Michael Görner, Mike Lautman, Niklas Fiedler, Robert Haschke, Sean Yen, Yu, Yan

1.0.2 (2019-06-28)
------------------
* [fix] Removed MessageFilter for /collision_object messages (`ros-planning:moveit#1406 <https://github.com/ros-planning/moveit/issues/1406>`_)
* [fix] Update robot state transforms when initializing a planning scene (`ros-planning:moveit#1474 <https://github.com/ros-planning/moveit/issues/1474>`_)
* [fix] Fix segfault when detaching attached collision object (`ros-planning:moveit#1438 <https://github.com/ros-planning/moveit/issues/1438>`_)
* [fix] Normalize quaternions when adding new or moving collision objects (`ros-planning:moveit#1420 <https://github.com/ros-planning/moveit/issues/1420>`_)
* [fix] Minor bug fixes in (collision) distance field (`ros-planning:moveit#1392 <https://github.com/ros-planning/moveit/issues/1392>`_)
* [fix] Remove obsolete moveit_resources/config.h ()
* [fix] Fix test utilities in moveit_core (`ros-planning:moveit#1391 <https://github.com/ros-planning/moveit/issues/1391>`_, `ros-planning:moveit#1409 <https://github.com/ros-planning/moveit/issues/1409>`_, `ros-planning:moveit#1412 <https://github.com/ros-planning/moveit/issues/1412>`_)
* Contributors: Bryce Willey, Henning Kayser, Mike Lautman, Robert Haschke, tsijs

1.0.1 (2019-03-08)
------------------
* [capability] Graphically print current robot joint states with joint limits (`ros-planning:moveit#1358 <https://github.com/ros-planning/moveit/issues/1358>`_)
* [improve] Apply clang tidy fix to entire code base (Part 1) (`ros-planning:moveit#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Dave Coleman, Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`ros-planning:moveit#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* [fix] invert waypoint velocities on reverse (`ros-planning:moveit#1335 <https://github.com/ros-planning/moveit/issues/1335>`_)
* [fix] Added missing robot state update to iterative spline parameterization to prevent warnings. (`ros-planning:moveit#1298 <https://github.com/ros-planning/moveit/issues/1298>`_)
* [fix] robot_model_test_utils depends on message generation (`ros-planning:moveit#1286 <https://github.com/ros-planning/moveit/issues/1286>`_)
* [improve] cleanup LMA kinematics solver `ros-planning:moveit#1318 <https://github.com/ros-planning/moveit/issues/1318>`_
* [improve] Remove (redundant) random seeding and ros-planning:moveit#attempts from RobotState::setFromIK() as the IK solver perform random seeding themselves. `ros-planning:moveit#1288 <https://github.com/ros-planning/moveit/issues/1288>`_
* [improve] Make FCL shape cache thread-local (`ros-planning:moveit#1316 <https://github.com/ros-planning/moveit/issues/1316>`_)
* [improve] Kinematics tests, kdl cleanup `ros-planning:moveit#1272 <https://github.com/ros-planning/moveit/issues/1272>`_, `ros-planning:moveit#1294 <https://github.com/ros-planning/moveit/issues/1294>`_
* [maintenance] Add coverage analysis for moveit_core (`ros-planning:moveit#1133 <https://github.com/ros-planning/moveit/issues/1133>`_)
* [improve] computeCartesianPath: limit joint-space jumps with IK consistency limits (`ros-planning:moveit#1293 <https://github.com/ros-planning/moveit/issues/1293>`_)
* Contributors: Alexander Gutenkunst, Dave Coleman, Jonathan Binney, Martin Oehler, Michael Görner, Mike Lautman, Robert Haschke, Simon Schmeisser

0.10.8 (2018-12-24)
-------------------
* [enhancement] Tool to generate constraint approximation databases (`ros-planning:moveit#1253 <https://github.com/ros-planning/moveit/issues/1253>`_)
* [fix] Fixed uninitialized RobotState transforms (`ros-planning:moveit#1271 <https://github.com/ros-planning/moveit/issues/1271>`_)
* Contributors: Michael Görner, Robert Haschke

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [fix] Fixed various memory leaks (`ros-planning:moveit#1104 <https://github.com/ros-planning/moveit/issues/1104>`_)
* [fix] Fixed computation of Jacobian for prismatic joints (`ros-planning:moveit#1192 <https://github.com/ros-planning/moveit/issues/1192>`_)
* [enhancement] Add support for FCL 0.6 (`ros-planning:moveit#1156 <https://github.com/ros-planning/moveit/issues/1156>`_)
* [enhancement] Pass RobotModel to IK, avoiding multiple loading (`ros-planning:moveit#1166 <https://github.com/ros-planning/moveit/issues/1166>`_)
* [enhancement] RobotTrajectory: Allow appending part of other trajectory (`ros-planning:moveit#1213 <https://github.com/ros-planning/moveit/issues/1213>`_)
* [maintenance] Rearranged CHOMP-related modules within moveit_planners/chomp (`ros-planning:moveit#1251 <https://github.com/ros-planning/moveit/issues/1251>`_)
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`ros-planning:moveit#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Use C++14 (`ros-planning:moveit#1146 <https://github.com/ros-planning/moveit/issues/1146>`_)
* [maintenance] Code Cleanup
  * `ros-planning:moveit#1179 <https://github.com/ros-planning/moveit/issues/1179>`_
  * `ros-planning:moveit#1180 <https://github.com/ros-planning/moveit/issues/1180>`_
  * `ros-planning:moveit#1185 <https://github.com/ros-planning/moveit/issues/1185>`_
  * `ros-planning:moveit#1193 <https://github.com/ros-planning/moveit/issues/1193>`_
  * `ros-planning:moveit#1194 <https://github.com/ros-planning/moveit/issues/1194>`_
  * `ros-planning:moveit#1196 <https://github.com/ros-planning/moveit/issues/1196>`_
* [maintenance] RobotModelBuilder to facilitate testing (`ros-planning:moveit#1176 <https://github.com/ros-planning/moveit/issues/1176>`_)
* Contributors: Robert Haschke, 2scholz, Alex Moriarty, Bryce Willey, Dave Coleman, Immanuel Martini, Michael Görner, Milutin Nikolic

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------
* [fix] compiler warnings (`ros-planning:moveit#1089 <https://github.com/ros-planning/moveit/issues/1089>`_)
* [code] cleanup (`ros-planning:moveit#1107 <https://github.com/ros-planning/moveit/issues/1107>`_, `ros-planning:moveit#1099 <https://github.com/ros-planning/moveit/issues/1099>`_, `ros-planning:moveit#1108 <https://github.com/ros-planning/moveit/issues/1108>`_)
* Contributors: Robert Haschke, Simon Schmeisser

0.10.2 (2018-10-24)
-------------------
* [fix] TFs in subgroups of rigidly-connected links (`ros-planning:moveit#912 <https://github.com/ros-planning/moveit/issues/912>`_)
* [fix] Chomp package handling issue `ros-planning:moveit#1086 <https://github.com/ros-planning/moveit/issues/1086>`_ that was introduced in `ubi-agni/hotfix-ros-planning:moveit#1012 <https://github.com/ubi-agni/hotfix-/issues/1012>`_
* [fix] CurrentStateMonitor update callback for floating joints to handle non-identity joint origins `ros-planning:moveit#984 <https://github.com/ros-planning/moveit/issues/984>`_
* [fix] Eigen alignment issuses due to missing aligned allocation (`ros-planning:moveit#1039 <https://github.com/ros-planning/moveit/issues/1039>`_)
* [fix] illegal pointer access (`ros-planning:moveit#989 <https://github.com/ros-planning/moveit/issues/989>`_)
* [fix] reset moveit_msgs::RobotState.is_diff to false (`ros-planning:moveit#968 <https://github.com/ros-planning/moveit/issues/968>`_) This fixes a regression introduced in `ros-planning:moveit#939 <https://github.com/ros-planning/moveit/issues/939>`_.
* [fix] continous joint limits are always satisfied (`ros-planning:moveit#729 <https://github.com/ros-planning/moveit/issues/729>`_)
* [maintenance] using LOGNAME variable rather than strings (`ros-planning:moveit#1079 <https://github.com/ros-planning/moveit/issues/1079>`_)
* [capability][chomp] Addition of CHOMP planning adapter for optimizing result of other planners (`ros-planning:moveit#1012 <https://github.com/ros-planning/moveit/issues/1012>`_)
* [enhancement] Add missing distance check functions to allValid collision checker (`ros-planning:moveit#986 <https://github.com/ros-planning/moveit/issues/986>`_)
* [enhancement] Allow chains to have only one active joint (`ros-planning:moveit#983 <https://github.com/ros-planning/moveit/issues/983>`_)
* [enhancement] collision_detection convenience (`ros-planning:moveit#957 <https://github.com/ros-planning/moveit/issues/957>`_)
* [doc] Document why to use only one IK attempt in computeCartesianPath (`ros-planning:moveit#1076 <https://github.com/ros-planning/moveit/issues/1076>`_)
* Contributors: Adrian Zwiener, Andrey Troitskiy, Dave Coleman, Jonathan Binney, Michael Görner, Mike Lautman, Mohmmad Ayman, Raghavender Sahdev, Robert Haschke, Simon Schmeisser, dcconner, mike lautman

0.10.1 (2018-05-25)
-------------------
* Clang tidy moveit_core (`ros-planning:moveit#880 <https://github.com/ros-planning/moveit/issues/880>`_) (`ros-planning:moveit#911 <https://github.com/ros-planning/moveit/issues/911>`_)
* Allow to retrieve Jacobian of a child link of a move group. (`ros-planning:moveit#877 <https://github.com/ros-planning/moveit/issues/877>`_)
* migration from tf to tf2 API (`ros-planning:moveit#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* Switch to ROS_LOGGER from CONSOLE_BRIDGE (`ros-planning:moveit#874 <https://github.com/ros-planning/moveit/issues/874>`_)
* Add ability to request detailed distance information from fcl (`ros-planning:moveit#662 <https://github.com/ros-planning/moveit/issues/662>`_)
* allow checking for absolute joint-space jumps in Cartesian path (`ros-planning:moveit#843 <https://github.com/ros-planning/moveit/issues/843>`_)
* Simplify adding colored CollisionObjects (`ros-planning:moveit#810 <https://github.com/ros-planning/moveit/issues/810>`_)
* updateMimicJoint(group->getMimicJointModels()) -> updateMimicJoints(group)
* improve RobotState::updateStateWithLinkAt() (`ros-planning:moveit#765 <https://github.com/ros-planning/moveit/issues/765>`_)
* fix computation of shape_extents\_ of links w/o shapes (`ros-planning:moveit#766 <https://github.com/ros-planning/moveit/issues/766>`_)
* Fix computation of axis-aligned bounding box (`ros-planning:moveit#703 <https://github.com/ros-planning/moveit/issues/703>`_)
* RobotModel::getRigidlyConnectedParentLinkModel()
  ... to compute earliest parent link that is rigidly connected to a given link
* Iterative cubic spline interpolation (`ros-planning:moveit#441 <https://github.com/ros-planning/moveit/issues/441>`_)
* Contributors: Bryce Willey, Ian McMahon, Ken Anderson, Levi Armstrong, Maarten de Vries, Martin Pecka, Michael Görner, Mike Lautman, Patrick Holthaus, Robert Haschke, Victor Lamoine, Xiaojian Ma

0.9.11 (2017-12-25)
-------------------
* [fix] ros-planning:moveit#723; attached bodies are not shown in trajectory visualization anymore `ros-planning:moveit#724 <https://github.com/ros-planning/moveit/issues/724>`_
* [fix] Shortcomings in kinematics plugins `ros-planning:moveit#714 <https://github.com/ros-planning/moveit/issues/714>`_
* Contributors: Henning Kayser, Michael Görner, Robert Haschke

0.9.10 (2017-12-09)
-------------------
* [fix] Add missing logWarn argument (`ros-planning:moveit#707 <https://github.com/ros-planning/moveit/issues/707>`_)
* [fix] IKConstraintSampler: Fixed transform from end-effector to ik chain tip. `ros-planning:moveit#582 <https://github.com/ros-planning/moveit/issues/582>`_
* [fix] robotStateMsgToRobotState: is_diff==true => not empty `ros-planning:moveit#589 <https://github.com/ros-planning/moveit/issues/589>`_
* [capability] Multi DOF Trajectory only providing translation not velocity (`ros-planning:moveit#555 <https://github.com/ros-planning/moveit/issues/555>`_)
* [capability] Adds parameter lookup function for kinematics plugins (`ros-planning:moveit#701 <https://github.com/ros-planning/moveit/issues/701>`_)
* [improve] Make operator bool() explicit `ros-planning:moveit#696 <https://github.com/ros-planning/moveit/pull/696>`_
* [improve] Get msgs from Planning Scene `ros-planning:moveit#663 <https://github.com/ros-planning/moveit/issues/663>`_
* [improve] moveit_core: export DEPENDS on LIBFCL `ros-planning:moveit#632 <https://github.com/ros-planning/moveit/pull/632>`_
* [improve] RobotState: Changed multi-waypoint version of computeCartesianPath to test joint space jumps after all waypoints are generated. (`ros-planning:moveit#576 <https://github.com/ros-planning/moveit/issues/576>`_)
* [improve] Better debug output for IK tip frames (`ros-planning:moveit#603 <https://github.com/ros-planning/moveit/issues/603>`_)
* [improve] New debug console colors YELLOW PURPLE (`ros-planning:moveit#604 <https://github.com/ros-planning/moveit/issues/604>`_)
* Contributors: Dave Coleman, Dennis Hartmann, Henning Kayser, Isaac I.Y. Saito, Jorge Nicho, Michael Görner, Phil, Sarah Elliott, Simon Schmeisser, TroyCordie, v4hn

0.9.9 (2017-08-06)
------------------
* [fix][moveit_core] segfault due to missing string format parameter. (`ros-planning:moveit#547 <https://github.com/ros-planning/moveit/issues/547>`_)
* [fix][moveit_core] doc-comment for robot_state::computeAABB (`ros-planning:moveit#516 <https://github.com/ros-planning/moveit/issues/516>`_)
* Contributors: Martin Pecka, henhenhen

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------
* [fix] checks for empty name arrays messages before parsing the robot state message data (`ros-planning:moveit#499 <https://github.com/ros-planning/moveit/issues/499>`_)
* Contributors: Jorge Nicho, Michael Goerner

0.9.6 (2017-04-12)
------------------
* [fix] PlanarJointModel::getVariableRandomPositionsNearBy (`ros-planning:moveit#464 <https://github.com/ros-planning/moveit/issues/464>`_)
* Contributors: Tamaki Nishino

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `ros-planning:moveit#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`ros-planning:moveit#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [fix] PlanningScene: Don't reset color information of existing objects when new entries are added (`ros-planning:moveit#410 <https://github.com/ros-planning/moveit/issues/410>`_)
* [fix] update link transforms in UnionConstraintSampler::project (`ros-planning:moveit#384 <https://github.com/ros-planning/moveit/issues/384>`_)
* [capability Addition of Set Joint Model Group Velocities and Accelerations Functions (`ros-planning:moveit#402 <https://github.com/ros-planning/moveit/issues/402>`_)
* [capability] time parameterization: use constants (`ros-planning:moveit#380 <https://github.com/ros-planning/moveit/issues/380>`_)
* [enhancement] multiple shapes in an attached collision object `ros-planning:moveit#421 <https://github.com/ros-planning/moveit/pull/421>`_
* [maintenance] Use static_cast to cast to const. (`ros-planning:moveit#433 <https://github.com/ros-planning/moveit/issues/433>`_)
* [maintenance] ompl_interface: uniform & simplified handling of the default planner (`ros-planning:moveit#371 <https://github.com/ros-planning/moveit/issues/371>`_)
* Contributors: Dave Coleman, Maarten de Vries, Michael Goerner, Mike Lautman, Ruben

0.9.3 (2016-11-16)
------------------
* [fix] Replace unused service dependency with msg dep (`ros-planning:moveit#361 <https://github.com/ros-planning/moveit/issues/361>`_)
* [fix] cleanup urdfdom compatibility (`ros-planning:moveit#319 <https://github.com/ros-planning/moveit/issues/319>`_)
* [fix] Fix missing compatibility header for Wily `ros-planning:moveit#364 <https://github.com/ros-planning/moveit/issues/364>`_)
* [enhancement] Improved RobotState feedback for setFromIK() (`ros-planning:moveit#342 <https://github.com/ros-planning/moveit/issues/342>`_)
* [maintenance] Updated package.xml maintainers and author emails `ros-planning:moveit#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon, Robert Haschke

0.9.2 (2016-11-05)
------------------
* [Fix] CHANGELOG encoding for 0.9.1 (Fix `ros-planning:moveit#318 <https://github.com/ros-planning/moveit/issues/318>`_). (`ros-planning:moveit#327 <https://github.com/ros-planning/moveit/issues/327>`_)
* [Capability] compatibility to urdfdom < 0.4 (`ros-planning:moveit#317 <https://github.com/ros-planning/moveit/issues/317>`_)
* [Capability] New isValidVelocityMove() for checking maximum velocity between two robot states given time delta
* [Maintenance] Travis check code formatting (`ros-planning:moveit#309 <https://github.com/ros-planning/moveit/issues/309>`_)
* [Maintenance] Auto format codebase using clang-format (`ros-planning:moveit#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman, Isaac I. Y. Saito, Robert Haschke

0.8.2 (2016-06-17)
------------------
* [feat] planning_scene updates: expose success state to caller. This is required to get the information back for the ApplyPlanningSceneService. `ros-planning:moveit_core#296 <https://github.com/ros-planning/moveit_core/issues/297>`_
* [sys] replaced cmake_modules dependency with eigen
* Contributors: Michael Ferguson, Robert Haschke, Michael Goerner, Isaac I. Y. Saito

0.8.1 (2016-05-19)
------------------
* Corrected check in getStateAtDurationFromStart (cherry-picking `ros-planning:moveit_core#291 <https://github.com/ros-planning/moveit_core/issues/291>`_ from indigo-devel)
* Contributors: Hamal Marino

0.8.0 (2016-05-18)
------------------
* [feat] Added file and trajectory_msg to RobotState conversion functions `ros-planning:moveit_core#267 <https://github.com/ros-planning/moveit_core/issues/267>`_
* [feat] Added setJointVelocity and setJointEffort functions `ros-planning:moveit_core#261 <https://github.com/ros-planning/moveit_core/issues/261>`_
* [feat] KinematicsBase changes `ros-planning:moveit_core#248 <https://github.com/ros-planning/moveit_core/issues/248>`_
* [feat] added an ik_seed_state argument to the new getPositionIK(...) method
* [feat] added new interface method for computing multiple ik solutions for a single pose
* [fix] RevoluteJointModel::computeVariablePositions `ros-planning:moveit_core#282 <https://github.com/ros-planning/moveit_core/issues/282>`_
* [fix] getStateAtDurationFromStart would never execute as the check for number of waypoints was inverted `ros-planning:moveit_core#289 <https://github.com/ros-planning/moveit_core/issues/289>`_
* [fix] Revert "Use libfcl-dev rosdep key in kinetic" `ros-planning:moveit_core#287 <https://github.com/ros-planning/moveit_core/issues/287>`_
* [fix] memory leak in RobotState::attachBody `ros-planning:moveit_core#276 <https://github.com/ros-planning/moveit_core/issues/276>`_. Fixing `ros-planning:moveit_core#275 <https://github.com/ros-planning/moveit_core/issues/275>`_
* [fix] New getOnlyOneEndEffectorTip() function `ros-planning:moveit_core#262 <https://github.com/ros-planning/moveit_core/issues/262>`_
* [fix] issue `ros-planning:moveit_core#258 <https://github.com/ros-planning/moveit_core/issues/258>`_ in jade-devel `ros-planning:moveit_core#266 <https://github.com/ros-planning/moveit_core/issues/266>`_
* [fix] Segfault in parenthesis operator `ros-planning:moveit_core#254 <https://github.com/ros-planning/moveit_core/issues/254>`_
* [fix] API Change of shape_tools `ros-planning:moveit_core#242 <https://github.com/ros-planning/moveit_core/issues/242>`_
* [fix] Fixed bug in KinematicConstraintSet::decide that makes it evaluate only joint_constraints. `ros-planning:moveit_core#250 <https://github.com/ros-planning/moveit_core/issues/250>`_
* [fix] Prevent divide by zero `ros-planning:moveit_core#246 <https://github.com/ros-planning/moveit_core/issues/246>`_
* [fix] removed the 'f' float specifiers and corrected misspelled method name
* [fix] typo MULTIPLE_TIPS_NO_SUPPORTED -> MULTIPLE_TIPS_NOT_SUPPORTED
* [sys] Upgrade to Eigen3 as required in Jade `ros-planning:moveit_core#293 <https://github.com/ros-planning/moveit_core/issues/293>`_
* [sys] [cmake] Tell the compiler about FCL include dirs `ros-planning:moveit_core#263 <https://github.com/ros-planning/moveit_core/issues/263>`_
* [sys] Install static libs `ros-planning:moveit_core#251 <https://github.com/ros-planning/moveit_core/issues/251>`_
* [enhance] Allow a RobotTrajectory to be initialized with a pointer joint model group `ros-planning:moveit_core#245 <https://github.com/ros-planning/moveit_core/issues/245>`_
* [doc] Better documentation and formatting `ros-planning:moveit_core#244 <https://github.com/ros-planning/moveit_core/issues/244>`_
* Contributors: Alexis Ballier, Bastian Gaspers, Christian Dornhege, Dave Coleman, Gary Servin, Ioan A Sucan, Isaac I.Y. Saito, Jim Mainprice, Levi Armstrong, Michael Ferguson, Mihai Pomarlan, Robert Haschke, Sachin Chitta, Sam Pfeiffer, Steven Peters, Severin Lemaignan, jrgnicho, ros-devel, simonschmeisser

0.6.15 (2015-01-20)
-------------------
* add ptr/const ptr types for distance field
* update maintainers
* Contributors: Ioan A Sucan, Michael Ferguson

0.6.14 (2015-01-15)
-------------------
* Add time factor to iterative_time_parametrization
* Contributors: Dave Coleman, Michael Ferguson, kohlbrecher

0.6.13 (2014-12-20)
-------------------
* add getShapePoints() to distance field
* update distance_field API to no longer use geometry_msgs
* Added ability to remove all collision objects directly through API (without using ROS msgs)
* Planning Scene: Ability to offset geometry loaded from stream
* Namespaced pr2_arm_kinematics_plugin tests to allow DEBUG output to be suppressed during testing
* Contributors: Dave Coleman, Ioan A Sucan, Michael Ferguson

0.6.12 (2014-12-03)
-------------------
* Merge pull request `ros-planning:moveit_core#214 <https://github.com/ros-planning/moveit_core/issues/214>`_ from mikeferguson/collision_plugin
  moveit_core components of collision plugins
* Merge pull request `ros-planning:moveit_core#210 <https://github.com/ros-planning/moveit_core/issues/210>`_ from davetcoleman/debug_model
  Fix truncated debug message
* Fixed a number of tests, all are now passing on buildfarm
* Merge pull request `ros-planning:moveit_core#208 <https://github.com/ros-planning/moveit_core/issues/208>`_ from mikeferguson/update_fcl_api
  update to use non-deprecated call
* Contributors: Dave Coleman, Ioan A Sucan, Michael Ferguson

0.6.11 (2014-11-03)
-------------------
* Merge pull request `ros-planning:moveit_core#204 <https://github.com/ros-planning/moveit_core/issues/204>`_ from mikeferguson/indigo-devel
* forward port `ros-planning:moveit_core#198 <https://github.com/ros-planning/moveit_core/issues/198>`_ to indigo
* Contributors: Ioan A Sucan, Michael Ferguson

0.6.10 (2014-10-27)
-------------------
* Made setVerbose virtual in constraint_sampler so that child classes can override
* Manipulability Index Error for few DOF
  When the group has fewer than 6 DOF, the Jacobian is of the form 6xM and when multiplied by its transpose, forms a 6x6 matrix that is singular and its determinant is always 0 (or NAN if the solver cannot calculate it).
  Since calculating the SVD of a Jacobian is a costly operation, I propose to retain the calculation of the Manipulability Index through the determinant for 6 or more DOF (where it produces the correct result), but use the product of the singular values of the Jacobian for fewer DOF.
* Fixed missing test depends for tf_conversions
* Allow setFromIK() with multiple poses to single IK solver
* Improved debug output
* Removed duplicate functionality poseToMsg function
* New setToRandomPositions function with custom rand num generator
* Moved find_package angles to within CATKIN_ENABLE_TESTING
* Getter for all tips (links) of every end effector in a joint model group
* New robot state to (file) stream conversion functions
* Added default values for iostream in print statements
* Change PlanningScene constructor to RobotModelConstPtr
* Documentation and made printTransform() public
* Reduced unnecessary joint position copying
* Added getSubgroups() helper function to joint model groups
* Maintain ordering of poses in order that IK solver expects
* Added new setToRandomPositions function that allows custom random number generator to be specified
* Split setToIKSolverFrame() into two functions
* Add check for correct solver type
* Allowed setFromIK to do whole body IK solving with multiple tips
* Contributors: Acorn, Dave Coleman, Ioan A Sucan, Jonathan Weisz, Konstantinos Chatzilygeroudis, Sachin Chitta, hersh

0.5.10 (2014-06-30)
-------------------
* making Saucy and Trusty version of includes to be compatible with upstream packaging. re: https://github.com/ros/rosdistro/issues/4633
* Contributors: Tully Foote

0.5.9 (2014-06-23)
------------------
* Fixed bug in RevoluteJointModel::distance() giving large negative numbers.
* kinematics_base: added an optional RobotState for context.
* fix pick/place approach/retreat on indigo/14.04
* Fixed bug in RevoluteJointModel::distance() giving large negative numbers.
* IterativeParabolicTimeParameterization now ignores virtual joints.
* kinematics_base: added an optional RobotState for context.
* Removed check for multi-dof joints in iterative_time_parameterization.cpp.
* fix pick/place approach/retreat on indigo/14.04
* IterativeParabolicTimeParameterization now ignores virtual joints.
  When checking if all joints are single-DOF, it accepts multi-DOF joints only if they are
  also virtual.
* Fix compiler warnings
* Address [cppcheck: unreadVariable] warning.
* Address [cppcheck: postfixOperator] warning.
* Address [cppcheck: stlSize] warning.
* Address [-Wunused-value] warning.
* Address [-Wunused-variable] warning.
* Address [-Wreturn-type] warning.
* Address [-Wsign-compare] warning.
* Address [-Wreorder] warning.
* Allow joint model group to have use IK solvers with multiple tip frames
* KinematicsBase support for multiple tip frames and IK requests with multiple poses
* dynamics_solver: fix crashbug
  Ignore joint that does not exist (including the virtual joint if it is part of
  the group).
* Changed KinematicsBase::supportsGroup() to use a more standard call signature.
* Merged with hydro-devel
* Removed unnecessary error output
* Removed todo
* Added support for legacy IK calls without solution_callback
* Merge branch 'hydro-devel' into kinematic_base
* Changed KinematicsBase::supportsGroup() to use a more standard call signature.
* Added empty check.
* computeCartesianPath waypoints double-up fix
  computeCartesianPath appends full trajectories between waypoints when given a vector of waypoints. As trajectories include their endpoints, this leads to the combined trajectory being generated with duplicate points at waypoints, which can lead to pauses or stuttering.
  This change skips the first point in trajectories generated between waypoints.
* avoid unnecessary calculations
* Created supportsGroup() test for IK solvers
* from ros-planning/more-travis-tests
  More Travis test fixes.
* Commented out failing test.
  run_tests_moveit_ros_perception requires glut library, and thus a video card or X server, but I haven't had any luck making such things work on Travis.
* avoid unnecessary calculations
  If we are not going to use the missing vector then we should not create it
  (avoid an expensive operation).
* Code cleanup
* Allow joint model group to have use IK solvers with multiple tip frames
* Authorship
* Fixed missing removeSlash to setValues()
* Feedback and cleaned up comment lengths
* Cleaned up commit
* KinematicsBase support for multiple tip frames and IK requests with multiple poses
* More Travis test fixes.
  Switched test_constraint_samplers.cpp from build-time to run-time reference to moveit_resources.
  Added passing run_tests_moveit_core_gtest_test_robot_state_complex test to .travis.yml.
  Added 'make tests' to .travis.yml to make all tests, even failing ones.
* Contributors: Acorn Pooley, Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Dave Hershberger, Martin Szarski, Michael Ferguson, Sachin Chitta, hersh, sachinc

0.5.8 (2014-03-03)
------------------
* Dix bad includes after upstream catkin fix
* update how we find eigen: this is needed for indigo
* Contributors: Ioan A Sucan, Dirk Thomas, Vincent Rabaud

0.5.7 (2014-02-27)
------------------
* Constraint samplers bug fix and improvements
* fix for reverting PR ros-planning:moveit_core#148
* Fix joint variable location segfault
* Better enforce is_valid as a flag that indicated proper configuration has been completed, added comments and warning
* Fix fcl dependency in CMakeLists.txt
* Fixed asymmetry between planning scene read and write.
* Improved error output for state conversion
* Added doxygen for RobotState::attachBody() warning of danger.
* Improved error output for state converstion
* Debug and documentation
* Added new virtual getName() function to constraint samplers
* Made getName() const with static variable
* KinematicsMetrics crashes when called with non-chain groups.
* Added prefixes to debug messages
* Documentation / comments
* Fixed asymmetry between planning scene read and write.
* Added new virtual getName function to constraint samplers for easier debugging and plugin management
* KinematicsMetrics no longer crashes when called with non-chain groups.
* Added doxygen for RobotState::attachBody() warning of danger.
* resolve full path of fcl library
  Because it seems to be common practice to ignore ${catkin_LIBRARY_DIRS}
  it's more easy to resolve the full library path here instead.
* Fix fcl dependency in CMakeLists.txt
  See http://answers.ros.org/question/80936 for details
  Interestingly collision_detection_fcl already uses the correct
  variable ${LIBFCL_LIBRARIES} although it wasn't even set before
* Contributors: Dave Coleman, Dave Hershberger, Ioan A Sucan, Sachin Chitta, sachinc, v4hn

0.5.6 (2014-02-06)
------------------
* fix mix-up comments, use getCollisionRobotUnpadded() since this function is checkCollisionUnpadded.
* Updated tests to new run-time usage of moveit_resources.
* robot_state: comment meaning of default
* Trying again to fix broken tests.
* document RobotState methods
* transforms: clarify comment
* Fixed build of test which depends on moveit_resources.
* Removed debug-write in CMakeLists.txt.
* Added running of currently passing tests to .travis.yml.
* Add kinematic options when planning for CartesianPath
* -Fix kinematic options not getting forwarded, which can lead to undesired behavior in some cases
* Added clarifying doxygen to collision_detection::World::Object.

0.5.5 (2013-12-03)
------------------
* Fix for computing jacobian when the root_joint is not an active joint.
* RobotState: added doxygen comments clarifying action of attachBody().
* Always check for dirty links.
* Update email addresses.
* Robot_state: fix copy size bug.
* Corrected maintainer email.
* Fixed duration in robottrajectory.swap.
* Fixing distance field bugs.
* Compute associated transforms bug fixed.
* Fixing broken tests for changes in robot_state.
* Fixed doxygen function-grouping.
* Fix `ros-planning:moveit_core#95 <https://github.com/ros-planning/moveit_core/issues/95>`_.
* More docs for RobotState.

0.5.4 (2013-10-11)
------------------
* Add functionality for enforcing velocity limits; update API to better naming to cleanly support the new additions
* Adding Travis Continuous Integration to MoveIt
* remember if a group could be a parent of an eef, even if it is not the default one

0.5.3 (2013-09-25)
------------------
* remove use of flat_map

0.5.2 (2013-09-23)
------------------
* Rewrite RobotState and significantly update RobotModel; lots of optimizations
* add support for diffs in RobotState
* fix `ros-planning:moveit_core#87 <https://github.com/ros-planning/moveit_core/issues/87>`_
* add non-const variants for getRobotMarkers
* use trajectory_msgs::JointTrajectory for object attach information instead of sensor_msgs::JointState
* add effort to robot state
* do not include mimic joints or fixed joints in the set of joints in a robot trajectory
* voxel_grid: finish adding Eigen accessors
* voxel_grid: add Eigen accessors
* eliminate determineCollisionPoints() and distance_field_common.h
* propagation_distance_field: make getNearestCell() work with max_dist cells
* distance_field: fix bug in adding shapes
* propagation_distance_field: add getNearestCell()

0.5.1 (2013-08-13)
------------------
* remove CollisionMap message, allow no link name in for AttachedCollisionObject REMOVE operations
* make headers and author definitions aligned the same way; white space fixes
* move background_processing lib to core
* enable RTTI for CollisionRequest
* added ability to find attached objects for a group
* add function for getting contact pairs

0.5.0 (2013-07-15)
------------------
* move msgs to common_msgs

0.4.7 (2013-07-12)
------------------
* doc updates
* white space fixes (tabs are now spaces)
* update root joint if needed, after doing backward fk
* adding options struct to kinematics base
* expose a planning context in the planning_interface base library

0.4.6 (2013-07-03)
------------------
* Added ability to change planner configurations in the interface
* add docs for controller manager
* fix computeTransformBackward()

0.4.5 (2013-06-26)
------------------
* add computeBackwardTransform()
* bugfixes for voxel_grid, distance_field
* slight improvements to profiler
* Fixes compile failures on OS X with clang
* minor speedup in construction of RobotState
* fix time parametrization crash due to joints that have ros-planning:moveit_core#variables!=1
* remove re-parenting of URDF models feature (we can do it cleaner in a different way)

0.4.4 (2013-06-03)
------------------
* fixes for hydro
* be careful about when to add a / in front of the frame name

0.4.3 (2013-05-31)
------------------
* remove distinction of loaded and active controllers

0.4.2 (2013-05-29)
------------------
* generate header with version information

0.4.1 (2013-05-27)
------------------
* fix `ros-planning:moveit_core#66 <https://github.com/ros-planning/moveit_core/issues/66>`_
* rename getTransforms() to copyTransforms()
* refactor how we deal with frames; add a separate library
* remove direction from CollisionResult

0.4.0 (2013-05-23)
------------------
* attempt to fix `ros-planning:moveit_core#241 <https://github.com/ros-planning/moveit_core/issues/241>`_ from moveit_ros
* update paths so that files are found in the globally installed moveit_resources package
* remove magical 0.2 and use of velocity_map
* Work on issue `ros-planning:moveit_core#35 <https://github.com/ros-planning/moveit_core/issues/35>`_.

0.3.19 (2013-05-02)
-------------------
* rename getAttachPosture to getDetachPosture
* add support for attachment postures and implement MOVE operation for CollisionObject
* add ability to fill in planning scene messages by component
* when projection from start state fails for IK samplers, try random states
* bugfixes

0.3.18 (2013-04-17)
-------------------
* allow non-const access to kinematic solver
* bugfix: always update variable transform

0.3.17 (2013-04-16)
-------------------
* bugfixes
* add console colors
* add class fwd macro
* cleanup API of trajectory lookup
* Added method to get joint type as string
* fixing the way mimic joints are updated
* fixed tests

0.3.16 (2013-03-15)
-------------------
* bugfixes
* robot_state::getFrameTransform now returns a ref instead of a pointer; fixed a bug in transforming Vector3 with robot_state::Transforms, add planning_scene::getFrameTransform
* add profiler tool (from ompl)

0.3.15 (2013-03-08)
-------------------
* Remove configure from PlanningScene
* return shared_ptr from getObject() (was ref to shared_ptr)
* use NonConst suffix on PlanningScene non-const get functions.
* make setActiveCollisionDetector(string) return bool status
* use CollisionDetectorAllocator in PlanningScene
* add World class
* bodies attached to the same link should not collide
* include velocities in conversions
* Added more general computeCartesianPath, takes vector of waypoints
* efficiency improvements

0.3.14 (2013-02-05)
-------------------
* initialize controller state by default
* fix `ros-planning:moveit_core#157 <https://github.com/ros-planning/moveit_core/issues/157>`_ in moveit_ros
* fix moveit_ros/`ros-planning:moveit_core#152 <https://github.com/ros-planning/moveit_core/issues/152>`_

0.3.13 (2013-02-04 23:25)
-------------------------
* add a means to get the names of the known states (as saved in SRDF)
* removed kinematics planner

0.3.12 (2013-02-04 13:16)
-------------------------
* Adding comments to voxel grid
* Adding in octree constructor and some additional fields and tests
* Getting rid of obstacle_voxel set as it just slows things down
* Removing pf_distance stuff, adding some more performance, getting rid of addCollisionMapToField function
* Fixing some bugs for signed distance field and improving tests
* Merging signed functionality into PropagateDistanceField, adding remove capabilities, and adding a few comments and extra tests

0.3.11 (2013-02-02)
-------------------
* rename KinematicState to RobotState, KinematicTrajectory to RobotTrajectory
* remove warnings about deprecated functions, use a deque instead of vector to represent kinematic trajectories

0.3.10 (2013-01-28)
-------------------
* fix `ros-planning:moveit_core#28 <https://github.com/ros-planning/moveit_core/issues/28>`_
* improves implementation of metaball normal refinement for octomap
* add heuristic to detect jumps in joint-space distance
* make it such that when an end effector is looked up by group name OR end effector name, things work as expected
* removed urdf and srdf from configure function since kinematic model is also passed in
* make sure decoupling of scenes from parents that are themselves diffs to other scenes actually works
* Fix KinematicState::printStateInfo to actually print to the ostream given.
* add option to specify whether the reference frame should be global or not when computing Cartesian paths
* update API for trajectory smoother
* add interpolation function that takes joint velocities into account, generalize setDiffFromIK
* add option to reverse trajectories
* add computeCartesianPath()
* add ability to load & save scene geometry as text
* compute jacobian with kdl
* fix `ros-planning:moveit_core#15 <https://github.com/ros-planning/moveit_core/issues/15>`_

0.3.9 (2013-01-05)
------------------
* adding logError when kinematics solver not instantiated, also changing @class
* move some functions to a anonymous namespace
* add doc for kinematic_state ns

0.3.8 (2013-01-03)
------------------
* add one more CATKIN dep

0.3.7 (2012-12-31)
------------------
* add capabilities related to reasoning about end-effectors

0.3.6 (2012-12-20)
------------------
* add ability to specify external sampling constraints for constraint samplers

0.3.5 (2012-12-19 01:40)
------------------------
* fix build system

0.3.4 (2012-12-19 01:32)
------------------------
* add notion of default number of IK attempts
* added ability to use IK constraints in sampling with IK samplers
* fixing service request to take proper group name, check for collisions
* make setFromIK() more robust

0.3.3 (2012-12-09)
------------------
* adding capability for constraint aware kinematics + consistency limits to joint state group
* changing the way consistency limits are specified
* speed up implementation of infinityNormDistance()
* adding distance functions and more functions to sample near by
* remove the notion of PlannerCapabilities

0.3.2 (2012-12-04)
------------------
* robustness checks + re-enabe support for octomaps
* adding a bunch of functions to sample near by

0.3.1 (2012-12-03)
------------------
* update debug messages for dealing with attached bodies, rely on the conversion functions more
* changing manipulability calculations
* adding docs
* log error if joint model group not found
* cleaning up code, adding direct access api for better efficiency

0.3.0 (2012-11-30)
------------------
* added a helper function

0.2.12 (2012-11-29)
-------------------
* fixing payload computations
* Changing pr2_arm_kinematics test plugin for new kinematics_base changes
* Finished updating docs, adding tests, and making some small changes to the function of UnionConstraintSampler and ConstraintSamplerManager
* Some extra logic for making sure that a set of joint constraints has coverage for all joints, and some extra tests and docs for constraint sampler manager
* adding ik constraint sampler tests back in, and modifying dependencies such that everything builds
* Changing the behavior of default_constraint_sampler JointConstraintSampler to support detecting conflicting constraints or one constraint that narrows another value, and adding a new struct for holding data.  Also making kinematic_constraint ok with values that are within 2*epsilon of the limits

0.2.11 (2012-11-28)
-------------------
* update kinematics::KinematicBase API and add the option to pass constraints to setFromIK() in KinematicState

0.2.10 (2012-11-25)
-------------------
* minor reorganization of code
* fix `ros-planning:moveit_core#10 <https://github.com/ros-planning/moveit_core/issues/10>`_

0.2.9 (2012-11-23)
------------------
* minor bugfix

0.2.8 (2012-11-21)
------------------
* removing deprecated functions

0.2.7 (2012-11-19)
------------------
* moving sensor_manager and controller_manager from moveit_ros

0.2.6 (2012-11-16 14:19)
------------------------
* reorder includes
* add group name option to collision checking via planning scene functions

0.2.5 (2012-11-14)
------------------
* update DEPENDS
* robustness checks

0.2.4 (2012-11-12)
------------------
* add setVariableBounds()
* read information about passive joints from srdf

0.2.3 (2012-11-08)
------------------
* using srdf info for `ros-planning:moveit_core#6 <https://github.com/ros-planning/moveit_core/issues/6>`_
* fix `ros-planning:moveit_core#6 <https://github.com/ros-planning/moveit_core/issues/6>`_

0.2.2 (2012-11-07)
------------------
* add processPlanningSceneWorldMsg()
* Adding and fixing tests
* Adding docs
* moves refineNormals to new file in collision_detection
* Fixed bugs in PositionConstraint, documented Position and Orientation constraint, extended tests for Position and OrientationConstraint and started working on tests for VisibilityConstraint
* more robust checking of joint names in joint constraints
* adds smoothing to octomap normals; needs better testing

0.2.1 (2012-11-06)
------------------
* revert some of the install location changes

0.2.0 (2012-11-05)
------------------
* update install target locations

0.1.19 (2012-11-02)
-------------------
* add dep on kdl_parser

0.1.18 (2012-11-01)
-------------------
* add kinematics_metrics & dynamics_solver to build process

0.1.17 (2012-10-27 18:48)
-------------------------
* fix DEPENDS libs

0.1.16 (2012-10-27 16:14)
-------------------------
* more robust checking of joint names in joint constraints
* KinematicModel and KinematicState are independent; need to deal with transforms and conversions next

0.1.15 (2012-10-22)
-------------------
* moving all headers under include/moveit/ and using console_bridge instead of rosconsole

0.1.14 (2012-10-20 11:20)
-------------------------
* fix typo

0.1.13 (2012-10-20 10:51)
-------------------------
* removing no longer needed deps
* add ``moveit_`` prefix for all generated libs

0.1.12 (2012-10-18)
-------------------
* porting to new build system
* moved some libraries to moveit_planners
* add access to URDF and SRDF in planning_models
* Adding in path constraints for validating states, needs more testing

0.1.11 (2012-09-20 12:55)
-------------------------
* update conversion functions for kinematic states to support attached bodies

0.1.10 (2012-09-20 10:34)
-------------------------
* making JointConstraints + their samplers work with local variables for multi_dof joints
* Remove fast time parameterization and zero out waypoint times
* setting correct error codes
* bugfixes
* changing the way subgroups are interpreted

0.1.9 (2012-09-14)
------------------
* bugfixes

0.1.8 (2012-09-12 20:56)
------------------------
* bugfixes

0.1.7 (2012-09-12 18:56)
------------------------
* bugfixes

0.1.6 (2012-09-12 18:39)
------------------------
* add install targets, fix some warnings and errors

0.1.5 (2012-09-12 17:25)
------------------------
* first release
