^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_hybrid_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.9 (2025-04-15)
------------------

2.5.8 (2025-02-09)
------------------
* Fix clang-tidy warnings in hybrid planning package (`#3305 <https://github.com/ros-planning/moveit2/issues/3305>`_)
* Contributors: Sebastian Castro

2.5.7 (2024-12-29)
------------------

2.5.6 (2024-11-17)
------------------

2.5.5 (2023-09-10)
------------------
* Replaced numbers with SystemDefaultsQos() (`#2271 <https://github.com/ros-planning/moveit2/issues/2271>`_) (`#2277 <https://github.com/ros-planning/moveit2/issues/2277>`_)
  (cherry picked from commit 5506dd516a91bc145e462b493668ef8623d43521)
  Co-authored-by: Shobuj Paul <72087882+Shobuj-Paul@users.noreply.github.com>
* Fix clang compiler warnings (backport of `#1712 <https://github.com/ros-planning/moveit2/issues/1712>`_) (`#1896 <https://github.com/ros-planning/moveit2/issues/1896>`_)
  - Fix warning: definition of implicit copy assignment operator is deprecated
  - Fix warning: expression with side effects will be evaluated
  - Fix warning: passing by value
  - Enable -Werror
  - Fix -Wdelete-non-abstract-non-virtual-dtor
  - Fix more clang warnings
  - Modernize gtest: TYPED_TEST_CASE -> TYPED_TEST_SUITE
  - Fix GoogleTestVerification.UninstantiatedTypeParameterizedTestSuite
  - Add default copy/move constructors/assignment operators
  As a user-declared destructor deletes any implicitly-defined move constructor/assignment operator,
  we need to declared them manually. This in turn requires to declare the copy constructor/assignment as well.
  - Explicitly declare overrides
  - Add default constructors as they are not implicitly declared anymore
  - Declare selected classes as final
  - Add noexcept specifier to constructors
  - Fixup gmock/gtest warnings
* Cleanup msg includes: Use C++ instead of C header (backport `#1844 <https://github.com/ros-planning/moveit2/issues/1844>`_)
  * Cleanup msg includes: Use C++ instead of C header
  * Remove obsolete include: moveit_msgs/srv/execute_known_trajectory.hpp
* Use <> for non-local headers (`#1765 <https://github.com/ros-planning/moveit2/issues/1765>`_)
  Unless a header lives in the same or a child directory of the file
  including it, it's recommended to use <> for the #include statement.
  For more information, see the C++ Core Guidelines item SF.12
  https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#sf12-prefer-the-quoted-form-of-include-for-files-relative-to-the-including-file-and-the-angle-bracket-form-everywhere-else
  (cherry picked from commit 7a1f2a101f9aeb8557e8a31656bbe1a6d53b430e)
* Re-enable clang-tidy check `performance-unnecessary-value-param` (backport `#1703 <https://github.com/ros-planning/moveit2/issues/1703>`_)
  * Re-enable clang-tidy check performance-unnecessary-value-param (`#1703 <https://github.com/ros-planning/moveit2/issues/1703>`_)
  * Fix clang-tidy issues (`#1706 <https://github.com/ros-planning/moveit2/issues/1706>`_)
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  Co-authored-by: Robert Haschke <rhaschke@users.noreply.github.com>
* Contributors: Chris Thrasher, Robert Haschke, mergify[bot]

2.5.4 (2022-11-04)
------------------
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_) (`#1555 <https://github.com/ros-planning/moveit2/issues/1555>`_)
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Contributors: mergify[bot]

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Merge remote-tracking branch 'origin/main' into feature/msa
* Launch file cleanup (`#1380 <https://github.com/ros-planning/moveit2/issues/1380>`_)
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* [Hybrid Planning] Improve action cancellation (`#1272 <https://github.com/ros-planning/moveit2/issues/1272>`_)
* Contributors: AndyZe, Jafar, Vatan Aksoy Tezer

2.5.1 (2022-05-31)
------------------
* Remove position controllers from CMake (`#1285 <https://github.com/ros-planning/moveit2/issues/1285>`_)
* Contributors: Vatan Aksoy Tezer

2.5.0 (2022-05-26)
------------------
* Fix hybrid planning launching (`#1271 <https://github.com/ros-planning/moveit2/issues/1271>`_)
* Enable cppcheck (`#1224 <https://github.com/ros-planning/moveit2/issues/1224>`_)
  Co-authored-by: jeoseo <jeongwooseo2012@gmail.com>
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
  Adaption of https://github.com/ros-planning/moveit/pull/3106
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* RCLCPP Upgrade Bugfixes (`#1181 <https://github.com/ros-planning/moveit2/issues/1181>`_)
* Enable rolling / jammy CI (again) (`#1134 <https://github.com/ros-planning/moveit2/issues/1134>`_)
  * Use ros2_control binaries
  * Use output screen instead of explicitly stating stderr
* [hybrid planning] Adjust planning scene locking (`#1087 <https://github.com/ros-planning/moveit2/issues/1087>`_)
  * Create a copy of the planning scene. const robot state.
  * Use LockedPlanningSceneRO over lockSceneRead()
  * Use lambda function
* [Hybrid Planning] configurable planning scene topics (`#1052 <https://github.com/ros-planning/moveit2/issues/1052>`_)
* [hybrid planning] Use a map of expected feedback codes (`#1065 <https://github.com/ros-planning/moveit2/issues/1065>`_)
  * Use a map of expected feedback codes
  * Use a constexpr function instead of unordered_map
  * Don't need this #include
  * Minor function renaming
* Disable hybrid planning test, don't cache ci docker at all and don't cache upstream_ws if repos file is changed (`#1051 <https://github.com/ros-planning/moveit2/issues/1051>`_)
  * Don't cache docker builds
  * Don't cache upstream ws
  * Use new action for getting the latest timestamp .repos file has been edited
  * Debug
  * Fix repos path
  * Disable hybrid planning test
  * Use more verbose name
  Co-authored-by: Tyler Weaver <tylerjw@gmail.com>
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
* [hybrid planning] Delete the pass-through option (`#986 <https://github.com/ros-planning/moveit2/issues/986>`_)
  * Delete the pass-through option
  * Suppress clang warning
  * Handle (waypoint_count < 0) possibility
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
* Remove unused parameters. (`#1018 <https://github.com/ros-planning/moveit2/issues/1018>`_)
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Contributors: AndyZe, Cory Crean, David V. Lu!!, Henning Kayser, Jafar, Vatan Aksoy Tezer, jeoseo, v4hn

2.4.0 (2022-01-20)
------------------
* hybrid_planning: Fix global_planner action name (`#960 <https://github.com/ros-planning/moveit2/issues/960>`_)
* Put hybrid planning actions under a common namespace (`#932 <https://github.com/ros-planning/moveit2/issues/932>`_)
  * Put hybrid planning actions under a common namespace
  * Use ~
  * New pkg for common resources. Does not work.
  * Use inline rather than anonymous namespace
  Co-authored-by: Jafar Abdi <cafer.abdi@gmail.com>
  * Revert "Use inline rather than anonymous namespace"
  This reverts commit 29a7d279776be21f4666c7e0fadeaa6b7ef8debf.
  * Revert "New pkg for common resources. Does not work."
  This reverts commit 68a173baee4b7f8b2c1f74285f96c8e3892c5909.
  * Some progress toward loading common parameters
  Co-authored-by: Jafar Abdi <cafer.abdi@gmail.com>
* Contributors: AndyZe

2.3.2 (2021-12-29)
------------------
* Fix syntax (`#939 <https://github.com/ros-planning/moveit2/issues/939>`_)
  Co-authored-by: AndyZe <zelenak@picknik.ai>
* Contributors: Sebastian Jahr

2.3.1 (2021-12-23)
------------------
* Bump new packages to 2.3.0
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* CMake fix and cleanup abstract class (`#6 <https://github.com/ros-planning/moveit2/issues/6>`_)
  * Install hybrid_planning_demo_node separately to avoid exporting it
  * Fix exported include directory
  * Remove getTargetWayPointIndex() from abstract trajectory operator class
  * Delete unused getTargetWayPointIndex()
* Standardize with moveit_cpp parameters. Fix some parameter loading errors
* Enable reaction to planner failure in the planner logic (`#3 <https://github.com/ros-planning/moveit2/issues/3>`_)
  * Add unsuccessful action Hybrid Planning events and handle them in logic
  * Replace std::once with simple bool variable
  * Remove unneeded variable and update comments
  Don't const& for built-in types
  Co-authored-by: Tyler Weaver <tylerjw@gmail.com>
* Update robot state properly
  Update robot state properly; remove debug prints; clang format
  Check if the local planner is stuck within the forward_traj plugin, not local_planner_component
* Pass GlobalPlanner failing to HybridPlanningManager
* Move common launch elements to a Python file, for easy re-use
  Refactor Global and Local Planner Components into NodeClasses
  Add a simple launch test (`#1 <https://github.com/ros-planning/moveit2/issues/1>`_)
  Try to fix plugin export; add helpful debug when stuck
  Error if global planning fails
  READY and AWAIT_TRAJ states are redundant
  Lock the planning scene as briefly as possible
  Specify joint group when checking waypoint distance
  Implement a reset() function in the local planner
  Detect when the local planner gets stuck
* Add generic global planner plugin, support MotionSequenceRequest (`#585 <https://github.com/ros-planning/moveit2/issues/585>`_)
  Fix hybrid planning include folders (`#675 <https://github.com/ros-planning/moveit2/issues/675>`_)
  Order stuff in the CMakeLists.txt and remove control_box package
  Update README
  Move member initialization to initializer lists
  Remove control_box include dependency
  Replace "loop-run" with "iteration"
  Remove cpp interface class constructors and destructors
  Use joint_state_broadcaster, clean up test node, add execution dependencies
  Use only plugin interface header files and add missing dependencies
  Clean up constructor and destructor definitions
  Declare missing parameter in moveit_planning_pipeline_plugin
  Move rclcpp::Loggers into anonymous namespaces
  Switch CI branches to feature/hybrid_planning
  Update message name
  Remove moveit_msgs from .repos file
  Update github workflows
  Remove note from readme about building from source
  Minor renamings, make reset() noexcept
  Check for vector length of zero
  Load moveit_cpp options with the Options helper. Reduces LOC.
  Set the planning parameters within plan()
  Function renaming
  Authors and descriptions in header files only. New header file for error code interface.
  Update namespacing
  Use default QoS for subscribers
  Better progress comparison
  Add publish_joint_positions and publish_joint_velocities param
  Grammar and other minor nitpicks
  Restore moveit_msgs to .repos, for now
* Refactor local planner plugins (`#447 <https://github.com/ros-planning/moveit2/issues/447>`_)
  * Add reset method for trajectory operator and local constraint sampler
  * Refactor next_waypoint_sampler into simple_sampler
  * Include collision checking to forward_trajectory and remove unneeded plugin
  * Fix formatting and plugin description
  * Update README and add missing planner logic plugin description
  Add TODO to use lifecycle components nodes to trigger initialization
  Return a reaction result instead of bool on react()
  Set invalidation flag to false on reset() in ForwardTrajectory local solver
  Return local planner feedback from trajectory operator function calls
  Fix segfault caused by passing through the global trajectory
  Update comment, unify logging and add missing config parameters
  Update to rolling
* Restructure hybrid_planning package (`#426 <https://github.com/ros-planning/moveit2/issues/426>`_)
  * Add forward_trajectory local solver plugin (`#359 <https://github.com/ros-planning/moveit2/issues/359>`_)
  * Use ros2_control based joint simulation and remove unnecessary comment
  * Update copyrights
  * Restructure hybrid planning package
  * Fix formatting and add missing time stamp in local solver plugin
  * Remove unnecessary logging and param loading
  * Enable different interfaces between local planner and controller
  * Use JointGroupPositionController as demo hardware controller
* Code cleanup & MoveIt best practices (`#351 <https://github.com/ros-planning/moveit2/issues/351>`_)
  * Export missing plugins
  * Use std::chrono_literals
  * Construct smart pointers with std::make\_* instead of 'new'
  * Fixup const-ref in function signatures, reduce copies
  * Improve planning scene locking, robot state processing, controller logic
* Refine local planner component (`#326 <https://github.com/ros-planning/moveit2/issues/326>`_)
  * Make local planner component generic
  * Add next_waypoint_sampler trajectory operator
  * Update hybrid planning test to include collision object
  * Clean up code and fix minor bugs.
  * Update local planner component parameter
  * Add local collision constraint solver
  * Update planning scene topic name and test node
  * Fix bugs and runtime errors in local planner component and it's plugins
  * Add collision object that invalidates global trajectory
  * Add simple "stop in front of collision object" demo
  * Add hybrid planning manager reaction to local planner feedback
  * Fix ament_lint_cmake
  * Ensure that local planner start command and collision event are send once
  * Add simple replanning logic plugin
  * Use current state as start state for global planner
  * Use RobotTrajectory instead of constraint vector describe local problem
  * Add PlanningSceneMonitorPtr to local solver plugin
  * Add local planner frequency parameter
  * Use PID controller to create control outputs for robot controller
  * Add hybrid_planning_manager config file
  * Add more complex test node
  * Update README
  * Reset index in next_waypoint_sampler
  * Use correct isPathValid() interface
  * Rename path_invalidation flag
  * Read planning scene instead of cloning it in local planner
  * Add TODO creator
  * Rename local constraint solver plugin
  * Use read-locked access to the planning scene for collision checking
  * Rename constraint_solver into local_constraint_solver
  * Add missing pointer initialization
* Hybrid planning architecture (`#311 <https://github.com/ros-planning/moveit2/issues/311>`_)
  * Add hybrid_planning architecture skeleton code
  * Add simple state machines to hybrid planning manager and local planner
  * Initial global planner prototype implementation
  * Forward joint_trajectory with local planner
  * Forward hybrid planning motion request to global planner
  * Abstract planner logic from hybrid planning manager by using a plugin
  * Implement single plan execution logic
  * Add test launch files, RViz and demo config
  * Add test for motion_planning_request
* Contributors: AndyZe, David V. Lu!!, Henning Kayser, Jens Vanhooydonck, Sebastian Jahr
