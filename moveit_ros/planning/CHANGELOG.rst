^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.13.1 (2025-04-15)
-------------------
* Make the destructors of the base classes of planning adapters virtual and close move_group gracefully (`#3435 <https://github.com/ros-planning/moveit2/issues/3435>`_)
* fix: ensure attached objects update during motion execution (`#3327 <https://github.com/ros-planning/moveit2/issues/3327>`_)
* Planning scene monitor: reliable QoS (`#3400 <https://github.com/ros-planning/moveit2/issues/3400>`_)
* Ports moveit1 `#3689 <https://github.com/ros-planning/moveit/issues/3689>`_ (`#3357 <https://github.com/ros-planning/moveit2/issues/3357>`_)
  * Publish planning scene while planning (`#3689 <https://github.com/ros-planning/moveit/issues/3689>`_)
* fix: explicitly add the same namespace as the parent node (`#3360 <https://github.com/ros-planning/moveit2/issues/3360>`_)
* Contributors: Aleksey Nogin, Cihat Kurtuluş Altıparmak, Kazuya Oguma, Marco Magri, Mark Johnson

2.13.0 (2025-02-15)
-------------------
* Enable allowed_execution_duration_scaling and allowed_goal_duration_margin for each controller (`#3335 <https://github.com/ros-planning/moveit2/issues/3335>`_)
* Add allowed_start_tolerance_joints parameter (Port moveit`#3287 <https://github.com/ros-planning/moveit2/issues/3287>`_) (`#3309 <https://github.com/ros-planning/moveit2/issues/3309>`_)
* load robot_description from other namespace (`#3269 <https://github.com/ros-planning/moveit2/issues/3269>`_)
* Ports moveit `#3676 <https://github.com/ros-planning/moveit2/issues/3676>`_ and `#3682 <https://github.com/ros-planning/moveit2/issues/3682>`_ (`#3283 <https://github.com/ros-planning/moveit2/issues/3283>`_)
* Update includes for generate_parameter_library 0.4.0 (`#3255 <https://github.com/ros-planning/moveit2/issues/3255>`_)
* [moveit_ros] fix race condition when stopping trajectory execution (`#3198 <https://github.com/ros-planning/moveit2/issues/3198>`_)
* move TrajectoryExecutionManager::clear() to private (`#3226 <https://github.com/ros-planning/moveit2/issues/3226>`_)
* Don't destroy objects on attach (`#3205 <https://github.com/ros-planning/moveit2/issues/3205>`_)
* Simplify scene update that does not include new robot_state (`#3206 <https://github.com/ros-planning/moveit2/issues/3206>`_)
* Fix planning_scene_monitor sync when passed empty robot state (`#3187 <https://github.com/ros-planning/moveit2/issues/3187>`_)
* Update deprecated tf2 imports from .h to .hpp (`#3197 <https://github.com/ros-planning/moveit2/issues/3197>`_)
* Fix logic in CheckStartStateBounds adapter (`#3143 <https://github.com/ros-planning/moveit2/issues/3143>`_)
* Contributors: Daniel García López, Dongya Jiang, Mark Johnson, Marq Rasmussen, Michael Görner, RLi43, Robert Haschke, Sebastian Castro

2.12.0 (2024-11-29)
-------------------
* Enhancement/use hpp for headers (`#3113 <https://github.com/ros-planning/moveit2/issues/3113>`_)
* Re-enable flaky PSM test (`#3124 <https://github.com/ros-planning/moveit2/issues/3124>`_)
* Add use_padding flag + deprecate checkCollisionUnpadded() functions (`#3088 <https://github.com/ros-planning/moveit2/issues/3088>`_)
* Remove plugins from export set (`#3024 <https://github.com/ros-planning/moveit2/issues/3024>`_)
* Update urdf/model.h -> urdf/model.hpp (`#3003 <https://github.com/ros-planning/moveit2/issues/3003>`_)
* Contributors: Paul Gesel, Robert Haschke, Sebastian Castro, Sebastian Jahr, Tom Noble

2.11.0 (2024-09-16)
-------------------

2.10.0 (2024-06-13)
-------------------
* Apply clang-tidy fixes
* Migrate ros-planning org to moveit (`#2847 <https://github.com/moveit/moveit2/issues/2847>`_)
  * Rename github.com/ros-planning -> github.com/moveit
  * Rename ros-planning.github.io -> moveit.github.io
  * Rename ros-planning organization in docker and CI workflow files
  - ghcr.io/ros-planning -> ghcr.io/moveit
  - github.repository == 'moveit/*''
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
  ---------
  Co-authored-by: Paul Gesel <paulgesel@gmail.com>
  Co-authored-by: Abishalini Sivaraman <abi.gpuram@gmail.com>
  Co-authored-by: Ezra Brooks <ezra@brooks.cx>
* Skip flaky PSM launch test (`#2822 <https://github.com/moveit/moveit2/issues/2822>`_)
* change default to 1e308 (`#2801 <https://github.com/moveit/moveit2/issues/2801>`_)
* PSM: keep references to scene\_ valid upon receiving full scenes (`#2745 <https://github.com/moveit/moveit2/issues/2745>`_)
  plan_execution-related modules rely on `plan.planning_scene\_` in many places
  to point to the currently monitored scene (or a diff on top of it).
  Before this patch, if the PSM would receive full scenes during execution,
  `plan.planning_scene\_` would not include later incremental updates anymore
  because the monitor created a new diff scene.
  ---------
  Co-authored-by: v4hn <me@v4hn.de>
* Unify log names (`#2720 <https://github.com/moveit/moveit2/issues/2720>`_)
  Co-authored-by: Abishalini Sivaraman <abi.gpuram@gmail.com>
* Print links in collision (`#2727 <https://github.com/moveit/moveit2/issues/2727>`_)
* Do not overwrite the error code in planWithSinglePipeline (`#2723 <https://github.com/moveit/moveit2/issues/2723>`_)
  * Do not overwrite the error code in planWithSinglePipeline
  Return the `MotionPlanResponse` as-is.
  * Do not rely on generatePlan() to set error code
  Do not rely on generatePlan() to set the error code in all cases and
  ensure that the error code is set to FAILURE if `generatePlan()` returns
  false.
  ---------
  Co-authored-by: Gaël Écorchard <gael@km-robotics.cz>
* Get configuration values of traj_exec_man (`#2702 <https://github.com/moveit/moveit2/issues/2702>`_)
  * (ros_planning) get configuration values of traj_exec_man
  * (py) get configuration values of traj_exec_man
  ---------
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Exit earlier on failure in `generatePlan` (`#2726 <https://github.com/moveit/moveit2/issues/2726>`_)
  With this change, the `PlanningPipeline::generatePlan()` exits as soon
  as a failure is detected. Before this, `break` was used to exit the
  current loop of request adapters, planners, or response adapters, but
  the function continued to the next loop. For example, if a planner would
  fail, the response adapters would still be executed.
  Co-authored-by: Gaël Écorchard <gael@km-robotics.cz>
* srdf publisher node (`#2682 <https://github.com/moveit/moveit2/issues/2682>`_)
* CMake format and lint in pre-commit (`#2683 <https://github.com/moveit/moveit2/issues/2683>`_)
* Get robot description from topic in GetUrdfService (`#2681 <https://github.com/moveit/moveit2/issues/2681>`_)
* Shut down PSM publishing before starting to publish on a potentially new topic (`#2680 <https://github.com/moveit/moveit2/issues/2680>`_)
* Contributors: Abishalini Sivaraman, Ezra Brooks, Gaël Écorchard, Henning Kayser, Matthijs van der Burgh, Paul Gesel, Robert Haschke, Sebastian Castro, Sebastian Jahr, Tyler Weaver

2.9.0 (2024-01-09)
------------------
* [PSM] Process collision object color when adding object trough the planning scene monitor (`#2567 <https://github.com/ros-planning/moveit2/issues/2567>`_)
  * Added an optional Collision Object color object to set the coller of the collision object when adding the collision object trough the PSM.
  * Fixes for clang-tidy warnings
  * fix pre-commit
  * Pass by reference
* [Servo] Make listening to octomap updates optional (`#2627 <https://github.com/ros-planning/moveit2/issues/2627>`_)
  * [Servo] Make listening to octomap updates optional
  * Update moveit_ros/moveit_servo/config/panda_simulated_config.yaml
* Node logging for the rest of MoveIt (`#2599 <https://github.com/ros-planning/moveit2/issues/2599>`_)
* [Planning Pipeline Refactoring] `#2 <https://github.com/ros-planning/moveit2/issues/2>`_ Enable chaining planners (`#2457 <https://github.com/ros-planning/moveit2/issues/2457>`_)
  * Enable chaining multiple planners
* Node logging in moveit_core (`#2503 <https://github.com/ros-planning/moveit2/issues/2503>`_)
* Fix trajectory execution manager comments for docs builds (`#2563 <https://github.com/ros-planning/moveit2/issues/2563>`_)
* Change default topic name for display contacts (`#2561 <https://github.com/ros-planning/moveit2/issues/2561>`_)
* [PSM] Add proccess Collision Object to PSM and request planning scene to moveit py to allow syncing of mutliple PSM (`#2536 <https://github.com/ros-planning/moveit2/issues/2536>`_)
  * PlanningSceneMonitor and request planning scene to moveit py to allow syncing of multiple planning scene monitors
  * pre-commit fixes
  * Update moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp
  First catch empty scene to not have a unneeded indents.
  Co-authored-by: Sebastian Jahr <sebastian.jahr@tuta.io>
  * Removed unneeded callback functions
  ---------
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* [Planning Pipeline Refactoring] `#1 <https://github.com/ros-planning/moveit2/issues/1>`_ Simplify Adapter - Planner chain (`#2429 <https://github.com/ros-planning/moveit2/issues/2429>`_)
* Use node logging in moveit_ros (`#2482 <https://github.com/ros-planning/moveit2/issues/2482>`_)
* Add new clang-tidy style rules (`#2177 <https://github.com/ros-planning/moveit2/issues/2177>`_)
* Re-enable waiting for current state in MoveItCpp (`#2419 <https://github.com/ros-planning/moveit2/issues/2419>`_)
* Protect against zero frequency in TrajectoryMonitorMiddlewareHandler (`#2423 <https://github.com/ros-planning/moveit2/issues/2423>`_)
* Small planning pipeline class fixes (`#2416 <https://github.com/ros-planning/moveit2/issues/2416>`_)
* Use generate parameters library in PlanningPipelineClass + general cleanups (`#2288 <https://github.com/ros-planning/moveit2/issues/2288>`_)
  * Don't discard stuff
  * Move constants into source file
  * Move static consts into header
  * Don't ignore pipeline result
  * Use generate parameter library for planning pipeline parameters
  * Fix CI
  * More CI fixes
  * Remove more state from planning pipeline
  * Small cleanups
  * Assert planner_instance\_ is not a nullptr
  * Remove valid variable
  * Simplify logic for trajectory printing
  * More helpful comments
  * Small logic simplification by using break
  * Fix clang-tidy
  * Pre-commit + Deprecate functions instead of removing them
  * Fix CI
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Remove old deprecated functions (`#2384 <https://github.com/ros-planning/moveit2/issues/2384>`_)
* [PSM] Get the parameter values of the main node when declaring them in the private node. (`#2392 <https://github.com/ros-planning/moveit2/issues/2392>`_)
  * Get the values of the main node when declaring them in the private node.
  * [chore] linting
  * Removed logging
  * Update formatting
  * Removed whitespace
  ---------
* Update clang-format-14 with QualifierAlignment (`#2362 <https://github.com/ros-planning/moveit2/issues/2362>`_)
  * Set qualifier order in .clang-format
  * Ran pre-commit to update according to new style guide
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Contributors: Abishalini Sivaraman, Henning Kayser, Jens Vanhooydonck, Marq Rasmussen, Rayene Messaoud, Sebastian Castro, Sebastian Jahr, Shobuj Paul, Tyler Weaver

2.8.0 (2023-09-10)
------------------
* Remove added path index from planner adapter function signature (`#2285 <https://github.com/ros-planning/moveit2/issues/2285>`_)
* Replaced boost::algorithm::join with fmt::join (`#2273 <https://github.com/ros-planning/moveit2/issues/2273>`_)
  * Replaced boost::algorithm::join with fmt::join
  * Made changes in CMakeLists.txt to accomodate fmt
  * Updated package.xml files
  * removed redundant boost dependencies
  * Rename variables -> variable
  ---------
  Co-authored-by: Sebastian Castro <4603398+sea-bass@users.noreply.github.com>
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Replaced numbers with SystemDefaultsQos() (`#2271 <https://github.com/ros-planning/moveit2/issues/2271>`_)
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
* Prefer to use the active controller if multiple controllers apply (`#2251 <https://github.com/ros-planning/moveit2/issues/2251>`_)
* Don't default to random algorithm if no plugin is defined (`#2228 <https://github.com/ros-planning/moveit2/issues/2228>`_)
  * Don't default to random algorithm if no plugin is defined
  * Simplify selection logic & initialize default values in constructor
  * Increase message severity
* Always set response planner id and warn if it is not set (`#2236 <https://github.com/ros-planning/moveit2/issues/2236>`_)
* Suppress redundant error message in CSM (`#2222 <https://github.com/ros-planning/moveit2/issues/2222>`_)
  The CSM would spam the log if /joint_states messages
  includes unkonwn joints. RobotModel::hasJointModel()
  allows for verifying joint names in a safe way without
  the error message.
* Minor cleanup to ros_control_interface and trajectory execution (`#2208 <https://github.com/ros-planning/moveit2/issues/2208>`_)
* Ensure that planning pipeline id is set (`#2202 <https://github.com/ros-planning/moveit2/issues/2202>`_)
* Add @brief descriptions for plan_request_adapters (`#2185 <https://github.com/ros-planning/moveit2/issues/2185>`_)
* Make loggers static or move into anonymous namespace (`#2184 <https://github.com/ros-planning/moveit2/issues/2184>`_)
  * Make loggers static or move into anonymous namespace
  * Update moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp
  * Update moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp
  * Move LOGGER out of class template
* Contributors: Henning Kayser, Sebastian Jahr, Shobuj Paul, Stephanie Eng

2.7.4 (2023-05-18)
------------------
* Update default planning configs to use AddTimeOptimalParameterization (`#2167 <https://github.com/ros-planning/moveit2/issues/2167>`_)
* Deprecate MoveItCpp::execute() use of blocking flag (`#1984 <https://github.com/ros-planning/moveit2/issues/1984>`_)
* Contributors: Anthony Baker, Lucas Wendland

2.7.3 (2023-04-24)
------------------
* Replace check for the ROS_DISTRO env variable with a check for the rclcpp version (`#2135 <https://github.com/ros-planning/moveit2/issues/2135>`_)
* Replace Variable PROJECT_NAME in CMakeLists.txt with the actual name (`#2020 <https://github.com/ros-planning/moveit2/issues/2020>`_)
* Contributors: Jafar, Shobuj Paul

2.7.2 (2023-04-18)
------------------
* Switch from qos_event.hpp to event_handler.hpp (`#2111 <https://github.com/ros-planning/moveit2/issues/2111>`_)
  * Switch from qos_event.hpp to event_handler.hpp
  * moveit_common: Add a cmake interface library to keep humble support on main
  * Include qos_event.hpp or event_handler.hpp depending on the ROS 2 version
  * Fix ament_lint_cmake
  * Fix clang-tidy
  * PRIVATE linking in some cases
  * Update moveit_common/cmake/moveit_package.cmake
  Co-authored-by: Chris Thrasher <chrisjthrasher@gmail.com>
  * Fix servo and cleanup excessive CMake variable usage
  * Cleanup & make compiling
  * Small variable naming and const cleanup
  * Restore OpenCV linking
  * Public/private linking fixup
  * Revert "Restore OpenCV linking"
  This reverts commit 57a9efa806e59223e35a1f7e998d7b52f930c263.
  ---------
  Co-authored-by: JafarAbdi <jafar.uruc@gmail.com>
  Co-authored-by: Jafar <cafer.abdi@gmail.com>
  Co-authored-by: AndyZe <andyz@utexas.edu>
  Co-authored-by: Chris Thrasher <chrisjthrasher@gmail.com>
* Fix MoveItCpp issues (port from MoveIt1) (`#2001 <https://github.com/ros-planning/moveit2/issues/2001>`_)
  * Fix MoveitCpp's const member accessors
  They should return a ConstPtr instead of a const Ptr&!
  * Fix SEVERE ClassLoader warning when releasing MoveItCpp
  - PSM was released before copy of its RobotModel -> removed extra RobotModel copy
  - clearContents() was broken:
  - resets in wrong order: psm\_ should be last
  - trajectory_execution_manager\_ was missing
  I suggest to omit clearContents() and rely on the (correct) ordering of member variables.
  While this is not explicit, we ensure that we don't miss any newly added member variable.
  Fix: https://github.com/ros-planning/moveit2/issues/1597
  ---------
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
  Co-authored-by: Jafar <cafer.abdi@gmail.com>
  Co-authored-by: Sebastian Jahr <sebastian.jahr@tuta.io>
  Co-authored-by: JafarAbdi <jafar.uruc@gmail.com>
* Extract parallel planning from moveit cpp (`#2043 <https://github.com/ros-planning/moveit2/issues/2043>`_)
  * Add parallel_planning_interface
  * Add parallel planning interface
  * Rename package to pipeline_planning_interface
  * Move plan_responses_container into own header + source file
  * Add plan_responses_contrainer source file
  * Add solution selection and stopping criterion function files
  * Remove parallel planning from moveit_cpp
  * Move parallel planning into planning package
  * Update moveit_cpp
  * Drop planning_interface changes
  * Add documentation
  * Update other moveit packages
  * Remove removed header
  * Address CI complains
  * Address clang-tidy complains
  * Address clang-tidy complains 2
  * Address clang-tidy complains 3
  * Extract planning pipeline map creation function from moveit_cpp
  * Cleanup comment
  * Use const moveit::core::RobotModelConstPtr&
  * Formatting
  * Add header descriptions
  * Remove superfluous TODOs
  * Cleanup
* Move displaced launch file into planning_component_tools (`#2044 <https://github.com/ros-planning/moveit2/issues/2044>`_)
* Contributors: Robert Haschke, Sebastian Jahr

2.7.1 (2023-03-23)
------------------
* Fix member naming (`#1949 <https://github.com/ros-planning/moveit2/issues/1949>`_)
  * Update clang-tidy rules for readability-identifier-naming
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Benchmark parallel planning pipelines (`#1539 <https://github.com/ros-planning/moveit2/issues/1539>`_)
  * Remove launch and config files (moved to moveit_resources)
* Merge pull request `#1546 <https://github.com/ros-planning/moveit2/issues/1546>`_ from peterdavidfagan/moveit_py
  Python Bindings - moveit_py
* add new python bindings
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
  Co-authored-by: Michael Gorner <me@v4hn.de>
  Co-authored-by: Robert Haschke <rhaschke@techfak.uni-bielefeld.de>
  Co-authored-by: AndyZe <zelenak@picknik.ai>
  Co-authored-by: Peter Mitrano <mitranopeter@gmail.com>
  Co-authored-by: Sebastian Castro <4603398+sea-bass@users.noreply.github.com>
  Co-authored-by: Jafar <jafar.uruc@gmail.com>
  Co-authored-by: Shahwas Khan <shahwazk@usc.edu>
* moveit_cpp: handle the case where blocking==false (`#1834 <https://github.com/ros-planning/moveit2/issues/1834>`_)
* remove underscore from public member in MotionPlanResponse (`#1939 <https://github.com/ros-planning/moveit2/issues/1939>`_)
  * remove underscore from private members
  * fix more uses of the suffix notation
* Contributors: AlexWebb, AndyZe, Jafar, Robert Haschke, Sebastian Jahr, peterdavidfagan

2.7.0 (2023-01-29)
------------------
* converted characters from string format to character format (`#1881 <https://github.com/ros-planning/moveit2/issues/1881>`_)
* Add a default stopping criterion for parallel planning (`#1876 <https://github.com/ros-planning/moveit2/issues/1876>`_)
  * Add a default callback for parallel planning termination
  * Delete long-deprecated "using"
  * A new translation unit for the new callback
  * inline
* Switch to clang-format-14 (`#1877 <https://github.com/ros-planning/moveit2/issues/1877>`_)
  * Switch to clang-format-14
  * Fix clang-format-14
* Do not allow traj execution from PlanningComponent (`#1835 <https://github.com/ros-planning/moveit2/issues/1835>`_)
  * Do not allow traj execution from PlanningComponent
  * Deprecate, don't delete
  * Get the group_name from RobotTrajectory
  * Rebase
* Add optional list of controllers to MoveItCpp::execute() (`#1838 <https://github.com/ros-planning/moveit2/issues/1838>`_)
  * Add optional list of controllers
  * The default is an empty vector
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
* No default IK solver (`#1816 <https://github.com/ros-planning/moveit2/issues/1816>`_)
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Remove Iterative Spline and Iterative Parabola time-param algorithms (v2) (`#1780 <https://github.com/ros-planning/moveit2/issues/1780>`_)
  * Iterative parabolic parameterization fails for nonzero initial/final conditions
  * Iterative spline parameterization fails, too
  * Delete Iterative Spline & Iterative Parabola algorithms
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
* Cleanup lookup of planning pipelines in MoveItCpp (`#1710 <https://github.com/ros-planning/moveit2/issues/1710>`_)
  * Revert "Add planner configurations to CHOMP and PILZ (`#1522 <https://github.com/ros-planning/moveit2/issues/1522>`_)"
  * Cleanup lookup of planning pipelines
  Remove MoveItCpp::getPlanningPipelineNames(), which was obviously intended initially to provide a planning-group-based filter for all available planning pipelines: A pipeline was discarded for a group, if there were no `planner_configs` defined for that group on the parameter server.
  As pointed out in `#1522 <https://github.com/ros-planning/moveit2/issues/1522>`_, only OMPL actually explicitly declares planner_configs on the parameter server.
  To enable all other pipelines as well (and thus circumventing the original filter mechanism), `#1522 <https://github.com/ros-planning/moveit2/issues/1522>`_ introduced empty dummy planner_configs for all other planners as well (CHOMP + Pilz).
  This, obviously, renders the whole filter mechanism useless. Thus, here we just remove the function getPlanningPipelineNames() and the corresponding member groups_pipelines_map\_.
* Fix clang-tidy issues (`#1706 <https://github.com/ros-planning/moveit2/issues/1706>`_)
  * Blindly apply automatic clang-tidy fixes
  * Exemplarily cleanup a few automatic clang-tidy fixes
  * Clang-tidy fixups
  * Missed const-ref fixups
  * Fix unsupported non-const -> const
  * More fixes
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Contributors: Abhijeet Das Gupta, AndyZe, Chris Thrasher, Christian Henkel, Cory Crean, Henning Kayser, Michael Wiznitzer, Robert Haschke, Sameer Gupta, Tyler Weaver

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
* Fixup for `#1420 <https://github.com/ros-planning/moveit2/issues/1420>`_: Restore constness of generatePlan() (`#1699 <https://github.com/ros-planning/moveit2/issues/1699>`_)
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
* Fixed typo in deprecation warning in ControllerManager (`#1688 <https://github.com/ros-planning/moveit2/issues/1688>`_)
  * fixed typo as suggested by  @AndyZe
  * Update naming
  Co-authored-by: AndyZe <andyz@utexas.edu>
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
* Rename MoveItControllerManager. Add deprecation warning (`#1601 <https://github.com/ros-planning/moveit2/issues/1601>`_)
  * Rename MoveItControllerManager->Ros2ControlManager. Add deprecation warning.
  * Do not rename base class
  * Still allow users to load plugins by the old names
  Co-authored-by: Jafar <jafar.uruc@gmail.com>
* Use generate_parameter_library to load kinematics parameters (`#1568 <https://github.com/ros-planning/moveit2/issues/1568>`_)
* Merge PR `#1553 <https://github.com/ros-planning/moveit2/issues/1553>`_: Improve cmake files
* Use standard exported targets: export\_${PROJECT_NAME} -> ${PROJECT_NAME}Targets
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* Disable flaky test_servo_singularity + test_rdf_integration (`#1530 <https://github.com/ros-planning/moveit2/issues/1530>`_)
* Remove sensor manager (`#1172 <https://github.com/ros-planning/moveit2/issues/1172>`_)
* Removed plan_with_sensing (`#1142 <https://github.com/ros-planning/moveit2/issues/1142>`_)
* Remove __has_include statements (`#1481 <https://github.com/ros-planning/moveit2/issues/1481>`_)
* Merge https://github.com/ros-planning/moveit/commit/a63580edd05b01d9480c333645036e5b2b222da9
* Default destructor for PlanningComponent (`#1470 <https://github.com/ros-planning/moveit2/issues/1470>`_)
* trajectory execution manager: reactivate tests (`#3177 <https://github.com/ros-planning/moveit2/issues/3177>`_)
* Clean up TrajectoryExecutionManager API: remove unused code (`#3178 <https://github.com/ros-planning/moveit2/issues/3178>`_)
  * Clean up unused code
  * Add a comment to MIGRATION.md
  Co-authored-by: Cristian Beltran <cristianbehe@gmail.com>
* Merge PR `#3172 <https://github.com/ros-planning/moveit2/issues/3172>`_: Fix CI
* Load robot_description via planning_context.launch
* TrajectoryExecutionManager: Use local LOGNAME instead of member name\_ (`#3168 <https://github.com/ros-planning/moveit2/issues/3168>`_)
* Contributors: Abhijeet Das Gupta, Abishalini Sivaraman, AndyZe, Robert Haschke, Sebastian Jahr, Stephanie Eng, Tyler Weaver, Vatan Aksoy Tezer, abishalini, cambel

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Merge remote-tracking branch 'origin/main' into feature/msa
* Removing more boost usage (`#1372 <https://github.com/ros-planning/moveit2/issues/1372>`_)
* Merge remote-tracking branch 'upstream/main' into feature/msa
* Removing some boost usage (`#1331 <https://github.com/ros-planning/moveit2/issues/1331>`_)
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Add missing headers
* MoveItCpp: Allow multiple pipelines (`#3131 <https://github.com/ros-planning/moveit2/issues/3131>`_)
  * Fix config of multiple pipelines
  * Simply MoveItCpp::getPlanningPipelineNames()
* Merge pull request `#3106 <https://github.com/ros-planning/moveit2/issues/3106>`_ from v4hn/pr-master-bind-them-all / banish bind()
* banish bind()
* various: prefer objects and references over pointers
* Contributors: David V. Lu, Henry Moore, Jafar, Jochen Sprickerhof, Michael Görner, Vatan Aksoy Tezer, v4hn

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Enable cppcheck (`#1224 <https://github.com/ros-planning/moveit2/issues/1224>`_)
  Co-authored-by: jeoseo <jeongwooseo2012@gmail.com>
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Remove unused includes for boost::bind (`#1220 <https://github.com/ros-planning/moveit2/issues/1220>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
  Adaption of https://github.com/ros-planning/moveit/pull/3106
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* various: prefer object and references over pointers
  source: https://github.com/ros-planning/moveit/pull/3106/commits/1a8e5715e3142a92977ac585031b9dc1871f8718; this commit contains minor changes when compared to the source commit which it is based on, these changes are limited to ensuring compatibility with ROS2.
* Delete an unused variable and a redundant log message (`#1179 <https://github.com/ros-planning/moveit2/issues/1179>`_)
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Fix failing test
* Comment failing rdf integration test
* Merge https://github.com/ros-planning/moveit/commit/424a5b7b8b774424f78346d1e98bf1c9a33f0e78
* Return `ExecutionStatus` from `MoveItCpp::execute()` (`#1147 <https://github.com/ros-planning/moveit2/issues/1147>`_)
  Return an `ExecutionStatus` from `MoveItCpp::execute()`, which is
  convertible to a bool in the caller code.
  This change is forward compatible.
* Set controller status before it is checked on trajectory execution (`#1014 <https://github.com/ros-planning/moveit2/issues/1014>`_)
* Remove new operators (`#1135 <https://github.com/ros-planning/moveit2/issues/1135>`_)
  replace new operator with make_shared
* RDFLoader Broken with Xacro Files (`#1132 <https://github.com/ros-planning/moveit2/issues/1132>`_)
  * A broken RDFLoader test
  * Bugfix: Add space between executable and path (if no arguments)
* Simply MoveItCpp::getPlanningPipelineNames() (`#1114 <https://github.com/ros-planning/moveit2/issues/1114>`_)
* [moveit_cpp] Fix config of multiple pipelines (`#1096 <https://github.com/ros-planning/moveit2/issues/1096>`_)
* Merge https://github.com/ros-planning/moveit/commit/a25515b73d682df03ed3eccd839110c296aa79fc
* Merge https://github.com/ros-planning/moveit/commit/ab42a1d7017b27eb6c353fb29331b2da08ab0039
* Make lockSceneRead() and lockSceneWrite() protected member functions (`#1100 <https://github.com/ros-planning/moveit2/issues/1100>`_)
  * No lock in planning_component.cpp
  * Make lockSceneRead(), lockSceneWrite() protected
  * Add a migration note
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
* Fix mixed-up implementations in TfSubscription creation (`#1073 <https://github.com/ros-planning/moveit2/issues/1073>`_)
  Co-authored-by: Jean-Christophe Ruel <jeanchristophe.ruel@elmec.ca>
* Get parameter on initialize (rebased version of `#893 <https://github.com/ros-planning/moveit2/issues/893>`_) (`#996 <https://github.com/ros-planning/moveit2/issues/996>`_)
  Get parameter `trajectory_execution.execution_duration_monitoring` in
  initialize().
  Co-authored-by: Gaël Écorchard <gael.ecorchard@cvut.cz>
* Misc fixes for time and transforms (`#768 <https://github.com/ros-planning/moveit2/issues/768>`_)
  * Fix setting shape_transform_cache_lookup_wait_time from seconds
  * Fix setting last_update_time from seconds
  * Check the return value of canTransform
* Fix race condition in SynchronizedStringParameter::waitForMessage (`#1050 <https://github.com/ros-planning/moveit2/issues/1050>`_)
  Co-authored-by: Tyler Weaver <squirrel428@protonmail.com>
* 1.1.8
* Delete profiler (`#998 <https://github.com/ros-planning/moveit2/issues/998>`_)
  * Delete profiler and evaluator tools
* Remove unused parameters. (`#1018 <https://github.com/ros-planning/moveit2/issues/1018>`_)
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Fix deprecation warning in moveit_cpp (`#3019 <https://github.com/ros-planning/moveit2/issues/3019>`_)
  Fixup for `#3009 <https://github.com/ros-planning/moveit2/issues/3009>`_.
* 1.1.7
* Move MoveItErrorCode class to moveit_core (`#3009 <https://github.com/ros-planning/moveit2/issues/3009>`_)
  ... reducing code duplication and facilitating re-use
* Merge `#2944 <https://github.com/ros-planning/moveit2/issues/2944>`_: various fixes to the rviz plugins
* Switch to std::bind (`#2967 <https://github.com/ros-planning/moveit2/issues/2967>`_)
  * boost::bind -> std::bind
  grep -rlI --exclude-dir=.git "boost::bind" | xargs sed -i 's/boost::bind/std::bind/g'
  * Convert bind placeholders
  grep -rlI --exclude-dir=.git " _[0-9]" | xargs sed -i 's/ _\([0-9]\)/ std::placeholders::_\1/g'
  * Update bind include header
  grep -rlI --exclude-dir=.git "boost/bind" | xargs sed -i 's#boost/bind.hpp#functional#'
* MoveitCpp - added ability to set path constraints for PlanningComponent. (`#2959 <https://github.com/ros-planning/moveit2/issues/2959>`_)
* RDFLoader: clear buffer before reading content (`#2963 <https://github.com/ros-planning/moveit2/issues/2963>`_)
* 1.1.6
* Reset markers on display_contacts topic after a new planning attempt
* Contributors: Abishalini, AndyZe, Colin Kohler, Cory Crean, David V. Lu!!, Denis Štogl, Gaël Écorchard, Henning Kayser, Jafar, Jafar Abdi, JafarAbdi, Jean-Christophe Ruel, Jeroen, Jochen Sprickerhof, Rick Staa, Robert Haschke, Sencer Yazıcı, Stephanie Eng, Tyler Weaver, Vatan Aksoy Tezer, jeoseo, v4hn

2.4.0 (2022-01-20)
------------------
* Merge https://github.com/ros-planning/moveit/commit/f3ac6070497da90da33551fc1dc3a68938340413
* Replace NULL with nullptr (`#961 <https://github.com/ros-planning/moveit2/issues/961>`_)
  * Fixes `#841 <https://github.com/ros-planning/moveit2/issues/841>`_
* Add jerk to the robot model (`#683 <https://github.com/ros-planning/moveit2/issues/683>`_)
  * Add jerk to the robot model
  * Add joint limit parsing to a unit test
  * Add jerk to computeVariableBoundsMsg and <<, too
* Silent clang-tidy's -Wpotentially-evaluated-expression
  https://stackoverflow.com/questions/46494928/clang-warning-on-expression-side-effects
* moveit_build_options()
  Declare common build options like CMAKE_CXX_STANDARD, CMAKE_BUILD_TYPE,
  and compiler options (namely warning flags) once.
  Each package depending on moveit_core can use these via moveit_build_options().
* Contributors: Abishalini, AndyZe, Robert Haschke, Stephanie Eng

2.3.2 (2021-12-29)
------------------
* Add ros_testsing to moveit_ros_planning for rdf_loader (`#943 <https://github.com/ros-planning/moveit2/issues/943>`_)
* Contributors: Tyler Weaver

2.3.1 (2021-12-23)
------------------
* Replaced C-Style Cast with C++ Style Cast. (`#935 <https://github.com/ros-planning/moveit2/issues/935>`_)
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* Get rid of "std::endl" (`#918 <https://github.com/ros-planning/moveit2/issues/918>`_)
* changed post-increments in loops to preincrements (`#888 <https://github.com/ros-planning/moveit2/issues/888>`_)
* Latched Strings for URDF and SRDF (`#765 <https://github.com/ros-planning/moveit2/issues/765>`_)
* Consider simulated time (`#883 <https://github.com/ros-planning/moveit2/issues/883>`_)
* Make controller management logic more tolerant of missing or late ros2_control nodes (`#792 <https://github.com/ros-planning/moveit2/issues/792>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* Tests for TrajectoryMonitor using dependency injection (`#570 <https://github.com/ros-planning/moveit2/issues/570>`_)
* Update controller_manager_plugin to fix MoveIt-managed controller switching (`#785 <https://github.com/ros-planning/moveit2/issues/785>`_)
* MoveitCpp - path constraints added from PlanningComponent (backport `#752 <https://github.com/ros-planning/moveit2/issues/752>`_) (`#781 <https://github.com/ros-planning/moveit2/issues/781>`_)
* Split CollisionPluginLoader (`#2834 <https://github.com/ros-planning/moveit/issues/2834>`_)
* Bugfix in RDFLoader (`#2806 <https://github.com/ros-planning/moveit/issues/2806>`_)
* Fix obvious typo (`#2787 <https://github.com/ros-planning/moveit/issues/2787>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: Abishalini Sivaraman, Dave Coleman, David V. Lu!!, Felix von Drigalski, Gaël Écorchard, Henning Kayser, Joseph Schornak, Kaustubh, Mathias Lüdtke, Michael Görner, Parthasarathy Bana, Robert Haschke, Sencer Yazıcı, pvanlaar, werner291

2.3.0 (2021-10-08)
------------------
* Make TF buffer & listener in PSM private (`#654 <https://github.com/ros-planning/moveit2/issues/654>`_)
* kinematics_plugin_loader: Revert accidental change in logging level (`#692 <https://github.com/ros-planning/moveit2/issues/692>`_)
* Add Ruckig trajectory_processing plugin (jerk-limited) (`#571 <https://github.com/ros-planning/moveit2/issues/571>`_)
* PlanningSceneMonitor: Fix warning about having two publisher with the same node (`#662 <https://github.com/ros-planning/moveit2/issues/662>`_)
* Port moveit ros control interface to ROS2 (`#545 <https://github.com/ros-planning/moveit2/issues/545>`_)
* OccupancyMapMonitor tests using Dependency Injection (`#569 <https://github.com/ros-planning/moveit2/issues/569>`_)
* Fix reversed check (`#623 <https://github.com/ros-planning/moveit2/issues/623>`_)
* follow_joint_trajectory_controller_handle: publish new multi_dof_trajectory field (`#492 <https://github.com/ros-planning/moveit2/issues/492>`_)
* Fix warnings in Galactic and Rolling (`#598 <https://github.com/ros-planning/moveit2/issues/598>`_)
  * Use __has_includes preprocessor directive for deprecated headers
  * Fix parameter template types
  * Proper initialization of smart pointers, rclcpp::Duration
* Create a transform subscribers to enable virtual joints (`#310 <https://github.com/ros-planning/moveit2/issues/310>`_)
* Minor documentation and cleanup of TOTG plugin (`#584 <https://github.com/ros-planning/moveit2/issues/584>`_)
* Wait for complete state duration fix (`#590 <https://github.com/ros-planning/moveit2/issues/590>`_)
* Fix some format strings (`#587 <https://github.com/ros-planning/moveit2/issues/587>`_)
* Fixes for Windows (`#530 <https://github.com/ros-planning/moveit2/issues/530>`_)
* Tests for CurrentStateMonitor using dependency injection (`#562 <https://github.com/ros-planning/moveit2/issues/562>`_)
* Fix joint's position limits loading (`#553 <https://github.com/ros-planning/moveit2/issues/553>`_)
* Refactors for OccMapTree in PlanningScene (`#2684 <https://github.com/ros-planning/moveit2/issues/2684>`_)
* Move OccMapTree to moveit_core/collision_detection
* Contributors: Akash, AndyZe, Bjar Ne, Henning Kayser, Jafar Abdi, Nathan Brooks, Simon Schmeisser, Tyler Weaver, Vatan Aksoy Tezer, Wyatt Rees, Jack, Dave Coleman,  Joe Schornak, Nisala Kalupahana, Lior Lustgarten, Jorge Nicho

2.2.1 (2021-07-12)
------------------
* Fix unwanted override of URDF joint limit defaults (`#546 <https://github.com/ros-planning/moveit2/issues/546>`_)
* Contributors: Jafar Abdi

2.2.0 (2021-06-30)
------------------
* Fix stopping the TrajectoryExecutionManager's execution (`#506 <https://github.com/ros-planning/moveit2/issues/506>`_)
* Enable Rolling and Galactic CI (`#494 <https://github.com/ros-planning/moveit2/issues/494>`_)
* [sync] MoveIt's master branch up-to https://github.com/ros-planning/moveit/commit/0d0a6a171b3fbea97a0c4f284e13433ba66a4ea4
  * PSM: Don't read padding parameters from private namespace (`#2706 <https://github.com/ros-planning/moveit/issues/2706>`_)
  * MSA: Fix template (max_safe_path_cost) (`#2703 <https://github.com/ros-planning/moveit/issues/2703>`_)
  * CI: Use compiler flag --pedantic (`#2691 <https://github.com/ros-planning/moveit/issues/2691>`_)
  * CI: Fail on warnings (`#2687 <https://github.com/ros-planning/moveit/issues/2687>`_)
  * Refine CSM::haveCompleteState (`#2663 <https://github.com/ros-planning/moveit/issues/2663>`_)
  * Use private namespace instead of child for PlanningPipeline topics (`#2652 <https://github.com/ros-planning/moveit/issues/2652>`_)
  * Print error before returning (`#2639 <https://github.com/ros-planning/moveit/issues/2639>`_)
  * Simplify logic in PSM (`#2632 <https://github.com/ros-planning/moveit/issues/2632>`_)
  * PlanExecution: Correctly handle preempt-requested flag (`#2554 <https://github.com/ros-planning/moveit/issues/2554>`_)
  * Support multiple planning pipelines with MoveGroup via MoveItCpp (`#2127 <https://github.com/ros-planning/moveit/issues/2127>`_)
    * Deprecate namespace moveit::planning_interface in favor of moveit_cpp
  * thread safety in clear octomap & only update geometry (`#2500 <https://github.com/ros-planning/moveit/issues/2500>`_)
* Contributors: Henning Kayser, Jafar Abdi, JafarAbdi, Luc Bettaieb, Martin Günther, Michael Görner, Robert Haschke, Simon Schmeisser, Tyler Weaver, Vatan Aksoy Tezer, v4hn

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
* Contributors: AndyZe

2.1.2 (2021-04-20)
------------------
* Unify PickNik name in copyrights (`#419 <https://github.com/ros-planning/moveit2/issues/419>`_)
* Contributors: Tyler Weaver

2.1.1 (2021-04-12)
------------------
* Declare joint limit parameters (`#408 <https://github.com/ros-planning/moveit2/issues/408>`_)
* Add initialize function for moveit_sensor_manager plugin (`#386 <https://github.com/ros-planning/moveit2/issues/386>`_)
* Eliminate ability to keep multiple collision detectors updated (`#364 <https://github.com/ros-planning/moveit2/issues/364>`_)
* Fix EXPORT install in CMake (`#372 <https://github.com/ros-planning/moveit2/issues/372>`_)
* Sync main branch with MoveIt 1 from previous head https://github.com/ros-planning/moveit/commit/0247ed0027ca9d7f1a7f066e62c80c9ce5dbbb5e up to https://github.com/ros-planning/moveit/commit/74b3e30db2e8683ac17b339cc124675ae52a5114
* [fix] export cmake library install (`#339 <https://github.com/ros-planning/moveit2/issues/339>`_)
* MTC compatibility fixes (`#323 <https://github.com/ros-planning/moveit2/issues/323>`_)
* trajectory_execution_manager: Fix creating duration from double
* current_state_monitor: Fix creating duration from double & converting duration to seconds
* Fix some typos in comments (`#2466 <https://github.com/ros-planning/moveit2/issues/2466>`_)
* Fix repo URLs in package.xml files
* Contributors: AndyZe, Boston Cleek, Henning Kayser, Jafar Abdi, Tyler Weaver, Udbhavbisarya23, Yu Yan

2.1.0 (2020-11-23)
------------------
* [improvement] Planning Scene Monitor Node Executor (`#230 <https://github.com/ros-planning/moveit2/issues/230>`_, `#257 <https://github.com/ros-planning/moveit2/issues/257>`_, `#262 <https://github.com/ros-planning/moveit2/issues/262>`_, `#266 <https://github.com/ros-planning/moveit2/issues/266>`_)
  * Fix PSM private node name
  * Initializes all ros interfaces with the private node
  * Runs timer callback using async single threaded executor
  * Fix duplicate PSM ndes (from `ros-planning/navigation2#1940 <https://github.com/ros-planning/navigation2/issues/1940>`_)
* [improvement] Enable MoveIt fake controller in demo (`#231 <https://github.com/ros-planning/moveit2/issues/231>`_)
* [fix] Interactive markers not visible in motion planning plugin (`#299 <https://github.com/ros-planning/moveit2/issues/299>`_)
* [maint] Remove deprecated namespaces robot_model, robot_state  (`#276 <https://github.com/ros-planning/moveit2/issues/276>`_)
* [maint] Wrap common cmake code in 'moveit_package()' macro (`#285 <https://github.com/ros-planning/moveit2/issues/285>`_)
  * New moveit_package() macro for compile flags, Windows support etc
  * Add package 'moveit_common' as build dependency for moveit_package()
  * Added -Wno-overloaded-virtual compiler flag for moveit_ros_planners_ompl
* [maint] Compilation fixes for macOS (`#271 <https://github.com/ros-planning/moveit2/issues/271>`_)
* [maint] kinematics_base: remove deprecated initialize function (`#232 <https://github.com/ros-planning/moveit2/issues/232>`_)
* [maint] Update to new moveit_resources layout (`#247 <https://github.com/ros-planning/moveit2/issues/247>`_)
* [maint] Cleanup and fix CMakeLists target dependencies (`#226 <https://github.com/ros-planning/moveit2/issues/226>`_, `#228 <https://github.com/ros-planning/moveit2/issues/228>`_)
* [maint] Enable clang-tidy-fix and ament_lint_cmake (`#210 <https://github.com/ros-planning/moveit2/issues/210>`_, `#215 <https://github.com/ros-planning/moveit2/issues/215>`_, `#264 <https://github.com/ros-planning/moveit2/issues/264>`_)
* [ros2-migration] Port MoveGroupInterface and MotionPlanning display to ROS 2 (`#272 <https://github.com/ros-planning/moveit2/issues/272>`_)
  * Add private executor for the internal trajectory_execution_manager node
  * Use private MGI node, cleanup & fixes
* [ros2-migration] Port move_group to ROS 2 (`#217 <https://github.com/ros-planning/moveit2/issues/217>`_)
* [ros2-migration] Port planning_pipeline to ROS 2 (`#75 <https://github.com/ros-planning/moveit2/issues/75>`_)
* Contributors: Adam Pettinger, Edwin Fan, Henning Kayser, Jafar Abdi, Jorge Nicho, Lior Lustgarten, Mark Moll, Tyler Weaver, Yu Yan, anasarrak

2.0.0 (2020-02-17)
------------------
* [fix] Fix double node executor exceptions
* [fix] PlanningSceneMonitor: Fix double executor exception for service call
* [sys] Fix export of moveit_ros_planning dependencies
* [improve] Support passing parameter subnamespace in PlanningPipeline
* [improve] Load planning request adapter parameters from subnamespace
* [fix] Fix parameter lookup: kinematics_plugin_loader
* [fix] Fix moveit_ros_visualization (`#167 <https://github.com/ros-planning/moveit2/issues/167>`_)
* [fix] rdf_loader: Fix parameter lookup
* [port] Port moveit_cpp to ROS 2 (`#163 <https://github.com/ros-planning/moveit2/issues/163>`_)
* [port] Port plan_execution to ROS 2 (`#111 <https://github.com/ros-planning/moveit2/issues/111>`_)
* [fix] trajectory_execution_manager: Make library shared
* [fix] planning_pipeline: Make library shared
* [port] Port planning_components_tools to ROS 2 (`#149 <https://github.com/ros-planning/moveit2/issues/149>`_)
* [port] Port moveit ros visualization to ROS 2 (`#160 <https://github.com/ros-planning/moveit2/issues/160>`_)
* [sys] moveit_ros_planning: Fix export dependencies
* [port] Port moveit_simple_controller_manager to ROS 2 (`#158 <https://github.com/ros-planning/moveit2/issues/158>`_)
* [fix] Fix and compile planning_pipeline (`#162 <https://github.com/ros-planning/moveit2/issues/162>`_)
* [port] Port trajectory_execution_manager to ROS2 (`#110 <https://github.com/ros-planning/moveit2/issues/110>`_)
* [fix] Fix linking issue in planning_scene_monitor (`#161 <https://github.com/ros-planning/moveit2/issues/161>`_)
* [port] Port planning_scene_monitor to ROS2 (`#112 <https://github.com/ros-planning/moveit2/issues/112>`_)
* [sys] Re-enable moveit_ros_planning (`#144 <https://github.com/ros-planning/moveit2/issues/144>`_)
* [sys] Comment moveit_ros_occupancy_map_monitor as depend
* [sys] Upgrade to ROS 2 Eloquent (`#152 <https://github.com/ros-planning/moveit2/issues/152>`_)
* [sys] Fix CMakeLists.txt files for Eloquent
* [port] Port collision_plugin_loader to ROS 2 (`#137 <https://github.com/ros-planning/moveit2/issues/137>`_)
* [improve] Pass node to KinematicsBase initialization (`#145 <https://github.com/ros-planning/moveit2/issues/145>`_)
* [sys] Fix docker images (`#139 <https://github.com/ros-planning/moveit2/issues/139>`_)
* [sys] COLCON_IGNORE moveit_ros_planning
* [port] Port robot_model_loader to ROS2 (`#109 <https://github.com/ros-planning/moveit2/issues/109>`_)
* [port] Port constraint_sampler_manager_loader to ROS2 (`#113 <https://github.com/ros-planning/moveit2/issues/113>`_)
* [port] Port kinematics_plugin_loader to ROS2  (`#107 <https://github.com/ros-planning/moveit2/issues/107>`_)
  * Update CMakeLists.txt
  * Update parameter resolution
  * Update logger
* [port] Port planning_request_adapter_plugins to ROS2 (`#114 <https://github.com/ros-planning/moveit2/issues/114>`_)
* [improve] Initialize parameters from node
* [sys] Export plugin description file in new way
* [fix] Fix warnings
* [sys] Enable planning_request_adapter_plugins for colcon build
* [port] Migrate to ROS 2 Logger
* [sys] Update CMakeLists.txt
* [port] Port planning_request_adapter_plugins to ROS 2
* [port] Port rdf_loader to ROS2 (`#104 <https://github.com/ros-planning/moveit2/issues/104>`_)
* Contributors: Henning Kayser, Jafar Abdi, Robert Haschke, Yu Yan, Víctor Mayoral Vilches,

1.1.1 (2020-10-13)
------------------
* [fix] some clang-tidy issues on Travis (`#2337 <https://github.com/ros-planning/moveit/issues/2337>`_)
* [fix] various issues with Noetic build (`#2327 <https://github.com/ros-planning/moveit/issues/2327>`_)
* [fix] "Clear Octomap" button, disable when no octomap is published (`#2320 <https://github.com/ros-planning/moveit/issues/2320>`_)
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski, Robert Haschke

1.1.0 (2020-09-04)
------------------
* [feature] Use Eigen::Transform::linear() instead of rotation() (`#1964 <https://github.com/ros-planning/moveit/issues/1964>`_)
* [feature] Bullet collision detection (`#1839 <https://github.com/ros-planning/moveit/issues/1839>`_) (`#1504 <https://github.com/ros-planning/moveit/issues/1504>`_)
* [feature] Allow different controllers for execution (`#1832 <https://github.com/ros-planning/moveit/issues/1832>`_)
* [feature] Adding continuous collision detection to Bullet (`#1551 <https://github.com/ros-planning/moveit/issues/1551>`_)
* [feature] plan_execution: refine logging for invalid paths (`#1705 <https://github.com/ros-planning/moveit/issues/1705>`_)
* [feature] Unified Collision Environment Integration (`#1584 <https://github.com/ros-planning/moveit/issues/1584>`_)
* [feature] Allow ROS namespaces for planning request adapters (`#1530 <https://github.com/ros-planning/moveit/issues/1530>`_)
* [feature] Add named frames to CollisionObjects (`#1439 <https://github.com/ros-planning/moveit/issues/1439>`_)
* [feature] get_planning_scene_service: return full scene when nothing was requested (`#1424 <https://github.com/ros-planning/moveit/issues/1424>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Initialize zero dynamics in CurrentStateMonitor (`#1883 <https://github.com/ros-planning/moveit/issues/1883>`_)
* [fix] memory leak (`#1526 <https://github.com/ros-planning/moveit/issues/1526>`_)
* [maint] Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint] partially transition unused test bin to rostest (`#2158 <https://github.com/ros-planning/moveit/issues/2158>`_)
* [maint] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `#2004 <https://github.com/ros-planning/moveit/issues/2004>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Fix usage of panda_moveit_config (`#1904 <https://github.com/ros-planning/moveit/issues/1904>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Adapt cmake for Bullet (`#1744 <https://github.com/ros-planning/moveit/issues/1744>`_)
* [maint] Readme for speed benchmark (`#1648 <https://github.com/ros-planning/moveit/issues/1648>`_)
* [maint] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] Improve variable naming in RobotModelLoader (`#1759 <https://github.com/ros-planning/moveit/issues/1759>`_)
* [maint] Move isEmpty() test functions to moveit_core/utils (`#1627 <https://github.com/ros-planning/moveit/issues/1627>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Ayush Garg, Bianca Homberg, Dave Coleman, Felix von Drigalski, Henning Kayser, Jens P, Jonathan Binney, Markus Vieth, Martin Pecka, Max Krichenbauer, Michael Görner, Robert Haschke, Sean Yen, Simon Schmeisser, Tyler Weaver, Yu, Yan, jschleicher, livanov93, llach

1.0.6 (2020-08-19)
------------------
* [fix]   Fix segfault in PSM::clearOctomap() (`#2193 <https://github.com/ros-planning/moveit/issues/2193>`_)
* [maint] Migrate to clang-format-10
* [maint] Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* Contributors: Henning Kayser, Markus Vieth, Robert Haschke

1.0.5 (2020-07-08)
------------------
* [feature] Trajectory Execution: fix check for start state position (`#2157 <https://github.com/ros-planning/moveit/issues/2157>`_)
* [feature] Improve responsiveness of PlanningSceneDisplay (`#2049 <https://github.com/ros-planning/moveit/issues/2049>`_)
  - PlanningSceneMonitor: increate update frequency from 10Hz to 30Hz
  - send RobotState diff if only position changed
* Contributors: Michael Görner, Robert Haschke, Simon Schmeisser

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [fix]     `CurrentStateMonitor`: Initialize velocity/effort with unset dynamics
* [fix]     Fix spurious warning message (# IK attempts) (`#1876 <https://github.com/ros-planning/moveit/issues/1876>`_)
* [maint]   Move `get_planning_scene` service into `PlanningSceneMonitor` for reusability (`#1854 <https://github.com/ros-planning/moveit/issues/1854>`_)
* [feature] Forward controller names to TrajectoryExecutionManager
* [fix]     Always copy dynamics if enabled in CurrentStateMonitor (`#1676 <https://github.com/ros-planning/moveit/issues/1676>`_)
* [feature] TrajectoryMonitor: zero sampling frequency disables trajectory recording (`#1542 <https://github.com/ros-planning/moveit/issues/1542>`_)
* [feature] Add user warning when planning fails with multiple constraints (`#1443 <https://github.com/ros-planning/moveit/issues/1443>`_)
* [maint]   Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint]   Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint]   Windows build fixes
  * Fix header inclusion and other MSVC build errors (`#1636 <https://github.com/ros-planning/moveit/issues/1636>`_)
  * Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
  * Favor ros::Duration.sleep over sleep. (`#1634 <https://github.com/ros-planning/moveit/issues/1634>`_)
  * Remove GCC extensions (`#1583 <https://github.com/ros-planning/moveit/issues/1583>`_)
  * Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix]     Fix potential memory leak in `RDFLoader` (`#1828 <https://github.com/ros-planning/moveit/issues/1828>`_)
  [maint]   Use smart pointers to avoid explicit new/delete
* [fix]     `TrajectoryExecutionManager`: fix race condition (`#1709 <https://github.com/ros-planning/moveit/issues/1709>`_)
* [fix]     Correctly propagate error if time parameterization fails (`#1562 <https://github.com/ros-planning/moveit/issues/1562>`_)
* [maint]   move `occupancy_map_monitor` into its own package (`#1533 <https://github.com/ros-planning/moveit/issues/1533>`_)
* [feature] `PlanExecution`: return executed trajectory (`#1538 <https://github.com/ros-planning/moveit/issues/1538>`_)
* Contributors: Felix von Drigalski, Henning Kayser, Max Krichenbauer, Michael Görner, Robert Haschke, Sean Yen, Yu, Yan, jschleicher, livanov93, Luca Lach

1.0.2 (2019-06-28)
------------------
* [fix] Removed MessageFilter for /collision_object messages (`#1406 <https://github.com/ros-planning/moveit/issues/1406>`_)
* Contributors: Robert Haschke

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [maintenance] Travis: enable warnings and catkin_lint checker (`#1332 <https://github.com/ros-planning/moveit/issues/1332>`_)
* [improve] Remove (redundant) random seeding and #attempts from RobotState::setFromIK() as the IK solver perform random seeding themselves. `#1288 <https://github.com/ros-planning/moveit/issues/1288>`_
* Contributors: Robert Haschke

0.10.8 (2018-12-24)
-------------------
* [maintenance] RDFLoader / RobotModelLoader: remove TinyXML API (`#1254 <https://github.com/ros-planning/moveit/issues/1254>`_)
* [enhancement] Cmdline tool to print planning scene info (`#1239 <https://github.com/ros-planning/moveit/issues/1239>`_)
* Contributors: Dave Coleman, Robert Haschke

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [fix] Fixed various memory leaks (`#1104 <https://github.com/ros-planning/moveit/issues/1104>`_)
  * KinematicsPluginLoader: only cache the latest instance
  * Use createUniqueInstance()
* [fix] Use correct trajectory_initialization_method parameter (`#1237 <https://github.com/ros-planning/moveit/issues/1237>`_)
* [enhancement] Pass RobotModel to IK, avoiding multiple loading (`#1166 <https://github.com/ros-planning/moveit/issues/1166>`_)
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Use C++14 (`#1146 <https://github.com/ros-planning/moveit/issues/1146>`_)
* [maintenance] Code Cleanup
  * `#1179 <https://github.com/ros-planning/moveit/issues/1179>`_
  * `#1180 <https://github.com/ros-planning/moveit/issues/1180>`_
  * `#1196 <https://github.com/ros-planning/moveit/issues/1196>`_
* [maintenance] Change dynamic reconfigure limits for allowed_goal_duration_margin to 30s (`#993 <https://github.com/ros-planning/moveit/issues/993>`_)
* Contributors: Alex Moriarty, Dave Coleman, Hamal Marino, Michael Görner, Robert Haschke, Stephan

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------
* [fix] Build regression (`#1170 <https://github.com/ros-planning/moveit/issues/1170>`_)
* Contributors: Robert Haschke

0.10.3 (2018-10-29)
-------------------
* [fix] Build regression (`#1134 <https://github.com/ros-planning/moveit/issues/1134>`_)
* Contributors: Robert Haschke

0.10.2 (2018-10-24)
-------------------
* [fix] Chomp package handling issue `#1086 <https://github.com/ros-planning/moveit/issues/1086>`_ that was introduced in `ubi-agni/hotfix-#1012 <https://github.com/ubi-agni/hotfix-/issues/1012>`_
* [fix] PlanningSceneMonitor lock `#1033 <https://github.com/ros-planning/moveit/issues/1033>`_: Fix `#868 <https://github.com/ros-planning/moveit/issues/868>`_ (`#1057 <https://github.com/ros-planning/moveit/issues/1057>`_)
* [fix] CurrentStateMonitor update callback for floating joints to handle non-identity joint origins `#984 <https://github.com/ros-planning/moveit/issues/984>`_
* [fix] Eigen alignment issuses due to missing aligned allocation (`#1039 <https://github.com/ros-planning/moveit/issues/1039>`_)
* [fix] reset moveit_msgs::RobotState.is_diff to false (`#968 <https://github.com/ros-planning/moveit/issues/968>`_) This fixes a regression introduced in `#939 <https://github.com/ros-planning/moveit/issues/939>`_.
* [capability][chomp] Addition of CHOMP planning adapter for optimizing result of other planners (`#1012 <https://github.com/ros-planning/moveit/issues/1012>`_)
* [capability] new dynamic-reconfigure parameter wait_for_trajectory_completion to disable waiting for convergence independently from start-state checking. (`#883 <https://github.com/ros-planning/moveit/issues/883>`_)
* [capability] Option for controller-specific duration parameters (`#785 <https://github.com/ros-planning/moveit/issues/785>`_)
* [enhancement] do not wait for robot convergence, when trajectory_execution_manager finishes with status != SUCCEEDED (`#1011 <https://github.com/ros-planning/moveit/issues/1011>`_)
* [enhancement] allow execution of empty trajectories (`#940 <https://github.com/ros-planning/moveit/issues/940>`_)
* [enhancement] avoid warning spam: "Unable to update multi-DOF joint" (`#935 <https://github.com/ros-planning/moveit/issues/935>`_)
* Contributors: 2scholz, Adrian Zwiener, Kei Okada, Michael Görner, Mohmmad Ayman, Raghavender Sahdev, Robert Haschke, Simon Schmeisser, Xaver Kroischke, mike lautman, srsidd

0.10.1 (2018-05-25)
-------------------
* [fix] explicitly enforce updateSceneWithCurrentState() in waitForCurrentRobotState() (`#824 <https://github.com/ros-planning/moveit/issues/824>`_)
* Support static TFs for multi-DOF joints in CurrentStateMonitor (`#799 <https://github.com/ros-planning/moveit/issues/799>`_)
* support xacro args (`#796 <https://github.com/ros-planning/moveit/issues/796>`_)
* CSM: wait for *active* joint states only (`#792 <https://github.com/ros-planning/moveit/issues/792>`_)
* skip non-actuated joints for execution (`#754 <https://github.com/ros-planning/moveit/issues/754>`_)
* Iterative cubic spline interpolation (`#441 <https://github.com/ros-planning/moveit/issues/441>`_)
* Floating Joint Support in CurrentStateMonitor (`#748 <https://github.com/ros-planning/moveit/issues/748>`_)
* validate multi-dof trajectories before execution (`#713 <https://github.com/ros-planning/moveit/issues/713>`_)
* Contributors: Bruno Brito, Dave Coleman, Ian McMahon, Ken Anderson, Michael Görner, Mikael Arguedas, Robert Haschke

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] Avoid segfault when validating a multidof-only trajectory (`#691 <https://github.com/ros-planning/moveit/issues/691>`_). Fixes `#539 <https://github.com/ros-planning/moveit/issues/539>`_
* [fix] find and link against tinyxml where needed (`#569 <https://github.com/ros-planning/moveit/issues/569>`_)
* [capability] Multi DOF Trajectory only providing translation not velocity (`#555 <https://github.com/ros-planning/moveit/issues/555>`_)
* Contributors: Isaac I.Y. Saito, Michael Görner, Mikael Arguedas, Troy Cordie

0.9.9 (2017-08-06)
------------------
* [fix] Change getCurrentExpectedTrajectory index so collision detection is still performed even if the path timing is not known (`#550 <https://github.com/ros-planning/moveit/issues/550>`_)
* [fix] Support for MultiDoF only trajectories `#553 <https://github.com/ros-planning/moveit/pull/553>`_
* [fix] ros_error macro name (`#544 <https://github.com/ros-planning/moveit/issues/544>`_)
* [fix] check plan size for plan length=0 `#535 <https://github.com/ros-planning/moveit/issues/535>`_
* Contributors: Cyrille Morin, Michael Görner, Mikael Arguedas, Notou, Unknown

0.9.8 (2017-06-21)
------------------
* [fix] Include callback of execution status if trajectory is invalid. (`#524 <https://github.com/ros-planning/moveit/issues/524>`_)
* Contributors: dougsm

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [fix] gcc6 build error (`#471 <https://github.com/ros-planning/moveit/issues/471>`_, `#458 <https://github.com/ros-planning/moveit/issues/458>`_)
* [fix] undefined symbol in planning_scene_monitor (`#463 <https://github.com/ros-planning/moveit/issues/463>`_)
* Contributors: Dmitry Rozhkov, Ruben Burger

0.9.5 (2017-03-08)
------------------
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar

0.9.4 (2017-02-06)
------------------
* [fix] race conditions when updating PlanningScene (`#350 <https://github.com/ros-planning/moveit/issues/350>`_)
* [maintenance] Use static_cast to cast to const. (`#433 <https://github.com/ros-planning/moveit/issues/433>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Maarten de Vries, Robert Haschke

0.9.3 (2016-11-16)
------------------
* [fix] cleanup urdfdom compatibility (`#319 <https://github.com/ros-planning/moveit/issues/319>`_)
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon, Robert Haschke

0.9.2 (2016-11-05)
------------------
* [Capability] compatibility to urdfdom < 0.4 (`#317 <https://github.com/ros-planning/moveit/issues/317>`_)
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman, Robert Haschke

0.6.6 (2016-06-08)
------------------
* Add library moveit_collision_plugin_loader as an exported catkin library (`ros-planning:moveit_ros#678 <https://github.com/ros-planning/moveit_ros/issues/678>`_)
* replaced cmake_modules dependency with eigen
* [jade] eigen3 adjustment
* Fix compilation with C++11.
* Enable optional build against Qt5, use -DUseQt5=On to enable it
* merge indigo-devel changes (PR `ros-planning:moveit_ros#633 <https://github.com/ros-planning/moveit_ros/issues/633>`_ trailing whitespace) into jade-devel
* Removed trailing whitespace from entire repository
* Optional ability to copy velocity and effort to RobotState
* cherry-picked PR `ros-planning:moveit_ros#614 <https://github.com/ros-planning/moveit_ros/issues/614>`_
  fixed segfault on shutdown
* fixed segfault on shutdown
  use of pluginlib's createUnmanagedInstance() is strongly discouraged:
  http://wiki.ros.org/class_loader#Understanding_Loading_and_Unloading
  here, the kinematics plugin libs were unloaded before destruction of corresponding pointers
* Deprecate shape_tools
* CurrentStateMonitor no longer requires hearing mimic joint state values.
* Fix crash due to robot state not getting updated (moveit_ros `ros-planning:moveit_ros#559 <https://github.com/ros-planning/moveit_ros/issues/559>`_)
* Contributors: Dave Coleman, Dave Hershberger, Isaac I.Y. Saito, Levi Armstrong, Maarten de Vries, Robert Haschke, Simon Schmeisser (isys vision), kohlbrecher

0.6.5 (2015-01-24)
------------------
* update maintainers
* perception: adding RAII-based locking for OccMapTree
* perception: adding locks to planning scene monitor
* Add time factor support for iterative_time_parametrization
* Contributors: Jonathan Bohren, Michael Ferguson, kohlbrecher

0.6.4 (2014-12-20)
------------------
* Namespaced "traj_execution" for all trajectory_execution_manager logging
* planning_scene_monitor: add ros parameter for adding a wait-for-transform lookup time
  fixes `ros-planning:moveit_ros#465 <https://github.com/ros-planning/moveit_ros/issues/465>`_
* Contributors: Dave Coleman, Jonathan Bohren

0.6.3 (2014-12-03)
------------------
* add plugin interface for collision detectors
* fix missing return value
* trivial fixes for warnings
* Contributors: Michael Ferguson

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------
* re-add libqt4 dependency (previously came from pcl-all)
* Contributors: Michael Ferguson

0.6.0 (2014-10-27)
------------------
* Removed leadings slash from rosparam for robot padding
* Added move_group capability for clearing octomap.
* Made loading octomap_monitor optional in planning_scene_monitor when using WorldGeometryMonitor
* Contributors: Dave Coleman, Dave Hershberger, Sachin Chitta, ahb

0.5.19 (2014-06-23)
-------------------
* Updated doxygen comment in TrajectoryExecutionManager.
* Added more informative error message text when cant' find controllers.
* robot_model_loader.cpp: added call to KinematicsBase::supportsGroup().
* Fix [-Wreorder] warning.
* Fix broken log & output statements.
  - Address [cppcheck: coutCerrMisusage] and [-Werror=format-extra-args] errors.
  - ROS_ERROR -> ROS_ERROR_NAMED.
  - Print size_t values portably.
* Address [-Wreturn-type] warning.
* Address [cppcheck: postfixOperator] warning.
* Address [cppcheck: duplicateIf] error.
  The same condition was being checked twice, and the same action was being taken.
* Add check for planning scene monitor connection, with 5 sec delay
* Fix for building srv_kinematics_plugin
* New ROS service call-based IK plugin
* Allow planning groups to have more than one tip
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Dave Hershberger

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* update maintainer e-mail
* Namespace a debug message
* Minor non-functional changes to KDL
* Contributors: Dave Coleman, Ioan Sucan

0.5.16 (2014-02-27)
-------------------
* Copy paste error fix
* Contributors: fivef

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------
* remove debug printfs
* planning_scene_monitor: add requestPlanningSceneState()
* planning_scene_monitor: fix race condition
* planning_scene_monitor: fix state update bug
  The rate of state updates is limited to dt_state_update per second.
  When an update arrived it was not processed if another was recently
  processed.  This meant that if a quick sequence of state updates
  arrived and then no updates arrive for a while that the last update(s)
  were not seen until another arrives (which may be much later or
  never). This fixes the bug by periodically checking for pending
  updates and running them if they have been pending longer than
  dt_state_update.
* add default_robot_link_padding/scale, set padding/scale value for each robot link, see https://github.com/ros-planning/moveit_ros/issues/402
* fix LockedPlanningSceneRW docs
  fix the text that was originally copied from another class
  (from LockedPlanningSceneRO)
  it mentioned an incorrect return value type of
  LockedPlanningSceneRW::operator->()
* Contributors: Acorn Pooley, Filip Jares, Kei Okada

0.5.12 (2014-01-03)
-------------------
* Fixed trailing underscores in CHANGELOGs.
* Contributors: Dave Hershberger

0.5.11 (2014-01-03)
-------------------
* planning_scene_monitor: slight code simplification
* planning_scene_monitor: fix scope of local vars
* planning_scene_monitor: fix init bug
  scene_const\_ not set if scene passed to constructor.
* kdl_kinematics_plugin: fix warning
* Contributors: Acorn Pooley

0.5.10 (2013-12-08)
-------------------
* fixing how joint names are filled up, fixed joints were getting included earlier, also resizing consistency limits for when random positions near by function is being called
* Contributors: Sachin Chitta

0.5.9 (2013-12-03)
------------------
* Doxygen: added warnings and details to planning_scene_monitor.h
* correcting maintainer email
* remove duplicate header
* Fixed exported targets
* Fixed dependency issue
* fixing joint limits setup for mimic joints
* implement feature requests
* clear monitored octomap when needed (see `ros-planning:moveit_ros#315 <https://github.com/ros-planning/moveit_ros/issues/315>`_)
* fix the adapter for fixing path constraints for initial states
* fixed computation of dimension\_.
* bugfixes in indexing added states for path adapters
* fixes for mimic joints and redundant joints

0.5.8 (2013-10-11)
------------------
* add executable for publishing scene geometry from text
* Made the goal duration margin and scaling optional rosparameters
* bugfixes

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* fix the event triggered on updating attached objects
* make scene monitor trigger updates only when needed
* fix loading of default params
* port to new RobotState API, new messages
* make sure we do not overwrite attached bodies, when updating the current state
* fix `ros-planning:moveit_ros#308 <https://github.com/ros-planning/moveit_ros/issues/308>`_
* fix `ros-planning:moveit_ros#304 <https://github.com/ros-planning/moveit_ros/issues/304>`_
* fix issue with sending trajectories for passive/mimic/fixed joints
* pass effort along

0.5.4 (2013-08-14)
------------------

* remove CollisionMap, expose topic names in PlanningSceneMonitor, implement detach / attach operations as requested by `ros-planning:moveit_ros#280 <https://github.com/ros-planning/moveit_ros/issues/280>`_
* make headers and author definitions aligned the same way; white space fixes
* move background_processing lib to core
* add option to disable trajectory monitoring

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* minor doc fixes
* add docs for planning pipeline
* cleanup build system
* fixing approximate ik calculation
* white space fixes (tabs are now spaces)
* adding check for approximate solution flag
* adding options struct to kinematics base
* port to new base class for planning_interface (using planning contexts)

0.4.5 (2013-07-03)
------------------
* Namespaced ROS_* log messages for better debug fitering - added 'kdl' namespace
* remove dep
* make searchPositionIK actually const, and thread-safe
* Made debug output look better

0.4.4 (2013-06-26)
------------------
* fix `ros-planning:moveit_ros#210 <https://github.com/ros-planning/moveit_ros/issues/210>`_
* added dynamic reconfigure parameters to allow enabling/disabling of trajectory duration monitoring. fixes `ros-planning:moveit_ros#256 <https://github.com/ros-planning/moveit_ros/issues/256>`_
* add state operations evaluation tool
* warn when time parametrization fails
* moved exceptions headers
