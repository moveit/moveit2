^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.11.0 (2024-09-16)
-------------------
* Add python bindings for saving and loading geometry from a .scene file (`#2971 <https://github.com/moveit/moveit2/issues/2971>`_)
* Add namespace to MoveitPy (`#2884 <https://github.com/moveit/moveit2/issues/2884>`_)
* New planning scene message (`#2885 <https://github.com/moveit/moveit2/issues/2885>`_)
* Contributors: Abhiroop Bhavsar, Bilal Gill, Jens Vanhooydonck

2.10.0 (2024-06-13)
-------------------
* Migrate ros-planning org to moveit (`#2847 <https://github.com/moveit/moveit2/issues/2847>`_)
  * Rename github.com/ros-planning -> github.com/moveit
  * Rename ros-planning.github.io -> moveit.github.io
  * Rename ros-planning organization in docker and CI workflow files
  - ghcr.io/ros-planning -> ghcr.io/moveit
  - github.repository == 'moveit/*''
* Unify log names (`#2720 <https://github.com/moveit/moveit2/issues/2720>`_)
  Co-authored-by: Abishalini Sivaraman <abi.gpuram@gmail.com>
* Get configuration values of traj_exec_man (`#2702 <https://github.com/moveit/moveit2/issues/2702>`_)
  * (ros_planning) get configuration values of traj_exec_man
  * (py) get configuration values of traj_exec_man
* CMake format and lint in pre-commit (`#2683 <https://github.com/moveit/moveit2/issues/2683>`_)
* log after rclcpp init
* Contributors: Henning Kayser, Matthijs van der Burgh, Robert Haschke, Sebastian Jahr, Tyler Weaver, peterdavidfagan

2.9.0 (2024-01-09)
------------------
* [PSM] Process collision object color when adding object trough the planning scene monitor (`#2567 <https://github.com/ros-planning/moveit2/issues/2567>`_)
  * Added an optional Collision Object color object to set the coller of the collision object when adding the collision object trough the PSM.
  * Fixes for clang-tidy warnings
  * fix pre-commit
  * Pass by reference
* Node logging for the rest of MoveIt (`#2599 <https://github.com/ros-planning/moveit2/issues/2599>`_)
* Fix moveit_py Policy docs build (`#2584 <https://github.com/ros-planning/moveit2/issues/2584>`_)
* init policy class (`#2494 <https://github.com/ros-planning/moveit2/issues/2494>`_)
  update command interfaces
  Update moveit_py/moveit/policies/policy.py
  Co-authored-by: Sebastian Castro <4603398+sea-bass@users.noreply.github.com>
  Update moveit_py/moveit/policies/policy.py
  Co-authored-by: Sebastian Castro <4603398+sea-bass@users.noreply.github.com>
  Update moveit_py/moveit/policies/policy.py
  Co-authored-by: Sebastian Castro <4603398+sea-bass@users.noreply.github.com>
  Update moveit_py/moveit/policies/policy.py
  Co-authored-by: Sebastian Castro <4603398+sea-bass@users.noreply.github.com>
  Update moveit_py/moveit/policies/policy.py
  Co-authored-by: Sebastian Castro <4603398+sea-bass@users.noreply.github.com>
  Update moveit_py/moveit/policies/policy.py
  Co-authored-by: Sebastian Castro <4603398+sea-bass@users.noreply.github.com>
  update script description
* (moveit_py) Extend Trajectory Execution Manager (`#2569 <https://github.com/ros-planning/moveit2/issues/2569>`_)
  * (moveit_py) Extend Trajectory Execution Manager
  Added part of the functions from `#2442 <https://github.com/ros-planning/moveit2/issues/2442>`_
  * PR-remarks
  * Update moveit_py/src/moveit/moveit_ros/trajectory_execution_manager/trajectory_execution_manager.cpp
  Co-authored-by: Matthijs van der Burgh <matthijs.vander.burgh@live.nl>
  * Update moveit_py/src/moveit/moveit_ros/trajectory_execution_manager/trajectory_execution_manager.cpp
  Co-authored-by: Matthijs van der Burgh <matthijs.vander.burgh@live.nl>
  * Update planning.pyi - Removed unused import
  * Fixes whitespace issues
  ---------
  Co-authored-by: Matthijs van der Burgh <matthijs.vander.burgh@live.nl>
* (moveit_py) execute needs a `gil_scoped_release` (`#2573 <https://github.com/ros-planning/moveit2/issues/2573>`_)
* Fix trajectory execution manager comments for docs builds (`#2563 <https://github.com/ros-planning/moveit2/issues/2563>`_)
* [PSM] Add proccess Collision Object to PSM and request planning scene to moveit py to allow syncing of mutliple PSM (`#2536 <https://github.com/ros-planning/moveit2/issues/2536>`_)
  * PlanningSceneMonitor and request planning scene to moveit py to allow syncing of multiple planning scene monitors
  * pre-commit fixes
  * Update moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp
  First catch empty scene to not have a unneeded indents.
  Co-authored-by: Sebastian Jahr <sebastian.jahr@tuta.io>
  * Removed unneeded callback functions
  ---------
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Allow editing allowed collision matrix in python + fix get_entry function (`#2551 <https://github.com/ros-planning/moveit2/issues/2551>`_)
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Update README.md (`#2552 <https://github.com/ros-planning/moveit2/issues/2552>`_)
  replace moveit wiht moveit_py
* (moveit_py) fix pyi files (`#2526 <https://github.com/ros-planning/moveit2/issues/2526>`_)
  * (moveit_py) fix planning.pyi
  * (moveit_py) add missing functions to robot_trajectory.pyi
  * (moveit_py) fix command to generate stubs
* (moveit_py) Add Trajectory Execution Manager (`#2406 <https://github.com/ros-planning/moveit2/issues/2406>`_)
  * (moveit_py) add trajectory execution manager
  * (moveit_py) add __bool_\_ to ExecutionStatus
  * (moveit_py) Update copyright header of changed files
  * (moveit_py) add comment referencing issue
  * Rename init_trajectory_execution_manager -> initTrajectoryExecutionManager
  * (moveit_py) python functions snake_case
  * (moveit_py) fix styling
  ---------
* (moveit_py) add update_frame_transforms to planning_scene_monitor (`#2521 <https://github.com/ros-planning/moveit2/issues/2521>`_)
* (moveit_py) remove unused applyPlanningScene (`#2505 <https://github.com/ros-planning/moveit2/issues/2505>`_)
* [moveit_py] add missing constructor of CollisionResult (`#2500 <https://github.com/ros-planning/moveit2/issues/2500>`_)
  Co-authored-by: Dongya Jiang <jiangdongya@xiaoyubot.com>
* Fix wrong rename of set_start_state in 63e0c3a (`#2497 <https://github.com/ros-planning/moveit2/issues/2497>`_)
* Add new clang-tidy style rules (`#2177 <https://github.com/ros-planning/moveit2/issues/2177>`_)
* Finally fix errors building new RobotTrajectory Python bindings docs (`#2481 <https://github.com/ros-planning/moveit2/issues/2481>`_)
  * Add missing parenthesis in Python bindings docstring
  * Fix more docstrings
* [Python] Add Allowed Collision Matrix to planning scene and optional planning scene during planning (`#2387 <https://github.com/ros-planning/moveit2/issues/2387>`_)
* More fixes to Python bindings docstrings (`#2474 <https://github.com/ros-planning/moveit2/issues/2474>`_)
* Fix docstring spacing in newly added trajectory Python bindings (`#2471 <https://github.com/ros-planning/moveit2/issues/2471>`_)
* (moveit_py) node can have multiple param files (`#2393 <https://github.com/ros-planning/moveit2/issues/2393>`_)
  A node is often started with multiple param files. Using `list.index` only returns the first occurrence. The new code searches for all occurrences.
* [Python] Add RetimeTrajectory to RobotTrajectory (`#2411 <https://github.com/ros-planning/moveit2/issues/2411>`_)
  * [Python] Add RetimeTrajectory to RobotTrajectory
  * Split retime trajecotry in multiple functions
  Moved logic to trajectory_tools
  Added Docstrings
  * Removed retime function from python binding
  ---------
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Contributors: Dongya Jiang, Jens Vanhooydonck, Matthijs van der Burgh, Nils-Christian Iseke, Peter David Fagan, Sebastian Castro, Sebastian Jahr, Tyler Weaver

2.8.0 (2023-09-10)
------------------
* Fix moveit_py rclcpp::init() (`#2223 <https://github.com/ros-planning/moveit2/issues/2223>`_)
  * Fix moveit_py rclcpp::init()
  Rclcpp has been initialized without args which was problematic
  for some use cases like clock simulation. Parameters like
  use_sim_time:=true need to be passed to rclcpp, also
  NodeOptions access the global rcl state on construction.
  Co-authored-by: Jafar Uruç <jafar.uruc@gmail.com>
* Export moveit_py_utils' cmake target (`#2207 <https://github.com/ros-planning/moveit2/issues/2207>`_)
* fix typo in name
* Contributors: Henning Kayser, Michael Görner, Robert Haschke

2.7.4 (2023-05-18)
------------------
* Rename named_target_state_values to get_named_target_state_values (`#2181 <https://github.com/ros-planning/moveit2/issues/2181>`_)
* Deprecate MoveItCpp::execute() use of blocking flag (`#1984 <https://github.com/ros-planning/moveit2/issues/1984>`_)
* Add Python binding for link_model_names and get_only_one_end_effector_tip + update stubs (`#1985 <https://github.com/ros-planning/moveit2/issues/1985>`_)
* Contributors: Jafar, Lucas Wendland

2.7.3 (2023-04-24)
------------------

2.7.2 (2023-04-18)
------------------
* Fix Formatting in Python Documentation (`#2085 <https://github.com/ros-planning/moveit2/issues/2085>`_)
  * fix formatting in docs
  * Fix clang-tidy warnings
  ---------
  Co-authored-by: Tyler Weaver <maybe@tylerjw.dev>
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Update moveit_py 'get_planning_scene_monitor' to return NonConst (`#2098 <https://github.com/ros-planning/moveit2/issues/2098>`_)
  Co-authored-by: Jafar <cafer.abdi@gmail.com>
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
* moveit_py citation (`#2029 <https://github.com/ros-planning/moveit2/issues/2029>`_)
* Added set_robot_trajectory_msg to python bindings (`#2050 <https://github.com/ros-planning/moveit2/issues/2050>`_)
* Contributors: Jens Vanhooydonck, Peter David Fagan, Robert Haschke, Sebastian Jahr
