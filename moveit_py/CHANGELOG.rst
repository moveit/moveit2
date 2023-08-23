^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
