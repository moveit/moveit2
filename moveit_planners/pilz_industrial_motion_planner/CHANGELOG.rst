^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pilz_industrial_motion_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.0 (2022-01-20)
------------------
* Remove 'using namespace' from header files. (`#994 <https://github.com/ros-planning/moveit2/issues/994>`_)
* Fix missing ament_cmake_gtest dependency (`#981 <https://github.com/ros-planning/moveit2/issues/981>`_)
* Remove some Maintainers from Pilz Planner (`#971 <https://github.com/ros-planning/moveit2/issues/971>`_)
* Fix usage of boost placeholder (`#958 <https://github.com/ros-planning/moveit2/issues/958>`_)
* Merge https://github.com/ros-planning/moveit/commit/a0ee2020c4a40d03a48044d71753ed23853a665d
* Remove '-W*' options from cmake files (`#2903 <https://github.com/ros-planning/moveit2/issues/2903>`_)
* Add test for pilz planner with attached objects (`#2878 <https://github.com/ros-planning/moveit2/issues/2878>`_)
  * Add test case for `#2824 <https://github.com/ros-planning/moveit2/issues/2824>`_
  Co-authored-by: Cristian Beltran <cristianbehe@gmail.com>
  Co-authored-by: Joachim Schleicher <joachimsl@gmx.de>
  Co-authored-by: jschleicher <j.schleicher@pilz.de>
* Contributors: Abishalini, Cory Crean, Leroy Rügemer, Tyler Weaver, Wolf Vollprecht, cambel, jschleicher

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Convert to modern include guard `#882 <https://github.com/ros-planning/moveit2/issues/882>`_ (`#891 <https://github.com/ros-planning/moveit2/issues/891>`_)
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* Get rid of "std::endl" (`#918 <https://github.com/ros-planning/moveit2/issues/918>`_)
* changed post-increments in loops to preincrements (`#888 <https://github.com/ros-planning/moveit2/issues/888>`_)
* Consider simulated time (`#883 <https://github.com/ros-planning/moveit2/issues/883>`_)
* Use CallbackGroup for MoveGroupSequenceAction
* PILZ: Build fixups, silence warnings, fix unit tests
* PILZ: Migrate and Restructure test directory
* PILZ: Migrate planner and testutils packages to ROS 2
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Consider attached bodies in Pilz planner `#2773 <https://github.com/ros-planning/moveit/issues/2773>`_ (`#2824 <https://github.com/ros-planning/moveit/issues/2824>`_)
  - Remove convertToRobotTrajectory() and integrate its line of code into setSuccessResponse()
  - Pass the final start_state into setSuccessResponse()
* Fix Pilz planner's collision detection (`#2803 <https://github.com/ros-planning/moveit/issues/2803>`_)
  We need to pass the current PlanningScene down to the actual collision checking methods
* Add planning_pipeline_id to MotionSequence service (`#2755 <https://github.com/ros-planning/moveit/issues/2755>`_)
  * Add planning_pipeline_id to MotionSequence action and service
  * check for empty request
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Improve readability of comment
* Contributors: David V. Lu!!, Felix von Drigalski, Gaël Écorchard, Henning Kayser, Parthasarathy Bana, Robert Haschke, Sebastian Jahr, Sencer Yazıcı, aa-tom, cambel, predystopic-dev, pvanlaar

* [feature] Add Pilz industrial motion planner (`#1893 <https://github.com/tylerjw/moveit/issues/1893>`_)
* Contributors: Pilz GmbH and Co. KG, Christian Henkel, Immanuel Martini, Joachim Schleicher, rfeistenauer
