^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pilz_industrial_motion_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.0 (2022-11-10)
------------------
* Use generate_parameter_library to load pilz cartesian limit parameters (`#1577 <https://github.com/ros-planning/moveit2/issues/1577>`_)
* Add joint acceleration validator methods to Pilz limits container (`#1638 <https://github.com/ros-planning/moveit2/issues/1638>`_)
* Use MoveItConfigsBuilder in Pilz test launch file (`#1571 <https://github.com/ros-planning/moveit2/issues/1571>`_)
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* Add planner configurations to CHOMP and PILZ (`#1522 <https://github.com/ros-planning/moveit2/issues/1522>`_)
* Use pragma once as header include guard (`#1525 <https://github.com/ros-planning/moveit2/issues/1525>`_)
* Removed plan_with_sensing (`#1142 <https://github.com/ros-planning/moveit2/issues/1142>`_)
* Remove __has_include statements (`#1481 <https://github.com/ros-planning/moveit2/issues/1481>`_)
* Merge https://github.com/ros-planning/moveit/commit/a63580edd05b01d9480c333645036e5b2b222da9
* Add missing header for std::unique_ptr (`#3180 <https://github.com/ros-planning/moveit2/issues/3180>`_)
* Contributors: Abishalini Sivaraman, J. Javan, Jochen Sprickerhof, Sebastian Castro, Sebastian Jahr, Stephanie Eng, Vatan Aksoy Tezer, abishalini

2.5.3 (2022-07-28)
------------------
* rename header files so debs are installable (`#1443 <https://github.com/ros-planning/moveit2/issues/1443>`_)
* Contributors: Michael Ferguson

2.5.2 (2022-07-18)
------------------
* Rename cartesian_limits.yaml (`#1422 <https://github.com/ros-planning/moveit2/issues/1422>`_)
  Co-authored-by: Tyler Weaver <maybe@tylerjw.dev>
* Merge remote-tracking branch 'origin/main' into feature/msa
* Removing more boost usage (`#1372 <https://github.com/ros-planning/moveit2/issues/1372>`_)
* Merge remote-tracking branch 'upstream/main' into feature/msa
* Removing some boost usage (`#1331 <https://github.com/ros-planning/moveit2/issues/1331>`_)
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Switch to hpp headers of pluginlib
* Merge pull request `#3106 <https://github.com/ros-planning/moveit2/issues/3106>`_ from v4hn/pr-master-bind-them-all / banish bind()
* banish bind()
* Contributors: AndyZe, David V. Lu, Henry Moore, Jafar, Jochen Sprickerhof, Michael Görner, Vatan Aksoy Tezer, v4hn

2.5.1 (2022-05-31)
------------------
* Fix exporting PILZ's move_group capabilities (`#1281 <https://github.com/ros-planning/moveit2/issues/1281>`_)
* Contributors: Jafar

2.5.0 (2022-05-26)
------------------
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
  Adaption of https://github.com/ros-planning/moveit/pull/3106
* Fix double delete in PILZ CIRC generation (`#1229 <https://github.com/ros-planning/moveit2/issues/1229>`_)
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* Use orocos_kdl_vendor package (`#1207 <https://github.com/ros-planning/moveit2/issues/1207>`_)
* Remove new operators (`#1135 <https://github.com/ros-planning/moveit2/issues/1135>`_)
  replace new operator with make_shared
* [moveit_cpp] Fix double param declaration (`#1097 <https://github.com/ros-planning/moveit2/issues/1097>`_)
* Merge https://github.com/ros-planning/moveit/commit/a25515b73d682df03ed3eccd839110c296aa79fc
* Fix missing boost::ref -> std::ref
* Merge https://github.com/ros-planning/moveit/commit/ab42a1d7017b27eb6c353fb29331b2da08ab0039
* Compilation fixes for Jammy and bring back Rolling CI (`#1095 <https://github.com/ros-planning/moveit2/issues/1095>`_)
* Merge https://github.com/ros-planning/moveit/commit/25a63b920adf46f0a747aad92ada70d8afedb3ec
* Merge https://github.com/ros-planning/moveit/commit/0d7462f140e03b4c319fa8cce04a47fe3f650c60
* Avoid downgrading default C++ standard (`#3043 <https://github.com/ros-planning/moveit2/issues/3043>`_)
* Resolve ambiguous function specification (`#3040 <https://github.com/ros-planning/moveit2/issues/3040>`_)
  As Eigen introduced construction from brace-initializers as well, we do need to distinguish between
  void setJointGroupPositions(const JointModelGroup* group, const std::vector<double>&) and
  void setJointGroupPositions(const JointModelGroup* group, const Eigen::VectorXd&)
* Add missing test dependencies (`#1027 <https://github.com/ros-planning/moveit2/issues/1027>`_)
* Add moveit_configs_utils package to simplify loading paramters (`#591 <https://github.com/ros-planning/moveit2/issues/591>`_)
* Merge pr `#3000 <https://github.com/ros-planning/moveit2/issues/3000>`_: Pilz planner: improve reporting of invalid start joints
* pilz: restrict start state check to active group
* pilz: report joint name with invalid limits in start state
  it does not provide enough feedback, is almost trivial and does redundant checks in the single case it's called from.
* Switch to std::bind (`#2967 <https://github.com/ros-planning/moveit2/issues/2967>`_)
* Fix orientation of subframe offset in Pilz planners (`#2890 <https://github.com/ros-planning/moveit2/issues/2890>`_)
  Fix `#2879 <https://github.com/ros-planning/moveit2/issues/2879>`_ by reorienting the subframe offset applied to a goal pose in the PTP planner,
* Merge PRs `#2948 <https://github.com/ros-planning/moveit2/issues/2948>`_ (improve CI) and `#2949 <https://github.com/ros-planning/moveit2/issues/2949>`_ (simplify ROS .test files)
* Remove unused moveit_planning_execution.launch
* Use test_environment.launch in unittests
* Rename launch argument execution_type -> fake_execution_type
  ... to clarify that this parameter is only used for fake controllers
* Pilz unittests: use test_environment.launch
* Merge PR `#2940 <https://github.com/ros-planning/moveit2/issues/2940>`_: Improve error messages of Pilz planner
* Fix typo: demangel -> demangle
* Remove deprecated xacro --inorder
* Fix unittests by providing a valid JMG
* Don't complain about missing limits for irrelevant JMGs
  When planning an arm motion, Pilz's PTP planner shouldn't complain (and bail out)
  on missing joint limits of hand joints!
* Avoid duplicate error messages
* Improve error messages
  - Downgrade ERROR to WARN
  - Report affected joint name
  - Quote (possibly empty) planner id
* Contributors: Abishalini, Gaël Écorchard, Henning Kayser, Jafar, Jafar Abdi, Jochen Sprickerhof, Robert Haschke, Sencer Yazıcı, Tom Noble, Tyler Weaver, Vatan Aksoy Tezer, jeoseo, v4hn

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
