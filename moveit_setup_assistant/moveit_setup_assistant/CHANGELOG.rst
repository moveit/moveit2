^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_setup_assistant
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.0 (2022-11-10)
------------------
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* Contributors: Sebastian Jahr

2.5.3 (2022-07-28)
------------------
* Organize package xml of MSA (`#1454 <https://github.com/ros-planning/moveit2/issues/1454>`_)
* Contributors: Vatan Aksoy Tezer

2.5.2 (2022-07-18)
------------------
* [MSA] Testing Framework for MoveItSetupAssistant (`#1383 <https://github.com/ros-planning/moveit2/issues/1383>`_)
* Merge pull request `#1254 <https://github.com/ros-planning/moveit2/issues/1254>`_ from ros-planning/feature/msa
  MoveIt Setup Assistant - Merge the Feature branch
* Fix compile issues
* Merge remote-tracking branch 'upstream/main' into feature/msa
* [MSA] ros2_control Integration (`#1299 <https://github.com/ros-planning/moveit2/issues/1299>`_)
* [MSA] Workaround to launch files without controllers (`#1275 <https://github.com/ros-planning/moveit2/issues/1275>`_)
* [MSA] PR Feedback
* [MSA] Initial Controllers and Simulation Steps Port (`#1252 <https://github.com/ros-planning/moveit2/issues/1252>`_)
* [MSA] Launch Updates (`#1247 <https://github.com/ros-planning/moveit2/issues/1247>`_)
* [MSA] Merge main into feature/msa (Part II) (`#1240 <https://github.com/ros-planning/moveit2/issues/1240>`_)
* [MSA] Three small edits (`#1203 <https://github.com/ros-planning/moveit2/issues/1203>`_)
* [MSA] Add Setup Step for Generating Launch Files (`#1129 <https://github.com/ros-planning/moveit2/issues/1129>`_)
* [MSA] Fix arg parsing in main (`#1110 <https://github.com/ros-planning/moveit2/issues/1110>`_)
* [MSA] Simplify loading of new SRDF (`#1102 <https://github.com/ros-planning/moveit2/issues/1102>`_)
* [MSA] SRDF Setup (`#1057 <https://github.com/ros-planning/moveit2/issues/1057>`_)
* [MSA] Navigation Widget Fixes (`#1054 <https://github.com/ros-planning/moveit2/issues/1054>`_)
* [MSA] Initial Refactor
* [MSA] Move Files Around (split into multiple packages, change to hpp extension)
* Contributors: David V. Lu!!, Vatan Aksoy Tezer

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
  Adaption of https://github.com/ros-planning/moveit/pull/3106
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* Merge https://github.com/ros-planning/moveit/commit/424a5b7b8b774424f78346d1e98bf1c9a33f0e78
* Merge `#3081 <https://github.com/ros-planning/moveit2/issues/3081>`_: Improve Gazebo-compatible URDF generation in MSA
* Add inertial/origin tag
* Don't overwrite existing attributes
* Merge https://github.com/ros-planning/moveit/commit/ab42a1d7017b27eb6c353fb29331b2da08ab0039
* Move getGazeboCompatibleURDF() from MoveItConfigData to SimulationWidget
  This is a function specific to the SimulationWidget
* Provide button to open original URDF file
* Fix widget layout
* Use focusGiven() + focusLost() to generate and validate Gazebo URDF
* XmlSyntaxHighlighter: allow nested highlighting
* Replace manual highlighting with a SyntaxHighlighter
* Simplify getGazeboCompatibleURDF()
  Use a new utility function uniqueInsert() to avoid code duplication
  when inserting XML elements uniquely.
* getGazeboCompatibleURDF(): Compare original and final XML
  Instead of manually keeping track of changes, compare the two docs.
  This is much more robust.
* getGazeboCompatibleURDF(): Skip catching YAML exceptions
  There is no YAML involved!
* fixup: config_path\_ -> static const CONFIG_PATH
* fixup: Simplify saving
  - new_gazebo_urdf\_ -> save_gazebo_urdf\_
  - directly save content, avoid extra parsing
  - hide overwrite button if doc is empty
  - disable overwrite button if saving is not possible due to xacro
* fixup: Drop hidden_func\_ from ConfigurationFilesWidget
  but (re)create the list of to-be-generated files each time
  entering the widget to allow for dynamic adaption of the file list
* fixup: Improve message boxes
* fixup: avoid code duplication
* Allow (over)writing the Gazebo-compatible URDF
* Use more specific check for correct tag
* static_cast<std::string>(*) -> std::string(*)
* fixup: (slightly) improve comment
* fixup: avoid segfaults if expected XML elements are missing
* fixup: fix variable name: transitions_elements -> transmission_elements
* 1.1.9
* Avoid creating duplicate transmission tags
  Only add Gazebo transmission tags for joints if they are not yet present.
* Fix collisions_updater's set comparison (`#3076 <https://github.com/ros-planning/moveit2/issues/3076>`_)
  Use operator< of std::pair(string,string) for comparing two link pairs.
* Compilation fixes for Jammy and bring back Rolling CI (`#1095 <https://github.com/ros-planning/moveit2/issues/1095>`_)
  * Use jammy dockers and clang-format-12
  * Fix unused depend, and move to python3-lxml
  * add ompl to repos, fix versions and ogre
  * Remove ogre keys
  * Fix boolean node operator
  * Stop building dockers on branch and fix servo null pointer
  * update pre-commit to clang-format-12 and pre-commit fixes
  * clang-format workaround and more pre-commit fixes
* MSA: boost::bind -> std::bind (`#3039 <https://github.com/ros-planning/moveit2/issues/3039>`_)
* Do not automatically load robot description in move_group.launch (`#3065 <https://github.com/ros-planning/moveit2/issues/3065>`_)
  MoveIt should not overwrite a previously uploaded robot description.
  It should only provide it optionally in demo mode.
* Merge https://github.com/ros-planning/moveit/commit/25a63b920adf46f0a747aad92ada70d8afedb3ec
* Merge https://github.com/ros-planning/moveit/commit/0d7462f140e03b4c319fa8cce04a47fe3f650c60
* 1.1.8
* Merge PR `#2938 <https://github.com/ros-planning/moveit2/issues/2938>`_: Rework ACM
  Implement ACM defaults as a fallback instead of an override.
  Based on `ros-planning/srdfdom#97 <https://github.com/ros-planning/srdfdom/issues/97>`_, this allows disabling collisions for specific links/objects by default and re-enabling individual pairs if necessary.
* MSA: Add STOMP + OMPL-CHOMP configs (`#2955 <https://github.com/ros-planning/moveit2/issues/2955>`_)
  - Add stomp planner to MSA
  - Add OMPL-CHOMP planner to MSA
  - Remove obsolete CHOMP parameters
  - Update CHOMP config parameters to match code defaults
  - Create CHOMP config via template (instead of code)
  Co-authored-by: Robert Haschke <rhaschke@techfak.uni-bielefeld.de>
* 1.1.7
* Move MoveItConfigData::setCollisionLinkPairs to collisions_updater.cpp
  This method is only used there to update disabled collision entries.
* Unify initialization of ACM from SRDF
* Adapt to API changes in srdfdom
  @v4hn requested splitting of collision_pairs into (re)enabled and disabled.
* Adapt to API changes in srdfdom
* Merge PR `#3013 <https://github.com/ros-planning/moveit2/issues/3013>`_: MSA cleanup
* Modernize loops
* Pass xacro_args to both, urdf and srdf loading
* Switch to std::bind (`#2967 <https://github.com/ros-planning/moveit2/issues/2967>`_)
  * boost::bind -> std::bind
  grep -rlI --exclude-dir=.git "boost::bind" | xargs sed -i 's/boost::bind/std::bind/g'
  * Convert bind placeholders
  grep -rlI --exclude-dir=.git " _[0-9]" | xargs sed -i 's/ _\([0-9]\)/ std::placeholders::_\1/g'
  * Update bind include header
  grep -rlI --exclude-dir=.git "boost/bind" | xargs sed -i 's#boost/bind.hpp#functional#'
* MSA: Notice file updates (`#2964 <https://github.com/ros-planning/moveit2/issues/2964>`_)
  This commit fixes a MSA bug causing files in a loaded MoveIt config to be incorrectly classified as externally modified
  after being written by the "Generate Package" button.
  As this status is solely based on the file timestamp relative to the timestamp stored in the .setupassistant file,
  we need to update this timestamp when we wrote the files.
* Upload controller_list for simple controller manager (`#2954 <https://github.com/ros-planning/moveit2/issues/2954>`_)
* 1.1.6
* Various improvements to MSA: `#2932 <https://github.com/ros-planning/moveit2/issues/2932>`_, `#2945 <https://github.com/ros-planning/moveit2/issues/2945>`_, `#2946 <https://github.com/ros-planning/moveit2/issues/2946>`_
* Pilz: Define default planner
* Simplify definition of `planning_plugin` parameter
  There is no means to declare the planning_plugin as an arg first.
* Allow checking/unchecking multiple files for generation
* Improve instructions
* moveit.rviz: Use Orbit view controller
* moveit.rviz template: remove link names
* Rename launch argument execution_type -> fake_execution_type
  ... to clarify that this parameter is only used for fake controllers
* gazebo.launch: delayed unpause
  Only unpause simulation when robot model was loaded.
  This ensures that the initial pose is actually held.
* gazebo.yaml: Allow initial_joint_positions
* gazebo.launch: Load URDF via xacro if neccessary
* Modularize demo_gazebo.launch: draw on demo.launch
* demo.launch: start joint + robot-state publishers in fake mode only
  This will facilitate re-use of demo.launch.
* Formatting
* Fix controller choice
  - Provide all types of JointTrajectoryController as well as
  FollowJointTrajectory and GripperCommand (use by simple manager)
  - Use effort_controllers/JointTrajectoryController as default
  - Create FollowJointTrajectory entries for any JointTrajectoryController
* Simplify code
* Update widget texts to speak about generic controllers
* Rename ROSControllersWidget -> ControllersWidget
* Rename files ros_controllers_widget.* -> controllers_widget.*
* Rename ros_controllers_config\_ -> controller_configs\_
* Rename functions *ROSController* -> *Controller*
* Rename ROSControlConfig -> ControllerConfig
* Fix ros_controllers.yaml: always handle joints as sequence
* Rework controller config generation
  We should write separate controller config files for different controller managers:
  - simple_moveit_controllers.yaml handles everything relevant for SimpleMoveItControllerManager
  - ros_controllers.yaml handles ros_control config
  - gazebo_controllers.yaml handles controllers required for Gazebo
* Add gazebo_controllers.yaml
* Fix handling of sensors_3d.yaml
  - Reading both, the default and the existing package's sensors_3d.yaml
  into the config, the config file was growing by 2 configs each time.
  - Not visiting the Perception tab, was writing the default config with 2 entries
  - Selecting "None" was writing an invalid config:
  sensors:
  - {}
  - {}
* Cleanup generation of ros_controllers.yaml
* Rework moveit_controller_manager handling
  There are 3 basic MoveIt controller manager plugins:
  - fake = `moveit_fake_controller_manager::MoveItFakeControllerManager`
  Used in demo.launch. Doesn't really control the robot, but just
  interpolates between via points. Allows these execution_types:
  - via points: just jumps to the via points
  - interpolate: linearly interpolates between via points (default)
  - last point: jumps to the final trajectory point (used for fast execution testing)
  - ros_control = `moveit_ros_control_interface::MoveItControllerManager`
  Interfaces to ros_control controllers.
  - simple = `moveit_simple_controller_manager/MoveItSimpleControllerManager`
  Interfaces to action servers for `FollowJointTrajectory` and/or `GripperCommand`
  that in turn interface to the low-level robot controllers (typically based on ros_control)
  However, so far move_group.launch distinguished between `fake` and `robot` only.
  The argument moveit_controller_manager now allows switching between all 3 variants.
  Adding more *_moveit_controller_manager.launch files allows for an extension of this scheme.
* Fix definition of real-robot moveit_controller_manager
  Fixes the following error (occurring since 61d18f2f073aa4c8a13c2278c41a63591d401c4a)
  ```
  [FATAL] ros.moveit_ros_planning.trajectory_execution_manager:
  Exception while loading controller manager 'robot':
  According to the loaded plugin descriptions the class robot
  with base class type moveit_controller_manager::MoveItControllerManager does not exist.
  ```
  As we introduced `pass_all_args="true"`, the value of the argument
  `moveit_controller_manager` was the robot name.
* Remove execution_type argument from real-robot controller_manager.launch
* moveit_controller_manager.launch: pass execution_type via pass_all_args (`#2928 <https://github.com/ros-planning/moveit2/issues/2928>`_)
  While we need to pass execution_type to fake_moveit_controller_manager.launch,
  the controller_manager.launch files of real-robot shouldn't be required
  to define this argument. However, if they don't roslaunch fails with an
  `unused args` exception (see `#2786 <https://github.com/ros-planning/moveit2/issues/2786>`_).
  Passing arguments via pass_all_args should solve that issue.
* Contributors: Abishalini, Henning Kayser, Jafar, Jochen Sprickerhof, Loy van Beek, Michael Görner, Rick Staa, Robert Haschke, Vatan Aksoy Tezer, jeoseo, rickstaa, v4hn

2.4.0 (2022-01-20)
------------------
* Replace NULL with nullptr (`#961 <https://github.com/ros-planning/moveit2/issues/961>`_)
  * Fixes `#841 <https://github.com/ros-planning/moveit2/issues/841>`_
* Merge https://github.com/ros-planning/moveit/commit/a0ee2020c4a40d03a48044d71753ed23853a665d
* moveit_build_options()
  Declare common build options like CMAKE_CXX_STANDARD, CMAKE_BUILD_TYPE,
  and compiler options (namely warning flags) once.
  Each package depending on moveit_core can use these via moveit_build_options().
* Load all planning pipelines into their own namespace (`#2888 <https://github.com/ros-planning/moveit2/issues/2888>`_)
  Reduce code redundancy, specifying the namespace once in planning_pipeline.launch.
* MSA: Correctly state not-found package name
  The warning message was accessing the config_data\_ variable, which
  was assigned just a few lines later.
* Contributors: Abishalini, Robert Haschke, Stephanie Eng

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Replaced C-Style Cast with C++ Style Cast. (`#935 <https://github.com/ros-planning/moveit2/issues/935>`_)
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* Get rid of "std::endl" (`#918 <https://github.com/ros-planning/moveit2/issues/918>`_)
* changed post-increments in loops to preincrements (`#888 <https://github.com/ros-planning/moveit2/issues/888>`_)
* Update README (`#812 <https://github.com/ros-planning/moveit2/issues/812>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* Ported the collision updater from ros1 to ros2 in the moveit_setup_assistant (`#732 <https://github.com/ros-planning/moveit2/issues/732>`_)
* Adds jiggle fraction arg to trajopt template (`#2858 <https://github.com/ros-planning/moveit/issues/2858>`_)
* Fixes _planning_pipeline.launch template input args defaults (`#2849 <https://github.com/ros-planning/moveit/issues/2849>`_)
* Fixes setup_assistant custom planner ns problem (`#2842 <https://github.com/ros-planning/moveit/issues/2842>`_)
* MSA: Mention optional Gazebo deps in package.xml templates (`#2839 <https://github.com/ros-planning/moveit/issues/2839>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
  Co-authored-by: Robert Haschke <rhaschke@techfak.uni-bielefeld.de>
* Revert $(dirname) use for rviz config file
  ... due to this bug: https://github.com/ros/ros_comm/issues/1487
  This partially reverts 442c3202a4124877afbb6e2bdee682c537f25553
* Fix MSA templates (`#2769 <https://github.com/ros-planning/moveit/issues/2769>`_)
  * create static_transform_publisher for each virtual joint type
  * another $(dirname)
  augmenting `#2748 <https://github.com/ros-planning/moveit/issues/2748>`_
  * formatting
* Contributors: Brennand Pierce, Dave Coleman, David V. Lu!!, Henning Kayser, Kaustubh, Parthasarathy Bana, Rick Staa, Robert Haschke, Sencer Yazıcı, Stephanie Eng, pvanlaar

1.1.1 (2020-10-13)
------------------
* [feature] Allow showing both, visual and collision geometry (`#2352 <https://github.com/ros-planning/moveit/issues/2352>`_)
* [fix] layout (`#2349 <https://github.com/ros-planning/moveit/issues/2349>`_)
* [fix] group editing (`#2350 <https://github.com/ros-planning/moveit/issues/2350>`_)
* [fix] only write default_planner_config field if any is selected (`#2293 <https://github.com/ros-planning/moveit/issues/2293>`_)
* [fix] Segfault when editing pose in moveit_setup_assistant (`#2340 <https://github.com/ros-planning/moveit/issues/2340>`_)
* [fix] disappearing robot on change of reference frame (`#2335 <https://github.com/ros-planning/moveit/issues/2335>`_)
* [fix] robot_description is already loaded in move_group.launch (`#2313 <https://github.com/ros-planning/moveit/issues/2313>`_)
* [maint] Cleanup MSA includes (`#2351 <https://github.com/ros-planning/moveit/issues/2351>`_)
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski, Michael Görner, Robert Haschke, Tyler Weaver, Yoan Mollard

1.1.0 (2020-09-04)
------------------
* [feature] Start new joint_state_publisher_gui on param use_gui (`#2257 <https://github.com/ros-planning/moveit/issues/2257>`_)
* [feature] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [feature] Add default velocity/acceleration scaling factors (`#1890 <https://github.com/ros-planning/moveit/issues/1890>`_)
* [feature] MSA: use matching group/state name for default controller state (`#1936 <https://github.com/ros-planning/moveit/issues/1936>`_)
* [feature] MSA: Restore display of current directory (`#1932 <https://github.com/ros-planning/moveit/issues/1932>`_)
* [feature] Cleanup: use range-based for-loop (`#1830 <https://github.com/ros-planning/moveit/issues/1830>`_)
* [feature] Add delete process to the doneEditing() function in end_effectors_widgets (`#1829 <https://github.com/ros-planning/moveit/issues/1829>`_)
* [feature] Fix Rviz argument in demo_gazebo.launch (`#1797 <https://github.com/ros-planning/moveit/issues/1797>`_)
* [feature] Allow user to specify planner termination condition. (`#1695 <https://github.com/ros-planning/moveit/issues/1695>`_)
* [feature] Add OMPL planner 'AnytimePathShortening' (`#1686 <https://github.com/ros-planning/moveit/issues/1686>`_)
* [feature] MVP TrajOpt Planner Plugin (`#1593 <https://github.com/ros-planning/moveit/issues/1593>`_)
* [feature] Use QDir::currentPath() rather than getenv("PWD") (`#1618 <https://github.com/ros-planning/moveit/issues/1618>`_)
* [feature] Add named frames to CollisionObjects (`#1439 <https://github.com/ros-planning/moveit/issues/1439>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Fix ordering of request adapters (`#2053 <https://github.com/ros-planning/moveit/issues/2053>`_)
* [fix] Fix some clang tidy issues (`#2004 <https://github.com/ros-planning/moveit/issues/2004>`_)
* [fix] Fix usage of panda_moveit_config (`#1904 <https://github.com/ros-planning/moveit/issues/1904>`_)
* [fix] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [fix] Use portable string() on filesystem::path. (`#1571 <https://github.com/ros-planning/moveit/issues/1571>`_)
* [fix] Fix test utilities in moveit core (`#1409 <https://github.com/ros-planning/moveit/issues/1409>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* [maint] remove obsolete moveit_resources/config.h (`#1412 <https://github.com/ros-planning/moveit/issues/1412>`_)
* Contributors: AndyZe, Ayush Garg, Daniel Wang, Dave Coleman, Felix von Drigalski, Henning Kayser, Jafar Abdi, Jonathan Binney, Mark Moll, Max Krichenbauer, Michael Görner, Mike Lautman, Mohmmad Ayman, Omid Heidari, Robert Haschke, Sandro Magalhães, Sean Yen, Simon Schmeisser, Tejas Kumar Shastha, Tyler Weaver, Yoan Mollard, Yu, Yan, jschleicher, tnaka, v4hn

1.0.6 (2020-08-19)
------------------
* [maint] Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint] Migrate to clang-format-10, fix warnings
* [fix]   Define planning adapters for chomp planning pipeline (`#2242 <https://github.com/ros-planning/moveit/issues/2242>`_)
* [maint] Remove urdf package as build_depend from package.xml (`#2207 <https://github.com/ros-planning/moveit/issues/2207>`_)
* Contributors: Jafar Abdi, Robert Haschke, tnaka, Michael Görner

1.0.5 (2020-07-08)
------------------
* [fix]     Fix catkin_lint issues (`#2120 <https://github.com/ros-planning/moveit/issues/2120>`_)
* [feature] Add use_rviz to demo.launch in setup_assistant (`#2019 <https://github.com/ros-planning/moveit/issues/2019>`_)
* Contributors: Henning Kayser, Jafar Abdi, Michael Görner, Robert Haschke, Tyler Weaver

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [feature] Allow loading of additional kinematics parameters file (`#1997 <https://github.com/ros-planning/moveit/issues/1997>`_)
* [feature] Allow adding initial poses to fake_controllers.yaml (`#1892 <https://github.com/ros-planning/moveit/issues/1892>`_)
* [feature] Display robot poses on selection, not only on click (`#1930 <https://github.com/ros-planning/moveit/issues/1930>`_)
* [fix]     Fix invalid iterator (`#1623 <https://github.com/ros-planning/moveit/issues/1623>`_)
* [maint]   Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint]   Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint]   Windows build fixes
  * Fix header inclusion and other MSVC build errors (`#1636 <https://github.com/ros-planning/moveit/issues/1636>`_)
  * Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
  * Favor ros::Duration.sleep over sleep. (`#1634 <https://github.com/ros-planning/moveit/issues/1634>`_)
  * Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [feature] Add support for pos_vel_controllers and pos_vel_acc_controllers (`#1806 <https://github.com/ros-planning/moveit/issues/1806>`_)
* [feature] Add joint state controller config by default (`#1024 <https://github.com/ros-planning/moveit/issues/1024>`_)
* Contributors: AndyZe, Daniel Wang, Felix von Drigalski, Jafar Abdi, Max Krichenbauer, Michael Görner, Mohmmad Ayman, Robert Haschke, Sandro Magalhães, Sean Yen, Simon Schmeisser, Tejas Kumar Shastha, Yu, Yan, v4hn

1.0.2 (2019-06-28)
------------------
* [fix]     static transform publisher does not take a rate (`#1494 <https://github.com/ros-planning/moveit/issues/1494>`_)
* [feature] Add arguments `load_robot_description`, `pipeline`, `rviz config_file`  to launch file templates (`#1397 <https://github.com/ros-planning/moveit/issues/1397>`_)
* Contributors: Mike Lautman, Robert Haschke, jschleicher

1.0.1 (2019-03-08)
------------------
* [fix] re-add required build dependencies (`#1373 <https://github.com/ros-planning/moveit/issues/1373>`_)
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Isaac I.Y. Saito, Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* [fix] memory leaks (`#1292 <https://github.com/ros-planning/moveit/issues/1292>`_)
* [improve] Remove (redundant) random seeding and #attempts from RobotState::setFromIK() as the IK solver perform random seeding themselves. `#1288 <https://github.com/ros-planning/moveit/issues/1288>`_
* [improve] support dark themes (`#1173 <https://github.com/ros-planning/moveit/issues/1173>`_)
* Contributors: Dave Coleman, Robert Haschke, Victor Lamoine

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [enhancement] Create demo_gazebo.launch (`#1051 <https://github.com/ros-planning/moveit/issues/1051>`_)
* [maintenance] Cleanup includes to speedup compiling (`#1205 <https://github.com/ros-planning/moveit/issues/1205>`_)
* [maintenance] Use C++14 (`#1146 <https://github.com/ros-planning/moveit/issues/1146>`_)
* [maintenance] Code Cleanup
  * `#1179 <https://github.com/ros-planning/moveit/issues/1179>`_
  * `#1196 <https://github.com/ros-planning/moveit/issues/1196>`_
* Contributors: Alex Moriarty, Dave Coleman, Michael Görner, Robert Haschke

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------
* [fix] Build regression (`#1170 <https://github.com/ros-planning/moveit/issues/1170>`_)
* Contributors: Robert Haschke

0.10.3 (2018-10-29)
-------------------
* [fix] compiler warnings (`#1089 <https://github.com/ros-planning/moveit/issues/1089>`_)
* Contributors: Robert Haschke

0.10.2 (2018-10-24)
-------------------
* [fix] Some bugs (`#1022 <https://github.com/ros-planning/moveit/issues/1022>`_, `#1013 <https://github.com/ros-planning/moveit/issues/1013>`_, `#1040 <https://github.com/ros-planning/moveit/issues/1040>`_)
* [capability][chomp] Failure recovery options for CHOMP by tweaking parameters (`#987 <https://github.com/ros-planning/moveit/issues/987>`_)
* [capability] New screen for automatically generating interfaces to low level controllers(`#951 <https://github.com/ros-planning/moveit/issues/951>`_, `#994 <https://github.com/ros-planning/moveit/issues/994>`_, `#908 <https://github.com/ros-planning/moveit/issues/908>`_)
* [capability] Perception screen for using laser scanner point clouds. (`#969 <https://github.com/ros-planning/moveit/issues/969>`_)
* [enhancement][GUI] Logo for MoveIt 2.0, cleanup appearance (`#1059 <https://github.com/ros-planning/moveit/issues/1059>`_)
* [enhancement][GUI] added a setup assistant window icon (`#1028 <https://github.com/ros-planning/moveit/issues/1028>`_)
* [enhancement][GUI] Planning Groups screen (`#1017 <https://github.com/ros-planning/moveit/issues/1017>`_)
* [enhancement] use panda for test, and write test file in tmp dir (`#1042 <https://github.com/ros-planning/moveit/issues/1042>`_)
* [enhancement] Added capabilties as arg to move_group.launch (`#998 <https://github.com/ros-planning/moveit/issues/998>`_)
* [enhancement] Add moveit_setup_assistant as depenency of all *_moveit_config pkgs (`#1029 <https://github.com/ros-planning/moveit/issues/1029>`_)
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* [enhancement] Improving gazebo integration. (`#956 <https://github.com/ros-planning/moveit/issues/956>`_, `#936 <https://github.com/ros-planning/moveit/issues/936>`_)
* [maintenance] Renamed wedgits in setup assistant wedgit to follow convention (`#995 <https://github.com/ros-planning/moveit/issues/995>`_)
* [capability][chomp] cleanup of unused parameters and code + addition of trajectory initialization methods (linear, cubic, quintic-spline) (`#960 <https://github.com/ros-planning/moveit/issues/960>`_)
* Contributors: Alexander Gutenkunst, Dave Coleman, Mike Lautman, MohmadAyman, Mohmmad Ayman, Raghavender Sahdev, Robert Haschke, Sohieb Abdelrahman, mike lautman

0.10.1 (2018-05-25)
-------------------
* [maintenance] migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* [maintenance] cleanup yaml parsing, remove yaml-cpp 0.3 support (`#795 <https://github.com/ros-planning/moveit/issues/795>`_)
* [feature] allow editing of xacro args (`#796 <https://github.com/ros-planning/moveit/issues/796>`_)
* Contributors: Dave Coleman, Ian McMahon, Michael Görner, Mikael Arguedas, Robert Haschke, Will Baker

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix][kinetic onward] msa: use qt4-compatible API for default font (`#682 <https://github.com/ros-planning/moveit/issues/682>`_)
* [fix][kinetic onward] replace explicit use of Arial with default application font (`#668 <https://github.com/ros-planning/moveit/issues/668>`_)
* [fix] add moveit_fake_controller_manager to run_depend of moveit_config_pkg_template/package.xml.template (`#613 <https://github.com/ros-planning/moveit/issues/613>`_)
* [fix] find and link against tinyxml where needed (`#569 <https://github.com/ros-planning/moveit/issues/569>`_)
* Contributors: Kei Okada, Michael Görner, Mikael Arguedas, William Woodall

0.9.9 (2017-08-06)
------------------
* [setup_assistant] Fix for lunar (`#542 <https://github.com/ros-planning/moveit/issues/542>`_) (fix `#506 <https://github.com/ros-planning/moveit/issues/506>`_)
* Contributors: Dave Coleman

0.9.8 (2017-06-21)
------------------
* [enhance] setup assistant: add use_gui param to demo.launch (`#532 <https://github.com/ros-planning/moveit/issues/532>`_)
* [build] add Qt-moc guards for boost 1.64 compatibility (`#534 <https://github.com/ros-planning/moveit/issues/534>`_)
* Contributors: Michael Goerner

0.9.7 (2017-06-05)
------------------
* [fix] Build for Ubuntu YZ by adding BOOST_MATH_DISABLE_FLOAT128 (`#505 <https://github.com/ros-planning/moveit/issues/505>`_)
* [improve][MSA] Open a directory where setup_assistant.launch was started. (`#509 <https://github.com/ros-planning/moveit/issues/509>`_)
* Contributors: Isaac I.Y. Saito, Mikael Arguedas

0.9.6 (2017-04-12)
------------------
* [improve] Add warning if no IK solvers found (`#485 <https://github.com/ros-planning/moveit/issues/485>`_)
* Contributors: Dave Coleman

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* Contributors: Dave Coleman

0.9.4 (2017-02-06)
------------------
* [fix] Qt4/Qt5 compatibility `#413 <https://github.com/ros-planning/moveit/pull/413>`_
* [fix] show disabled collisions as matrix  (`#394 <https://github.com/ros-planning/moveit/issues/394>`_)
* Contributors: Dave Coleman, Robert Haschke, Michael Goerner

0.9.3 (2016-11-16)
------------------
* [capability] Exposed planners from latest ompl release. (`#338 <https://github.com/ros-planning/moveit/issues/338>`_)
* [enhancement] Increase collision checking interval (`#337 <https://github.com/ros-planning/moveit/issues/337>`_)
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon, Ruben Burger

0.9.2 (2016-11-05)
------------------
* [Fix] xacro warnings in Kinetic (`#334 <https://github.com/ros-planning/moveit/issues/334>`_)
  [Capability] Allows for smaller collision objects at the cost of increased planning time
* [Improve] Increase the default discretization of collision checking motions (`#321 <https://github.com/ros-planning/moveit/issues/321>`_)
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman

0.7.1 (2016-06-24)
------------------
* [sys] Qt adjustment.
  * relax Qt-version requirement.  Minor Qt version updates are ABI-compatible with each other:  https://wiki.qt.io/Qt-Version-Compatibility
  * auto-select Qt version matching the one from rviz `#114 <https://github.com/ros-planning/moveit_setup_assistant/issues/114>`_
  * Allow to conditionally compile against Qt5 by setting -DUseQt5=On
* [sys] Add line for supporting CMake 2.8.11 as required for Indigo
* [sys][travis] Update CI conf for ROS Jade (and optionally added Kinetic) `#116 <https://github.com/ros-planning/moveit_setup_assistant/issues/116>`_
* [feat] add ApplyPlanningScene capability to template
* Contributors: Dave Coleman, Isaac I.Y. Saito, Robert Haschke, Simon Schmeisser (isys vision), v4hn

0.7.0 (2016-01-30)
------------------
* Merge pull request from ipa-mdl/indigo-devel
  Added command-line SRDF updater
* renamed target output to collisions_updater
* formatted code to roscpp style
* More verbose error descriptions, use ROS_ERROR_STREAM
* moved file loader helpers into tools
* added licence header
* Missed a negation sign
* CollisionUpdater class was not really needed
* factored out createFullURDFPath and createFullSRDFPath
* factored out MoveItConfigData::getSetupAssistantYAMLPath
* factored out MoveItConfigData::setPackagePath
* factored out setCollisionLinkPairs into MoveItConfigData
* require output path to be set if SRDF path is overwritten by a xacro file path
* separated xacro parsing from loadFileToString
* make disabled_collisions entries unique
* Added command-line SRDF updater
* Merge pull request from 130s/fix/windowsize
  Shrink window height
* Add scrollbar to the text area that could be squashed.
* Better minimum window size.
* Merge pull request #103  from gavanderhoorn/issue102_cfgrble_db_path
  Fix for issue #102 : allow user to set mongodb db location
* Update warehouse launch file to accept non-standard db location. Fix #102.
  Also update generated demo.launch accordingly.
  The default directory could be located on a non-writable file system, leading
  to crashes of the mongodb wrapper script. This change allows the user to specify
  an alternative location using the 'db_path' argument.
* Update configuration_files_widget.cpp
  Fix link
* Contributors: Dave Coleman, Ioan A Sucan, Isaac IY Saito, Mathias Lüdtke, Nathan Bellowe, Sachin Chitta, gavanderhoorn, hersh

0.6.0 (2014-12-01)
------------------
* Values are now read from kinematics.yaml correctly.
* Simplified the inputKinematicsYAML() code.
* Debug and octomap improvements in launch file templates
* Values are now read from kinematics.yaml correctly. Previously, keys such
  as "kinematics_solver" were not found.
* Added clear octomap service to move_group launch file template
* Added gdb debug helper that allows easier break point addition
* Add launch file for joystick control of MotionPlanningPlugin
* Joint limits comments
* Removed velocity scaling factor
* Added a new 'velocity_scaling_factor' parameter to evenly reduce max joint velocity for all joints. Added documentation.
* Simply renamed kin_model to robot_model for more proper naming convension
* Added new launch file for controll Rviz with joystick
* use relative instead of absolute names for topics (to allow for namespaces)
* Added planner specific parameters to ompl_planning.yaml emitter.
* Added space after every , in function calls
  Added either a space or a c-return before opening {
  Moved & next to the variable in the member function declarations
* Added planner specific parameters to ompl_planning.yaml emitter.
  Each parameter is set to current defaults. This is fragile, as defaults may change.
* Contributors: Chris Lewis, Dave Coleman, Ioan A Sucan, Jim Rothrock, ahb, hersh

0.5.9 (2014-03-22)
------------------
* Fixed bug 82 in a quick way by reducing min size.
* Fix for issue `#70 <https://github.com/ros-planning/moveit_setup_assistant/issues/70>`_: support yaml-cpp 0.5+ (new api).
* Generate joint_limits.yaml using ordered joints
* Ensures that group name changes are reflected in the end effectors and robot poses screens as well
* Prevent dirty transforms warning
* Cleaned up stray cout's
* Contributors: Benjamin Chretien, Dave Coleman, Dave Hershberger, Sachin Chitta

0.5.8 (2014-02-06)
------------------
* Update move_group.launch
  Adding get planning scene service to template launch file.
* Fix `#42 <https://github.com/ros-planning/moveit_setup_assistant/issues/42>` plus cosmetic param name change.
* Contributors: Acorn, Dave Hershberger, sachinchitta

0.5.7 (2014-01-03)
------------------
* Added back-link to tutorial and updated moveit website URL.
* Ported tutorial from wiki to sphinx in source repo.

0.5.6 (2013-12-31)
------------------
* Fix compilation on OS X 10.9 (clang)
* Contributors: Nikolaus Demmel, isucan

0.5.5 (2013-12-03)
------------------
* fix `#64 <https://github.com/ros-planning/moveit_setup_assistant/issues/64>`_.
* Added Travis Continuous Integration

0.5.4 (2013-10-11)
------------------
* Added optional params so user knows they exist - values remain same

0.5.3 (2013-09-23)
------------------
* enable publishing more information for demo.launch
* Added 2 deps needed for some of the launch files generated by the setup assistant
* add source param for joint_state_publisher
* Added default octomap_resolution to prevent warning when move_group starts. Added comments.
* generate config files for fake controllers
* port to new robot state API

0.5.2 (2013-08-16)
------------------
* fix `#50 <https://github.com/ros-planning/moveit_setup_assistant/issues/50>`_
* fix `#52 <https://github.com/ros-planning/moveit_setup_assistant/issues/52>`_

0.5.1 (2013-08-13)
------------------
* make headers and author definitions aligned the same way; white space fixes
* add debug flag to demo.launch template
* default scene alpha is now 1.0
* add robot_state_publisher dependency for generated pkgs
* disable mongodb creation by default in demo.launch
* add dependency on joint_state_publisher for generated config pkgs

0.5.0 (2013-07-15)
------------------
* white space fixes (tabs are now spaces)
* fix `#49 <https://github.com/ros-planning/moveit_setup_assistant/issues/49>`_

0.4.1 (2013-06-26)
------------------
* fix `#44 <https://github.com/ros-planning/moveit_setup_assistant/issues/44>`_
* detect when xacro needs to be run and generate planning_context.launch accordingly
* fix `#46 <https://github.com/ros-planning/moveit_setup_assistant/issues/46>`_
* refactor how planners are added to ompl_planning.yaml; include PRM & PRMstar, remove LazyRRT
* change defaults per `#47 <https://github.com/ros-planning/moveit_setup_assistant/issues/47>`_
* SRDFWriter: add initModel() method for initializing from an existing urdf/srdf model in memory.
* SRDFWriter: add INCLUDE_DIRS to catkin_package command so srdf_writer.h can be used by other packages.
* git add option for minimum fraction of 'sometimes in collision'
* fix `#41 <https://github.com/ros-planning/moveit_setup_assistant/issues/41>`_
