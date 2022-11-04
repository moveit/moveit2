^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.4 (2022-11-04)
------------------
* Use pragma once as header include guard (`#1525 <https://github.com/ros-planning/moveit2/issues/1525>`_) (`#1652 <https://github.com/ros-planning/moveit2/issues/1652>`_)
  (cherry picked from commit 7d758de1b2f2904b8c85520129fa8d48aad93713)
  Co-authored-by: J. Javan <J-Javan@users.noreply.github.com>
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_) (`#1555 <https://github.com/ros-planning/moveit2/issues/1555>`_)
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Remove __has_include statements (`#1481 <https://github.com/ros-planning/moveit2/issues/1481>`_) (`#1483 <https://github.com/ros-planning/moveit2/issues/1483>`_)
* Contributors: mergify[bot]

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Merge https://github.com/ros-planning/moveit/commit/c88f6fb64e9057a4b9a8f6fafc01060e8c48a216
* Merge remote-tracking branch 'origin/main' into feature/msa
* fix regression from `#1331 <https://github.com/ros-planning/moveit2/issues/1331>`_ (`#1384 <https://github.com/ros-planning/moveit2/issues/1384>`_)
* Removing more boost usage (`#1372 <https://github.com/ros-planning/moveit2/issues/1372>`_)
* Merge remote-tracking branch 'upstream/main' into feature/msa
* Removing some boost usage (`#1331 <https://github.com/ros-planning/moveit2/issues/1331>`_)
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Fix rviz segfault when changing move group during execution (`#3123 <https://github.com/ros-planning/moveit2/issues/3123>`_)
* Merge pull request `#3106 <https://github.com/ros-planning/moveit/issues/3106>`_ from v4hn/pr-master-bind-them-all / banish bind()
* Fix clang-tidy
* banish bind()
* Replace obsolete distutils.core with setuptools (`#3103 <https://github.com/ros-planning/moveit2/issues/3103>`_)
  http://wiki.ros.org/noetic/Migration#Setuptools_instead_of_Distutils
* Contributors: Abishalini, David V. Lu, Henry Moore, Jafar, Michael Ferguson, Michael Görner, Robert Haschke, Vatan Aksoy Tezer, bsygo, v4hn

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Declare the default_planning_pipeline parameter (`#1227 <https://github.com/ros-planning/moveit2/issues/1227>`_)
  Co-authored-by: AndyZe <zelenak@picknik.ai>
* Merge https://github.com/ros-planning/moveit/commit/72d919299796bffc21f5eb752d66177841dc3442
* Enable cppcheck (`#1224 <https://github.com/ros-planning/moveit2/issues/1224>`_)
  Co-authored-by: jeoseo <jeongwooseo2012@gmail.com>
* Make TOTG the default time-parameterization algorithm everywhere (`#1218 <https://github.com/ros-planning/moveit2/issues/1218>`_)
  Co-authored-by: Jafar <cafer.abdi@gmail.com>
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
  Adaption of https://github.com/ros-planning/moveit/pull/3106
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* Merge https://github.com/ros-planning/moveit/commit/424a5b7b8b774424f78346d1e98bf1c9a33f0e78
* Remove new operators (`#1135 <https://github.com/ros-planning/moveit2/issues/1135>`_)
  replace new operator with make_shared
* Merge https://github.com/ros-planning/moveit/commit/a25515b73d682df03ed3eccd839110c296aa79fc
* Remove include of OgrePrerequisites header (`#1099 <https://github.com/ros-planning/moveit2/issues/1099>`_)
  * Remove OgrePrerequisites include
  * octomap_render.h includes itself
  * Include OgrePrerequisites.h
* Merge https://github.com/ros-planning/moveit/commit/ab42a1d7017b27eb6c353fb29331b2da08ab0039
* 1.1.9
* moveit joy: add PS3 dual shock model (`#3025 <https://github.com/ros-planning/moveit2/issues/3025>`_)
  * Added PS3 dual shock
  * Simplified if-else statements with as one-liners
* Compilation fixes for Jammy and bring back Rolling CI (`#1095 <https://github.com/ros-planning/moveit2/issues/1095>`_)
  * Use jammy dockers and clang-format-12
  * Fix unused depend, and move to python3-lxml
  * add ompl to repos, fix versions and ogre
  * Remove ogre keys
  * Fix boolean node operator
  * Stop building dockers on branch and fix servo null pointer
  * update pre-commit to clang-format-12 and pre-commit fixes
  * clang-format workaround and more pre-commit fixes
* Add option to use simulation time for rviz trajectory display (`#3055 <https://github.com/ros-planning/moveit2/issues/3055>`_)
* Fix object interactive marker in wrong pose after changing the fixed frame (`#680 <https://github.com/ros-planning/moveit2/issues/680>`_)
* Merge https://github.com/ros-planning/moveit/commit/0d7462f140e03b4c319fa8cce04a47fe3f650c60
* 1.1.8
* Remove unused parameters. (`#1018 <https://github.com/ros-planning/moveit2/issues/1018>`_)
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* 1.1.7
* Move MoveItErrorCode class to moveit_core (`#3009 <https://github.com/ros-planning/moveit2/issues/3009>`_)
  ... reducing code duplication and facilitating re-use
* RobotState::attachBody: Migrate to unique_ptr argument (`#3011 <https://github.com/ros-planning/moveit2/issues/3011>`_)
  ... to indicate transfer of ownership and simplify pointer handling
* Merge PR `#2925 <https://github.com/ros-planning/moveit2/issues/2925>`_: Fix "ClassLoader: SEVERE WARNING" on reset of MPD
  Resetting the MotionPlanningDisplay in rviz (or disabling+enabling it) issues a warning, because the IK plugin is unloaded (when resetting the RobotModelLoader) while there are still pending references to the RobotModel.
* Remove all remaining usage of robot_model
* Merge `#2944 <https://github.com/ros-planning/moveit2/issues/2944>`_: various fixes to the rviz plugins
* Switch to std::bind (`#2967 <https://github.com/ros-planning/moveit2/issues/2967>`_)
  * boost::bind -> std::bind
  grep -rlI --exclude-dir=.git "boost::bind" | xargs sed -i 's/boost::bind/std::bind/g'
  * Convert bind placeholders
  grep -rlI --exclude-dir=.git " _[0-9]" | xargs sed -i 's/ _\([0-9]\)/ std::placeholders::_\1/g'
  * Update bind include header
  grep -rlI --exclude-dir=.git "boost/bind" | xargs sed -i 's#boost/bind.hpp#functional#'
* MPD: Avoid flickering of the progress bar
  The progress bar shows the number of pending background jobs.
  If there is only one job pending, the progress bar is shown and
  immediately hidden as soon as the process is finished.
  Thus, we shouldn't show the progress bar if there is only one job
  and thus no actual progress to show.
  Use the default size and color scheme.
* Joints widget: avoid flickering of the nullspace slider
  Show a (disabled) dummy slider if there is no nullspace.
  This avoids flickering between zero and one slider, which is the most common case.
  Also provide some tooltips to explain the usage.
* 1.1.6
* Fix MotionPlanningFrame's namespace handling (`#2922 <https://github.com/ros-planning/moveit2/issues/2922>`_)
  * waitForAction(): remove NodeHandle argument
  * The NodeHandle was just for NodeHandle::ok(), which can be handled by ros::ok() as well.
  * Fix initialization of params, etc. that depend on MoveGroupNS
  * When the MoveGroupNS has changed, we should re-initialize all these
  params, subscribers, and topics.
  Thus having them in a central place is helpful ;-)
  * Fix namespaces as pointed out by @v4hn
  * Simplify nh\_ naming
  * update comments
* Fix ClassLoader: SEVERE WARNING
  Clear all references to RobotModel before destroying the corresponding
  RobotModelLoader.
* Modernize: std::make_shared
* Contributors: Abishalini, AndyZe, Cory Crean, Henning Kayser, Jafar, Jafar Abdi, JafarAbdi, Job van Dieten, Jochen Sprickerhof, Martin Oehler, Robert Haschke, Sencer Yazıcı, Stephanie Eng, Vatan Aksoy Tezer, jeoseo, pvanlaar, v4hn

2.4.0 (2022-01-20)
------------------
* Move background_processing (`#997 <https://github.com/ros-planning/moveit2/issues/997>`_)
* Merge https://github.com/ros-planning/moveit/commit/f3ac6070497da90da33551fc1dc3a68938340413
* Merge https://github.com/ros-planning/moveit/commit/a0ee2020c4a40d03a48044d71753ed23853a665d
* moveit_build_options()
  Declare common build options like CMAKE_CXX_STANDARD, CMAKE_BUILD_TYPE,
  and compiler options (namely warning flags) once.
  Each package depending on moveit_core can use these via moveit_build_options().
* MPD: do not save/restore warehouse parameters (`#2865 <https://github.com/ros-planning/moveit2/issues/2865>`_)
  If we reload these values from the config, setting the ROS parameters is much less useful.
  At least the *type* of warehouse_ros plugin (mongo or sqlite) cannot be selected
  in the display, so you will probably need to meddle with the parameters anyway if you want
  to connect to a different db.
  search for parameters warehouse_host/port because they are usually set at the top level, but
  you might want to set them differently for different move_groups.
* PlanningSceneDisplay: always update the main scene node's pose (`#2876 <https://github.com/ros-planning/moveit2/issues/2876>`_)
* Contributors: Abishalini, Michael Görner, Robert Haschke, Tyler Weaver

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Convert to modern include guard `#882 <https://github.com/ros-planning/moveit2/issues/882>`_ (`#891 <https://github.com/ros-planning/moveit2/issues/891>`_)
* Replaced C-Style Cast with C++ Style Cast. (`#935 <https://github.com/ros-planning/moveit2/issues/935>`_)
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* Get rid of "std::endl" (`#918 <https://github.com/ros-planning/moveit2/issues/918>`_)
* changed post-increments in loops to preincrements (`#888 <https://github.com/ros-planning/moveit2/issues/888>`_)
* Latched Strings for URDF and SRDF (`#765 <https://github.com/ros-planning/moveit2/issues/765>`_)
* Consider simulated time (`#883 <https://github.com/ros-planning/moveit2/issues/883>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* Reduce log verbosity, improved info message (`#714 <https://github.com/ros-planning/moveit2/issues/714>`_)
* Fix Python2: convert keys() into list (`#2862 <https://github.com/ros-planning/moveit/issues/2862>`_)
* MP panel: fix order of input widgets for shape size (`#2847 <https://github.com/ros-planning/moveit/issues/2847>`_)
* Makes rviz trajectory visualization topic relative (`#2835 <https://github.com/ros-planning/moveit/issues/2835>`_)
* MotionPlanningFrame: Gracefully handle undefined parent widget (`#2833 <https://github.com/ros-planning/moveit/issues/2833>`_)
* more fixes for the clang-tidy job (`#2813 <https://github.com/ros-planning/moveit/issues/2813>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: Dave Coleman, David V. Lu!!, Felix von Drigalski, Gaël Écorchard, Henning Kayser, Kaustubh, Michael Görner, Parthasarathy Bana, Rick Staa, Robert Haschke, Sencer Yazıcı, Yuri Rocha, lorepieri8, predystopic-dev, pvanlaar

2.3.0 (2021-10-08)
------------------
* Support passing MoveGroup's namespace to MoveGroupInterface (`#533 <https://github.com/ros-planning/moveit2/issues/533>`_)
* Add getSharedRobotModelLoader to fix race condition when having multiple displays for the same node (`#525 <https://github.com/ros-planning/moveit2/issues/525>`_)
* Make TF buffer & listener in PSM private (`#654 <https://github.com/ros-planning/moveit2/issues/654>`_)
  * Add private buffer & tf listener to PSM
  * Remove coupled deleter
  * Decouple PSM from CSM
  * Deprecate old constructors
* mesh_shape: Fix resource group for meshes (`#672 <https://github.com/ros-planning/moveit2/issues/672>`_)
* Fix warnings in Galactic and Rolling (`#598 <https://github.com/ros-planning/moveit2/issues/598>`_)
  * Use __has_includes preprocessor directive for deprecated headers
  * Fix parameter template types
  * Proper initialization of smart pointers, rclcpp::Duration
* Add option to disable Octomap in Rviz Rendering Tools (`#606 <https://github.com/ros-planning/moveit2/issues/606>`_)
* Fixes for Windows (`#530 <https://github.com/ros-planning/moveit2/issues/530>`_)
* Support arbitrary realtime-factors in trajectory visualization (`#2745 <https://github.com/ros-planning/moveit2/issues/2745>`_)
* Fix joints tab
* MP frame: Fix handling of mimic + passive joints
* Switch order of manipulation and joints tab
* Fix trajectory panel (`#2737 <https://github.com/ros-planning/moveit2/issues/2737>`_)
  * TrajectoryPanel: Only set paused\_ via pauseButton() to keep "Pause/Play" button in correct state
  * simplify code on the side
* moveit_joy: RuntimeError: dictionary changed size during iteration (`#2628 <https://github.com/ros-planning/moveit2/issues/2628>`_)
* Contributors: AdamPettinger, Akash, Henning Kayser, Jafar Abdi, Michael Görner, Nisala Kalupahana, Jorge Nicho, Henning Kayser, Robert Haschke, Vatan Aksoy Tezer, Tyler Weaver, Lior Lustgarten

2.2.1 (2021-07-12)
------------------

2.2.0 (2021-06-30)
------------------
* Declare warehouse params in rviz plugin (`#513 <https://github.com/ros-planning/moveit2/issues/513>`_)
* [sync] MoveIt's master branch up-to https://github.com/ros-planning/moveit/commit/0d0a6a171b3fbea97a0c4f284e13433ba66a4ea4
  * CI: Use compiler flag --pedantic (`#2691 <https://github.com/ros-planning/moveit/issues/2691>`_)
  * Runtime fixes to PlanningSceneDisplay, MotionPlanningDisplay (`#2618 <https://github.com/ros-planning/moveit/issues/2618>`_),(`#2588 <https://github.com/ros-planning/moveit2/issues/2588>`_)
  * Support multiple planning pipelines with MoveGroup via MoveItCpp (`#2127 <https://github.com/ros-planning/moveit/issues/2127>`_)
    * Allow selecting planning pipeline in RViz MotionPlanningDisplay
* Contributors: Bjar Ne, Henning Kayser, JafarAbdi, Michael Görner, Robert Haschke, Tyler Weaver

2.1.4 (2021-05-31)
------------------

2.1.3 (2021-05-22)
------------------

2.1.2 (2021-04-20)
------------------
* Fix robot_model & moveit_ros_visualization dependencies (`#421 <https://github.com/ros-planning/moveit2/issues/421>`_)
* Remove move_group namespace from MotionPlanning display (`#420 <https://github.com/ros-planning/moveit2/issues/420>`_)
* Contributors: Jafar Abdi, Vatan Aksoy Tezer

2.1.1 (2021-04-12)
------------------
* Fix EXPORT install in CMake (`#372 <https://github.com/ros-planning/moveit2/issues/372>`_)
* Add a private node to the interactive marker display (`#342 <https://github.com/ros-planning/moveit2/issues/342>`_)
* Sync main branch with MoveIt 1 from previous head https://github.com/ros-planning/moveit/commit/0247ed0027ca9d7f1a7f066e62c80c9ce5dbbb5e up to https://github.com/ros-planning/moveit/commit/74b3e30db2e8683ac17b339cc124675ae52a5114
* [fix] export cmake likbrary install (`#339 <https://github.com/ros-planning/moveit2/issues/339>`_)
* MTC compatibility fixes (`#323 <https://github.com/ros-planning/moveit2/issues/323>`_)
* Remove redundant exports
* moveit_ros_visualization: export libraries and include directory
* Catch exceptions during RobotModel loading in rviz (`#2468 <https://github.com/ros-planning/moveit2/issues/2468>`_)
* Fix repo URLs in package.xml files
* Contributors: Henning Kayser, Jafar Abdi, Simon Schmeisser, Tyler Weaver

2.1.0 (2020-11-23)
------------------
* [fix] Interactive markers not visible in motion planning plugin (`#299 <https://github.com/ros-planning/moveit2/issues/299>`_)
* [maint] Wrap common cmake code in 'moveit_package()' macro (`#285 <https://github.com/ros-planning/moveit2/issues/285>`_)
  * New moveit_package() macro for compile flags, Windows support etc
  * Add package 'moveit_common' as build dependency for moveit_package()
  * Added -Wno-overloaded-virtual compiler flag for moveit_ros_planners_ompl
* [maint] Compilation fixes for macOS (`#271 <https://github.com/ros-planning/moveit2/issues/271>`_)
* [ros2-migration] Port moveit_ros_warehouse to ROS 2 (`#273 <https://github.com/ros-planning/moveit2/issues/273>`_)
* [ros2-migration] Port trajectory_rviz_plugin to ROS 2 (`#201 <https://github.com/ros-planning/moveit2/issues/201>`_)
* Contributors: Henning Kayser, Jafar Abdi, Lior Lustgarten, Mark Moll, Yu Yan, Edwin Fan

2.0.0 (2020-02-17)
------------------
* [fix] moveit_ros_visualization fixes (`#168 <https://github.com/ros-planning/moveit2/issues/168>`_)
  * robot_state_display: Fix empty robot description field
  * planning scene plugin: Fix destroySceneNode
* [fix] Fix moveit_ros_visualization (`#167 <https://github.com/ros-planning/moveit2/issues/167>`_)
* [port] Port moveit ros visualization to ROS 2 (`#160 <https://github.com/ros-planning/moveit2/issues/160>`_)
* [port] Port rdf_loader to ROS2 (`#104 <https://github.com/ros-planning/moveit2/issues/104>`_)
* Contributors: Henning Kayser, Jafar Abdi

1.1.1 (2020-10-13)
------------------
* [feature] Clean up Rviz Motion Planning plugin, add tooltips (`#2310 <https://github.com/ros-planning/moveit/issues/2310>`_)
* [fix]     "Clear Octomap" button, disable when no octomap is published (`#2320 <https://github.com/ros-planning/moveit/issues/2320>`_)
* [fix]     clang-tidy warning (`#2334 <https://github.com/ros-planning/moveit/issues/2334>`_)
* [fix]     python3 issues (`#2323 <https://github.com/ros-planning/moveit/issues/2323>`_)
* [maint]   Cleanup MSA includes (`#2351 <https://github.com/ros-planning/moveit/issues/2351>`_)
* [maint]   Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski, Michael Görner, Robert Haschke

1.1.0 (2020-09-04)
------------------

1.0.6 (2020-08-19)
------------------
* [feature] MP display: add units to joints tab (`#2264 <https://github.com/ros-planning/moveit/issues/2264>`_)
* [feature] Allow adding planning scene shapes from rviz panel (`#2198 <https://github.com/ros-planning/moveit/issues/2198>`_)
* [feature] Default to Planning tab initially (`#2061 <https://github.com/ros-planning/moveit/issues/2061>`_)
* [fix]     Fix deferred robot model loading (`#2245 <https://github.com/ros-planning/moveit/issues/2245>`_)
* [maint]   Migrate to clang-format-10
* [maint]   Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* Contributors: Jorge Nicho, Markus Vieth, Michael Görner, Robert Haschke, Michael Görner

1.0.5 (2020-07-08)
------------------
* [feature] Improve rviz GUI to add PlanningScene objects. Ask for scaling large meshes. (`#2142 <https://github.com/ros-planning/moveit/issues/2142>`_)
* [maint]   Replace robot_model and robot_state namespaces with moveit::core (`#2135 <https://github.com/ros-planning/moveit/issues/2135>`_)
* [maint]   Fix catkin_lint issues (`#2120 <https://github.com/ros-planning/moveit/issues/2120>`_)
* [feature] PlanningSceneDisplay speedup (`#2049 <https://github.com/ros-planning/moveit/issues/2049>`_)
* [feature] Added support for PS4 joystick (`#2060 <https://github.com/ros-planning/moveit/issues/2060>`_)
* [fix]     MP display: planning attempts are natural numbers (`#2076 <https://github.com/ros-planning/moveit/issues/2076>`_, `#2082 <https://github.com/ros-planning/moveit/issues/2082>`_)
* Contributors: Felix von Drigalski, Henning Kayser, Jafar Abdi, Michael Görner, Robert Haschke, Simon Schmeisser, TrippleBender

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [fix]     `MotionPlanningDisplay`: change internal shortcut Ctrl+R to Ctrl+I (`#1967 <https://github.com/ros-planning/moveit/issues/1967>`_)
* [fix]     Remove `PlanningSceneInterface` from rviz display, but use its `PlanningSceneMonitor` instead
* [fix]     Fix segfault in `RobotStateVisualization` (`#1941 <https://github.com/ros-planning/moveit/issues/1941>`_)
* [feature] Provide visual feedback on success of requestPlanningSceneState()
* [feature] Wait for `get_planning_scene` in background (`#1934 <https://github.com/ros-planning/moveit/issues/1934>`_)
* [feature] Reduce step size for pose-adapting widgets
* [fix]     Reset `scene_marker` when disabling motion planning panel
* [fix]     Enable/disable motion planning panel with display
* [fix]     Enable/disable pose+scale group box when collision object is selected/deselected
* [fix]     Correctly populate the list of scene objects in the motion planning panel
* [feature] Resize scene marker with collision object
* [feature] Show attached bodies in trajectory trail (`#1766 <https://github.com/ros-planning/moveit/issues/1766>`_)
* [fix]     Fix `REALTIME` trajectory playback (`#1683 <https://github.com/ros-planning/moveit/issues/1683>`_)
* [maint]   Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint]   Notice changes in rviz planning panel requiring saving (`#1991 <https://github.com/ros-planning/moveit/issues/1991>`_)
* [maint]   Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint]   Improve Python 3 compatibility (`#1870 <https://github.com/ros-planning/moveit/issues/1870>`_)
  * Replaced StringIO with BytesIO for python msg serialization
  * Use py_bindings_tools::ByteString as byte-based serialization buffer on C++ side
* [maint]   Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix]     Fix pruning of enclosed nodes when rendering octomap in RViz (`#1685 <https://github.com/ros-planning/moveit/issues/1685>`_)
* [fix]     Fix missing `scene_manager` initialization in OcTreeRender's  constructor (`#1817 <https://github.com/ros-planning/moveit/issues/1817>`_)
* [feature] new `Joints` tab in RViz motion panel (`#1308 <https://github.com/ros-planning/moveit/issues/1308>`_)
* [feature] Add `<previous>` robot state to RViz motion panel (`#1742 <https://github.com/ros-planning/moveit/issues/1742>`_)
* Contributors: Bjar Ne, Dale Koenig, MarqRazz, Max Krichenbauer, Michael Görner, Robert Haschke, RyodoTanaka, Sean Yen, Takara Kasai, Yannick Jonetzko, Yu, Yan, v4hn

1.0.2 (2019-06-28)
------------------
* [maintenance] Removed unnecessary null pointer checks on deletion (`#1410 <https://github.com/ros-planning/moveit/issues/1410>`_)
* Contributors: Mahmoud Ahmed Selim

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Isaac Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* Contributors: Dave Coleman, Robert Haschke

0.10.8 (2018-12-24)
-------------------
* [fix] Handle exceptions in rviz plugins (`#1267 <https://github.com/ros-planning/moveit/issues/1267>`_)
* Contributors: Christian Rauch, Robert Haschke

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [enhancement] Add check box for CartesianPath planning (`#1238 <https://github.com/ros-planning/moveit/issues/1238>`_)
* [enhancement] Improve MotionPlanning panel (`#1198 <https://github.com/ros-planning/moveit/issues/1198>`_)
  * Allow selection of planning group in planning panel
  * Choose start and goal state directly from combobox
* [fix] rviz crash when changing the planning group while executing (`#1198 <https://github.com/ros-planning/moveit/issues/1198>`_)
* [fix] Fix several issues in rendering of attached bodies (`#1199 <https://github.com/ros-planning/moveit/issues/1199>`_)
  * Show / hide attached body together with robot
  * Force PlanningScene rendering on enable
  * Link SceneDisplay's attached-body-color to TrajectoryVisualization's one
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Use C++14 (`#1146 <https://github.com/ros-planning/moveit/issues/1146>`_)
* [maintenance] Cleanup Robot Interaction (`#1194 <https://github.com/ros-planning/moveit/issues/1194>`_)
  * Postpone subscription to trajectory topic
  * Fix memory leaks
* [maintenance] Simplify shared tf2 buffer usage (`#1196 <https://github.com/ros-planning/moveit/issues/1196>`_)
* [maintenance] Code Cleanup (`#1179 <https://github.com/ros-planning/moveit/issues/1179>`_)
* Remove obsolete eigen_conversions dependency (`#1181 <https://github.com/ros-planning/moveit/issues/1181>`_)
* Contributors: Alex Moriarty, Benjamin Scholz, Dave Coleman, Kei Okada, Michael Görner, Robert Haschke, Sven Krause

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------
* [maintenance] Store more settings of rviz' PlanningFrame (`#1135 <https://github.com/ros-planning/moveit/issues/1135>`_)
* [maintenance] Lint visualization (`#1144 <https://github.com/ros-planning/moveit/issues/1144>`_)
* Contributors: Alexander Gutenkunst, Dave Coleman

0.10.2 (2018-10-24)
-------------------
* [fix] build issue in boost/thread/mutex.hpp (`#1055 <https://github.com/ros-planning/moveit/issues/1055>`_)
* [fix] optional namespace args (`#929 <https://github.com/ros-planning/moveit/issues/929>`_)
* [maintenance] Python3 support (`#1103 <https://github.com/ros-planning/moveit/issues/1103>`_, `#1054 <https://github.com/ros-planning/moveit/issues/1054>`_)
* [maintenance] add minimum required pluginlib version (`#927 <https://github.com/ros-planning/moveit/issues/927>`_)
* Contributors: Michael Görner, Mikael Arguedas, Mohmmad Ayman, Robert Haschke, Timon Engelke, mike lautman

0.10.1 (2018-05-25)
-------------------
* [maintenance] migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* [feature] rviz plugin: set start/goal RobotState from external (`#823 <https://github.com/ros-planning/moveit/issues/823>`_)
  - /rviz/moveit/update_custom_start_state
  - /rviz/moveit/update_custom_goal_state
  stopping from external:
  - /rviz/moveit/stop
* [feature] namespace capabilities for moveit_commander (`#835 <https://github.com/ros-planning/moveit/issues/835>`_)
* [fix] consider shape transform for OcTree
* [fix] realtime trajectory display (`#761 <https://github.com/ros-planning/moveit/issues/761>`_)
* Contributors: Alexander Rössler, Dave Coleman, Ian McMahon, Mikael Arguedas, Pan Hy, Phy, Robert Haschke, Will Baker

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] don't crash on empty robot_description in RobotState plugin `#688 <https://github.com/ros-planning/moveit/issues/688>`_
* [fix] RobotState rviz previewer: First message from e.g. latching publishers is not applied to robot state correctly (`#596 <https://github.com/ros-planning/moveit/issues/596>`_)
* [doc] Document auto scale in Rviz plugin (`#602 <https://github.com/ros-planning/moveit/issues/602>`_)
* Contributors: Dave Coleman, Isaac I.Y. Saito, Simon Schmeisser, axelschroth

0.9.9 (2017-08-06)
------------------
* [fix] RobotStateVisualization: clear before load to avoid segfault `#572 <https://github.com/ros-planning/moveit/pull/572>`_
* Contributors: v4hn

0.9.8 (2017-06-21)
------------------
* [fix] TrajectoryVisualization crash if no window_context exists (`#523 <https://github.com/ros-planning/moveit/issues/523>`_, `#525 <https://github.com/ros-planning/moveit/issues/525>`_)
* [fix] robot display: Don't reload robot model upon topic change (Fixes `#528 <https://github.com/ros-planning/moveit/issues/528>`_)
* [build] add Qt-moc guards for boost 1.64 compatibility (`#534 <https://github.com/ros-planning/moveit/issues/534>`_)
* [enhance] rviz display: stop trajectory visualization on new plan. Fixes `#526 <https://github.com/ros-planning/moveit/issues/526>`_ (`#531 <https://github.com/ros-planning/moveit/issues/531>`_, `#510 <https://github.com/ros-planning/moveit/issues/510>`_).
* Contributors: Isaac I.Y. Saito, Simon Schmeisser, Yannick Jonetzko, henhenhen, v4hn


0.9.7 (2017-06-05)
------------------
* [capability] New panel with a slider to control the visualized trajectory (`#491 <https://github.com/ros-planning/moveit/issues/491>`_) (`#508 <https://github.com/ros-planning/moveit/issues/508>`_)
* [fix] Build for Ubuntu YZ by adding BOOST_MATH_DISABLE_FLOAT128 (`#505 <https://github.com/ros-planning/moveit/issues/505>`_)
* Contributors: Dave Coleman, Mikael Arguedas

0.9.6 (2017-04-12)
------------------
* [fix] RViz plugin some cosmetics and minor refactoring `#482 <https://github.com/ros-planning/moveit/issues/482>`_
* [fix] rviz panel: Don't add object marker if the wrong tab is selected `#454 <https://github.com/ros-planning/moveit/pull/454>`_
* [improve] RobotState display [kinetic] (`#465 <https://github.com/ros-planning/moveit/issues/465>`_)
* Contributors: Jorge Nicho, Michael Goerner, Yannick Jonetzko

0.9.5 (2017-03-08)
------------------
* [fix] correct "simplify widget handling" `#452 <https://github.com/ros-planning/moveit/pull/452>`_ This reverts "simplify widget handling (`#442 <https://github.com/ros-planning/moveit/issues/442>`_)"
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman, Isaac I.Y. Saito, Yannick Jonetzko

0.9.4 (2017-02-06)
------------------
* [fix] race conditions when updating PlanningScene (`#350 <https://github.com/ros-planning/moveit/issues/350>`_)
* [enhancement] Add colours to trajectory_visualisation display (`#362 <https://github.com/ros-planning/moveit/issues/362>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Bence Magyar, Dave Coleman, Robert Haschke

0.9.3 (2016-11-16)
------------------
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman

0.6.6 (2016-06-08)
------------------
* cleanup cmake tests, fix empty output
* added missing rostest dependency (`#680 <https://github.com/ros-planning/moveit_ros/issues/680>`_), fixes c6d0ede (`#639 <https://github.com/ros-planning/moveit_ros/issues/639>`_)
* [moveit joy] Add friendlier error message
* relax Qt-version requirement
  Minor Qt version updates are ABI-compatible with each other:
  https://wiki.qt.io/Qt-Version-Compatibility
* replaced cmake_modules dependency with eigen
* [jade] eigen3 adjustment
* always (re)create collision object marker
  other properties than pose (such as name of the marker) need to be adapted too
* use getModelFrame() as reference frame for markers
* moved "Publish Scene" button to "Scene Objects" tab
  previous location on "Context" tab was weird
* cherry-pick PR `#635 <https://github.com/ros-planning/moveit_ros/issues/635>`_ from indigo-devel
* unify Qt4 / Qt5 usage across cmake files
  - fetch Qt version from rviz
  - define variables/macros commonly used for Qt4 and Qt5
  - QT_LIBRARIES
  - qt_wrap_ui()
* leave frame transforms to rviz
  The old code
  (1.) reimplemented frame transforms in rviz
  although it could simply utilize rviz' FrameManager
  (2.) assumed the transform between the model-frame
  and the fixed_frame was constant and only needed to be updated
  if the frame changes (ever tried to make the endeffector
  your fixed frame?)
  (3.) was broken because on startup calculateOffsetPosition was called
  *before* the robot model is loaded, so the first (and usually only)
  call to calculateOffsetPosition failed.
  Disabling/Enabling the display could be used to work around this...
  This fixes all three issues.
* display planned path in correct rviz context
  This was likely a typo.
* Solved parse error with Boost 1.58. Fixes `#653 <https://github.com/ros-planning/moveit_ros/issues/653>`_
* Enable optional build against Qt5, use -DUseQt5=On to enable it
* explicitly link rviz' default_plugin library
  The library is not exported anymore and now is provided separately from rviz_LIBRARIES.
  See https://github.com/ros-visualization/rviz/pull/979 for details.
* merge indigo-devel changes (PR `#633 <https://github.com/ros-planning/moveit_ros/issues/633>`_ trailing whitespace) into jade-devel
* Removed trailing whitespace from entire repository
* correctly handle int and float parameters
  Try to parse parameter as int and float (in that series)
  and use IntProperty or FloatProperty on success to have
  input checking.
  Floats formatted without decimal dot, e.g. "0", will be
  considered as int!
  All other parameters will be handled as string.
* access planner params in rviz' MotionPlanningFrame
* new method MoveGroup::getDefaultPlannerId(const std::string &group)
  ... to retrieve default planner config from param server
  moved corresponding code from rviz plugin to MoveGroup interface
  to facilitate re-use
* correctly initialize scene robot's parameters after initialization
  - loaded parameters were ignored
  - changed default alpha value to 1 to maintain previous behaviour
* load default_planner_config from default location
  instead of loading from `/<ns>/default_planner_config`, use
  `/<ns>/move_group/<group>/default_planner_config`, which is the default
  location for `planner_configs` too
* Merge pull request `#610 <https://github.com/ros-planning/moveit_ros/issues/610>`_: correctly update all markers after robot motion
* fixing conflicts, renaming variable
* Merge pull request `#612 <https://github.com/ros-planning/moveit_ros/issues/612>`_ from ubi-agni/interrupt-traj-vis
  interrupt trajectory visualization on arrival of new display trajectory
* cherry-picked PR `#611 <https://github.com/ros-planning/moveit_ros/issues/611>`_: fix segfault when disabling and re-enabling TrajectoryVisualization
* cherry-picked PR `#609 <https://github.com/ros-planning/moveit_ros/issues/609>`_: load / save rviz' workspace config
* added missing initialization
* correctly setAlpha for new trail
* fixed race condition for trajectory-display interruption
* cleanup TrajectoryVisualization::update
  simplified code to switch to new trajectory / start over animation in loop mode
* new GUI property to allow immediate interruption of displayed trajectory
* immediately show trajectory after planning (interrupting current display)
* fix segfault when disabling and re-enabling TrajectoryVisualization
* update pose of all markers when any marker moved
  Having several end-effector markers attached to a group (e.g. a multi-
  fingered hand having an end-effector per fingertip and an end-effector
  for the hand base), all markers need to update their pose on any motion
  of any marker. In the example: if the hand base is moved, the fingertip
  markers should be moved too.
* use move_group/default_workspace_bounds as a fallback for workspace bounds
* code style cleanup
* fixed tab order of rviz plugin widgets
* load / save rviz' workspace config
* saves robot name to db from moveit. also robot name accessible through robot interface python wrapper
* Added install rule to install moveit_joy.py.
* motion_planning_frame_planning: use /default_planner_config parma to specify default planning algorithm
* Avoid adding a slash if getMoveGroupNS() is empty.
  If the getMoveGroupNS() returns an empty string, ros::names::append() inserts a slash in front of 'right', which changes it to a global name.
  Checking getMoveGroupNS() before calling append removes the issue.
  append() behaviour will not be changed in ros/ros_comm.
* Contributors: Ammar Najjar, Dave Coleman, Isaac I.Y. Saito, Jochen Welle, Kei Okada, Michael Ferguson, Michael Görner, Robert Haschke, Sachin Chitta, Simon Schmeisser (isys vision), TheDash, Thomas Burghout, dg, v4hn

0.6.5 (2015-01-24)
------------------
* update maintainers
* Created new trajectory display, split from motion planning display
* Added new trajectory display inside of motion planning display
* Fix bug with alpha property in trajectory robot
* Optimized number of URDFs loaded
* Changed motion planning Rviz icon to MoveIt icon
* Add time factor support for iterative_time_parametrization
* Contributors: Dave Coleman, Michael Ferguson, kohlbrecher

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------
* fix duplicate planning attempt box, also fix warning about name
* Contributors: Michael Ferguson

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------
* Fixed joystick documentation
* Joystick documentation and queue_size addition
* Contributors: Dave Coleman

0.6.0 (2014-10-27)
------------------
* Added move_group capability for clearing octomap.
* Fix coding style according to the moveit style
* Better user output, kinematic solver error handling, disclaimer
* Remove sample launch file for joystick and update
  joystick python script.
  1) Use moveit-python binding to parse SRDF.
  2) Make the speed slower to control the marker from joystick.
  3) Change joystick button mapping to be suitable for the users.
* Update joystick documentation and rename the
  the launch file for joy stick program.
  Shorten the message the check box to toggle
  communication with joy stick script.
* add checkbox to toggle if moveit rviz plugin subscribes
  the topics to be used for communication to the external ros nodes.
  update moveit_joy.py to parse srdf to know planning_groups and the
  names of the end effectors and support multi-endeffector planning groups.
* motion_planning_rviz_plugin: add move_group namespace option
  This allows multiple motion_planning_rviz_plugin /
  planning_scene_rviz_plugin to be used in RViz and connect to
  differently-namespaced move_group nodes.
* moved planning_attempts down one row in gui to maintain gui width
* Added field next to planning_time for planning_attempts
  Now, ParallelPlanner terminates either due to timeout, or due to this many attempts.
  Note, that ParallelPlanner run's Dijkstra's on all the nodes of all the sucessful plans (hybridize==true).
* adding PoseStamped topic to move the interactive marker from other ros nodes
  such as joystick programs.
* motion_planning_rviz_plugin: add move_group namespace option
  This allows multiple motion_planning_rviz_plugin /
  planning_scene_rviz_plugin to be used in RViz and connect to
  differently-namespaced move_group nodes.
* Contributors: Chris Lewis, Dave Coleman, Dave Hershberger, Jonathan Bohren, Ryohei Ueda, Sachin Chitta

0.5.19 (2014-06-23)
-------------------
* Changed rviz plugin action server wait to non-simulated time
* Fix [-Wreorder] warning.
* Fix RobotState rviz plugin to not display when disabled
* Add check for planning scene monitor connection, with 5 sec delay
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.18 (2014-03-23)
-------------------
* add pkg-config as dep
* find PkgConfig before using pkg_check_modules
  PC specific functions mustn't be used before including PkgConfig
* Contributors: Ioan Sucan, v4hn

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* update maintainer e-mail
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------
* back out problematic ogre fixes
* robot_interaction: split InteractionHandler into its own file
* Switched from isStateColliding to isStateValid
* Changed per PR review
* Clean up debug output
* Added ability to set a random <collision free> start/goal position
* Merge branch 'hydro-devel' of https://github.com/ros-planning/moveit_ros into acorn_rviz_stereo
* rviz: prepare for Ogre1.10
* Contributors: Acorn Pooley, Dave Coleman

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------
* remove debug printfs
* planning_scene_display: use requestPlanningSceneState()
  Get current planning scene state when planning scene display is
  enabled and/or model is loaded.
* Fix Parse error at "BOOST_JOIN" error
  See: https://bugreports.qt-project.org/browse/QTBUG-22829
* Contributors: Acorn Pooley, Benjamin Chretien

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------
* Added back-link to tutorial and updated moveit website URL.
* Ported MoveIt RViz plugin tutorial to sphinx.
* Contributors: Dave Hershberger

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* correcting maintainer email
* Fixed an occasional crash bug in rviz plugin caused by gui calls in non-gui thread.
* Added planning feedback to gui, refactored states tab
* Stored states are auto loaded when warehouse database is connected

0.5.8 (2013-10-11)
------------------
* Added option to rviz plugin to show scene robot collision geometry

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* Fix crash when the destructor is called before onInitialize
* remove call for getting the combined joint limits of a group
* bugfixes
* porting to new RobotState API
* use new helper class from rviz for rendering meshes

0.5.4 (2013-08-14)
------------------

* Added manipulation tab, added plan id to manipulation request
* make headers and author definitions aligned the same way; white space fixes
* using action client for object recognition instead of topic
* move background_processing lib to core
* display collision pairs instead of simply colliding links

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* fix `#275 <https://github.com/ros-planning/moveit_ros/issues/275>`_
* white space fixes (tabs are now spaces)

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
* remove root_link_name property
* add status tab to Rviz plugin
