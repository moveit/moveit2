^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_benchmarks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.12.3 (2025-04-15)
-------------------

2.12.2 (2025-02-15)
-------------------

2.12.1 (2024-12-18)
-------------------

2.12.0 (2024-11-29)
-------------------
* Enhancement/use hpp for headers (`#3113 <https://github.com/ros-planning/moveit2/issues/3113>`_)
* Add use_padding flag + deprecate checkCollisionUnpadded() functions (`#3088 <https://github.com/ros-planning/moveit2/issues/3088>`_)
* Contributors: Sebastian Jahr, Tom Noble

2.11.0 (2024-09-16)
-------------------

2.10.0 (2024-06-13)
-------------------
* Migrate ros-planning org to moveit (`#2847 <https://github.com/moveit/moveit2/issues/2847>`_)
  * Rename github.com/ros-planning -> github.com/moveit
  * Rename ros-planning.github.io -> moveit.github.io
  * Rename ros-planning organization in docker and CI workflow files
  - ghcr.io/ros-planning -> ghcr.io/moveit
  - github.repository == 'moveit/*''
* Fix CI for Rolling / Ubuntu Noble (`#2793 <https://github.com/moveit/moveit2/issues/2793>`_)
  * docker.yaml: Enable caching
  * [TEMP] moveit2_rolling.repos: add not yet released packages
  * Skip broken ci-testing image: osrf/ros2:testing doesn't contain /opt/ros!
  * use boost::timer::progress_display if available
  check for header to stay compatible with ubuntu 20.04.
  Support boost >= 1.83
  Slightly ugly due to the double alias, but boost::timer was a class
  before 1.72, so using `boost::timer::progress_display` in the code
  breaks with older versions.
  * cherry-pick of `#3547 <https://github.com/moveit/moveit2/issues/3547>`_ from MoveIt1
  * Tag ci image as ci-testing as well
  ---------
  Co-authored-by: Michael Görner <me@v4hn.de>
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Update deprecated include of boost/progress.hpp to boost/timer/progress_display.hpp (`#2811 <https://github.com/moveit/moveit2/issues/2811>`_)
* Unify log names (`#2720 <https://github.com/moveit/moveit2/issues/2720>`_)
  Co-authored-by: Abishalini Sivaraman <abi.gpuram@gmail.com>
* CMake format and lint in pre-commit (`#2683 <https://github.com/moveit/moveit2/issues/2683>`_)
* Contributors: Robert Haschke, Sebastian Jahr, Stephanie Eng, Tyler Weaver

2.9.0 (2024-01-09)
------------------
* [Planning Pipeline Refactoring] `#2 <https://github.com/ros-planning/moveit2/issues/2>`_ Enable chaining planners (`#2457 <https://github.com/ros-planning/moveit2/issues/2457>`_)
  * Enable chaining multiple planners
* Node logging in moveit_core (`#2503 <https://github.com/ros-planning/moveit2/issues/2503>`_)
* Use node logging in moveit_ros (`#2482 <https://github.com/ros-planning/moveit2/issues/2482>`_)
* Add new clang-tidy style rules (`#2177 <https://github.com/ros-planning/moveit2/issues/2177>`_)
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
* Enable running parallel or single pipeline benchmarks (`#2385 <https://github.com/ros-planning/moveit2/issues/2385>`_)
  * Enable running single or parallel planning pipeline benchmarks
  * Decrease log severity
* Update clang-format-14 with QualifierAlignment (`#2362 <https://github.com/ros-planning/moveit2/issues/2362>`_)
  * Set qualifier order in .clang-format
  * Ran pre-commit to update according to new style guide
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Contributors: Marq Rasmussen, Sebastian Jahr, Shobuj Paul, Tyler Weaver

2.8.0 (2023-09-10)
------------------

2.7.4 (2023-05-18)
------------------

2.7.3 (2023-04-24)
------------------
* Replace Variable PROJECT_NAME in CMakeLists.txt with the actual name (`#2020 <https://github.com/ros-planning/moveit2/issues/2020>`_)
* Contributors: Shobuj Paul

2.7.2 (2023-04-18)
------------------
* Update pre-commit (`#2094 <https://github.com/ros-planning/moveit2/issues/2094>`_)
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
* Contributors: Sebastian Jahr, Shobuj Paul

2.7.1 (2023-03-23)
------------------
* Fix member naming (`#1949 <https://github.com/ros-planning/moveit2/issues/1949>`_)
  * Update clang-tidy rules for readability-identifier-naming
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Benchmark parallel planning pipelines (`#1539 <https://github.com/ros-planning/moveit2/issues/1539>`_)
  * Remove launch and config files (moved to moveit_resources)
* remove underscore from public member in MotionPlanResponse (`#1939 <https://github.com/ros-planning/moveit2/issues/1939>`_)
  * remove underscore from private members
  * fix more uses of the suffix notation
* Contributors: AlexWebb, Robert Haschke, Sebastian Jahr

2.7.0 (2023-01-29)
------------------
* converted characters from string format to character format (`#1881 <https://github.com/ros-planning/moveit2/issues/1881>`_)
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Remove `MOVEIT_LIB_NAME` (`#1751 <https://github.com/ros-planning/moveit2/issues/1751>`_)
  It's more readable and searchable if we just spell out the target
  name.
* Add braces around blocks. (`#999 <https://github.com/ros-planning/moveit2/issues/999>`_)
* Used C++ style cast instead of C style cast  (`#1628 <https://github.com/ros-planning/moveit2/issues/1628>`_)
  Co-authored-by: Henning Kayser <henningkayser@picknik.ai>
* Contributors: Abhijeet Das Gupta, Chris Thrasher, Christian Henkel, Cory Crean, Sameer Gupta

2.6.0 (2022-11-10)
------------------
* Remove unused benchmark_execution.cpp file (`#1535 <https://github.com/ros-planning/moveit2/issues/1535>`_)
* Merge PR `#1553 <https://github.com/ros-planning/moveit2/issues/1553>`_: Improve cmake files
* Use standard exported targets: export\_${PROJECT_NAME} -> ${PROJECT_NAME}Targets
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* Free functions for calculating properties of trajectories (`#1503 <https://github.com/ros-planning/moveit2/issues/1503>`_)
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Remove __has_include statements (`#1481 <https://github.com/ros-planning/moveit2/issues/1481>`_)
* Contributors: Robert Haschke, Sebastian Jahr, Tyler Weaver, Vatan Aksoy Tezer

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Merge https://github.com/ros-planning/moveit/commit/c88f6fb64e9057a4b9a8f6fafc01060e8c48a216
* Merge remote-tracking branch 'origin/main' into feature/msa
* Removing more boost usage (`#1372 <https://github.com/ros-planning/moveit2/issues/1372>`_)
* Merge remote-tracking branch 'upstream/main' into feature/msa
* Removing some boost usage (`#1331 <https://github.com/ros-planning/moveit2/issues/1331>`_)
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Remove unnecessary rclcpp.hpp includes (`#1333 <https://github.com/ros-planning/moveit2/issues/1333>`_)
* Fix namespace of planning plugin for benchmarks examples (`#3128 <https://github.com/ros-planning/moveit2/issues/3128>`_)
  Since `#2888 <https://github.com/ros-planning/moveit2/issues/2888>`_ / https://github.com/ros-planning/moveit_resources/pull/92
  `planning_pipeline.launch.xml` loads into `pipeline` namespace by default,
  thus making an explicit specification of `ompl` redundant.
* Merge pull request `#3106 <https://github.com/ros-planning/moveit2/issues/3106>`_ from v4hn/pr-master-bind-them-all / banish bind()
* various: prefer objects and references over pointers
* Contributors: Abishalini, David V. Lu, Henry Moore, Jafar, Michael Görner, Robert Haschke, Vatan Aksoy Tezer, v4hn

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Merge https://github.com/ros-planning/moveit/commit/72d919299796bffc21f5eb752d66177841dc3442
* Enable cppcheck (`#1224 <https://github.com/ros-planning/moveit2/issues/1224>`_)
  Co-authored-by: jeoseo <jeongwooseo2012@gmail.com>
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
  Adaption of https://github.com/ros-planning/moveit/pull/3106
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* [moveit_ros_benchmarks] Add missing moveit_core dependency (`#1157 <https://github.com/ros-planning/moveit2/issues/1157>`_)
* Merge https://github.com/ros-planning/moveit/commit/a25515b73d682df03ed3eccd839110c296aa79fc
* Merge https://github.com/ros-planning/moveit/commit/ab42a1d7017b27eb6c353fb29331b2da08ab0039
* Filter more invalid values in moveit_benchmark_statistics.py (`#3084 <https://github.com/ros-planning/moveit2/issues/3084>`_)
  Count "-nan" and "-inf" as null value in the database.
* 1.1.9
* 1.1.8
* Remove unused parameters. (`#1018 <https://github.com/ros-planning/moveit2/issues/1018>`_)
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Add moveit_configs_utils package to simplify loading paramters (`#591 <https://github.com/ros-planning/moveit2/issues/591>`_)
  Co-authored-by: AndyZe <zelenak@picknik.ai>
  Co-authored-by: Stephanie Eng <stephanie-eng@users.noreply.github.com>
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
* 1.1.7
* Provide MOVEIT_VERSION_CHECK macro (`#2997 <https://github.com/ros-planning/moveit2/issues/2997>`_)
  - Rename MOVEIT_VERSION -> MOVEIT_VERSION_STR
  - MOVEIT_VERSION becomes a numeric identifier
  - Use like: #if MOVEIT_VERSION >= MOVEIT_VERSION_CHECK(1, 0, 0)
* Switch to std::bind (`#2967 <https://github.com/ros-planning/moveit2/issues/2967>`_)
  * boost::bind -> std::bind
  grep -rlI --exclude-dir=.git "boost::bind" | xargs sed -i 's/boost::bind/std::bind/g'
  * Convert bind placeholders
  grep -rlI --exclude-dir=.git " _[0-9]" | xargs sed -i 's/ _\([0-9]\)/ std::placeholders::_\1/g'
  * Update bind include header
  grep -rlI --exclude-dir=.git "boost/bind" | xargs sed -i 's#boost/bind.hpp#functional#'
* 1.1.6
* Contributors: Abishalini, Cory Crean, Gaël Écorchard, Henning Kayser, Hugal31, Jafar, Jafar Abdi, Jochen Sprickerhof, Robert Haschke, jeoseo, v4hn

2.4.0 (2022-01-20)
------------------
* Merge https://github.com/ros-planning/moveit/commit/f3ac6070497da90da33551fc1dc3a68938340413
* moveit_build_options()
  Declare common build options like CMAKE_CXX_STANDARD, CMAKE_BUILD_TYPE,
  and compiler options (namely warning flags) once.
  Each package depending on moveit_core can use these via moveit_build_options().
* Contributors: Abishalini, Robert Haschke

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Replaced C-Style Cast with C++ Style Cast. (`#935 <https://github.com/ros-planning/moveit2/issues/935>`_)
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* Get rid of "std::endl" (`#918 <https://github.com/ros-planning/moveit2/issues/918>`_)
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* Update Maintainers of MoveIt package (`#697 <https://github.com/ros-planning/moveit2/issues/697>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: Dave Coleman, David V. Lu!!, Henning Kayser, Kaustubh, Parthasarathy Bana, Robert Haschke, pvanlaar

2.3.0 (2021-10-08)
------------------
* Fix warnings in Galactic and Rolling (`#598 <https://github.com/ros-planning/moveit2/issues/598>`_)
  * Use __has_includes preprocessor directive for deprecated headers
  * Fix parameter template types
  * Proper initialization of smart pointers, rclcpp::Duration
* Fix loading joint_limits.yaml in demo and test launch files (`#544 <https://github.com/ros-planning/moveit2/issues/544>`_)
* Fixes for Windows (`#530 <https://github.com/ros-planning/moveit2/issues/530>`_)
* Fix predefined poses benchmark example (`#2718 <https://github.com/ros-planning/moveit2/issues/2718>`_)
* Contributors: Akash, Captain Yoshi, Jafar Abdi, Vatan Aksoy Tezer, Nisala Kalupahana, Jorge Nicho, Henning Kayser, Tyler Weaver, Lior Lustgarten

2.2.1 (2021-07-12)
------------------

2.2.0 (2021-06-30)
------------------

2.1.4 (2021-05-31)
------------------

2.1.3 (2021-05-22)
------------------

2.1.2 (2021-04-20)
------------------
* Unify PickNik name in copyrights (`#419 <https://github.com/ros-planning/moveit2/issues/419>`_)
* Contributors: Tyler Weaver

2.1.1 (2021-04-12)
------------------
* Fix EXPORT install in CMake (`#372 <https://github.com/ros-planning/moveit2/issues/372>`_)
* Fix repo URLs in package.xml files
* Contributors: Henning Kayser, Tyler Weaver

2.1.0 (2020-11-23)
------------------
* [maint] Wrap common cmake code in 'moveit_package()' macro (`#285 <https://github.com/ros-planning/moveit2/issues/285>`_)
  * New moveit_package() macro for compile flags, Windows support etc
  * Add package 'moveit_common' as build dependency for moveit_package()
  * Added -Wno-overloaded-virtual compiler flag for moveit_ros_planners_ompl
* [ros2-migration] Enable warehouse in moveit_ros_benchmarks (`#301 <https://github.com/ros-planning/moveit2/issues/301>`_)
* [ros2-migration] Port moveit_ros_benchmarks to ROS 2 (`#225 <https://github.com/ros-planning/moveit2/issues/225>`_)
* Contributors: Lior Lustgarten, Yu Yan

1.1.1 (2020-10-13)
------------------
* [fix] python3 issues (`#2323 <https://github.com/ros-planning/moveit/issues/2323>`_)
* [maint] Cleanup MSA includes (`#2351 <https://github.com/ros-planning/moveit/issues/2351>`_)
* Contributors: Michael Görner, Robert Haschke

1.1.0 (2020-09-04)
------------------
* [feature] Benchmark combinations of predefined poses (`#1548 <https://github.com/ros-planning/moveit/issues/1548>`_)
* [feature] Support benchmarking of full planning pipelines (`#1531 <https://github.com/ros-planning/moveit/issues/1531>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Fix plot details, correcting xlabels positions and cleaning the graph (`#1658 <https://github.com/ros-planning/moveit/issues/1658>`_) (`#1668 <https://github.com/ros-planning/moveit/issues/1668>`_)
* [maint] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `#2004 <https://github.com/ros-planning/moveit/issues/2004>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Fix usage of panda_moveit_config (`#1904 <https://github.com/ros-planning/moveit/issues/1904>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] Do not install helper scripts in global bin destination (`#1704 <https://github.com/ros-planning/moveit/issues/1704>`_)
* [maint] Cleanup launch + config files (`#1631 <https://github.com/ros-planning/moveit/issues/1631>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Ayush Garg, Dave Coleman, Henning Kayser, Jonathan Binney, Mahmoud Ahmed Selim, Markus Vieth, Michael Görner, Robert Haschke, Sean Yen, Tyler Weaver, Yu, Yan

1.0.6 (2020-08-19)
------------------
* [maint] Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint] Migrate to clang-format-10, Fix warnings
* [maint] Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* Contributors: Markus Vieth, Robert Haschke

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [feature] MoveIt benchmark improvements (`#1510 <https://github.com/ros-planning/moveit/issues/1510>`_)
  * Add pseudo experiment all_experiments to allow comparing all entries
  * Expose loadBenchmarkQueryData() for setting up custom queries
  * Add benchmark entry for comparing the 'final' result trajectory
  * Add trajectory similarity function to measure repeatability
  * Address requested changes
  * Fill empty fields in all_experiments
  * Improve variable and function names
  * Add helper function computeTrajectoryDistance()
* [maint]   Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint]   Windows build fixes
  * Fix header inclusion and other MSVC build errors (`#1636 <https://github.com/ros-planning/moveit/issues/1636>`_)
  * Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* Contributors: Henning Kayser, Michael Görner, Robert Haschke, Sean Yen, Yu, Yan

1.0.2 (2019-06-28)
------------------
* [maintenance] Removed unnecessary null pointer checks on deletion (`#1410 <https://github.com/ros-planning/moveit/issues/1410>`_)
* Contributors: Mahmoud Ahmed Selim

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* Contributors: Dave Coleman, Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* Contributors: Robert Haschke

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------
* [maintenance] Use locale independent conversion from double to string (`#1099 <https://github.com/ros-planning/moveit/issues/1099>`_)
* Contributors: Simon Schmeisser

0.10.2 (2018-10-24)
-------------------
* [capability] Benchmarking with different Motion Planners (STOMP, CHOMP, OMPL) (`#992 <https://github.com/ros-planning/moveit/issues/992>`_)
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* Contributors: Mohmmad Ayman, Raghavender Sahdev, Robert Haschke, mike lautman

0.10.1 (2018-05-25)
-------------------
* migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* Contributors: Ian McMahon

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] benchmarks: always prefer local header over system installations `#630 <https://github.com/ros-planning/moveit/issues/630>`_
* Contributors: Jorge Nicho, v4hn

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [improve] Add install rule for examples, statistics script
* Contributors: Bence Magyar

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman

* [enhancement] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman

0.9.3 (2016-11-16)
------------------
* 0.9.3 (catkin_prepare_release again missed increment as http://answers.ros.org/question/245969/catkin_prepare_release-not-bumping-packages-in-a-certain-folder
* Merge pull request `#330 <https://github.com/ros-planning/moveit/issues/330>`_ from davetcoleman/kinetic-package.xml
  Updated package.xml maintainers and author emails
* Updated package.xml maintainers and author emails
* Contributors: Dave Coleman, Ian McMahon, Isaac I.Y. Saito

0.9.2 (2016-11-05)
------------------
* Versions that didn't get bumped by catkin_prepare_release.
* Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman, Isaac I.Y. Saito

0.9.1 (2016-10-21)
------------------
* add full VERSIONs / SONAMEs to all libraries (`#273 <https://github.com/ros-planning/moveit/issues/273>`_)
* More version consolidation for all package.xml in the moveit repo, which are not even going to be released (addition to https://github.com/ros-planning/moveit/commit/fcb8df12dead9e5a62b276c46bb0ac6e2411daca).
* More version down for release preparation to consolidate version of to-be released packages (addition to https://github.com/ros-planning/moveit/commit/56a3c6fcd39ca0b548998f04a688655d5133abe0)
* Cleanup readme (`#258 <https://github.com/ros-planning/moveit/issues/258>`_)
* Convert assorted internal shared_ptrs.
* Switch to std::unique_ptr (instead of boost::scoped_ptr).
* Use shared_ptr typedefs in BenchmarkExecutor.cpp
* Convert pluginlibs ``shared_ptrs`` to ``std::``
* Code review fixup
  Remove package benchmark_gui
  clang-format Benchmarks package
* Changes for warehouse refactor to single "moveit" repo
* New benchmarks suite from Rice
* [package.xml] Fix repository URLs. (`#194 <https://github.com/ros-planning/moveit/issues/194>`_)
* Use MOVEIT_CLASS_FORWARD for moveit classes in moveit_ros. (`#182 <https://github.com/ros-planning/moveit/issues/182>`_)
* Switched to C++11
* Contributors: Dave Coleman, Isaac I.Y. Saito, Maarten de Vries, Michael Görner, Sachin Chitta, root

0.8.3 (2016-08-21)
------------------
* [jade] More Manual adjustment of package.xml versions to 0.8.3. Remove moveit_ikfast for now (see https://github.com/ros-planning/moveit/issues/22#issuecomment-241199671). (`#96 <https://github.com/ros-planning/moveit/issues/96>`_)
* [Jade] Unify package version numbers (see https://github.com/davetcoleman/moveit_merge/issues/9). (`#79 <https://github.com/ros-planning/moveit/issues/79>`_)
* Modifications for warehouse_ros refactor (`#699 <https://github.com/ros-planning/moveit/issues/699>`_)
* Contributors: Dave Coleman, Isaac I.Y. Saito, Michael Ferguson

0.7.6 (2016-12-30)
------------------
* changelog 0.7.6
* Contributors: Isaac I.Y. Saito

0.7.5 (2016-12-25)
------------------
* changelog 0.7.5
* Contributors: Isaac I.Y. Saito

0.7.4 (2016-12-22)
------------------
* [indigo][changelog] Add blank 0.7.3 section to those that are missing it.
  Reason why doing this:
  - catkin_generate_changelog gets stuck for some reason so batch generating changelog isn't possible now.
  - Since this is the first release since 6 month ago for Indigo, lots of commit logs since then that shouldn't be wasted.
  - Decided to bump version of all packages uniformely to 0.7.4 in the hope for catkin_generate_changelog to function...
  - Turned out the accumulated commit logs are not retrieved...But we might as well want to move forward to fix https://github.com/ros-planning/moveit/issues/386
* Contributors: Isaac I.Y. Saito

0.7.3 (2016-12-20)
------------------
* add full VERSIONs / SONAMEs to all libraries (`#273 <https://github.com/ros-planning/moveit/issues/273>`_)
  This is similar to `#273 <https://github.com/ros-planning/moveit/issues/273>`_ / 0a7a895bb2ae9e171efa101f354826366fa5eaff,
  but hard-codes the version for each library instead of using the project's version.
  Thus, we have to bump the version of a library *manually* if we break ABI in a release.
  === Below is the original commit message of the patch targeting the kinetic branch.
  * add full VERSIONs / SONAMEs to all core libraries
  As a result the libraries do not install as `libmoveit_xyz.so` anymore,
  but as `libmoveit_xyz.so.${MOVEIT_VERSION}` and only provide `libmoveit_xyz.so`
  as a symlink pointing to the versioned file.
  Because this sets each library's SONAME to the *full version*, this enforces
  that *every* binary links with the versioned library file from now on and
  has to be relinked with *each* new release of MoveIt.
  The alternative would be to set the SONAME to `$MAJOR.$MINOR` and ignore the patch version,
  but because we currently stay with one `$MAJOR.$MINOR` number within each ROS distribution,
  we had (and likely will have) ABI changes in the `$PATCH` version releases too.
  The reason for this commit is that it is practically impossible to maintain full ABI compatibility
  within each ROS distribution and still add the the features/patches the community asks for.
  This has resulted in more than one ABI-incompatible MoveIt release in the recent past
  within a ROS distribution. Because the libraries have not been versioned up to now,
  there was no way to indicate the incompatible changes and users who did not rebuild
  their whole workspace with the new release encountered weird and hard-to-track segfaults
  or broken behavior.
  * add SONAMES to all non-core libraries too
* Auto code formatted Indigo branch using clang-format (`#313 <https://github.com/ros-planning/moveit/issues/313>`_)
* [package.xml] Fix repository URLs. (`#194 <https://github.com/ros-planning/moveit/issues/194>`_)
* Use MOVEIT_CLASS_FORWARD for moveit classes in moveit_ros. (`#182 <https://github.com/ros-planning/moveit/issues/182>`_) (`#183 <https://github.com/ros-planning/moveit/issues/183>`_)
* 0.7.2
* changelog 0.7.2
* 0.7.1
* changelog 0.7.1
* 0.7.0
* preparing for 0.7
* Removed trailing whitespace from entire repository
* Adding tf dep fixes `#572 <https://github.com/ros-planning/moveit/issues/572>`_
* 0.6.5
* update changelogs
* add myself as maintainer, update/remove old maintainer emails
* 0.6.4
* update changelogs
* install moveit_benchmark_statistics.py
* 0.6.3
* update changelogs
* Add missing include of scoped_ptr
* 0.6.2
* update changelog
* 0.6.1
* update changelog
* 0.6.0
* update changelog
* Removed PlanningContext clear before planning call
* 0.5.19
* 0.5.19
* Removed PlanningContext clear before planning call
* 0.5.19
* 0.5.19
* benchmarks: add missing include.
* Fix broken log & output statements.
  - Address [cppcheck: coutCerrMisusage] and [-Werror=format-extra-args] errors.
  - ROS_ERROR -> ROS_ERROR_NAMED.
  - Print size_t values portably.
* Address [-Wsign-compare] warning.
* 0.5.18
* update changelog
* 0.5.17
* update changelog
* update build system for ROS indigo
* update maintainer e-mail
* 0.5.16
* changes for release
* 0.5.15
* 0.5.14
* preparing for 0.5.14
* 0.5.13
* changelogs for release
* "0.5.12"
* Changelogs for release.
* "0.5.11"
* Changelogs for release.
* "0.5.10"
* update changelogs
* "0.5.9"
* changelogs for 0.5.9
* Cleaned up var names and debug output
* 0.5.8
* update changelog
* update changelog
* 0.5.7
* update changelog
* 0.5.6
* update changelog
* 0.5.5
* update changelog
* update changelog
* add missing include
* more porting to new APi
* more porting to new API
* 0.5.4
* update changelog
* 0.5.3
* update changelog
* make headers and author definitions aligned the same way; white space fixes
* 0.5.2
* update changelog
* 0.5.1
* update changelog
* update changelog
* 0.5.0
* white space fixes (tabs are now spaces)
* 0.4.5
* update changelog
* port to new base class for planning_interface (using planning contexts)
* Fixed per Ioan's code review
* 0.4.4
* add changelog files
* Code cleanup
* Merge branch 'groovy-devel' of github.com:davetcoleman/moveit_ros into groovy-devel
* merge fixes
* 0.4.3
* 0.4.2
* 0.4.1
* 0.4.0
* 0.3.32
* 0.3.31
* Changed for fractional factorial analysis
* More advanced parameter sweeping implmented, workspace bounds added
* Added parameter sweeping to benchmarking
* Added ability to store the goal name - the query, constraint, traj constraint, etc
* Added new command line arguments and ability to export all experiments to csv file
* remove obsolete files
* Fixed building of benchmarks for boost program_options 1.49.0.1
* 0.3.30
* 0.3.29
* 0.3.28
* 0.3.27
* 0.3.26
* using new namespace parameter in planner plugin configuration
* move benchmark gui to a separate package
* change default plugin name
* robustness fix
* refactor benchmarks into lib + executable
* using new namespace parameter in planner plugin configuration
* move benchmark gui to a separate package
* change default plugin name
* robustness fix
* refactor benchmarks into lib + executable
* add names for background jobs (eases debugging), changed the threading for how robot model is loaded (previous version had race conditions), fix some issues with incorrect usage of marker scale
* moved job management to planning scene rviz plugin, moved scene monitor initialization to background
* reorder some includes
* Fixed github url name
* Renamed variable to be more specific
* Added debug output if user tries wrong planner. This is useful if they forget the 'left_arm[' part
* Made help the default option if no params passed
* 0.3.25
* 0.3.24
* remove alignment tag from .ui, only supported in recent versions
* 0.3.23
* added goal existance checks
* show progress bar when loading a robot
* benchmark tool now includes goal offsets in the output config file
* Multiple fixes in benchmark tool. Added end effector offsets
* 0.3.22
* Use NonConst suffix
* Add multi-collision to PlanningScene
* Switch from CollisionWorld to World
* minor fix
* minor bugfix
* bugfix for benchmarking
* minor bugfix
* generate benchmark config file dialog
* new run benchmark dialog, functionality to be implemented
* fixes and interpolated ik visualization
* Merge branch 'groovy-devel' of https://github.com/ros-planning/moveit_ros into animate_trajectory
* renamed kinematic_model to robot_model, robot_model_loader to rdf_loader and planning_models_loader to robot_model_loader
* call to computeCartesianPath and visualize results
* 0.3.21
* 0.3.20
* 0.3.19
* build fixes for quantal
* 0.3.18
* missling lib for linking
* 0.3.17
* complete renaming process
* fix merge conflict
* support for cartesian trajectories in benchmarks
* load benchmark results for cartesian trajectories, only reachability for now
* sets trajectory waypoint names
* rename KinematicState to RobotState, KinematicTrajectory to RobotTrajectory
* Reset goals and trajectories when switching scenes
* Update trajectory regex when loading a scene
* History of most used databases
* Remember database url, ui fixes
* Store and load cartesian trajectories to/from the warehouse
* use new robot_trajectory lib
* waypoints for trajectories
* remove trajectories, ui fixes
* cleaning and authors
* use kinematic_state_visualization from render_tools
* fixed cmake warning
* Merge branch 'groovy-devel' of https://github.com/ros-planning/moveit_ros into marioprats/render_shapes_fix
* ui fixes
* Cleaning and better handling of signal connection
* 0.3.16
* specify start and endpoints of a trajectory
* started trajectories
* added robot_interaction and some fixes
* update to moveit changes
* ui improvements, some error checking
* Added goals and states. Switch between robots
* Double clicking on a scene loads it
* Set alpha to 1.0 by default. GUI fixes
* Use PlanningSceneDisplay for the scene monitor and rendering
* First version of benchmark tool
* API updates needed for planning interface changes in moveit_core; more importantly, plan_execution is now split into plan_with_sensing plan_execution; there is now the notion of an ExecutableMotionPlan, which can also represent results from pick& place actions; this allows us to reuse the replanning code & looking around code we had for planning in pick& place. Added callbacks for repairing motion plans
* 0.3.15
* Author names
* upadte build flags
* 0.3.14
* 0.3.13
* fixing typo
* 0.3.12
* Fix kinematic state initialization in kinematic benchmark
* 0.3.11
* 0.3.10
* 0.3.9
* 0.3.8
* 0.3.7
* 0.3.6
* 0.3.5
* 0.3.4
* overload getPlanningQueriesNames for regex use
* Include translation offset in the transform
* Added translation offsets and optionality
* Option to specify a rotation offset to apply to the goals
* Print progress info in call_benchmark
* added option for default number of ik attempts
* refactor benchmarking code
* a bit of cleaning
* call_kinematic_benchmark and benchmark_config refactor
* run_kinematic_benchmark service
* Output to file
* Initial kinematic bencharking tool
* fix buildtool tag
* fix `#83 <https://github.com/ros-planning/moveit/issues/83>`_
* warehouse now overwrites records with the same name
* 0.3.3
* Warn the user before removing constraints on the database
* handling exceptions during benchmarking as well
* Clear previous start states when loading a scene
* making some includes SYSTEM and re-adding link_directories
* fixes catkin cmake issues
* add timeout option
* add planning frame option
* remove references to PlannerCapabilities
* 0.3.2
* add the option to specify the link to constrain
* change how we return results to avoid apparent ros::service issue
* 0.3.1
* add group override option
* minor fixes for running benchmarks
* 0.3.0
* using the new warehouse functionality in the benchmarks
* 0.2.29
* 0.2.28
* 0.2.27
* 0.2.26
* update example
* add construction of demo dbs; multiple feature enhancements for warehouse + benchmarks
* add demos
* add demos
* minor fixes for loading plugins
* 0.2.25
* minor fix
* 0.2.24
* using specification of start states in benchmarking
* more work on computing benchmarks when goal is specified as poses
* separate benchmark lib
* 0.2.23
* 0.2.22
* 0.2.21
* 0.2.20
* 0.2.19
* 0.2.18
* 0.2.17
* 0.2.16
* 0.2.15
* 0.2.14
* 0.2.13
* 0.2.12
* 0.2.11
* 0.2.10
* 0.2.9
* 0.2.8
* 0.2.7
* 0.2.6
* 0.2.5
* 0.2.4
* 0.2.3
* 0.2.2
* add some command line options
* fix include locations again
* add dummy manipulation pkg; bump versions, fix install targets
* update linked libs, install python pkgs + bump version
* rename folders
* build system for moveit_ros_benchmarks
* moving things around
* Contributors: Acorn, Adam Leeper, Adolfo Rodriguez Tsouroukdissian, Benjamin Chrétien, Dave Coleman, Dave Hershberger, Ioan Sucan, Isaac I.Y. Saito, Mario Prats, Michael Ferguson, Michael Görner, Mr-Yellow, Paul Mathieu, Sachin Chitta, arjungm, isucan, v4hn
