^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_configs_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.9 (2025-04-15)
------------------
* Update ompl_defaults.yaml to not have an invalid AnytimePathShortening configuration (`#3374 <https://github.com/ros-planning/moveit2/issues/3374>`_) (`#3375 <https://github.com/ros-planning/moveit2/issues/3375>`_)
* Contributors: Stephanie Eng

2.5.8 (2025-02-09)
------------------
* Switch to get for Dict lookup to prevent KeyError (`#3043 <https://github.com/ros-planning/moveit2/issues/3043>`_) (`#3193 <https://github.com/ros-planning/moveit2/issues/3193>`_)
* Contributors: Brendan Burns, mergify[bot]

2.5.7 (2024-12-29)
------------------
* Added joint limits to rviz launch file. (`#3091 <https://github.com/ros-planning/moveit2/issues/3091>`_) (`#3136 <https://github.com/ros-planning/moveit2/issues/3136>`_)
* Contributors: Matthew Elwin

2.5.6 (2024-11-17)
------------------
* fix move_group_capabilities usage (`#3018 <https://github.com/ros-planning/moveit2/issues/3018>`_) (`#3033 <https://github.com/ros-planning/moveit2/issues/3033>`_)
* Backport of `#2172 <https://github.com/ros-planning/moveit2/issues/2172>`_ and `#2684 <https://github.com/ros-planning/moveit2/issues/2684>`_ into Humble (`#2779 <https://github.com/ros-planning/moveit2/issues/2779>`_)
* Use different packages for launch and config packages in generate_demo_launch (backport `#2647 <https://github.com/ros-planning/moveit2/issues/2647>`_) (`#2650 <https://github.com/ros-planning/moveit2/issues/2650>`_)
* Pass along move_group_capabilities parameters (`#2587 <https://github.com/ros-planning/moveit2/issues/2587>`_) (`#2696 <https://github.com/ros-planning/moveit2/issues/2696>`_)
* Use $DISPLAY rather than assuming :0 (`#2049 <https://github.com/ros-planning/moveit2/issues/2049>`_) (`#2365 <https://github.com/ros-planning/moveit2/issues/2365>`_)
* Contributors: Michael Ferguson, Anthony Baker, Alex Navarro, Forrest Rogers-Marcovitz, Stephanie Eng, mergify[bot]

2.5.5 (2023-09-10)
------------------
* Do not add Pilz parameters to MoveIt Configs Utils if Pilz is not used (`#1583 <https://github.com/ros-planning/moveit2/issues/1583>`_) (`#2174 <https://github.com/ros-planning/moveit2/issues/2174>`_)
  (cherry picked from commit 1c7fa52edeef08bf8eb1e9cc73c1b0835aaf17e7)
  Co-authored-by: Stephanie Eng <stephanie-eng@users.noreply.github.com>
* Update default planning configs to use AddTimeOptimalParameterization (`#2167 <https://github.com/ros-planning/moveit2/issues/2167>`_) (`#2170 <https://github.com/ros-planning/moveit2/issues/2170>`_)
  (cherry picked from commit 895e9268bd5d9337bebdede07a7f68a99055a1df)
  Co-authored-by: Anthony Baker <abake48@users.noreply.github.com>
* Add xacro subsititution class and use it for loading urdf & srdf (backport `#1805 <https://github.com/ros-planning/moveit2/issues/1805>`_) (`#1937 <https://github.com/ros-planning/moveit2/issues/1937>`_)
  * Add xacro subsititution class and use it for loading urdf & srdf (`#1805 <https://github.com/ros-planning/moveit2/issues/1805>`_)
  * Add Xacro substitution type
  * Use Xacro substitution for robot description and robot description semantic
  * Install subsititution folder
  * Default to load_xacro if there's no launch substitution specified in the mappings
  (cherry picked from commit 4bc83c3c9e6bfa9efea8c431794a630fbf27dddc)
  # Conflicts:
  #	moveit_configs_utils/moveit_configs_utils/moveit_configs_builder.py
  * Fix merge conflicts
  ---------
  Co-authored-by: Jafar <cafer.abdi@gmail.com>
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
* Add support for multiple MoveItConfigBuilder instaces (`#1807 <https://github.com/ros-planning/moveit2/issues/1807>`_) (`#1808 <https://github.com/ros-planning/moveit2/issues/1808>`_)
  (cherry picked from commit 25d086cee9a7cf1c95a15ea12a27e5b7cbe50a1f)
  Co-authored-by: Marco Magri <94347649+MarcoMagriDev@users.noreply.github.com>
* Contributors: mergify[bot]

2.5.4 (2022-11-04)
------------------
* Use MoveItConfigsBuilder in Pilz test launch file (`#1571 <https://github.com/ros-planning/moveit2/issues/1571>`_) (`#1662 <https://github.com/ros-planning/moveit2/issues/1662>`_)
  (cherry picked from commit 5e880bacaad780f511ed99847050216a8b9905c1)
  Co-authored-by: Abishalini Sivaraman <abi.gpuram@gmail.com>
* Only require Cartesian limits if Pilz is used (`#1519 <https://github.com/ros-planning/moveit2/issues/1519>`_) (`#1653 <https://github.com/ros-planning/moveit2/issues/1653>`_)
  (cherry picked from commit 40f7f091cc77a683f3c0a4af64f6a463dd0846f1)
  Co-authored-by: Stephanie Eng <stephanie-eng@users.noreply.github.com>
* Typo fix (`#1518 <https://github.com/ros-planning/moveit2/issues/1518>`_) (`#1650 <https://github.com/ros-planning/moveit2/issues/1650>`_)
  (cherry picked from commit e89526de24d88fb05eb646a7925de69ba480bfe8)
  Co-authored-by: Stephanie Eng <stephanie-eng@users.noreply.github.com>
* Contributors: mergify[bot]

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Rename cartesian_limits.yaml (`#1422 <https://github.com/ros-planning/moveit2/issues/1422>`_)
  Co-authored-by: Tyler Weaver <maybe@tylerjw.dev>
* Merge pull request `#1402 <https://github.com/ros-planning/moveit2/issues/1402>`_ from Abishalini/pr-sync-a436a97
  Sync with MoveIt
* Merge https://github.com/ros-planning/moveit/commit/a436a9771f7445c162cc3090c4c7c57bdb5bf194
* load sensors_3d.yaml (`#1387 <https://github.com/ros-planning/moveit2/issues/1387>`_)
* Merge pull request `#1254 <https://github.com/ros-planning/moveit2/issues/1254>`_ from ros-planning/feature/msa
  MoveIt Setup Assistant - Merge the Feature branch
* [MSA] ros2_control Integration (`#1299 <https://github.com/ros-planning/moveit2/issues/1299>`_)
  Co-authored-by: AndyZe <andyz@utexas.edu>
* [MSA] Workaround to launch files without controllers (`#1275 <https://github.com/ros-planning/moveit2/issues/1275>`_)
  Co-authored-by: Jafar <jafar.uruc@gmail.com>
* Fix moveit_configs_utils parameter ordering (`#1315 <https://github.com/ros-planning/moveit2/issues/1315>`_)
* Remove TrajOpt Default Parameters (`#1332 <https://github.com/ros-planning/moveit2/issues/1332>`_)
  Co-authored-by: AndyZe <zelenak@picknik.ai>
* Decorate MoveItConfigs with dataclaass (`#1308 <https://github.com/ros-planning/moveit2/issues/1308>`_)
* moveit_config_utils: Change default rviz config file location (`#1316 <https://github.com/ros-planning/moveit2/issues/1316>`_)
* Contributors: Abishalini, AndyZe, David V. Lu!!, Jafar, Michael Ferguson, Tyler Weaver, Vatan Aksoy Tezer

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Add options to config for publishing description (`#1265 <https://github.com/ros-planning/moveit2/issues/1265>`_)
* Flexibility in moveit_configs_utils reading of .setup_assistant (`#1264 <https://github.com/ros-planning/moveit2/issues/1264>`_)
* Alternate Package Name Specification (`#1244 <https://github.com/ros-planning/moveit2/issues/1244>`_)
* Change condition for loading default OMPL config (`#1222 <https://github.com/ros-planning/moveit2/issues/1222>`_)
* Demo launch for moveit_configs_utils (`#1189 <https://github.com/ros-planning/moveit2/issues/1189>`_)
  * Demo launch for moveit_configs_utils
  * Don't respawn rviz
* move_group launch for moveit_configs_utils (`#1131 <https://github.com/ros-planning/moveit2/issues/1131>`_)
* Add launch file configurations for static tfs and spawning controllers (`#1176 <https://github.com/ros-planning/moveit2/issues/1176>`_)
* Fix Moveit Configs Utils Bug (`#1174 <https://github.com/ros-planning/moveit2/issues/1174>`_)
* New Launch Files using moveit_configs_utils (`#1113 <https://github.com/ros-planning/moveit2/issues/1113>`_)
  * Add Package Path
  * Launch Utils
  * Some Basic Launch Files
  * Remove circular dependencies
* [moveit_configs_utils] Minor fixes (`#1103 <https://github.com/ros-planning/moveit2/issues/1103>`_)
* Add moveit_configs_utils package to simplify loading paramters (`#591 <https://github.com/ros-planning/moveit2/issues/591>`_)
  Co-authored-by: AndyZe <zelenak@picknik.ai>
  Co-authored-by: Stephanie Eng <stephanie-eng@users.noreply.github.com>
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
* Contributors: David V. Lu!!, Jafar Abdi, Stephanie Eng, Tyler Weaver

* Add options to config for publishing description (`#1265 <https://github.com/ros-planning/moveit2/issues/1265>`_)
* Flexibility in moveit_configs_utils reading of .setup_assistant (`#1264 <https://github.com/ros-planning/moveit2/issues/1264>`_)
* Alternate Package Name Specification (`#1244 <https://github.com/ros-planning/moveit2/issues/1244>`_)
* Change condition for loading default OMPL config (`#1222 <https://github.com/ros-planning/moveit2/issues/1222>`_)
* Demo launch for moveit_configs_utils (`#1189 <https://github.com/ros-planning/moveit2/issues/1189>`_)
* move_group launch for moveit_configs_utils (`#1131 <https://github.com/ros-planning/moveit2/issues/1131>`_)
* Add launch file configurations for static tfs and spawning controllers (`#1176 <https://github.com/ros-planning/moveit2/issues/1176>`_)
* Fix Moveit Configs Utils Bug (`#1174 <https://github.com/ros-planning/moveit2/issues/1174>`_)
* New Launch Files using moveit_configs_utils (`#1113 <https://github.com/ros-planning/moveit2/issues/1113>`_)
* Contributors: David V. Lu!!, Jafar Abdi, Stephanie Eng, Tyler Weaver

2.4.0 (2022-01-20)
------------------

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------

2.3.0 (2021-10-08)
------------------

2.2.1 (2021-07-12)
------------------

2.2.0 (2021-06-30)
------------------

2.1.4 (2021-05-31)
------------------

2.1.3 (2021-05-22)
------------------

2.1.2 (2021-04-22)
------------------

2.1.1 (2021-04-13)
------------------

2.1.0 (2020-11-24)
------------------

2.0.0 (2020-05-13)
------------------
