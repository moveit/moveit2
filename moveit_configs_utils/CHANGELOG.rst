^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_configs_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.0 (2022-11-10)
------------------
* Do not add Pilz parameters to MoveIt Configs Utils if Pilz is not used (`#1583 <https://github.com/ros-planning/moveit2/issues/1583>`_)
* Use MoveItConfigsBuilder in Pilz test launch file (`#1571 <https://github.com/ros-planning/moveit2/issues/1571>`_)
* Only require Cartesian limits if Pilz is used (`#1519 <https://github.com/ros-planning/moveit2/issues/1519>`_)
* Typo fix (`#1518 <https://github.com/ros-planning/moveit2/issues/1518>`_)
* Contributors: Abishalini Sivaraman, Stephanie Eng

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
