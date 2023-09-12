^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_planners_stomp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.0 (2023-09-10)
------------------
* Remove added path index from planner adapter function signature (`#2285 <https://github.com/ros-planning/moveit2/issues/2285>`_)
* Always set response planner id and warn if it is not set (`#2236 <https://github.com/ros-planning/moveit2/issues/2236>`_)
* Contributors: Sebastian Jahr

2.7.4 (2023-05-18)
------------------
* Migrate STOMP from ros-planning/stomp_moveit (`#2158 <https://github.com/ros-planning/moveit2/issues/2158>`_)
* Fix clang-tidy warnings
* Improve Documentation and Readability
* Remove MVT example
* Migrate stomp_moveit into moveit_planners
  * Move package into moveit_planners subdirectory
  * Rename stomp_moveit package to moveit_planners_stomp
  * List moveit_planners_stomp as package dependency
* Contributors: Henning Kayser
