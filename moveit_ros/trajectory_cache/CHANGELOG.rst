^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_trajectory_cache
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.15.1 (2026-08-29)
-------------------

2.15.0 (2026-08-12)
-------------------
* resolute: add explicit <cstdint> / <fstream> includes for GCC 15 (`#3754 <https://github.com/moveit/moveit2/issues/3754>`_)
* resolute: GCC 15 / C++20 / rviz API build fixes, and octomap -Werror=cpp suppression (`#3760 <https://github.com/moveit/moveit2/issues/3760>`_)
* De-flake trajectory_cache _with_move_group tests (`#3777 <https://github.com/moveit/moveit2/issues/3777>`_)
* Use modern --frame-id/--child-frame-id args for test static_transform_publisher (`#3762 <https://github.com/moveit/moveit2/issues/3762>`_)
* Remove use of ament_target_dependencies: Take 2 (`#3726 <https://github.com/moveit/moveit2/issues/3726>`_)
  ament_target_dependencies is replaced by exported CMake targets. Downstream
  packages should link the namespaced targets (e.g. moveit_ros_planning::moveit_ros_planning).
* Fix deprecations in image_common and tf2_ros (`#3567 <https://github.com/moveit/moveit2/issues/3567>`_)
* Contributors: Nathan Brooks

2.14.1 (2025-09-09)
-------------------

2.14.0 (2025-06-13)
-------------------

2.13.2 (2025-04-16)
-------------------

2.13.1 (2025-04-15)
-------------------

2.13.0 (2025-02-15)
-------------------
* Fuzzy-matching Trajectory Cache Injectable Traits refactor 🔥🔥 (`#2941 <https://github.com/ros-planning/moveit2/issues/2941>`_)
* Contributors: methylDragon

2.12.0 (2024-11-29)
-------------------
* Enhancement/use hpp for headers (`#3113 <https://github.com/ros-planning/moveit2/issues/3113>`_)
* Contributors: Tom Noble

2.11.0 (2024-09-16)
-------------------

0.1.0 (2024-05-17)
------------------
* Add ``moveit_ros_trajectory_cache`` package for trajectory caching.
