^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_planners_stomp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Unify log names (`#2720 <https://github.com/moveit/moveit2/issues/2720>`_)
  Co-authored-by: Abishalini Sivaraman <abi.gpuram@gmail.com>
* CMake format and lint in pre-commit (`#2683 <https://github.com/moveit/moveit2/issues/2683>`_)
* missing destination path (`#2668 <https://github.com/moveit/moveit2/issues/2668>`_)
* Fix penalty-based cost function in STOMP (`#2625 <https://github.com/moveit/moveit2/issues/2625>`_)
  * Fix penalty-based cost function in STOMP
  This adds several test cases for STOMP's noise generation and cost
  functions, and provides the following fixes:
  * out-of-bounds vector access when tail states of trajectory are invalid
  * smoothed costs overriding values of previous invalid groups
  * missing validity check of last state in trajectory
  * inability to disable cost function interpolation steps
  * total cost of trajectory not summing up to sum of state penalties
  * bug in Gaussian producing infinite values with invalid start states
  * Improve documentation
  ---------
* Contributors: Henning Kayser, Robert Haschke, Sarvajith Adyanthaya, Sebastian Jahr, Tyler Weaver

2.9.0 (2024-01-09)
------------------
* Node logging for the rest of MoveIt (`#2599 <https://github.com/ros-planning/moveit2/issues/2599>`_)
* [Planning Pipeline Refactoring] `#2 <https://github.com/ros-planning/moveit2/issues/2>`_ Enable chaining planners (`#2457 <https://github.com/ros-planning/moveit2/issues/2457>`_)
  * Enable chaining multiple planners
* [Planning Pipeline Refactoring] `#1 <https://github.com/ros-planning/moveit2/issues/1>`_ Simplify Adapter - Planner chain (`#2429 <https://github.com/ros-planning/moveit2/issues/2429>`_)
* Add new clang-tidy style rules (`#2177 <https://github.com/ros-planning/moveit2/issues/2177>`_)
* Do not pass and return simple types by const ref (`#2453 <https://github.com/ros-planning/moveit2/issues/2453>`_)
  Co-authored-by: Nils <nilsmailiseke@gmail.com>
* Use constraint distance instead of bool validity in STOMP cost function (`#2418 <https://github.com/ros-planning/moveit2/issues/2418>`_)
  * Use constraint distance instead of bool validity in STOMP cost function
  * Fix comment
* Using std types instead of boost for Gaussian sampling (`#2351 <https://github.com/ros-planning/moveit2/issues/2351>`_)
  * *Changed boost namespace to std
  *Need to compare function implementations
  *Find an equivalent implementation for variate_generator
  * calling Distribution(Engine) directly
  * cleanup
  * Seed mersenne twister variable with random device
  * Updated with rsl random library
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Update clang-format-14 with QualifierAlignment (`#2362 <https://github.com/ros-planning/moveit2/issues/2362>`_)
  * Set qualifier order in .clang-format
  * Ran pre-commit to update according to new style guide
* Merge branch 'main' into dependabot/github_actions/SonarSource/sonarcloud-github-c-cpp-2
* Contributors: Henning Kayser, Marq Rasmussen, Sebastian Jahr, Shobuj Paul, Tyler Weaver

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
