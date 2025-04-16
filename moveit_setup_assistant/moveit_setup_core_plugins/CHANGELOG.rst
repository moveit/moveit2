^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_setup_core_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.9 (2025-04-15)
------------------

2.5.8 (2025-02-09)
------------------
* Explicit convert from std::filesystem::path to std::string for Windows compatibility (backport `#3249 <https://github.com/ros-planning/moveit2/issues/3249>`_) (`#3253 <https://github.com/ros-planning/moveit2/issues/3253>`_)
* Contributors: Silvio Traversaro, mergify[bot]

2.5.7 (2024-12-29)
------------------

2.5.6 (2024-11-17)
------------------

2.5.5 (2023-09-10)
------------------
* Use <> for non-local headers (`#1765 <https://github.com/ros-planning/moveit2/issues/1765>`_)
  Unless a header lives in the same or a child directory of the file
  including it, it's recommended to use <> for the #include statement.
  For more information, see the C++ Core Guidelines item SF.12
  https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#sf12-prefer-the-quoted-form-of-include-for-files-relative-to-the-including-file-and-the-angle-bracket-form-everywhere-else
  (cherry picked from commit 7a1f2a101f9aeb8557e8a31656bbe1a6d53b430e)
* Contributors: Chris Thrasher

2.5.4 (2022-11-04)
------------------
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_) (`#1555 <https://github.com/ros-planning/moveit2/issues/1555>`_)
  Co-authored-by: Sebastian Jahr <sebastian.jahr@picknik.ai>
* Contributors: mergify[bot]

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Merge pull request `#1254 <https://github.com/ros-planning/moveit2/issues/1254>`_ from ros-planning/feature/msa
  MoveIt Setup Assistant - Merge the Feature branch
* [MSA] PR Feedback
* [MSA] Existing Package Loading Tweaks (`#1212 <https://github.com/ros-planning/moveit2/issues/1212>`_)
* [MSA] Three small edits (`#1203 <https://github.com/ros-planning/moveit2/issues/1203>`_)
* [MSA] Add Setup Step for Generating Launch Files (`#1129 <https://github.com/ros-planning/moveit2/issues/1129>`_)
* [MSA] Simplify loading of new SRDF (`#1102 <https://github.com/ros-planning/moveit2/issues/1102>`_)
* [MSA] Merge Upstream into `feature/msa` (`#1119 <https://github.com/ros-planning/moveit2/issues/1119>`_)
* [MSA] Initial Refactor
* Move Files Around (split into multiple packages, change to hpp extension)
* Contributors: David V. Lu!!, Vatan Aksoy Tezer

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
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
