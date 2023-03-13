^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pilz_industrial_motion_planner_testutils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.7.0 (2023-01-29)
------------------
* Merge PR `#1712 <https://github.com/ros-planning/moveit2/issues/1712>`_: fix clang compiler warnings + stricter CI
* converted characters from string format to character format (`#1881 <https://github.com/ros-planning/moveit2/issues/1881>`_)
* Add noexcept specifier to constructors
* Add default constructors
  ... as they are not implicitly declared anymore
* Add default copy/move constructors/assignment operators
  As a user-declared destructor deletes any implicitly-defined move constructor/assignment operator,
  we need to declared them manually. This in turn requires to declare the copy constructor/assignment as well.
* Fix -Wdelete-non-abstract-non-virtual-dtor
* Fix BSD license in package.xml (`#1796 <https://github.com/ros-planning/moveit2/issues/1796>`_)
  * fix BSD license in package.xml
  * this must also be spdx compliant
* Minimize use of `this->` (`#1784 <https://github.com/ros-planning/moveit2/issues/1784>`_)
  It's often unnecessary. MoveIt already avoids this in most cases
  so this PR better cements that existing pattern.
* Use <> for non-local headers (`#1734 <https://github.com/ros-planning/moveit2/issues/1734>`_)
  Unless a header lives in the same or a child directory of the file
  including it, it's recommended to use <> for the #include statement.
  For more information, see the C++ Core Guidelines item SF.12
  https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#sf12-prefer-the-quoted-form-of-include-for-files-relative-to-the-including-file-and-the-angle-bracket-form-everywhere-else
* Contributors: Chris Thrasher, Christian Henkel, Robert Haschke, Sameer Gupta

2.6.0 (2022-11-10)
------------------
* Merge PR `#1553 <https://github.com/ros-planning/moveit2/issues/1553>`_: Improve cmake files
* Use standard exported targets: export\_${PROJECT_NAME} -> ${PROJECT_NAME}Targets
* Improve CMake usage (`#1550 <https://github.com/ros-planning/moveit2/issues/1550>`_)
* Remove __has_include statements (`#1481 <https://github.com/ros-planning/moveit2/issues/1481>`_)
* Contributors: Robert Haschke, Sebastian Jahr, Vatan Aksoy Tezer

2.5.3 (2022-07-28)
------------------

2.5.2 (2022-07-18)
------------------
* Merge remote-tracking branch 'origin/main' into feature/msa
* Removing more boost usage (`#1372 <https://github.com/ros-planning/moveit2/issues/1372>`_)
* Merge pull request `#3106 <https://github.com/ros-planning/moveit2/issues/3106>`_ from v4hn/pr-master-bind-them-all / banish bind()
* banish bind()
* Contributors: Henry Moore, Michael Görner, Vatan Aksoy Tezer, v4hn

2.5.1 (2022-05-31)
------------------

2.5.0 (2022-05-26)
------------------
* Enable cppcheck (`#1224 <https://github.com/ros-planning/moveit2/issues/1224>`_)
  Co-authored-by: jeoseo <jeongwooseo2012@gmail.com>
* Make moveit_common a 'depend' rather than 'build_depend' (`#1226 <https://github.com/ros-planning/moveit2/issues/1226>`_)
* Avoid bind(), use lambdas instead (`#1204 <https://github.com/ros-planning/moveit2/issues/1204>`_)
  Adaption of https://github.com/ros-planning/moveit/pull/3106
* banish bind()
  source:https://github.com/ros-planning/moveit/pull/3106/commits/a2911c80c28958c1fce8fb52333d770248c4ec05; required minor updates compared to original source commit in order to ensure compatibility with ROS2
* Merge https://github.com/ros-planning/moveit/commit/ab42a1d7017b27eb6c353fb29331b2da08ab0039
* 1.1.9
* 1.1.8
* Avoid downgrading default C++ standard (`#3043 <https://github.com/ros-planning/moveit2/issues/3043>`_)
* 1.1.7
* Switch to std::bind (`#2967 <https://github.com/ros-planning/moveit2/issues/2967>`_)
  * boost::bind -> std::bind
  grep -rlI --exclude-dir=.git "boost::bind" | xargs sed -i 's/boost::bind/std::bind/g'
  * Convert bind placeholders
  grep -rlI --exclude-dir=.git " _[0-9]" | xargs sed -i 's/ _\([0-9]\)/ std::placeholders::_\1/g'
  * Update bind include header
  grep -rlI --exclude-dir=.git "boost/bind" | xargs sed -i 's#boost/bind.hpp#functional#'
* 1.1.6
* Contributors: Abishalini, Henning Kayser, Jafar, Jochen Sprickerhof, Robert Haschke, jeoseo, v4hn

2.4.0 (2022-01-20)
------------------
* Remove some Maintainers from Pilz Planner (`#971 <https://github.com/ros-planning/moveit2/issues/971>`_)
* Remove '-W*' options from cmake files (`#2903 <https://github.com/ros-planning/moveit2/issues/2903>`_)
* Contributors: Leroy Rügemer, jschleicher

2.3.2 (2021-12-29)
------------------

2.3.1 (2021-12-23)
------------------
* Convert to modern include guard `#882 <https://github.com/ros-planning/moveit2/issues/882>`_ (`#891 <https://github.com/ros-planning/moveit2/issues/891>`_)
* Add codespell to precommit, fix A LOT of spelling mistakes (`#934 <https://github.com/ros-planning/moveit2/issues/934>`_)
* PILZ: Build fixups, silence warnings, fix unit tests
* PILZ: Migrate and Restructure test directory
* PILZ: Migrate planner and testutils packages to ROS 2
* Enforce package.xml format 3 Schema (`#779 <https://github.com/ros-planning/moveit2/issues/779>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: David V. Lu!!, Henning Kayser, Robert Haschke, Sebastian Jahr, predystopic-dev, pvanlaar, Pilz GmbH and Co. KG, Christian Henkel, Immanuel Martini, Joachim Schleicher, rfeistenauer
