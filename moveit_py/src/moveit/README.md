# MoveIt 2 Pybind11 Bindings

This directory contains the [Pybind11](https://pybind11.readthedocs.io/en/stable/) binding code used by the moveit_py package.
The root of this directory contains the actual module definitions (e.g. `core.cpp` defines the `moveit_py.core` module).

The structure of subfolders in this directory reflects that of the MoveIt 2 C++ codebase. Within each subfolder you will find the actual binding code that each module leverages.
For instance, to see how the `PlanningScene` object is being binded you can refer to the source files in `./moveit_core/planning_scene/`.
