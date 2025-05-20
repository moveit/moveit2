#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <moveit/collision_detection/world.h>

namespace py = pybind11;

namespace moveit_py
{
namespace bind_collision_detection
{
void initWorld(py::module& m);
}  // namespace bind_collision_detection
}  // namespace moveit_py