#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <moveit/collision_detection/collision_common.h>

namespace py = pybind11;

namespace moveit_py
{
namespace bind_collision_detection
{
void initCollisionRequest(py::module& m);
void initCollisionResult(py::module& m);
}  // namespace bind_collision_detection
}  // namespace moveit_py
