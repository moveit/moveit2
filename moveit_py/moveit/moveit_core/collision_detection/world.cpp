#include "world.hpp"

namespace moveit_py
{
namespace bind_collision_detection
{
void initWorld(py::module& m)
{
  py::module collision_detection = m.def_submodule("collision_detection");

  py::class_<collision_detection::World, collision_detection::WorldPtr>(m, "World").def(py::init<>());
}
}  // namespace bind_collision_detection
}  // namespace moveit_py