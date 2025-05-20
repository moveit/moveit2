#include "collision_matrix.hpp"

namespace moveit_py
{
namespace bind_collision_detection
{
// TODO: Create a custom typecaster/revise the current implementation to return std::pair<bool,
// collision_detection::AllowedCollision::Type>
std::pair<bool, std::string> getEntry(const collision_detection::AllowedCollisionMatrix& acm, const std::string& name1,
                                      const std::string& name2)
{
  // check acm for collision
  collision_detection::AllowedCollision::Type type;
  bool collision_allowed = acm.getEntry(name1, name2, type);
  std::string type_str;
  if (type == collision_detection::AllowedCollision::Type::NEVER)
  {
    type_str = "NEVER";
  }
  else if (type == collision_detection::AllowedCollision::Type::ALWAYS)
  {
    type_str = "ALWAYS";
  }
  else if (type == collision_detection::AllowedCollision::Type::CONDITIONAL)
  {
    type_str = "CONDITIONAL";
  }
  else
  {
    type_str = "UNKNOWN";
  }

  // should return a tuple true/false and the allowed collision type
  std::pair<bool, std::string> result = std::make_pair(collision_allowed, type_str);
  return result;
}

void initAcm(py::module& m)
{
  py::module collision_detection = m.def_submodule("collision_detection");

  py::class_<collision_detection::AllowedCollisionMatrix, std::shared_ptr<collision_detection::AllowedCollisionMatrix>>(
      collision_detection, "AllowedCollisionMatrix",
      R"(
          Definition of a structure for the allowed collision matrix. All elements in the collision world are referred to by their names. This class represents which collisions are allowed to happen and which are not.
          )")
      .def(py::init<std::vector<std::string>&, bool>(),
           R"(
       Initialize the allowed collision matrix using a list of names of collision objects.

       Args:
           names (list of str): A list of names of the objects in the collision world (corresponding to object IDs in the collision world).
           allowed (bool): If false, indicates that collisions between all elements must be checked for and no collisions will be ignored.
       )",
           py::arg("names"), py::arg("default_entry") = false)

      .def("get_entry", &moveit_py::bind_collision_detection::getEntry,
           R"(
           Get the allowed collision entry for a pair of objects.

           Args:
                name1 (str): The name of the first object.
                name2 (str): The name of the second object.

	   Returns:
                (bool, str): Whether the collision is allowed and the type of allowed collision.
       )",
           py::arg("name1"), py::arg("name2"))

      .def("set_entry",
           py::overload_cast<const std::string&, const std::string&, bool>(
               &collision_detection::AllowedCollisionMatrix::setEntry),
           py::arg("name1"), py::arg("name2"), py::arg("allowed"),
           R"(
           Set the allowed collision state between two objects.

           Args:
                name1 (str): The name of the first object.
                name2 (str): The name of the second object.
                allowed (bool): If true, indicates that the collision between the two objects is allowed. If false, indicates that the collision between the two objects is not allowed.
       )")

      .def("remove_entry",
           py::overload_cast<const std::string&, const std::string&>(
               &collision_detection::AllowedCollisionMatrix::removeEntry),
           py::arg("name1"), py::arg("name2"),
           R"(
           Remove an entry corresponding to a pair of elements. Nothing happens if the pair does not exist in the collision matrix.

           Args:
                name1 (str): The name of the first object.
                name2 (str): The name of the second object.
           )")

      .def("clear", &collision_detection::AllowedCollisionMatrix::clear, R"(Clear the allowed collision matrix.)");
}
// getEntry

}  // namespace bind_collision_detection
}  // namespace moveit_py