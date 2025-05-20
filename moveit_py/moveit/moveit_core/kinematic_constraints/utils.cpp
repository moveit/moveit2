#include "utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit_py/moveit_py_utils/ros_msg_typecasters.hpp>
#include <moveit/kinematic_constraints/utils.h>

namespace moveit_py
{
namespace bind_kinematic_constraints
{
moveit_msgs::msg::Constraints constructLinkConstraint(const std::string& link_name, const std::string& source_frame,
                                                      std::optional<std::vector<double>> cartesian_position,
                                                      std::optional<double> cartesian_position_tolerance,
                                                      std::optional<std::vector<double>> orientation,
                                                      std::optional<double> orientation_tolerance)
{
  // check that link cartesian and/or orientation constraints are specified
  if (!cartesian_position && !orientation)
  {
    throw std::invalid_argument("No link cartesian or orientation constraints specified");
  }

  moveit_msgs::msg::Constraints constraints_cpp;

  // merge constraints if necessary
  if (cartesian_position && orientation)
  {
    // define point stamped message
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = source_frame;
    point.point.x = cartesian_position.value()[0];
    point.point.y = cartesian_position.value()[1];
    point.point.z = cartesian_position.value()[2];

    moveit_msgs::msg::Constraints position_constraints =
        kinematic_constraints::constructGoalConstraints(link_name, point, cartesian_position_tolerance.value());

    // define quaternion message
    geometry_msgs::msg::QuaternionStamped quaternion;
    quaternion.header.frame_id = source_frame;
    quaternion.quaternion.x = orientation.value()[0];
    quaternion.quaternion.y = orientation.value()[1];
    quaternion.quaternion.z = orientation.value()[2];
    quaternion.quaternion.w = orientation.value()[3];

    moveit_msgs::msg::Constraints orientation_constraints =
        kinematic_constraints::constructGoalConstraints(link_name, quaternion, orientation_tolerance.value());

    constraints_cpp = kinematic_constraints::mergeConstraints(position_constraints, orientation_constraints);
  }

  // generate cartesian constraint
  else if (cartesian_position)
  {
    // define point stamped message
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = source_frame;
    point.point.x = cartesian_position.value()[0];
    point.point.y = cartesian_position.value()[1];
    point.point.z = cartesian_position.value()[2];

    // instantiate logger
    auto logger = rclcpp::get_logger("moveit_py");
    // check point with logger
    RCLCPP_DEBUG(rclcpp::get_logger("moveit_py"), "Point: %f, %f, %f", point.point.x, point.point.y, point.point.z);

    constraints_cpp = kinematic_constraints::constructGoalConstraints(link_name, point, *cartesian_position_tolerance);
  }

  // generate orientation constraint
  else
  {
    // define quaternion message
    geometry_msgs::msg::QuaternionStamped quaternion;
    quaternion.header.frame_id = source_frame;
    quaternion.quaternion.x = orientation.value()[0];
    quaternion.quaternion.y = orientation.value()[1];
    quaternion.quaternion.z = orientation.value()[2];
    quaternion.quaternion.w = orientation.value()[3];
    constraints_cpp =
        kinematic_constraints::constructGoalConstraints(link_name, quaternion, orientation_tolerance.value());
  }

  return constraints_cpp;
}

moveit_msgs::msg::Constraints constructJointConstraint(moveit::core::RobotState& robot_state,
                                                       moveit::core::JointModelGroup* joint_model_group,
                                                       double tolerance)
{
  // generate joint constraint message
  moveit_msgs::msg::Constraints joint_constraints =
      kinematic_constraints::constructGoalConstraints(robot_state, joint_model_group, tolerance);

  return joint_constraints;
}

moveit_msgs::msg::Constraints constructConstraintsFromNode(const std::shared_ptr<rclcpp::Node>& node_name,
                                                           const std::string& ns)
{
  // construct constraint message
  moveit_msgs::msg::Constraints constraints_cpp;
  kinematic_constraints::constructConstraints(node_name, ns, constraints_cpp);

  return constraints_cpp;
}

void initKinematicConstraints(py::module& m)
{
  py::module kinematic_constraints = m.def_submodule("kinematic_constraints");

  kinematic_constraints.def("construct_link_constraint", &constructLinkConstraint, py::arg("link_name"),
                            py::arg("source_frame"), py::arg("cartesian_position") = nullptr,
                            py::arg("cartesian_position_tolerance") = nullptr, py::arg("orientation") = nullptr,
                            py::arg("orientation_tolerance") = nullptr, "Construct a link constraint message");
  kinematic_constraints.def("construct_joint_constraint", &constructJointConstraint, py::arg("robot_state"),
                            py::arg("joint_model_group"), py::arg("tolerance") = 0.01,
                            "Construct a joint constraint message");
  kinematic_constraints.def("construct_constraints_from_node", &constructConstraintsFromNode, py::arg("node_name"),
                            py::arg("ns"), "Construct a constraint message from a node");
}

}  // namespace bind_kinematic_constraints
}  // namespace moveit_py