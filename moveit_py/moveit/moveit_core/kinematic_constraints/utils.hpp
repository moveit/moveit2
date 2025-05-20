#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/msg/constraints.hpp>

namespace py = pybind11;

namespace moveit_py
{
namespace bind_kinematic_constraints
{
moveit_msgs::msg::Constraints constructLinkConstraint(const std::string& link_name, const std::string& source_frame,
                                                      std::optional<std::vector<double>> cartesian_position,
                                                      std::optional<double> cartesian_position_tolerance,
                                                      std::optional<std::vector<double>> orientation,
                                                      std::optional<double> orientation_tolerance);

moveit_msgs::msg::Constraints constructJointConstraint(moveit::core::RobotState& robot_state,
                                                       moveit::core::JointModelGroup* joint_model_group,
                                                       double tolerance);

moveit_msgs::msg::Constraints constructConstraintsFromNode(const std::shared_ptr<rclcpp::Node>& node_name,
                                                           const std::string& ns);

void initKinematicConstraints(py::module& m);

}  // namespace bind_kinematic_constraints
}  // namespace moveit_py