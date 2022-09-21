#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>

namespace stomp_moveit
{
using Joints = std::vector<const moveit::core::JointModel*>;

std::vector<double> get_positions(const moveit::core::RobotState& state, const Joints& joints)
{
  std::vector<double> positions;
  for (const auto& joint : joints)
  {
    positions.push_back(*state.getJointPositions(joint));
  }

  return positions;
}

void set_joint_positions(const Eigen::VectorXd& values, const Joints& joints, moveit::core::RobotState& state)
{
  for (size_t joint_index = 0; joint_index < joints.size(); ++joint_index)
  {
    state.setJointPositions(joints[joint_index], &values[joint_index]);
  }
}

void fill_robot_trajectory(const Eigen::MatrixXd& trajectory_values, const moveit::core::RobotState& reference_state,
                           robot_trajectory::RobotTrajectory& trajectory)
{
  trajectory.clear();
  const auto& active_joints = trajectory.getGroup() ? trajectory.getGroup()->getActiveJointModels() :
                                                      trajectory.getRobotModel()->getActiveJointModels();
  assert(static_cast<std::size_t>(trajectory_values.rows()) == active_joints.size());

  for (int timestep = 0; timestep < trajectory_values.cols(); ++timestep)
  {
    const auto waypoint = std::make_shared<moveit::core::RobotState>(reference_state);
    set_joint_positions(trajectory_values.col(timestep), active_joints, *waypoint);

    trajectory.addSuffixWayPoint(waypoint, 0.1 /* placeholder dt */);
  }
}

robot_trajectory::RobotTrajectory matrix_to_robot_trajectory(const Eigen::MatrixXd& trajectory_values,
                                                             const moveit::core::RobotState& reference_state,
                                                             const moveit::core::JointModelGroup* group = nullptr)
{
  robot_trajectory::RobotTrajectory trajectory(reference_state.getRobotModel(), group);
  fill_robot_trajectory(trajectory_values, reference_state, trajectory);
  return trajectory;
}

Eigen::MatrixXd robot_trajectory_to_matrix(const robot_trajectory::RobotTrajectory& trajectory)
{
  const auto& active_joints = trajectory.getGroup() ? trajectory.getGroup()->getActiveJointModels() :
                                                      trajectory.getRobotModel()->getActiveJointModels();

  Eigen::MatrixXd trajectory_values(active_joints.size(), trajectory.getWayPointCount());

  for (int timestep = 0; timestep < trajectory_values.cols(); ++timestep)
  {
    const auto& waypoint = trajectory.getWayPoint(timestep);
    for (size_t joint_index = 0; joint_index < active_joints.size(); ++joint_index)
    {
      trajectory_values(joint_index, timestep) = *waypoint.getJointPositions(active_joints[joint_index]);
    }
  }

  return trajectory_values;
}

}  // namespace stomp_moveit
