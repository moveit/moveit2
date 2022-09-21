#pragma once

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <stomp_moveit/stomp_moveit_task.hpp>
#include <stomp_moveit/conversion_functions.hpp>

namespace stomp_moveit
{
namespace visualization
{
PostIterationFn
get_iteration_path_publisher(moveit_visual_tools::MoveItVisualTools& visual_tools,
                             const moveit::core::JointModelGroup* group,
                             const rviz_visual_tools::Colors color = rviz_visual_tools::TRANSLUCENT_LIGHT)
{
  assert(group != nullptr);

  std::shared_ptr<const moveit::core::RobotState> reference_state;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(visual_tools.getPlanningSceneMonitor());
    reference_state = std::make_shared<moveit::core::RobotState>(scene->getCurrentState());
  }

  PostIterationFn path_publisher = [=, &visual_tools](int iteration_number, double cost, const Eigen::MatrixXd& values) {
    static thread_local robot_trajectory::RobotTrajectory trajectory(visual_tools.getRobotModel(), group);
    fill_robot_trajectory(values, *reference_state, trajectory);
    visual_tools.publishTrajectoryLine(trajectory, group, color);
    visual_tools.trigger();
  };

  return path_publisher;
}

DoneFn get_success_trajectory_publisher(moveit_visual_tools::MoveItVisualTools& visual_tools,
                                        const moveit::core::JointModelGroup* group,
                                        const rviz_visual_tools::Colors color = rviz_visual_tools::GREEN)
{
  assert(group != nullptr);

  std::shared_ptr<const moveit::core::RobotState> reference_state;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(visual_tools.getPlanningSceneMonitor());
    reference_state = std::make_shared<moveit::core::RobotState>(scene->getCurrentState());
  }

  DoneFn path_publisher = [=, &visual_tools](bool success, int total_iterations, double final_cost,
                                             const Eigen::MatrixXd& values) {
    static thread_local robot_trajectory::RobotTrajectory trajectory(reference_state->getRobotModel(), group);
    if (success)
    {
      fill_robot_trajectory(values, *reference_state, trajectory);
      visual_tools.publishTrajectoryLine(trajectory, group, color);
      visual_tools.publishTrajectoryPath(trajectory);
      visual_tools.trigger();
    }
  };

  return path_publisher;
}
}  // namespace visualization
}  // namespace stomp_moveit
