#pragma once

#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.h>
#include <stomp_moveit/stomp_moveit_task.hpp>
#include <stomp_moveit/conversion_functions.hpp>

namespace stomp_moveit
{
namespace costs
{
CostFn get_collision_cost_function(const std::shared_ptr<const planning_scene::PlanningScene>& planning_scene,
                                   const moveit::core::JointModelGroup* group, double collision_penalty)
{
  const auto& joints = group ? group->getActiveJointModels() : planning_scene->getRobotModel()->getActiveJointModels();
  const auto& group_name = group ? group->getName() : "";

  CostFn cost_fn = [=](const Eigen::MatrixXd& values, Eigen::VectorXd& costs, bool& validity) {
    static thread_local moveit::core::RobotState sample_state(planning_scene->getCurrentState());

    costs.setZero(values.cols());

    validity = true;
    std::vector<std::pair<long, long>> collision_windows;
    bool in_collision = false;

    for (int timestep = 0; timestep < values.cols(); ++timestep)
    {
      set_joint_positions(values.col(timestep), joints, sample_state);
      sample_state.update();

      if (planning_scene->isStateColliding(sample_state, group_name))
      {
        costs(timestep) = collision_penalty;
        validity = false;

        if (!in_collision)
        {
          collision_windows.emplace_back(timestep, values.cols());
          in_collision = true;
        }
      }
      else
      {
        if (in_collision)
        {
          collision_windows.back().second = timestep - 1;
          in_collision = false;
        }
      }
      // TODO: Check intermediate collisions
    }

    // Smooth out cost of colliding segments using a gaussian
    // The standard deviation is picked so that neighboring states
    // before and after the collision are penalized as well.
    for (const auto& [start, end] : collision_windows)
    {
      const double window_size = static_cast<double>(end - start) + 1;
      const double sigma = window_size / 5.0;
      const double mu = 0.5 * (start + end);
      for (auto j = std::max(0l, start - static_cast<long>(sigma));
           j <= std::min(values.cols() - 1, end + static_cast<long>(sigma)); ++j)
      {
        costs(j) +=
            std::exp(-std::pow(j - mu, 2) / (2 * std::pow(sigma, 2))) / (sigma * std::sqrt(2 * mu)) * window_size;
      }
    }

    return true;
  };

  return cost_fn;
}
}  // namespace costs
}  // namespace stomp_moveit
