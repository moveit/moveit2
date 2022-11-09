#pragma once

#include <Eigen/Geometry>
#include <moveit/planning_scene/planning_scene.h>
#include <stomp_moveit/stomp_moveit_task.hpp>
#include <stomp_moveit/conversion_functions.hpp>

namespace stomp_moveit
{
namespace costs
{
// Interpolation step size for collision checking (joint space, L2 norm)
constexpr double COL_CHECK_DISTANCE = 0.05;

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
    bool in_collision_window = false;

    // Iterate over sample waypoint pairs and check for collisions in each segment.
    // If a collision is found, weighted penalty costs are applied to both waypoints.
    // Subsequent collisions are assumed to have the same cause (same object), so
    // we are keeping track of 'collision_windows' which are used for smoothing out
    // the costs per assumed 'object' with a gaussian, penalizing neighboring but
    // collision-free states as well.
    for (int timestep = 0; timestep < values.cols() - 1; ++timestep)
    {
      Eigen::VectorXd current = values.col(timestep);
      Eigen::VectorXd next = values.col(timestep + 1);
      const double segment_distance = (next - current).norm();
      double interpolation_fraction = 0.0;
      const double interpolation_step = std::min(0.5, COL_CHECK_DISTANCE / segment_distance);
      bool found_collision = false;
      while (!found_collision && interpolation_fraction < 1.0)
      {
        Eigen::VectorXd sample_vec = (1 - interpolation_fraction) * current + interpolation_fraction * next;
        set_joint_positions(sample_vec, joints, sample_state);
        sample_state.update();
        found_collision = planning_scene->isStateColliding(sample_state, group_name);
        interpolation_fraction += interpolation_step;
      }

      if (found_collision)
      {
        // Apply weighted collision penalties -> This trajectory is definitely invalid
        costs(timestep) = (1 - interpolation_fraction) * collision_penalty;
        costs(timestep + 1) = interpolation_fraction * collision_penalty;
        validity = false;

        // Open new collision window when this is the first detected collision in a group
        if (!in_collision_window)
        {
          collision_windows.emplace_back(timestep, values.cols());
          in_collision_window = true;
        }
      }
      else
      {
        // Close current collision window if the current state is collision-free
        if (in_collision_window)
        {
          collision_windows.back().second = timestep - 1;
          in_collision_window = false;
        }
      }
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
