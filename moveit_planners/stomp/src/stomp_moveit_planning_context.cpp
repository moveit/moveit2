/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** @file
 * @author Henning Kayser
 **/

#include <atomic>
#include <future>

#include <stomp/stomp.h>

#include <stomp_moveit/stomp_moveit_planning_context.hpp>
#include <stomp_moveit/trajectory_visualization.hpp>
#include <stomp_moveit/filter_functions.hpp>
#include <stomp_moveit/noise_generators.hpp>
#include <stomp_moveit/cost_functions.hpp>
#include <stomp_moveit/stomp_moveit_task.hpp>
#include <stomp_moveit_parameters.hpp>

#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/logger.hpp>

namespace stomp_moveit
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.planners.stomp.planning_context");
}
}  // namespace

// @brief Run a planning attempt with STOMP, either providing start and goal states or an optional seed trajectory
bool solveWithStomp(const std::shared_ptr<stomp::Stomp>& stomp, const moveit::core::RobotState& start_state,
                    const moveit::core::RobotState& goal_state, const moveit::core::JointModelGroup* group,
                    const robot_trajectory::RobotTrajectoryPtr& input_trajectory,
                    robot_trajectory::RobotTrajectoryPtr& output_trajectory)
{
  Eigen::MatrixXd waypoints;
  const auto& joints = group->getActiveJointModels();
  bool success = false;
  if (!input_trajectory || input_trajectory->empty())
  {
    success = stomp->solve(getPositions(start_state, joints), getPositions(goal_state, joints), waypoints);
  }
  else
  {
    auto input = robotTrajectoryToMatrix(*input_trajectory);
    success = stomp->solve(input, waypoints);
  }
  if (success)
  {
    output_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(start_state.getRobotModel(), group);
    fillRobotTrajectory(waypoints, start_state, *output_trajectory);
  }

  return success;
}

// @brief Extract a robot trajectory from the seed waypoints passed with a motion plan request
bool extractSeedTrajectory(const planning_interface::MotionPlanRequest& req,
                           const moveit::core::RobotModelConstPtr& robot_model,
                           robot_trajectory::RobotTrajectoryPtr& seed)
{
  if (req.trajectory_constraints.constraints.empty())
  {
    return false;
  }

  const auto* joint_group = robot_model->getJointModelGroup(req.group_name);
  const auto& names = joint_group->getActiveJointModelNames();
  const auto dof = names.size();

  trajectory_msgs::msg::JointTrajectory seed_traj;
  const auto& constraints = req.trajectory_constraints.constraints;  // alias to keep names short
  // Test the first point to ensure that it has all of the joints required
  for (size_t i = 0; i < constraints.size(); ++i)
  {
    auto n = constraints[i].joint_constraints.size();
    if (n != dof)
    {  // first test to ensure that dimensionality is correct
      RCLCPP_WARN(getLogger(), "Seed trajectory index %lu does not have %lu constraints (has %lu instead).", i, dof, n);
      return false;
    }

    trajectory_msgs::msg::JointTrajectoryPoint joint_pt;

    for (size_t j = 0; j < constraints[i].joint_constraints.size(); ++j)
    {
      const auto& c = constraints[i].joint_constraints[j];
      if (c.joint_name != names[j])
      {
        RCLCPP_WARN(getLogger(),
                    "Seed trajectory (index %lu, joint %lu) joint name '%s' does not match expected name '%s'", i, j,
                    c.joint_name.c_str(), names[j].c_str());
        return false;
      }
      joint_pt.positions.push_back(c.position);
    }

    seed_traj.points.push_back(joint_pt);
  }
  seed_traj.joint_names = names;

  moveit::core::RobotState robot_state(robot_model);
  moveit::core::robotStateMsgToRobotState(req.start_state, robot_state);
  seed = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, joint_group);
  seed->setRobotTrajectoryMsg(robot_state, seed_traj);

  return !seed->empty();
}

// @brief Build a STOMP task that uses MoveIt callback types for planning in STOMP
stomp::TaskPtr createStompTask(const stomp::StompConfiguration& config, StompPlanningContext& context)
{
  const size_t num_timesteps = config.num_timesteps;
  const auto planning_scene = context.getPlanningScene();
  const auto group = planning_scene->getRobotModel()->getJointModelGroup(context.getGroupName());

  // Check if we do have path constraints
  const auto& req = context.getMotionPlanRequest();
  kinematic_constraints::KinematicConstraintSet constraints(planning_scene->getRobotModel());
  constraints.add(req.path_constraints, planning_scene->getTransforms());

  // Create callback functions for STOMP task
  // Cost, noise and filter functions are provided for planning.
  // TODO(henningkayser): parameterize cost penalties
  using namespace stomp_moveit;
  CostFn cost_fn;
  if (!constraints.empty())
  {
    cost_fn = costs::sum({ costs::getCollisionCostFunction(planning_scene, group, 1.0 /* collision penalty */),
                           costs::getConstraintsCostFunction(planning_scene, group, constraints.getAllConstraints(),
                                                             1.0 /* constraint penalty */) });
  }
  else
  {
    cost_fn = costs::getCollisionCostFunction(planning_scene, group, 1.0 /* collision penalty */);
  }

  // TODO(henningkayser): parameterize stddev
  const std::vector<double> stddev(group->getActiveJointModels().size(), 0.1);
  auto noise_generator_fn = noise::getNormalDistributionGenerator(num_timesteps, stddev);
  auto filter_fn =
      filters::chain({ filters::simpleSmoothingMatrix(num_timesteps), filters::enforcePositionBounds(group) });
  auto iteration_callback_fn =
      visualization::getIterationPathPublisher(context.getPathPublisher(), planning_scene, group);
  auto done_callback_fn =
      visualization::getSuccessTrajectoryPublisher(context.getPathPublisher(), planning_scene, group);

  // Initialize and return STOMP task
  stomp::TaskPtr task =
      std::make_shared<ComposableTask>(noise_generator_fn, cost_fn, filter_fn, iteration_callback_fn, done_callback_fn);
  return task;
}

// @brief Create a valid STOMP configuration from runtime parameters and dimensions provided by the planning request
stomp::StompConfiguration getStompConfig(const stomp_moveit::Params& params, size_t num_dimensions)
{
  stomp::StompConfiguration config;
  config.num_dimensions = num_dimensions;  // Copied from joint count
  // TODO(henningkayser): set from request or params
  config.initialization_method = stomp::TrajectoryInitializations::LINEAR_INTERPOLATION;
  config.num_iterations = params.num_iterations;
  config.num_iterations_after_valid = params.num_iterations_after_valid;
  config.num_timesteps = params.num_timesteps;
  config.delta_t = params.delta_t;
  config.exponentiated_cost_sensitivity = params.exponentiated_cost_sensitivity;
  config.num_rollouts = params.num_rollouts;
  config.max_rollouts = params.max_rollouts;
  config.control_cost_weight = params.control_cost_weight;

  return config;
}

StompPlanningContext::StompPlanningContext(const std::string& name, const std::string& group,
                                           const stomp_moveit::Params& params)
  : planning_interface::PlanningContext(name, group), params_(params)
{
}

void StompPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  // Start time
  auto time_start = std::chrono::steady_clock::now();

  res.planner_id = std::string("stomp");
  // Default to happy path
  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  // Extract start and goal states
  const auto& req = getMotionPlanRequest();
  const moveit::core::RobotState start_state(*getPlanningScene()->getCurrentStateUpdated(req.start_state));
  moveit::core::RobotState goal_state(start_state);
  constraint_samplers::ConstraintSamplerManager sampler_manager;
  auto goal_sampler = sampler_manager.selectSampler(getPlanningScene(), getGroupName(), req.goal_constraints.at(0));
  if (!goal_sampler || !goal_sampler->sample(goal_state))
  {
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return;  // Can't plan without valid goal state
  }

  // STOMP config, task, planner instance
  const auto group = getPlanningScene()->getRobotModel()->getJointModelGroup(getGroupName());
  auto config = getStompConfig(params_, group->getActiveJointModels().size() /* num_dimensions */);
  robot_trajectory::RobotTrajectoryPtr input_trajectory;
  if (extractSeedTrajectory(request_, getPlanningScene()->getRobotModel(), input_trajectory))
  {
    config.num_timesteps = input_trajectory->size();
  }
  const auto task = createStompTask(config, *this);
  stomp_ = std::make_shared<stomp::Stomp>(config, task);

  std::condition_variable cv;
  std::mutex cv_mutex;
  bool finished = false;
  auto timeout_future = std::async(std::launch::async, [&, stomp = stomp_]() {
    std::unique_lock<std::mutex> lock(cv_mutex);
    cv.wait_for(lock, std::chrono::duration<double>(req.allowed_planning_time), [&finished] { return finished; });
    if (!finished)
    {
      stomp->cancel();
    }
  });

  // Solve
  if (!solveWithStomp(stomp_, start_state, goal_state, group, input_trajectory, res.trajectory))
  {
    // We timed out if the timeout task has completed so that the timeout future is valid and ready
    bool timed_out =
        timeout_future.valid() && timeout_future.wait_for(std::chrono::nanoseconds(1)) == std::future_status::ready;
    res.error_code.val =
        timed_out ? moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT : moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
  }
  stomp_.reset();
  {
    std::unique_lock<std::mutex> lock(cv_mutex);
    finished = true;
    cv.notify_all();
  }

  // Stop time
  std::chrono::duration<double> elapsed_seconds = std::chrono::steady_clock::now() - time_start;
  res.planning_time = elapsed_seconds.count();
}

void StompPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& /*res*/)
{
  // TODO(#2168): implement this function
  RCLCPP_ERROR(getLogger(),
               "StompPlanningContext::solve(planning_interface::MotionPlanDetailedResponse&) is not implemented!");
  return;
}

bool StompPlanningContext::terminate()
{
  // Copy shared pointer to avoid race conditions
  auto stomp = stomp_;
  if (stomp)
  {
    return stomp->cancel();
  }

  return true;
}

void StompPlanningContext::clear()
{
}

void StompPlanningContext::setPathPublisher(
    const std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>& path_publisher)
{
  path_publisher_ = path_publisher;
}

std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> StompPlanningContext::getPathPublisher()
{
  return path_publisher_;
}
}  // namespace stomp_moveit
