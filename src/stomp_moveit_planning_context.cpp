#include <atomic>
#include <future>

#include <stomp/stomp.h>

#include <stomp_moveit/stomp_moveit_planning_context.hpp>
// #include <stomp_moveit/trajectory_visualization.hpp>
#include <stomp_moveit/filter_functions.hpp>
#include <stomp_moveit/noise_generators.hpp>
#include <stomp_moveit/cost_functions.hpp>
#include <stomp_moveit/stomp_moveit_task.hpp>
#include <stomp_moveit_parameters.hpp>

#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/robot_state/conversions.h>

namespace stomp_moveit
{
bool solveWithStomp(const std::shared_ptr<stomp::Stomp>& stomp, const moveit::core::RobotState& start_state,
                    const moveit::core::RobotState& goal_state, const moveit::core::JointModelGroup* group,
                    const robot_trajectory::RobotTrajectoryPtr& input_trajectory,
                    robot_trajectory::RobotTrajectoryPtr& trajectory)
{
  Eigen::MatrixXd waypoints;
  const auto& joints = group->getActiveJointModels();
  bool success;
  if (!input_trajectory || input_trajectory->empty())
    success = stomp->solve(get_positions(start_state, joints), get_positions(goal_state, joints), waypoints);
  else
  {
    auto input = robot_trajectory_to_matrix(*input_trajectory);
    success = stomp->solve(input, waypoints);
  }
  if (success)
  {
    trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(start_state.getRobotModel(), group);
    fill_robot_trajectory(waypoints, start_state, *trajectory);
  }

  return success;
}

bool extractSeedTrajectory(const planning_interface::MotionPlanRequest& req,
                           const moveit::core::RobotModelConstPtr robot_model,
                           robot_trajectory::RobotTrajectoryPtr& seed)
{
  if (req.trajectory_constraints.constraints.empty())
    return false;

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
      RCLCPP_WARN(rclcpp::get_logger("stomp_moveit"),
                  "Seed trajectory index %lu does not have %lu constraints (has %lu instead).", i, dof, n);
      return false;
    }

    trajectory_msgs::msg::JointTrajectoryPoint joint_pt;

    for (size_t j = 0; j < constraints[i].joint_constraints.size(); ++j)
    {
      const auto& c = constraints[i].joint_constraints[j];
      if (c.joint_name != names[j])
      {
        RCLCPP_WARN(rclcpp::get_logger("stomp_moveit"),
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

stomp::TaskPtr createStompTask(const stomp::StompConfiguration& config, const StompPlanningContext& context)
{
  const size_t num_timesteps = config.num_timesteps;
  const double collision_penalty = 1.0;
  const auto planning_scene = context.getPlanningScene();
  const auto group = planning_scene->getRobotModel()->getJointModelGroup(context.getGroupName());
  const std::vector<double> stddev(group->getActiveJointModels().size(), 0.1);

  // Create STOMP planning task
  // Noise, cost, and filter functions are provided for planning.
  // The iteration and done callbacks are used for path and trajectory visualization.
  using namespace stomp_moveit;
  auto noise_generator_fn = noise::get_normal_distribution_generator(num_timesteps, stddev);
  auto cost_fn = costs::get_collision_cost_function(planning_scene, group, collision_penalty);
  auto filter_fn = filters::simple_smoothing_matrix(num_timesteps);
  // TODO: enable support for visualization
  // auto iteration_callback_fn = visualization::get_iteration_path_publisher(visual_tools, group);
  // auto done_callback_fn = visualization::get_success_trajectory_publisher(visual_tools, group);
  PostIterationFn iteration_callback_fn = [](auto, auto, const auto&) {};
  DoneFn done_callback_fn = [](auto, auto, auto, const auto&) {};
  stomp::TaskPtr task =
      std::make_shared<ComposableTask>(noise_generator_fn, cost_fn, filter_fn, iteration_callback_fn, done_callback_fn);
  return task;
}

stomp::StompConfiguration getStompConfig(const stomp_moveit::Params& params, size_t num_dimensions)
{
  stomp::StompConfiguration config;
  config.num_dimensions = num_dimensions;                                                 // Copied from joint count
  config.initialization_method = stomp::TrajectoryInitializations::LINEAR_INTERPOLATION;  // TODO: set from request
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

bool StompPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  // Start time
  auto time_start = std::chrono::steady_clock::now();

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
    return false;  // Can't plan without valid goal state
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

  return res.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

bool StompPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  return false;
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
}  // namespace stomp_moveit
