#include <stomp/stomp.h>

#include <stomp_moveit/stomp_moveit_planning_context.hpp>
// #include <stomp_moveit/trajectory_visualization.hpp>
#include <stomp_moveit/filter_functions.hpp>
#include <stomp_moveit/noise_generators.hpp>
#include <stomp_moveit/cost_functions.hpp>
#include <stomp_moveit/stomp_moveit_task.hpp>

#include <moveit/constraint_samplers/constraint_sampler_manager.h>

namespace stomp_moveit
{
bool solveWithStomp(const stomp::StompConfiguration& config, const stomp::TaskPtr& task,
                    const moveit::core::RobotState& start_state, const moveit::core::RobotState& goal_state,
                    const moveit::core::JointModelGroup* group, robot_trajectory::RobotTrajectoryPtr& trajectory)
{
  stomp::Stomp stomp(config, task);

  Eigen::MatrixXd waypoints;
  const auto& joints = group->getActiveJointModels();
  bool success = stomp.solve(get_positions(start_state, joints), get_positions(goal_state, joints), waypoints);
  if (success)
  {
    trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(start_state.getRobotModel(), group);
    fill_robot_trajectory(waypoints, start_state, *trajectory);
  }

  return success;
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
stomp::StompConfiguration loadStompConfig(size_t num_dimensions)
{
  stomp::StompConfiguration config;
  // General settings
  config.num_iterations = 1000;           /**< @brief Maximum number of iteration allowed */
  config.num_iterations_after_valid = 0;  /**< @brief Stomp will stop optimizing this many iterations after finding a
                                      valid solution */
  config.num_timesteps = 40;              /**< @brief Number of timesteps */
  config.num_dimensions = num_dimensions; /**< @brief Parameter dimensionality */
  config.delta_t = 0.1;                   /**< @brief Time change between consecutive points */
  config.initialization_method = stomp::TrajectoryInitializations::
      LINEAR_INTERPOLATION; /**< @brief TrajectoryInitializations::TrajectoryInitialization */

  // Probability Calculation
  config.exponentiated_cost_sensitivity = 0.5; /**< @brief Default exponetiated cost sensitivity coefficient */

  // Noisy trajectory generation
  config.num_rollouts = 15; /**< @brief Number of noisy trajectories*/
  config.max_rollouts = 25; /**< @brief The combined number of new and old rollouts during each iteration shouldn't
                       exceed this value */

  // Cost calculation
  config.control_cost_weight = 0.1; /**< @brief Percentage of the trajectory accelerations cost to be applied in the
                                 total cost calculation >*/
  return config;
}

bool StompPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  // Start time
  auto time_start = std::chrono::steady_clock::now();

  // Response output
  auto& trajectory = res.trajectory;
  auto& planning_time = res.planning_time;
  auto& result_code = res.error_code.val;
  result_code = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;  // Default to happy path

  // Extract start and goal states
  const auto& req = getMotionPlanRequest();
  const moveit::core::RobotState start_state(*getPlanningScene()->getCurrentStateUpdated(req.start_state));
  moveit::core::RobotState goal_state(start_state);
  constraint_samplers::ConstraintSamplerManager sampler_manager;
  auto goal_sampler = sampler_manager.selectSampler(getPlanningScene(), getGroupName(), req.goal_constraints.at(0));
  if (!goal_sampler || !goal_sampler->sample(goal_state))
  {
    result_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;  // Can't plan without valid goal state
  }

  // STOMP config and task
  const auto group = getPlanningScene()->getRobotModel()->getJointModelGroup(getGroupName());
  const auto config = loadStompConfig(group->getActiveJointModels().size() /* num_dimensions */);
  const auto task = createStompTask(config, *this);

  // Solve motion plan
  if (!solveWithStomp(config, task, start_state, goal_state, group, trajectory))
  {
    result_code = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
  }

  // Stop time
  std::chrono::duration<double> elapsed_seconds = std::chrono::steady_clock::now() - time_start;
  planning_time = elapsed_seconds.count();

  return result_code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

bool StompPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  return false;
}

bool StompPlanningContext::terminate()
{
  return false;
}

void StompPlanningContext::clear()
{
}
}  // namespace stomp_moveit
