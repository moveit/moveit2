#include <rclcpp/rclcpp.hpp>

#include <stomp/stomp.h>

#include <stomp_moveit/trajectory_visualization.hpp>
#include <stomp_moveit/filter_functions.hpp>
#include <stomp_moveit/noise_generators.hpp>
#include <stomp_moveit/cost_functions.hpp>
#include <stomp_moveit/stomp_moveit_task.hpp>

// MoveItCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

stomp::StompConfiguration getStompConfiguration(size_t num_dimensions)
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

int main(int argc, char** argv)
{
  // Initialize Node and Executor thread
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("stomp_moveit_example", "", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Setup MoveIt, MVT
  auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp->getPlanningSceneMonitor()->waitForCurrentRobotState(node->now(), 1.0 /* seconds */);
  moveit_cpp->getPlanningSceneMonitor()->providePlanningSceneService();
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "panda_link0", "stomp_moveit",
                                                      moveit_cpp->getPlanningSceneMonitor());
  const auto robot_model = moveit_cpp->getRobotModel();
  const auto group = robot_model->getJointModelGroup("panda_arm");
  const auto joints = group->getActiveJointModels();

  // Add collision box
  geometry_msgs::msg::Pose block_pose;
  block_pose.position.z = 1.0;
  visual_tools.publishCollisionBlock(block_pose, "my_block", 0.3);

  // Copy planning scene for collision checking
  const auto planning_scene =
      planning_scene_monitor::LockedPlanningSceneRO(moveit_cpp->getPlanningSceneMonitor())->diff();
  planning_scene->decoupleParent();

  // Start and Goal States
  const auto start_state = planning_scene->getCurrentState();
  auto goal_state = start_state;
  goal_state.setJointPositions(joints.at(3), { 0.5 });
  goal_state.setJointPositions(joints.at(2), { 0.5 });

  // Configure STOMP
  stomp::StompConfiguration config = getStompConfiguration(joints.size() /* num_dimensions */);

  // Create STOMP planning task
  // Noise, cost, and filter functions are provided for planning.
  // The iteration and done callbacks are used for path and trajectory visualization.
  using namespace stomp_moveit;
  auto noise_generator_fn = noise::get_normal_distribution_generator(config.num_timesteps, { 0.1, 0.1, 0.1, 0.1, 0.05,
                                                                                             0.05, 0.05 } /* stddev */);
  auto cost_fn = costs::get_collision_cost_function(planning_scene, group, 1.0 /* collision penalty */);
  auto filter_fn = filters::chain(
      { filters::simple_smoothing_matrix(config.num_timesteps), filters::enforce_position_bounds(group) });
  auto iteration_callback_fn = visualization::get_iteration_path_publisher(visual_tools, group);
  auto done_callback_fn = visualization::get_success_trajectory_publisher(visual_tools, group);
  stomp::TaskPtr task =
      std::make_shared<ComposableTask>(noise_generator_fn, cost_fn, filter_fn, iteration_callback_fn, done_callback_fn);

  // Keep on solving the same task until user interrupt
  while (rclcpp::ok())
  {
    stomp::Stomp stomp(config, task);

    Eigen::MatrixXd trajectory;
    if (stomp.solve(get_positions(start_state, joints), get_positions(goal_state, joints), trajectory))
    {
      std::cout << "STOMP succeeded" << std::endl;
    }
    else
    {
      std::cout << "A valid solution was not found" << std::endl;
    }

    // Cleanup RViZ
    rclcpp::sleep_for(std::chrono::seconds(1));
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  executor.cancel();

  return 0;
}
