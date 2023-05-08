#include <rclcpp/rclcpp.hpp>

#include <stomp/stomp.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include <stomp_moveit/trajectory_visualization.hpp>
#include <stomp_moveit/filter_functions.hpp>
#include <stomp_moveit/noise_generators.hpp>
#include <stomp_moveit/cost_functions.hpp>
#include <stomp_moveit/stomp_moveit_task.hpp>

// MoveIt
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std::chrono_literals;

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
  moveit_cpp->getPlanningSceneMonitorNonConst()->waitForCurrentRobotState(node->now(), 1.0 /* seconds */);
  moveit_cpp->getPlanningSceneMonitorNonConst()->updateFrameTransforms();
  moveit_cpp->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
  moveit_visual_tools::MoveItVisualTools visual_tools(node, "panda_link0", "stomp_moveit",
                                                      moveit_cpp->getPlanningSceneMonitorNonConst());

  auto markers_publisher =
      node->create_publisher<visualization_msgs::msg::MarkerArray>("/stomp_moveit", rclcpp::SystemDefaultsQoS());
  rclcpp::sleep_for(2s);

  const auto robot_model = moveit_cpp->getRobotModel();
  const auto group = robot_model->getJointModelGroup("panda_arm");
  const auto joints = group->getActiveJointModels();

  // Add collision box
  geometry_msgs::msg::Pose block_pose;
  block_pose.position.z = 0.4;
  block_pose.position.y = 0.3;
  visual_tools.publishCollisionBlock(block_pose, "my_block", 0.25);

  // Copy planning scene for collision checking
  const auto planning_scene =
      planning_scene_monitor::LockedPlanningSceneRO(moveit_cpp->getPlanningSceneMonitorNonConst())->diff();
  planning_scene->decoupleParent();
  planning_scene->getCurrentStateNonConst().update();

  // Start and Goal States
  // The goal state is set have the same eef orientation as the start state by rotating the base (front) by M_PI
  // and by counteracting the rotation in the endeffector (back)
  const auto start_state = planning_scene->getCurrentState();
  auto goal_state = start_state;
  goal_state.setJointPositions(joints.front(), { *goal_state.getJointPositions(joints.front()) + M_PI });
  goal_state.setJointPositions(joints.back(), { *goal_state.getJointPositions(joints.back()) + M_PI });

  // Path constraint keeping the end-effector in the same orientation over the whole trajectory
  goal_state.update();
  const std::string eef_link = "panda_hand";
  const Eigen::Isometry3d eef_transform = planning_scene->getFrameTransform(eef_link);
  geometry_msgs::msg::QuaternionStamped eef_orientation;
  eef_orientation.header.frame_id = "panda_link0";
  eef_orientation.quaternion = tf2::toMsg(Eigen::Quaterniond(eef_transform.linear()));
  moveit_msgs::msg::Constraints constraints =
      kinematic_constraints::constructGoalConstraints(eef_link, eef_orientation, 0.1 /* rad tolerance */);

  // Checks for verifying that constraints are satisfied by start and goal states
  if (!planning_scene->isStateConstrained(start_state, constraints, true))
    std::cout << "start state doesn't satisfy constraints" << std::endl;
  if (!planning_scene->isStateConstrained(goal_state, constraints, true))
    std::cout << "goal state doesn't satisfy constraints" << std::endl;

  // Configure STOMP
  stomp::StompConfiguration config = getStompConfiguration(joints.size() /* num_dimensions */);

  // Create STOMP planning task
  // Noise, cost, and filter functions are provided for planning.
  // The iteration and done callbacks are used for path and trajectory visualization.
  using namespace stomp_moveit;
  auto noise_generator_fn = noise::get_normal_distribution_generator(config.num_timesteps, { 0.1, 0.1, 0.1, 0.1, 0.05,
                                                                                             0.05, 0.05 } /* stddev */);
  auto cost_fn = costs::sum(
      { costs::get_collision_cost_function(planning_scene, group, 1.0 /* collision penalty */),
        costs::get_constraints_cost_function(planning_scene, group, constraints, 1.0 /* constraint penalty */) });
  auto filter_fn = filters::chain(
      { filters::simple_smoothing_matrix(config.num_timesteps), filters::enforce_position_bounds(group) });
  auto iteration_callback_fn = visualization::get_iteration_path_publisher(markers_publisher, planning_scene, group);
  auto done_callback_fn = visualization::get_success_trajectory_publisher(markers_publisher, planning_scene, group);
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
      rclcpp::sleep_for(std::chrono::seconds(5));
    }

    // Cleanup RViZ
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  executor.cancel();

  return 0;
}
