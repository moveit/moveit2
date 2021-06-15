/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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

/* Author: Sebastian Jahr
 */

#include <moveit/local_planner/local_planner_component.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/msg/constraints.hpp>
namespace moveit_hybrid_planning
{
using namespace std::chrono_literals;
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");

LocalPlannerComponent::LocalPlannerComponent(const rclcpp::NodeOptions& options)
  : Node("local_planner_component", options)
{
  state_ = moveit_hybrid_planning::LocalPlannerState::UNCONFIGURED;
  local_planner_feedback_ = std::make_shared<moveit_msgs::action::LocalPlanner::Feedback>();

  // Initialize local planner after construction
  // TODO(sjahr) Remove once life cycle component nodes are available
  timer_ = this->create_wall_timer(1ms, [this]() {
    switch (state_)
    {
      case moveit_hybrid_planning::LocalPlannerState::READY:
      {
        timer_->cancel();
        break;
      }
      case moveit_hybrid_planning::LocalPlannerState::UNCONFIGURED:
        if (this->initialize())
        {
          state_ = moveit_hybrid_planning::LocalPlannerState::READY;
        }
        else
        {
          const std::string error = "Failed to initialize global planner";
          RCLCPP_FATAL(LOGGER, error);
        }
      default:
        break;
    }
  });
}

bool LocalPlannerComponent::initialize()
{
  const auto node_ptr = shared_from_this();

  // Load planner parameter
  config_.load(node_ptr);

  // Configure planning scene monitor
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_ptr, "robot_description", tf_buffer_, "local_planner/planning_scene_monitor");
  if (!planning_scene_monitor_->getPlanningScene())
  {
    const std::string error = "Unable to configure planning scene monitor";
    RCLCPP_FATAL(LOGGER, error);
    throw std::runtime_error(error);
  }

  // Start state and scene monitors
  RCLCPP_INFO(LOGGER, "Starting planning scene monitors");
  planning_scene_monitor_->startSceneMonitor();
  planning_scene_monitor_->startWorldGeometryMonitor();
  planning_scene_monitor_->startStateMonitor();

  // Load trajectory operator plugin
  try
  {
    trajectory_operator_loader_ =
        std::make_unique<pluginlib::ClassLoader<moveit_hybrid_planning::TrajectoryOperatorInterface>>(
            "moveit_hybrid_planning", "moveit_hybrid_planning::TrajectoryOperatorInterface");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(LOGGER, "Exception while creating trajectory operator plugin loader %s", ex.what());
  }
  try
  {
    trajectory_operator_instance_ =
        trajectory_operator_loader_->createUniqueInstance(config_.trajectory_operator_plugin_name);
    if (!trajectory_operator_instance_->initialize(node_ptr, planning_scene_monitor_->getRobotModel(),
                                                   "panda_arm"))  // TODO(sjahr) add default group param
      throw std::runtime_error("Unable to initialize trajectory operator plugin");
    RCLCPP_INFO(LOGGER, "Using trajectory operator interface '%s'", config_.trajectory_operator_plugin_name.c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while loading trajectory operator '%s': %s",
                 config_.trajectory_operator_plugin_name.c_str(), ex.what());
  }

  // Load local constraint solver
  try
  {
    local_constraint_solver_plugin_loader_ =
        std::make_unique<pluginlib::ClassLoader<moveit_hybrid_planning::LocalConstraintSolverInterface>>(
            "moveit_hybrid_planning", "moveit_hybrid_planning::LocalConstraintSolverInterface");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(LOGGER, "Exception while creating constraint solver plugin loader %s", ex.what());
  }
  try
  {
    local_constraint_solver_instance_ =
        local_constraint_solver_plugin_loader_->createUniqueInstance(config_.local_constraint_solver_plugin_name);
    if (!local_constraint_solver_instance_->initialize(node_ptr, planning_scene_monitor_, "panda_arm"))
      throw std::runtime_error("Unable to initialize constraint solver plugin");
    RCLCPP_INFO(LOGGER, "Using constraint solver interface '%s'", config_.local_constraint_solver_plugin_name.c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while loading constraint solver '%s': %s",
                 config_.local_constraint_solver_plugin_name.c_str(), ex.what());
  }

  // Initialize local planning request action server
  using namespace std::placeholders;
  local_planning_request_server_ = rclcpp_action::create_server<moveit_msgs::action::LocalPlanner>(
      this->get_node_base_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(),
      this->get_node_waitables_interface(), "local_planning_action",
      [](const rclcpp_action::GoalUUID& /*unused*/,
         std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received local planning goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::LocalPlanner>>& /*unused*/) {
        RCLCPP_INFO(LOGGER, "Received request to cancel local planning goal");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::LocalPlanner>> goal_handle) {
        local_planning_goal_handle_ = std::move(goal_handle);
        // Start local planning loop when an action request is received
        timer_ = this->create_wall_timer(1s / config_.local_planning_frequency,
                                         std::bind(&LocalPlannerComponent::executePlanningLoopRun, this));
      });

  // Initialize global trajectory listener
  global_solution_subscriber_ = create_subscription<moveit_msgs::msg::MotionPlanResponse>(
      config_.global_solution_topic, 1, [this](const moveit_msgs::msg::MotionPlanResponse::SharedPtr msg) {
        // Add received trajectory to internal reference trajectory
        robot_trajectory::RobotTrajectory new_trajectory(planning_scene_monitor_->getRobotModel(), msg->group_name);
        moveit::core::RobotState start_state(planning_scene_monitor_->getRobotModel());
        moveit::core::robotStateMsgToRobotState(msg->trajectory_start, start_state);
        new_trajectory.setRobotTrajectoryMsg(start_state, msg->trajectory);
        *local_planner_feedback_ = trajectory_operator_instance_->addTrajectorySegment(new_trajectory);
        if (!local_planner_feedback_->feedback.empty())
        {
          local_planning_goal_handle_->publish_feedback(local_planner_feedback_);
        }

        // Update local planner state
        state_ = moveit_hybrid_planning::LocalPlannerState::LOCAL_PLANNING_ACTIVE;
      });

  // Initialize local solution publisher
  RCLCPP_INFO(LOGGER, "Using '%s' as local solution topic type", config_.local_solution_topic_type.c_str());
  if (config_.local_solution_topic_type == "trajectory_msgs/JointTrajectory")
  {
    local_trajectory_publisher_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(config_.local_solution_topic, 1);
  }
  else if (config_.local_solution_topic_type == "std_msgs/Float64MultiArray")
  {
    local_solution_publisher_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(config_.local_solution_topic, 1);
  }
  else if (config_.local_solution_topic_type == "CUSTOM")
  {
    // Local solution publisher is defined by the local constraint solver plugin
  }

  state_ = moveit_hybrid_planning::LocalPlannerState::READY;
  return true;
}

void LocalPlannerComponent::executePlanningLoopRun()
{
  auto result = std::make_shared<moveit_msgs::action::LocalPlanner::Result>();

  // Do different things depending on the planner's internal state
  switch (state_)
  {
    // If READY start waiting for trajectory
    case moveit_hybrid_planning::LocalPlannerState::READY:
    {
      state_ = moveit_hybrid_planning::LocalPlannerState::AWAIT_GLOBAL_TRAJECTORY;
      break;
    }
    // Wait for global solution to be published
    case moveit_hybrid_planning::LocalPlannerState::AWAIT_GLOBAL_TRAJECTORY:
      // Do nothing
      break;
    // Notify action client that local planning failed
    case moveit_hybrid_planning::LocalPlannerState::ABORT:
    {
      local_planning_goal_handle_->abort(result);
      local_constraint_solver_instance_->reset();
      trajectory_operator_instance_->reset();
      timer_->cancel();

      // TODO(sjahr) add proper reset function
      state_ = moveit_hybrid_planning::LocalPlannerState::READY;
      break;
    }
    // If the planner received an action request and a global solution it starts to plan locally
    case moveit_hybrid_planning::LocalPlannerState::LOCAL_PLANNING_ACTIVE:
    {
      // Read current planning scene
      planning_scene_monitor_->updateFrameTransforms();
      planning_scene_monitor_->lockSceneRead();  // LOCK planning scene
      planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor_->getPlanningScene();
      planning_scene_monitor_->unlockSceneRead();  // UNLOCK planning scene

      // Get current state
      auto current_robot_state = planning_scene->getCurrentStateNonConst();

      // Check if the global goal is reached
      if (trajectory_operator_instance_->getTrajectoryProgress(current_robot_state) == 1.0)
      {
        local_planning_goal_handle_->succeed(result);
        state_ = moveit_hybrid_planning::LocalPlannerState::READY;
        local_constraint_solver_instance_->reset();
        trajectory_operator_instance_->reset();
        timer_->cancel();
        break;
      }

      // Get local goal trajectory to follow
      robot_trajectory::RobotTrajectory local_trajectory =
          robot_trajectory::RobotTrajectory(planning_scene_monitor_->getRobotModel(), "panda_arm");
      *local_planner_feedback_ =
          trajectory_operator_instance_->getLocalTrajectory(current_robot_state, local_trajectory);
      if (!local_planner_feedback_->feedback.empty())
      {
        local_planning_goal_handle_->publish_feedback(local_planner_feedback_);
      }

      // Solve local planning problem
      trajectory_msgs::msg::JointTrajectory local_solution;
      *local_planner_feedback_ = local_constraint_solver_instance_->solve(
          local_trajectory, local_planning_goal_handle_->get_goal(), local_solution);

      if (!local_planner_feedback_->feedback.empty())
      {
        local_planning_goal_handle_->publish_feedback(local_planner_feedback_);
      }

      // Use the configuarable message interface like MoveIt serve
      // (See https://github.com/ros-planning/moveit2/blob/main/moveit_ros/moveit_servo/src/servo_calcs.cpp)
      // Format outgoing msg in the right format
      // (trajectory_msgs/JointTrajectory or std_msgs/Float64MultiArray).
      if (config_.local_solution_topic_type == "trajectory_msgs/JointTrajectory")
      {
        local_trajectory_publisher_->publish(local_solution);
      }
      else if (config_.local_solution_topic_type == "std_msgs/Float64MultiArray")
      {
        // Transform "trajectory_msgs/JointTrajectory" to "std_msgs/Float64MultiArray"
        auto joints = std::make_unique<std_msgs::msg::Float64MultiArray>();
        if (!local_solution.points.empty())
        {
          joints->data = local_solution.points[0].positions;
        }
        else if (!local_solution.points.empty())
        {
          joints->data = local_solution.points[0].velocities;
        }
        local_solution_publisher_->publish(std::move(joints));
      }
      else if (config_.local_solution_topic_type == "CUSTOM")
      {
        // Local solution publisher is defined by the local constraint solver plugin
      }
      break;
    }
    default:
    {
      local_planning_goal_handle_->abort(result);
      timer_->cancel();
      local_constraint_solver_instance_->reset();
      trajectory_operator_instance_->reset();
      RCLCPP_ERROR(LOGGER, "Local planner somehow failed :(");  // TODO(sjahr) Add more detailed failure information
      state_ = moveit_hybrid_planning::LocalPlannerState::READY;
      break;
    }
  }
};
}  // namespace moveit_hybrid_planning

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_hybrid_planning::LocalPlannerComponent)
