/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik Inc.
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

/* Author: Henning Kayser */

#include <stdexcept>

#include <moveit/controller_manager/controller_manager.h>
#include <moveit/planning_pipeline_interfaces/planning_pipeline_interfaces.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <moveit/utils/logger.hpp>

namespace moveit_cpp
{
MoveItCpp::MoveItCpp(const rclcpp::Node::SharedPtr& node) : MoveItCpp(node, Options(node))
{
}

MoveItCpp::MoveItCpp(const rclcpp::Node::SharedPtr& node, const Options& options)
  : node_(node), logger_(moveit::getLogger("moveit_cpp"))
{
  // Configure planning scene monitor
  if (!loadPlanningSceneMonitor(options.planning_scene_monitor_options))
  {
    const std::string error = "Unable to configure planning scene monitor";
    RCLCPP_FATAL_STREAM(logger_, error);
    throw std::runtime_error(error);
  }

  if (!getRobotModel())
  {
    const std::string error = "Unable to construct robot model. Please make sure all needed information is on the "
                              "parameter server.";
    RCLCPP_FATAL_STREAM(logger_, error);
    throw std::runtime_error(error);
  }

  bool load_planning_pipelines = true;
  if (load_planning_pipelines && !loadPlanningPipelines(options.planning_pipeline_options))
  {
    const std::string error = "Failed to load planning pipelines from parameter server";
    RCLCPP_FATAL_STREAM(logger_, error);
    throw std::runtime_error(error);
  }

  trajectory_execution_manager_ = std::make_shared<trajectory_execution_manager::TrajectoryExecutionManager>(
      node_, getRobotModel(), planning_scene_monitor_->getStateMonitor());

  RCLCPP_DEBUG(logger_, "MoveItCpp running");
}

MoveItCpp::~MoveItCpp()
{
  RCLCPP_INFO(logger_, "Deleting MoveItCpp");
}

bool MoveItCpp::loadPlanningSceneMonitor(const PlanningSceneMonitorOptions& options)
{
  planning_scene_monitor_ =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, options.robot_description, options.name);
  // Allows us to synchronize to Rviz and also publish collision objects to ourselves
  RCLCPP_DEBUG(logger_, "Configuring Planning Scene Monitor");
  if (planning_scene_monitor_->getPlanningScene())
  {
    // Start state and scene monitors
    RCLCPP_INFO(logger_, "Listening to '%s' for joint states", options.joint_state_topic.c_str());
    // Subscribe to JointState sensor messages
    planning_scene_monitor_->startStateMonitor(options.joint_state_topic, options.attached_collision_object_topic);
    // Publish planning scene updates to remote monitors like RViz
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                          options.monitored_planning_scene_topic);
    // Monitor and apply planning scene updates from remote publishers like the PlanningSceneInterface
    planning_scene_monitor_->startSceneMonitor(options.publish_planning_scene_topic);
    // Monitor requests for changes in the collision environment
    planning_scene_monitor_->startWorldGeometryMonitor();
  }
  else
  {
    RCLCPP_ERROR(logger_, "Planning scene not configured");
    return false;
  }

  // Wait for complete state to be received
  if (options.wait_for_initial_state_timeout > 0.0)
  {
    return planning_scene_monitor_->getStateMonitor()->waitForCurrentState(node_->now(),
                                                                           options.wait_for_initial_state_timeout);
  }

  return true;
}

bool MoveItCpp::loadPlanningPipelines(const PlanningPipelineOptions& options)
{
  // TODO(henningkayser): Use parent namespace for planning pipeline config lookup
  planning_pipelines_ =
      moveit::planning_pipeline_interfaces::createPlanningPipelineMap(options.pipeline_names, getRobotModel(), node_);

  if (planning_pipelines_.empty())
  {
    RCLCPP_ERROR(logger_, "Failed to load any planning pipelines.");
    return false;
  }
  return true;
}

moveit::core::RobotModelConstPtr MoveItCpp::getRobotModel() const
{
  return planning_scene_monitor_->getRobotModel();
}

const rclcpp::Node::SharedPtr& MoveItCpp::getNode() const
{
  return node_;
}

bool MoveItCpp::getCurrentState(moveit::core::RobotStatePtr& current_state, double wait_seconds)
{
  if (wait_seconds > 0.0 && !planning_scene_monitor_->getStateMonitor()->waitForCurrentState(node_->now(), wait_seconds))
  {
    RCLCPP_ERROR(logger_, "Did not receive robot state");
    return false;
  }
  {  // Lock planning scene
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    current_state = std::make_shared<moveit::core::RobotState>(scene->getCurrentState());
  }  // Unlock planning scene
  return true;
}

moveit::core::RobotStatePtr MoveItCpp::getCurrentState(double wait)
{
  moveit::core::RobotStatePtr current_state;
  getCurrentState(current_state, wait);
  return current_state;
}

const std::unordered_map<std::string, planning_pipeline::PlanningPipelinePtr>& MoveItCpp::getPlanningPipelines() const
{
  return planning_pipelines_;
}

planning_scene_monitor::PlanningSceneMonitorConstPtr MoveItCpp::getPlanningSceneMonitor() const
{
  return planning_scene_monitor_;
}

planning_scene_monitor::PlanningSceneMonitorPtr MoveItCpp::getPlanningSceneMonitorNonConst()
{
  return planning_scene_monitor_;
}

trajectory_execution_manager::TrajectoryExecutionManagerConstPtr MoveItCpp::getTrajectoryExecutionManager() const
{
  return trajectory_execution_manager_;
}

trajectory_execution_manager::TrajectoryExecutionManagerPtr MoveItCpp::getTrajectoryExecutionManagerNonConst()
{
  return trajectory_execution_manager_;
}

moveit_controller_manager::ExecutionStatus
MoveItCpp::execute(const robot_trajectory::RobotTrajectoryPtr& robot_trajectory, bool /* blocking */,
                   const std::vector<std::string>& /* controllers */)
{
  return execute(robot_trajectory);
}

moveit_controller_manager::ExecutionStatus
MoveItCpp::execute(const robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
                   const std::vector<std::string>& controllers)
{
  if (!robot_trajectory)
  {
    RCLCPP_ERROR(logger_, "Robot trajectory is undefined");
    return moveit_controller_manager::ExecutionStatus::ABORTED;
  }

  const std::string group_name = robot_trajectory->getGroupName();

  // Check if there are controllers that can handle the execution
  if (!trajectory_execution_manager_->ensureActiveControllersForGroup(group_name))
  {
    RCLCPP_ERROR(logger_, "Execution failed! No active controllers configured for group '%s'", group_name.c_str());
    return moveit_controller_manager::ExecutionStatus::ABORTED;
  }

  // Execute trajectory
  moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
  robot_trajectory->getRobotTrajectoryMsg(robot_trajectory_msg);
  trajectory_execution_manager_->push(robot_trajectory_msg, controllers);
  trajectory_execution_manager_->execute();
  return trajectory_execution_manager_->waitForExecution();
}

bool MoveItCpp::terminatePlanningPipeline(const std::string& pipeline_name)
{
  try
  {
    const auto& planning_pipeline = planning_pipelines_.at(pipeline_name);
    if (planning_pipeline->isActive())
    {
      planning_pipeline->terminate();
    }
    return true;
  }
  catch (const std::out_of_range& oor)
  {
    RCLCPP_ERROR(logger_, "Cannot terminate pipeline '%s' because no pipeline with that name exists",
                 pipeline_name.c_str());
    return false;
  }
}

std::shared_ptr<const tf2_ros::Buffer> MoveItCpp::getTFBuffer() const
{
  return planning_scene_monitor_->getTFClient();
}
std::shared_ptr<tf2_ros::Buffer> MoveItCpp::getTFBuffer()
{
  return planning_scene_monitor_->getTFClient();
}

}  // namespace moveit_cpp
