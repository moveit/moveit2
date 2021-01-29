/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit_msgs/action/global_planner.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>

namespace moveit_hybrid_planning
{
// Component node containing the global planner
class GlobalPlannerComponent : public rclcpp::Node
{
public:
  GlobalPlannerComponent(const rclcpp::NodeOptions& options);

  // TODO(sjahr) implement get_last_solution service

private:
  rclcpp::TimerBase::SharedPtr timer_;
  bool initialized_{ false };
  struct GlobalPlannerConfig  // TODO(sjahr) Implement this properly
  {
    // Planning scene monitor
    std::string publish_planning_scene_topic;

    // Planning pipelines
    std::vector<std::string> pipeline_names;
  };

  GlobalPlannerConfig config_;
  // Planning pipelines and scene monitor
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  std::map<std::string, planning_pipeline::PlanningPipelinePtr> planning_pipelines_;
  std::map<std::string, std::set<std::string>> groups_pipelines_map_;

  moveit_msgs::msg::MotionPlanResponse last_global_solution_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Global planning request action server
  rclcpp_action::Server<moveit_msgs::action::GlobalPlanner>::SharedPtr global_planning_request_server_;

  // Global trajectory publisher
  rclcpp::Publisher<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_trajectory_pub_;

  // Goal callback for global planning request action server
  void globalPlanningRequestCallback(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> goal_handle);

  // Initialize planning scene monitor and load pipelines
  bool init();

  // Plan global trajectory
  moveit_msgs::msg::MotionPlanResponse plan(const moveit_msgs::msg::MotionPlanRequest& planning_problem);
};

}  // namespace moveit_hybrid_planning
