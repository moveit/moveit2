/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#pragma once

#include <memory>
#include <rclcpp/version.h>
// For Rolling, Kilted, and newer
#if RCLCPP_VERSION_GTE(29, 6, 0)
#include <tf2_ros/buffer.hpp>
// For Jazzy and older
#else
#include <tf2_ros/buffer.h>
#endif
#include <moveit/planning_scene_monitor/current_state_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>

namespace moveit
{
namespace planning_interface
{
std::shared_ptr<tf2_ros::Buffer> getSharedTF();

robot_model_loader::RobotModelLoaderPtr getSharedRobotModelLoader(const rclcpp::Node::SharedPtr& node,
                                                                  const std::string& robot_description);

moveit::core::RobotModelConstPtr getSharedRobotModel(const rclcpp::Node::SharedPtr& node,
                                                     const std::string& robot_description);

/**
  @brief getSharedStateMonitor

  @param node A rclcpp::Node::SharePtr to pass node specific configurations, such as callbacks queues.
  @param robot_model
  @param tf_buffer
  @return
 */
planning_scene_monitor::CurrentStateMonitorPtr
getSharedStateMonitor(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
                      const std::shared_ptr<tf2_ros::Buffer>& tf_buffer);
}  // namespace planning_interface
}  // namespace moveit
