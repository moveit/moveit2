/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik, Inc.
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
 *   * Neither the name of PickNik nor the names of its
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

/* Author: Anthony Baker */

#pragma once
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace planning_pipeline
{
class PlanningPipelineMiddlewareHandle : public PlanningPipeline::MiddlewareHandle
{
public:
  PlanningPipelineMiddlewareHandle(const rclcpp::Node::SharedPtr& node);

  /**
   * @brief     Create a display path publisher
   * @param[in] topic the topic
   */
  void createDisplayPathPublisher(const std::string& topic) override;

  /**
   * @brief     Create a received request publisher
   * @param[in] topic the topic
   */
  void createReceivedRequestPublisher(const std::string& topic) override;

  /**
   * @brief     Create a contacts publisher
   * @param[in] topic the topic
   */
  void createContactsPublisher(const std::string& topic) override;

  std::string getContactsTopicName() const override;

  /**
   * @brief     Reset a display path publisher
   */
  void resetDisplayPathPublisher() override;

  /**
   * @brief     Reset a received request publisher
   */
  void resetReceivedRequestPublisher() override;

  /**
   * @brief     Reset a contacts publisher
   */
  void resetContactsPublisher() override;

  /**
   * @brief     Check if parameter exists
   */
  bool hasParameter(const std::string& name) const override;

  /**
   * @brief     Get parameter of specific plugin
   * @param[in] pluginName name of plugin that has parameter
   * @param[in] parameters container for parameter names
   */
  void getParameter(const std::string& pluginName, std::string& parameter) const override;

  /**
   * @brief     Create a planner plugin
   * @param[in] name plugin name
   */
  void createPlannerPlugin(const moveit::core::RobotModelConstPtr& robot_model) override;

  /**
   * @brief Retrieves a pointer to a planner manager
   */
  const planning_interface::PlannerManagerPtr& getPlannerManager() const override;

  /**
   * @brief     Create a adapter plugin
   * @param[in] name plugin name
   */
  void createAdapterPlugins(const std::vector<std::string>& name) override;

  /**
   * @brief     Given the following parameters, plan the requested motion and return whether the planned motion was successful
   * @param[in] planning_scene The current planning scene
   * @param[in] req The requested motion plan
   * @param[in] res The resulting motion plan
   * @param[in] adapter_added_state_index
   */
  bool plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
            const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
            std::vector<std::size_t>& adapter_added_state_index) override;

  /**
   * @brief     Create a contacts publisher
   * @param[in] topic the topic
   */
  void terminatePlannerPlugin() override;

  /**
   * @brief     Publish received motion plan request
   * @param[in] req Received Motion Plan to be published
   */
  void publishReceivedRequest(const planning_interface::MotionPlanRequest& req) override;

  /**
   * @brief     Publish contact markers
   * @param[in] markerArray Contact Marker Array to be published
   */
  void publishContactsMarkerArray(const visualization_msgs::msg::MarkerArray& markerArray) override;

  /**
   * @brief     Publish Display Trejectory Path
   * @param[in] displayTrajectory Display Trajectory to be published
   */
  void publishDisplayPathTrejectory(const moveit_msgs::msg::DisplayTrajectory& displayTrajectory) override;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_path_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::MotionPlanRequest>::SharedPtr received_request_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr contacts_publisher_;

  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader_;
  planning_interface::PlannerManagerPtr planner_instance_;
  std::string planner_plugin_name_;
  std::string parameter_namespace_;

  std::unique_ptr<pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter> > adapter_plugin_loader_;
  std::unique_ptr<planning_request_adapter::PlanningRequestAdapterChain> adapter_chain_;
  std::vector<std::string> adapter_plugin_names_;
};
}  // namespace planning_pipeline
