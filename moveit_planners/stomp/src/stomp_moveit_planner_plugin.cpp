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
 * @brief Planning Plugin implementation for STOMP
 */

#include <class_loader/class_loader.hpp>
#include <stomp_moveit/stomp_moveit_planning_context.hpp>
#include <moveit/utils/logger.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace stomp_moveit
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.planners.stomp.planner_manager");
}
}  // namespace

using namespace planning_interface;

class StompPlannerManager : public PlannerManager
{
public:
  StompPlannerManager() = default;
  ~StompPlannerManager() override = default;

  bool initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override
  {
    robot_model_ = model;
    node_ = node;
    parameter_namespace_ = parameter_namespace;
    param_listener_ = std::make_shared<stomp_moveit::ParamListener>(node, parameter_namespace);

    return true;
  }

  std::string getDescription() const override
  {
    return "STOMP";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs = { "STOMP" };
  }

  PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                        const MotionPlanRequest& req,
                                        moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    if (!canServiceRequest(req))
    {
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return nullptr;
    }

    const auto params = param_listener_->get_params();

    std::shared_ptr<StompPlanningContext> planning_context =
        std::make_shared<StompPlanningContext>("STOMP", req.group_name, params);
    planning_context->setPlanningScene(planning_scene);
    planning_context->setMotionPlanRequest(req);

    // Only create path publisher if topic parameter has been set
    if (!params.path_marker_topic.empty())
    {
      auto path_publisher = node_->create_publisher<visualization_msgs::msg::MarkerArray>(params.path_marker_topic,
                                                                                          rclcpp::SystemDefaultsQoS());
      planning_context->setPathPublisher(path_publisher);
    }

    return planning_context;
  }

  bool canServiceRequest(const MotionPlanRequest& req) const override
  {
    if (req.goal_constraints.empty())
    {
      RCLCPP_ERROR(getLogger(), "Invalid goal constraints");
      return false;
    }

    if (req.group_name.empty() || !robot_model_->hasJointModelGroup(req.group_name))
    {
      RCLCPP_ERROR(getLogger(), "Invalid joint group '%s'", req.group_name.c_str());
      return false;
    }
    return true;
  }

  void setPlannerConfigurations(const PlannerConfigurationMap& /*pcs*/) override
  {
  }

private:
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Node::SharedPtr node_;
  std::string parameter_namespace_;
  std::shared_ptr<stomp_moveit::ParamListener> param_listener_;
};

}  // namespace stomp_moveit

CLASS_LOADER_REGISTER_CLASS(stomp_moveit::StompPlannerManager, planning_interface::PlannerManager)
