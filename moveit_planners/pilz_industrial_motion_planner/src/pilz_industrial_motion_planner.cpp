/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#include <rclcpp/logging.hpp>

#include <pilz_industrial_motion_planner/pilz_industrial_motion_planner.hpp>

#include <pilz_industrial_motion_planner/planning_context_loader.hpp>
#include <pilz_industrial_motion_planner/planning_context_loader_ptp.hpp>
#include <pilz_industrial_motion_planner/planning_exceptions.hpp>

#include <pilz_industrial_motion_planner/cartesian_limits_parameters.hpp>
#include <pilz_industrial_motion_planner/joint_limits_aggregator.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <pluginlib/class_loader.hpp>

#include <memory>
#include <moveit/utils/logger.hpp>

namespace pilz_industrial_motion_planner
{
namespace
{
const std::string PARAM_NAMESPACE_LIMITS = "robot_description_planning";
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.planners.pilz.command_planner");
}
}  // namespace

bool CommandPlanner::initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                                const std::string& ns)
{
  // Call parent class initialize
  planning_interface::PlannerManager::initialize(model, node, ns);

  // Store the model and the namespace
  model_ = model;
  namespace_ = ns;

  // Obtain the aggregated joint limits
  aggregated_limit_active_joints_ = pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(
      node, PARAM_NAMESPACE_LIMITS, model->getActiveJointModels());

  // Obtain cartesian limits
  param_listener_ =
      std::make_shared<cartesian_limits::ParamListener>(node, PARAM_NAMESPACE_LIMITS + ".cartesian_limits");
  params_ = param_listener_->get_params();

  interpolation_param_listener_ =
      std::make_shared<interpolation::ParamListener>(node, PARAM_NAMESPACE_LIMITS + ".interpolation");

  // Load the planning context loader
  planner_context_loader_ = std::make_unique<pluginlib::ClassLoader<PlanningContextLoader>>(
      "pilz_industrial_motion_planner", "pilz_industrial_motion_planner::PlanningContextLoader");

  // List available plugins
  const std::vector<std::string>& factories = planner_context_loader_->getDeclaredClasses();
  std::stringstream ss;
  for (const auto& factory : factories)
  {
    ss << factory << ' ';
  }

  RCLCPP_INFO_STREAM(getLogger(), "Available plugins: " << ss.str());

  // Load each factory
  for (const auto& factory : factories)
  {
    RCLCPP_INFO_STREAM(getLogger(), "About to load: " << factory);
    PlanningContextLoaderPtr loader_pointer(planner_context_loader_->createSharedInstance(factory));

    pilz_industrial_motion_planner::LimitsContainer limits;
    limits.setJointLimits(aggregated_limit_active_joints_);
    limits.setCartesianLimits(params_);

    loader_pointer->setLimits(limits);
    loader_pointer->setModel(model_);
    loader_pointer->setInterpolationParamListener(interpolation_param_listener_);

    registerContextLoader(loader_pointer);
  }

  return true;
}

std::string CommandPlanner::getDescription() const
{
  return "Pilz Industrial Motion Planner";
}

void CommandPlanner::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();

  for (const auto& context_loader : context_loader_map_)
  {
    algs.push_back(context_loader.first);
  }
}

planning_interface::PlanningContextPtr
CommandPlanner::getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                   const moveit_msgs::msg::MotionPlanRequest& req,
                                   moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  // TODO(henningkayser): print req
  // RCLCPP_DEBUG_STREAM(getLogger(), "Loading PlanningContext for request\n<request>\n" << req << "\n</request>");
  RCLCPP_DEBUG(getLogger(), "Loading PlanningContext");

  // Check that a loaded for this request exists
  if (!canServiceRequest(req))
  {
    RCLCPP_ERROR_STREAM(getLogger(), "No ContextLoader for planner_id '" << req.planner_id.c_str()
                                                                         << "' found. Planning not possible.");
    return nullptr;
  }

  planning_interface::PlanningContextPtr planning_context;

  if (context_loader_map_.at(req.planner_id)->loadContext(planning_context, req.planner_id, req.group_name))
  {
    RCLCPP_DEBUG_STREAM(getLogger(),
                        "Found planning context loader for " << req.planner_id << " group:" << req.group_name);
    planning_context->setMotionPlanRequest(req);
    planning_context->setPlanningScene(planning_scene);
    return planning_context;
  }
  else
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return nullptr;
  }
}

bool CommandPlanner::canServiceRequest(const moveit_msgs::msg::MotionPlanRequest& req) const
{
  if (context_loader_map_.find(req.planner_id) == context_loader_map_.end())
  {
    RCLCPP_ERROR(getLogger(), "Cannot service planning request because planner ID '%s' does not exist.",
                 req.planner_id.c_str());
    return false;
  }

  if (req.group_name.empty())
  {
    RCLCPP_ERROR(getLogger(), "Cannot service planning request because group name is not specified.");
    return false;
  }

  auto joint_mode_group_ptr = model_->getJointModelGroup(req.group_name);
  if (joint_mode_group_ptr == nullptr)
  {
    RCLCPP_ERROR(getLogger(), "Cannot service planning request because group '%s' does not exist.",
                 req.group_name.c_str());
    return false;
  }

  if (joint_mode_group_ptr->getSolverInstance() == nullptr)
  {
    RCLCPP_ERROR(getLogger(), "Cannot service planning request because group '%s' does have an IK solver instance.",
                 req.group_name.c_str());
    return false;
  }

  if (!req.trajectory_constraints.constraints.empty())
  {
    RCLCPP_ERROR(getLogger(),
                 "Cannot service planning request because PILZ does not support 'trajectory constraints'.");
    return false;
  }

  return true;
}

void CommandPlanner::registerContextLoader(
    const pilz_industrial_motion_planner::PlanningContextLoaderPtr& planning_context_loader)
{
  // Only add if command is not already in list, throw exception if not
  if (context_loader_map_.find(planning_context_loader->getAlgorithm()) == context_loader_map_.end())
  {
    context_loader_map_[planning_context_loader->getAlgorithm()] = planning_context_loader;
    RCLCPP_INFO_STREAM(getLogger(), "Registered Algorithm [" << planning_context_loader->getAlgorithm() << ']');
  }
  else
  {
    throw ContextLoaderRegistrationException("The command [" + planning_context_loader->getAlgorithm() +
                                             "] is already registered");
  }
}

}  // namespace pilz_industrial_motion_planner

PLUGINLIB_EXPORT_CLASS(pilz_industrial_motion_planner::CommandPlanner, planning_interface::PlannerManager)
