/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_pipeline/planning_pipeline_middleware_handle.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/join.hpp>
#include <sstream>
#include <memory>

static const auto LOGGER = rclcpp::get_logger("moveit.ros_planning.planning_pipeline");

planning_pipeline::PlanningPipeline::PlanningPipeline(
    const moveit::core::RobotModelConstPtr& model,
    std::unique_ptr<PlanningPipeline::MiddlewareHandle> middleware_handle, const std::string& parameter_namespace,
    const std::string& planner_plugin_param_name, const std::string& adapter_plugins_param_name)
  : is_active_{ false }
  , middleware_handle_{ std::move(middleware_handle) }
  , parameter_namespace_{ parameter_namespace }
  , robot_model_{ model }
{
  std::string planner_plugin_fullname = parameter_namespace_ + "." + planner_plugin_param_name;
  if (parameter_namespace_.empty())
  {
    planner_plugin_fullname = planner_plugin_param_name;
  }
  if (middleware_handle_->hasParameter(planner_plugin_fullname))
  {
    middleware_handle_->getParameter(planner_plugin_fullname, planner_plugin_name_);
  }

  std::string adapter_plugins_fullname = parameter_namespace_ + "." + adapter_plugins_param_name;
  if (parameter_namespace_.empty())
  {
    adapter_plugins_fullname = adapter_plugins_param_name;
  }

  std::string adapters;
  if (middleware_handle_->hasParameter(adapter_plugins_fullname))
  {
    middleware_handle_->getParameter(adapter_plugins_fullname, adapters);
    boost::char_separator<char> sep(" ");
    boost::tokenizer<boost::char_separator<char>> tok(adapters, sep);
    for (boost::tokenizer<boost::char_separator<char>>::iterator beg = tok.begin(); beg != tok.end(); ++beg)
    {
      adapter_plugin_names_.push_back(*beg);
    }
  }

  configure();
}

planning_pipeline::PlanningPipeline::PlanningPipeline(const moveit::core::RobotModelConstPtr& model,
                                                      const std::shared_ptr<rclcpp::Node>& node,
                                                      const std::string& parameter_namespace,
                                                      const std::string& planner_plugin_param_name,
                                                      const std::string& adapter_plugins_param_name)
  : PlanningPipeline{ model, std::make_unique<PlanningPipelineMiddlewareHandle>(node), parameter_namespace,
                      planner_plugin_param_name, adapter_plugins_param_name }
{
}

planning_pipeline::PlanningPipeline::PlanningPipeline(
    const moveit::core::RobotModelConstPtr& model,
    std::unique_ptr<PlanningPipeline::MiddlewareHandle> middleware_handle, const std::string& parameter_namespace,
    const std::string& planner_plugin_name, const std::vector<std::string>& adapter_plugin_names)
  : is_active_{ false }
  , middleware_handle_{ std::move(middleware_handle) }
  , parameter_namespace_{ parameter_namespace }
  , planner_plugin_name_{ planner_plugin_name }
  , adapter_plugin_names_{ adapter_plugin_names }
  , robot_model_{ model }
{
  configure();
}

planning_pipeline::PlanningPipeline::PlanningPipeline(const moveit::core::RobotModelConstPtr& model,
                                                      const std::shared_ptr<rclcpp::Node>& node,
                                                      const std::string& parameter_namespace,
                                                      const std::string& planner_plugin_name,
                                                      const std::vector<std::string>& adapter_plugin_names)
  : PlanningPipeline{ model, std::make_unique<PlanningPipelineMiddlewareHandle>(node), parameter_namespace,
                      planner_plugin_name, adapter_plugin_names }
{
}

void planning_pipeline::PlanningPipeline::configure()
{
  is_checking_solution_paths_ = false;  // this is set to true below
  is_publishing_received_requests_ = false;
  is_displaying_computed_motion_plans_ = false;  // this is set to true below

  middleware_handle_->createPlannerPlugin(robot_model_);
  middleware_handle_->createAdapterPlugins(adapter_plugin_names_);

  displayComputedMotionPlans(true);
  checkSolutionPaths(true);
}

void planning_pipeline::PlanningPipeline::displayComputedMotionPlans(bool flag)
{
  if (is_displaying_computed_motion_plans_ && !flag)
  {
    middleware_handle_->resetDisplayPathPublisher();
  }
  else if (!is_displaying_computed_motion_plans_ && flag)
  {
    middleware_handle_->createDisplayPathPublisher(DISPLAY_PATH_TOPIC);
  }
  is_displaying_computed_motion_plans_ = flag;
}

void planning_pipeline::PlanningPipeline::publishReceivedRequests(bool flag)
{
  if (is_publishing_received_requests_ && !flag)
  {
    middleware_handle_->resetReceivedRequestPublisher();
  }
  else if (!is_publishing_received_requests_ && flag)
  {
    middleware_handle_->createReceivedRequestPublisher(MOTION_PLAN_REQUEST_TOPIC);
  }
  is_publishing_received_requests_ = flag;
}

void planning_pipeline::PlanningPipeline::checkSolutionPaths(bool flag)
{
  if (is_checking_solution_paths_ && !flag)
  {
    middleware_handle_->resetContactsPublisher();
  }
  else if (!is_checking_solution_paths_ && flag)
  {
    middleware_handle_->createContactsPublisher(MOTION_CONTACTS_TOPIC);
  }
  is_checking_solution_paths_ = flag;
}

bool planning_pipeline::PlanningPipeline::getDisplayComputedMotionPlans() const
{
  return is_displaying_computed_motion_plans_;
}

bool planning_pipeline::PlanningPipeline::getPublishReceivedRequests() const
{
  return is_publishing_received_requests_;
}

bool planning_pipeline::PlanningPipeline::getCheckSolutionPaths() const
{
  return is_checking_solution_paths_;
}

bool planning_pipeline::PlanningPipeline::generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                       const planning_interface::MotionPlanRequest& req,
                                                       planning_interface::MotionPlanResponse& res) const
{
  std::vector<std::size_t> dummy;
  return generatePlan(planning_scene, req, res, dummy);
}

bool planning_pipeline::PlanningPipeline::generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                       const planning_interface::MotionPlanRequest& req,
                                                       planning_interface::MotionPlanResponse& res,
                                                       std::vector<std::size_t>& adapter_added_state_index) const
{
  if (!middleware_handle_->getPlannerManager())
  {
    RCLCPP_ERROR(LOGGER, "No planning plugin loaded. Cannot plan.");
    // Set planning pipeline to inactive

    is_active_ = false;
    return false;
  }

  // Set planning pipeline active
  is_active_ = true;

  // broadcast the request we are about to work on, if needed
  if (is_publishing_received_requests_)
  {
    middleware_handle_->publishReceivedRequest(req);
  }
  adapter_added_state_index.clear();

  const auto is_solved = middleware_handle_->plan(planning_scene, req, res, adapter_added_state_index);

  if (!is_solved)
  {
    // This should alert the user if planning failed because of contradicting constraints.
    // Could be checked more thoroughly, but it is probably not worth going to that length.
    auto is_stacked_constraints{ false };
    if (req.path_constraints.position_constraints.size() > 1 || req.path_constraints.orientation_constraints.size() > 1)
    {
      is_stacked_constraints = true;
    }
    for (const auto& constraint : req.goal_constraints)
    {
      if (constraint.position_constraints.size() > 1 || constraint.orientation_constraints.size() > 1)
      {
        is_stacked_constraints = true;
      }
    }
    if (is_stacked_constraints)
    {
      RCLCPP_WARN(LOGGER, "More than one constraint is set. If your move_group does not have multiple end "
                          "effectors/arms, this is "
                          "unusual. Are you using a move_group_interface and forgetting to call clearPoseTargets() or "
                          "equivalent?");
    }
  }

  // display solution path if needed
  if (is_displaying_computed_motion_plans_)
  {
    moveit_msgs::msg::DisplayTrajectory disp;
    disp.model_id = robot_model_->getName();
    disp.trajectory.resize(1);
    res.trajectory_->getRobotTrajectoryMsg(disp.trajectory[0]);
    moveit::core::robotStateToRobotStateMsg(res.trajectory_->getFirstWayPoint(), disp.trajectory_start);
    middleware_handle_->publishDisplayPathTrejectory(disp);
  }

  if (res.trajectory_)
  {
    const auto state_count{ res.trajectory_->getWayPointCount() };
    RCLCPP_DEBUG(LOGGER, "Motion planner reported a solution path with %ld states", state_count);
    if (is_checking_solution_paths_)
    {
      visualization_msgs::msg::MarkerArray arr;
      visualization_msgs::msg::Marker m;
      m.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(m);

      std::vector<std::size_t> index;
      if (!planning_scene->isPathValid(*res.trajectory_, req.path_constraints, req.group_name, false, &index))
      {
        // check to see if there is any problem with the states that are found to be invalid
        // they are considered ok if they were added by a planning request adapter
        bool is_problem = false;
        for (std::size_t i = 0; i < index.size() && !is_problem; ++i)
        {
          bool is_found = false;
          for (std::size_t added_index : adapter_added_state_index)
          {
            if (index[i] == added_index)
            {
              is_found = true;
              break;
            }
            if (!is_found)
            {
              is_problem = true;
            }
          }
        }
        if (is_problem)
        {
          if (index.size() == 1 && index[0] == 0)
          {  // ignore cases when the robot starts at invalid location
            RCLCPP_DEBUG(LOGGER, "It appears the robot is starting at an invalid state, but that is ok.");
          }
          else
          {
            res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;

            // display error messages
            std::stringstream ss;
            for (std::size_t it : index)
            {
              ss << it << " ";
            }
            RCLCPP_ERROR_STREAM(LOGGER, "Computed path is not valid. Invalid states at index locations: [ "
                                            << ss.str() << "] out of " << state_count
                                            << ". Explanations follow in command line. Contacts are published on "
                                            << middleware_handle_->getContactsTopicName());

            // call validity checks in verbose mode for the problematic states
            for (std::size_t it : index)
            {
              // check validity with verbose on
              const moveit::core::RobotState& robot_state = res.trajectory_->getWayPoint(it);
              planning_scene->isStateValid(robot_state, req.path_constraints, req.group_name, true);

              // compute the contacts if any
              collision_detection::CollisionRequest c_req;
              collision_detection::CollisionResult c_res;
              c_req.contacts = true;
              c_req.max_contacts = 10;
              c_req.max_contacts_per_pair = 3;
              c_req.verbose = false;
              planning_scene->checkCollision(c_req, c_res, robot_state);
              if (c_res.contact_count > 0)
              {
                visualization_msgs::msg::MarkerArray arr_i;
                collision_detection::getCollisionMarkersFromContacts(arr_i, planning_scene->getPlanningFrame(),
                                                                     c_res.contacts);
                arr.markers.insert(arr.markers.end(), arr_i.markers.begin(), arr_i.markers.end());
              }
            }
            RCLCPP_ERROR(LOGGER, "Completed listing of explanations for invalid states.");
          }
          is_active_ = false;
          return false;
        }
        else
        {
          RCLCPP_DEBUG(LOGGER,
                       "Planned path was found to be valid, except for states that were added by planning request "
                       "adapters, but that is ok.");
        }
      }

      else
      {
        RCLCPP_DEBUG(LOGGER, "Planned path was found to be valid when rechecked");
      }
      middleware_handle_->publishContactsMarkerArray(arr);
    }
  }

  is_active_ = false;
  return is_solved;
}

void planning_pipeline::PlanningPipeline::terminate() const
{
  middleware_handle_->terminatePlannerPlugin();
}

const planning_interface::PlannerManagerPtr& planning_pipeline::PlanningPipeline::getPlannerManager()
{
  return middleware_handle_->getPlannerManager();
}
