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
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <boost/tokenizer.hpp>
#include <fmt/format.h>
#include <sstream>

#include <planning_pipeline_parameters.hpp>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.ros_planning.planning_pipeline");
}  // namespace

planning_pipeline::PlanningPipeline::PlanningPipeline(const moveit::core::RobotModelConstPtr& model,
                                                      const std::shared_ptr<rclcpp::Node>& node,
                                                      const std::string& parameter_namespace)
  : active_{ false }, node_(node), parameter_namespace_(parameter_namespace), robot_model_(model)
{
  auto param_listener = planning_pipeline_parameters::ParamListener(node, parameter_namespace);
  const auto params = param_listener.get_params();
  planner_plugin_name_ = params.planning_plugin;
  if (!params.request_adapters.empty())
  {
    boost::char_separator<char> sep(" ");
    boost::tokenizer<boost::char_separator<char>> tok(params.request_adapters, sep);
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
                                                      const std::string& planner_plugin_name,
                                                      const std::vector<std::string>& adapter_plugin_names)
  : active_{ false }
  , node_(node)
  , parameter_namespace_(parameter_namespace)
  , planner_plugin_name_(planner_plugin_name)
  , adapter_plugin_names_(adapter_plugin_names)
  , robot_model_(model)
{
  configure();
}

void planning_pipeline::PlanningPipeline::configure()
{
  // Optional publishers for debugging
  received_request_publisher_ = node_->create_publisher<moveit_msgs::msg::MotionPlanRequest>(
      MOTION_PLAN_REQUEST_TOPIC, rclcpp::SystemDefaultsQoS());
  contacts_publisher_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(MOTION_CONTACTS_TOPIC, rclcpp::SystemDefaultsQoS());
  display_path_publisher_ =
      node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(DISPLAY_PATH_TOPIC, rclcpp::SystemDefaultsQoS());

  // load the planning plugin
  try
  {
    planner_plugin_loader_ = std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
        "moveit_core", "planning_interface::PlannerManager");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
    throw;
  }

  if (planner_plugin_name_.empty() || planner_plugin_name_ == "UNKNOWN")
  {
    std::string classes_str = fmt::format("{}", fmt::join(planner_plugin_loader_->getDeclaredClasses(), ", "));
    throw std::runtime_error("Planning plugin name is empty or not defined in namespace '" + parameter_namespace_ +
                             "'. Please choose one of the available plugins: " + classes_str);
  }

  try
  {
    planner_instance_ = planner_plugin_loader_->createUniqueInstance(planner_plugin_name_);
    if (!planner_instance_->initialize(robot_model_, node_, parameter_namespace_))
    {
      throw std::runtime_error("Unable to initialize planning plugin");
    }
    RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance_->getDescription().c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    std::string classes_str = fmt::format("{}", fmt::join(planner_plugin_loader_->getDeclaredClasses(), ", "));
    RCLCPP_FATAL(LOGGER,
                 "Exception while loading planner '%s': %s"
                 "Available plugins: %s",
                 planner_plugin_name_.c_str(), ex.what(), classes_str.c_str());
    throw;
  }

  // load the planner request adapters
  if (!adapter_plugin_names_.empty())
  {
    std::vector<planning_request_adapter::PlanningRequestAdapterConstPtr> planning_request_adapter_vector;
    try
    {
      adapter_plugin_loader_ =
          std::make_unique<pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>>(
              "moveit_core", "planning_request_adapter::PlanningRequestAdapter");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
      throw;
    }

    if (adapter_plugin_loader_)
    {
      for (const std::string& adapter_plugin_name : adapter_plugin_names_)
      {
        planning_request_adapter::PlanningRequestAdapterPtr planning_request_adapter;
        try
        {
          planning_request_adapter = adapter_plugin_loader_->createUniqueInstance(adapter_plugin_name);
        }
        catch (pluginlib::PluginlibException& ex)
        {
          RCLCPP_FATAL(LOGGER, "Exception while loading planning adapter plugin '%s': %s", adapter_plugin_name.c_str(),
                       ex.what());
          throw;
        }
        if (planning_request_adapter)
        {
          planning_request_adapter->initialize(node_, parameter_namespace_);
          planning_request_adapter_vector.push_back(std::move(planning_request_adapter));
        }
      }
    }
    if (!planning_request_adapter_vector.empty())
    {
      adapter_chain_ = std::make_unique<planning_request_adapter::PlanningRequestAdapterChain>();
      for (planning_request_adapter::PlanningRequestAdapterConstPtr& planning_request_adapter :
           planning_request_adapter_vector)
      {
        RCLCPP_INFO(LOGGER, "Using planning request adapter '%s'", planning_request_adapter->getDescription().c_str());
        adapter_chain_->addAdapter(planning_request_adapter);
      }
    }
  }
  else
  {
    RCLCPP_WARN(LOGGER, "No planning request adapter names specified.");
  }
}

bool planning_pipeline::PlanningPipeline::generatePlan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                       const planning_interface::MotionPlanRequest& req,
                                                       planning_interface::MotionPlanResponse& res,
                                                       const bool publish_received_requests,
                                                       const bool check_solution_paths,
                                                       const bool display_computed_motion_plans) const
{
  assert(planner_instance_ != nullptr);

  // Set planning pipeline active
  active_ = true;

  // broadcast the request we are about to work on, if needed
  if (publish_received_requests)
  {
    received_request_publisher_->publish(req);
  }

  // ---------------------------------
  // Solve the motion planning problem
  // ---------------------------------
  bool solved = false;
  try
  {
    if (adapter_chain_)
    {
      solved = adapter_chain_->adaptAndPlan(planner_instance_, planning_scene, req, res);
      if (!res.added_path_index.empty())
      {
        std::stringstream ss;
        for (std::size_t added_index : res.added_path_index)
          ss << added_index << ' ';
        RCLCPP_INFO(LOGGER, "Planning adapters have added states at index positions: [ %s]", ss.str().c_str());
      }
    }
    else
    {
      planning_interface::PlanningContextPtr context =
          planner_instance_->getPlanningContext(planning_scene, req, res.error_code);
      solved = context ? context->solve(res) : false;
    }
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception caught: '%s'", ex.what());
    // Set planning pipeline to inactive
    active_ = false;
    return false;
  }

  // -----------------
  // Validate solution
  // -----------------
  if (solved && res.trajectory)
  {
    std::size_t state_count = res.trajectory->getWayPointCount();
    RCLCPP_DEBUG(LOGGER, "Motion planner reported a solution path with %ld states", state_count);
    if (check_solution_paths)
    {
      visualization_msgs::msg::MarkerArray arr;
      visualization_msgs::msg::Marker m;
      m.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(m);

      std::vector<std::size_t> indices;
      if (!planning_scene->isPathValid(*res.trajectory, req.path_constraints, req.group_name, false, &indices))
      {
        // check to see if there is any problem with the states that are found to be invalid
        // they are considered ok if they were added by a planning request adapter
        bool problem = false;
        for (const auto& index : indices)
        {
          bool found = false;
          for (const std::size_t& added_index : res.added_path_index)
          {
            if (index == added_index)
            {
              found = true;
              break;
            }
          }
          if (!found)
          {
            problem = true;
            break;
          }
        }
        if (problem)
        {
          if (indices.size() == 1 && indices.at(0) == 0)
          {  // ignore cases when the robot starts at invalid location
            RCLCPP_DEBUG(LOGGER, "It appears the robot is starting at an invalid state, but that is ok.");
          }
          else
          {
            res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;

            // display error messages
            std::stringstream ss;
            for (std::size_t it : indices)
            {
              ss << it << ' ';
            }

            RCLCPP_ERROR_STREAM(LOGGER, "Computed path is not valid. Invalid states at index locations: [ "
                                            << ss.str() << "] out of " << state_count
                                            << ". Explanations follow in command line. Contacts are published on "
                                            << contacts_publisher_->get_topic_name());

            // call validity checks in verbose mode for the problematic states
            for (std::size_t it : indices)
            {
              // check validity with verbose on
              const moveit::core::RobotState& robot_state = res.trajectory->getWayPoint(it);
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
        }
        else
        {
          RCLCPP_DEBUG(LOGGER,
                       "Planned path was found to be valid, except for states that were added by planning request "
                       "adapters, but that is ok.");
        }
      }
      else
        RCLCPP_DEBUG(LOGGER, "Planned path was found to be valid when rechecked");
      contacts_publisher_->publish(arr);
    }

    // Optionally publish DisplayTrajectory msg
    if (display_computed_motion_plans)
    {
      moveit_msgs::msg::DisplayTrajectory disp;
      disp.model_id = robot_model_->getName();
      disp.trajectory.resize(1);
      res.trajectory->getRobotTrajectoryMsg(disp.trajectory.at(0));
      moveit::core::robotStateToRobotStateMsg(res.trajectory->getFirstWayPoint(), disp.trajectory_start);
      display_path_publisher_->publish(disp);
    }
  }
  else  // If no trajectory exists, let's see if it might be related to stacked constraints
  {
    // This should alert the user if planning failed because of contradicting constraints.
    // Could be checked more thoroughly, but it is probably not worth going to that length.
    bool stacked_constraints = false;
    if (req.path_constraints.position_constraints.size() > 1 || req.path_constraints.orientation_constraints.size() > 1)
    {
      stacked_constraints = true;
    }
    for (const auto& constraint : req.goal_constraints)
    {
      if (constraint.position_constraints.size() > 1 || constraint.orientation_constraints.size() > 1)
      {
        stacked_constraints = true;
      }
    }
    if (stacked_constraints)
    {
      RCLCPP_WARN(LOGGER, "More than one constraint is set. If your move_group does not have multiple end "
                          "effectors/arms, this is "
                          "unusual. Are you using a move_group_interface and forgetting to call clearPoseTargets() or "
                          "equivalent?");
    }
  }

  // Make sure that planner id is set in the response
  if (res.planner_id.empty())
  {
    RCLCPP_WARN(LOGGER, "The planner plugin did not fill out the 'planner_id' field of the MotionPlanResponse. Setting "
                        "it to the planner ID name of the MotionPlanRequest assuming that the planner plugin does warn "
                        "you if it does not use the requested planner.");
    res.planner_id = req.planner_id;
  }

  // Set planning pipeline to inactive
  active_ = false;
  return solved && bool(res);
}

void planning_pipeline::PlanningPipeline::terminate() const
{
  if (planner_instance_)
  {
    planner_instance_->terminate();
  }
}
