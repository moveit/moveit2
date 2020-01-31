/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Michael Ferguson, Ioan Sucan, E. Gil Jones */

#include <rclcpp/rclcpp.hpp>
#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <moveit_simple_controller_manager/gripper_controller_handle.h>
#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>
#include <boost/algorithm/string/join.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <map>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.plugins.moveit_simple_controller_manager");

// TOOD(JafarAbdi): Should be in moveit/utils .?
/*
 * Check if the map of parameters contains all the input keys
 */
bool isSubParameter(const std::map<std::string, rclcpp::Parameter>& parameters, const std::vector<std::string>& keys)
{
  std::vector<std::string> missing;
  for (const std::string& key : keys)
  {
    if (parameters.find(key) == parameters.end())
      missing.push_back(key);
  }
  if (!missing.empty())
  {
    RCLCPP_WARN_STREAM(LOGGER, "The input parameter map doesn't contain the keys "
                                   << boost::join(keys, ",") << " (misses " << boost::join(missing, ",") << ")");
  }
  return missing.empty();
}

namespace moveit_simple_controller_manager
{
class MoveItSimpleControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  MoveItSimpleControllerManager() = default;

  ~MoveItSimpleControllerManager() override = default;

  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    node_ = node;
    if (!node_->has_parameter("controller_names"))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "No controller_names specified.");
      return;
    }
    rclcpp::Parameter controller_names_param;
    node_->get_parameter("controller_names", controller_names_param);
    if (controller_names_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
      RCLCPP_ERROR(LOGGER, "Parameter controller_names should be specified as a string array");
      return;
    }
    std::vector<std::string> controller_names = controller_names_param.as_string_array();
    /* actually create each controller */
    for (const std::string& controller_name : controller_names)
    {
      std::map<std::string, rclcpp::Parameter> controller_params;
      //
      node_->get_parameters(controller_name, controller_params);
      if (!isSubParameter(controller_params, { "joints", "action_ns", "type" }))
      {
        RCLCPP_ERROR_STREAM(LOGGER, "joints, action_ns, and type must be specifed for the " << controller_name
                                                                                            << " controller");
        continue;
      }

      try
      {
        if (controller_params.at("joints").get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
          RCLCPP_ERROR_STREAM(LOGGER, "The list of joints for controller " << controller_name
                                                                           << " is not specified as a string array");
          continue;
        }
        if (controller_params.at("action_ns").get_type() != rclcpp::ParameterType::PARAMETER_STRING)
        {
          RCLCPP_ERROR_STREAM(LOGGER, "The action_ns for controller " << controller_name
                                                                      << " is not specified as a string");
          continue;
        }
        if (controller_params.at("type").get_type() != rclcpp::ParameterType::PARAMETER_STRING)
        {
          RCLCPP_ERROR_STREAM(LOGGER, "The type for controller " << controller_name << " is not specified as a string");
          continue;
        }

        const std::string action_ns = controller_params.at("action_ns").as_string();
        const std::string type = controller_params.at("type").as_string();

        ActionBasedControllerHandleBasePtr new_handle;
        if (type == "GripperCommand")
        {
          new_handle = std::make_shared<GripperControllerHandle>(node_, controller_name, action_ns);
          if (static_cast<GripperControllerHandle*>(new_handle.get())->isConnected())
          {
            if (isSubParameter(controller_params, { "parallel" }))
            {
              std::vector<std::string> controller_joints = controller_params.at("joints").as_string_array();
              if (controller_joints.size() != 2)
              {
                RCLCPP_ERROR_STREAM(LOGGER, "Parallel Gripper requires exactly two joints");
                continue;
              }
              static_cast<GripperControllerHandle*>(new_handle.get())
                  ->setParallelJawGripper(controller_joints[0], controller_joints[1]);
            }
            else
            {
              if (isSubParameter(controller_params, { "command_joint" }))
                static_cast<GripperControllerHandle*>(new_handle.get())
                    ->setCommandJoint(controller_params.at("command_joint").as_string());
              else
                static_cast<GripperControllerHandle*>(new_handle.get())
                    ->setCommandJoint(controller_params.at("joints").as_string_array()[0]);
            }

            if (isSubParameter(controller_params, { "allow_failure" }))
              static_cast<GripperControllerHandle*>(new_handle.get())->allowFailure(true);

            RCLCPP_INFO_STREAM(LOGGER, "Added GripperCommand controller for " << controller_name);
            controllers_[controller_name] = new_handle;
          }
        }
        else if (type == "FollowJointTrajectory")
        {
          auto h = new FollowJointTrajectoryControllerHandle(node_, controller_name, action_ns);
          new_handle.reset(h);
          if (h->isConnected())
          {
            RCLCPP_INFO_STREAM(LOGGER, "Added FollowJointTrajectory controller for " << controller_name);
            controllers_[controller_name] = new_handle;
          }
        }
        else
        {
          RCLCPP_ERROR_STREAM(LOGGER, "Unknown controller type: " << type);
          continue;
        }
        if (!controllers_[controller_name])
        {
          controllers_.erase(controller_name);
          continue;
        }

        /* add list of joints, used by controller manager and MoveIt */
        const std::vector<std::string> controller_joints = controller_params.at("joints").as_string_array();
        for (const std::string& controller_joint : controller_joints)
          new_handle->addJoint(controller_joint);
      }
      catch (...)
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Caught unknown exception while parsing controller information");
      }
    }
  }
  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      return static_cast<moveit_controller_manager::MoveItControllerHandlePtr>(it->second);
    else
      RCLCPP_FATAL_STREAM(LOGGER, "No such controller: " << name);
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /*
   * Get the list of controller names.
   */
  void getControllersList(std::vector<std::string>& names) override
  {
    for (std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.begin();
         it != controllers_.end(); ++it)
      names.push_back(it->first);
    RCLCPP_INFO_STREAM(LOGGER, "Returned " << names.size() << " controllers in list");
  }

  /*
   * This plugin assumes that all controllers are already active -- and if they are not, well, it has no way to deal
   * with it anyways!
   */
  void getActiveControllers(std::vector<std::string>& names) override
  {
    getControllersList(names);
  }

  /*
   * Controller must be loaded to be active, see comment above about active controllers...
   */
  virtual void getLoadedControllers(std::vector<std::string>& names)
  {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control.
   */
  void getControllerJoints(const std::string& name, std::vector<std::string>& joints) override
  {
    std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
    {
      it->second->getJoints(joints);
    }
    else
    {
      RCLCPP_WARN(LOGGER, "The joints for controller '%s' are not known. Perhaps the controller configuration is "
                          "not loaded on the param server?",
                  name.c_str());
      joints.clear();
    }
  }

  /*
   * Controllers are all active and default -- that's what makes this thing simple.
   */
  moveit_controller_manager::MoveItControllerManager::ControllerState
  getControllerState(const std::string& /* name */) override
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  /* Cannot switch our controllers */
  bool switchControllers(const std::vector<std::string>& /* activate */,
                         const std::vector<std::string>& /* deactivate */) override
  {
    return false;
  }

protected:
  rclcpp::Node::SharedPtr node_;
  std::map<std::string, ActionBasedControllerHandleBasePtr> controllers_;
};

}  // end namespace moveit_simple_controller_manager

PLUGINLIB_EXPORT_CLASS(moveit_simple_controller_manager::MoveItSimpleControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
