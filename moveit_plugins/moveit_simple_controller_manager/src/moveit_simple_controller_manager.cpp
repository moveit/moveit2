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

#include <moveit_simple_controller_manager/action_based_controller_handle.h>
#include <moveit_simple_controller_manager/gripper_controller_handle.h>
#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>
#include <boost/algorithm/string/join.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <algorithm>
#include <map>

namespace
{
/**
 * @brief Create a string by adding a provided separator character between each of a variable number of input arguments.
 * @param separator Char to use as the separator.
 * @param content Variable number of input arguments to concatenate.
 * @return Concatenated result string.
 */
template <typename... T>
std::string concatenateWithSeparator(char separator, T... content)
{
  std::string result;
  (result.append(content).append({ separator }), ...);
  result.erase(result.end() - 1);  // delete trailing separator
  return result;
}

/**
 * @brief Compose a parameter name used to load controller configuration from a ROS parameter by concatenating strings
 * separated by the `.` character.
 * @param strings One or more strings, each corresponding to a part of the full parameter name.
 * @return std::string concatenating the input parameters separated by the `.` character. For example:
 * base_namespace.controller_name.param_name
 */
template <typename... T>
std::string makeParameterName(T... strings)
{
  return concatenateWithSeparator<T...>('.', strings...);
}

}  // namespace

namespace moveit_simple_controller_manager
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.plugins.moveit_simple_controller_manager");
static const std::string PARAM_BASE_NAME = "moveit_simple_controller_manager";
class MoveItSimpleControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  MoveItSimpleControllerManager() = default;

  ~MoveItSimpleControllerManager() override = default;

  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    node_ = node;
    if (!node_->has_parameter(makeParameterName(PARAM_BASE_NAME, "controller_names")))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "No controller_names specified.");
      return;
    }
    rclcpp::Parameter controller_names_param;
    node_->get_parameter(makeParameterName(PARAM_BASE_NAME, "controller_names"), controller_names_param);
    if (controller_names_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
      RCLCPP_ERROR(LOGGER, "Parameter controller_names should be specified as a string array");
      return;
    }
    std::vector<std::string> controller_names = controller_names_param.as_string_array();
    /* actually create each controller */
    for (const std::string& controller_name : controller_names)
    {
      try
      {
        std::string action_ns;
        const std::string& action_ns_param = makeParameterName(PARAM_BASE_NAME, controller_name, "action_ns");
        if (!node_->get_parameter(action_ns_param, action_ns))
        {
          RCLCPP_ERROR_STREAM(LOGGER, "No action namespace specified for controller `"
                                          << controller_name << "` through parameter `" << action_ns_param << "`");
          continue;
        }

        std::string type;
        if (!node_->get_parameter(makeParameterName(PARAM_BASE_NAME, controller_name, "type"), type))
        {
          RCLCPP_ERROR_STREAM(LOGGER, "No type specified for controller " << controller_name);
          continue;
        }

        std::vector<std::string> controller_joints;
        if (!node_->get_parameter(makeParameterName(PARAM_BASE_NAME, controller_name, "joints"), controller_joints) ||
            controller_joints.empty())
        {
          RCLCPP_ERROR_STREAM(LOGGER, "No joints specified for controller " << controller_name);
          continue;
        }

        ActionBasedControllerHandleBasePtr new_handle;
        if (type == "GripperCommand")
        {
          double max_effort;
          const std::string& max_effort_param = makeParameterName(PARAM_BASE_NAME, controller_name, "max_effort");
          if (!node->get_parameter(max_effort_param, max_effort))
          {
            RCLCPP_INFO_STREAM(LOGGER, "Max effort set to 0.0");
            max_effort = 0.0;
          }

          new_handle = std::make_shared<GripperControllerHandle>(node_, controller_name, action_ns, max_effort);
          bool parallel_gripper = false;
          if (node_->get_parameter(makeParameterName(PARAM_BASE_NAME, "parallel"), parallel_gripper) && parallel_gripper)
          {
            if (controller_joints.size() != 2)
            {
              RCLCPP_ERROR_STREAM(LOGGER, "Parallel Gripper requires exactly two joints, " << controller_joints.size()
                                                                                           << " are specified");
              continue;
            }
            static_cast<GripperControllerHandle*>(new_handle.get())
                ->setParallelJawGripper(controller_joints[0], controller_joints[1]);
          }
          else
          {
            std::string command_joint;
            if (!node_->get_parameter(makeParameterName(PARAM_BASE_NAME, "command_joint"), command_joint))
              command_joint = controller_joints[0];

            static_cast<GripperControllerHandle*>(new_handle.get())->setCommandJoint(command_joint);
          }

          bool allow_failure;
          node_->get_parameter_or(makeParameterName(PARAM_BASE_NAME, "allow_failure"), allow_failure, false);
          static_cast<GripperControllerHandle*>(new_handle.get())->allowFailure(allow_failure);

          RCLCPP_INFO_STREAM(LOGGER, "Added GripperCommand controller for " << controller_name);
          controllers_[controller_name] = new_handle;
        }
        else if (type == "FollowJointTrajectory")
        {
          new_handle = std::make_shared<FollowJointTrajectoryControllerHandle>(node_, controller_name, action_ns);
          RCLCPP_INFO_STREAM(LOGGER, "Added FollowJointTrajectory controller for " << controller_name);
          controllers_[controller_name] = new_handle;
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

        moveit_controller_manager::MoveItControllerManager::ControllerState state;
        node_->get_parameter_or(makeParameterName(PARAM_BASE_NAME, controller_name, "default"), state.default_, false);
        state.active_ = true;

        controller_states_[controller_name] = state;

        /* add list of joints, used by controller manager and MoveIt */
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
      RCLCPP_WARN(LOGGER,
                  "The joints for controller '%s' are not known. Perhaps the controller configuration is "
                  "not loaded on the param server?",
                  name.c_str());
      joints.clear();
    }
  }

  moveit_controller_manager::MoveItControllerManager::ControllerState
  getControllerState(const std::string& name) override
  {
    return controller_states_[name];
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
  std::map<std::string, moveit_controller_manager::MoveItControllerManager::ControllerState> controller_states_;
};

}  // end namespace moveit_simple_controller_manager

PLUGINLIB_EXPORT_CLASS(moveit_simple_controller_manager::MoveItSimpleControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
