/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
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
 *   * Neither the name of Ioan A. Sucan nor the names of its
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

/* Author: Ioan Sucan, Dave Coleman, Robert Haschke */

#include "moveit_fake_controllers.h"
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <map>

namespace moveit_fake_controller_manager
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.plugins.moveit_fake_controller_manager");
static const std::string DEFAULT_TYPE = "interpolate";
static const std::string ROBOT_DESCRIPTION = "robot_description";

class MoveItFakeControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  MoveItFakeControllerManager() : MoveItControllerManager()
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    const std::string param_base_name = "moveit_fake_controller_manager";
    node_ = node;
    if (!node_->has_parameter(param_base_name + ".controller_names"))
    {
      RCLCPP_ERROR(LOGGER, "No controller_names specified.");
      return;
    }

    rclcpp::Parameter controller_names_param;
    node_->get_parameter(param_base_name + ".controller_names", controller_names_param);
    if (controller_names_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
      RCLCPP_ERROR(LOGGER, "Parameter controller_names should be specified as a string array");
      return;
    }

    /* by setting latch to true we preserve the initial joint state while other nodes launch */
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("fake_controller_joint_states", 100);

    /* publish initial pose */
    rcl_interfaces::msg::ListParametersResult params_result = node->list_parameters(
        { param_base_name + ".initial" }, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
    if (!params_result.prefixes.empty())
    {
      sensor_msgs::msg::JointState js = loadInitialJointValues(params_result.names);
      js.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
      pub_->publish(js);
    }

    std::vector<std::string> controller_names = controller_names_param.as_string_array();
    /* actually create each controller */
    for (const std::string& controller_name : controller_names)
    {
      try
      {
        std::vector<std::string> controller_joints;
        if (!node_->get_parameter(param_base_name + "." + controller_name + ".joints", controller_joints))
        {
          RCLCPP_ERROR_STREAM(LOGGER, "No joints specified for controller " << controller_name);
          continue;
        }

        std::string type = DEFAULT_TYPE;
        if (node_->has_parameter(param_base_name + "." + controller_name + ".type"))
          node_->get_parameter(param_base_name + "." + controller_name + ".type", type);

        if (type == "last point")
          controllers_[controller_name].reset(new LastPointController(controller_name, controller_joints, pub_));
        else if (type == "via points")
          controllers_[controller_name].reset(new ViaPointController(controller_name, controller_joints, pub_));
        else if (type == "interpolate")
        {
          double rate = 10.0;
          std::string fake_interp_rate_param =
              param_base_name + "." + controller_name + ".fake_interpolating_controller_rate";
          if (node_->has_parameter(fake_interp_rate_param))
            node_->get_parameter(fake_interp_rate_param, rate);
          controllers_[controller_name].reset(
              new InterpolatingController(controller_name, controller_joints, pub_, rate));
        }
        else
          RCLCPP_ERROR_STREAM(LOGGER, "Unknown fake controller type: " << type);

        moveit_controller_manager::MoveItControllerManager::ControllerState state;
        state.active_ = true;

        std::string default_state_param = param_base_name + "." + controller_name + "default";
        if (node_->has_parameter(default_state_param))
          node_->get_parameter(default_state_param, state.default_);
        controller_states_[controller_name] = state;
      }
      catch (...)
      {
        RCLCPP_ERROR(LOGGER, "Caught unknown exception while parsing controller information");
      }
    }
  }

  // TODO: codebase wide refactoring for XmlRpc
  sensor_msgs::msg::JointState loadInitialJointValues(const std::vector<std::string>& param_names) const
  {
    sensor_msgs::msg::JointState js;

    robot_model_loader::RobotModelLoader robot_model_loader(node_, ROBOT_DESCRIPTION);
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    moveit::core::RobotState robot_state(robot_model);
    typedef std::map<std::string, double> JointPoseMap;
    JointPoseMap joints;

    robot_state.setToDefaultValues();  // initialize all joint values (just in case...)

    for (const auto& param_name : param_names)
    {
      try
      {
        rclcpp::Parameter param = node_->get_parameter(param_name);
        auto group_name = param_name.substr(param_name.find_last_of('.') + 1);
        auto pose_name = param.value_to_string();

        if (!robot_model->hasJointModelGroup(group_name))
        {
          RCLCPP_ERROR_STREAM(LOGGER, "Unknown joint model group: " << group_name);
          continue;
        }

        moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
        const std::vector<std::string>& joint_names = jmg->getActiveJointModelNames();
        if (!robot_state.setToDefaultValues(jmg, pose_name))
        {
          RCLCPP_WARN(LOGGER, "Unknown pose '%s' for group '%s'.", pose_name.c_str(), group_name.c_str());
          continue;
        }
        RCLCPP_WARN(LOGGER, "Set joints of group '%s' to pose '%s'.", group_name.c_str(), pose_name.c_str());

        for (const std::string& joint_name : joint_names)
        {
          const moveit::core::JointModel* jm = robot_state.getJointModel(joint_name);
          if (!jm)
          {
            RCLCPP_WARN_STREAM(LOGGER, "Unknown joint: " << joint_name);
            continue;
          }
          if (jm->getVariableCount() != 1)
          {
            RCLCPP_WARN_STREAM(LOGGER, "Cannot handle multi-variable joint: " << joint_name);
            continue;
          }

          joints[joint_name] = robot_state.getJointPositions(jm)[0];
        }
      }
      catch (...)
      {
        RCLCPP_ERROR_ONCE(LOGGER, "Caught unknown exception while reading initial pose "
                                  "information.");
      }
    }

    // fill the joint state
    for (const auto& joint : joints)
    {
      js.name.push_back(joint.first);
      js.position.push_back(joint.second);
    }
    return js;
  }

  ~MoveItFakeControllerManager() override = default;

  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    std::map<std::string, BaseFakeControllerPtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      return it->second;
    else
      RCLCPP_FATAL_STREAM(LOGGER, "No such controller: " << name);
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /*
   * Get the list of controller names.
   */
  void getControllersList(std::vector<std::string>& names) override
  {
    for (std::map<std::string, BaseFakeControllerPtr>::const_iterator it = controllers_.begin();
         it != controllers_.end(); ++it)
      names.push_back(it->first);
    RCLCPP_INFO_STREAM(LOGGER, "Returned " << names.size() << " controllers in list");
  }

  /*
   * Fake controllers are always active
   */
  void getActiveControllers(std::vector<std::string>& names) override
  {
    getControllersList(names);
  }

  /*
   * Fake controllers are always loaded
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
    std::map<std::string, BaseFakeControllerPtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
    {
      it->second->getJoints(joints);
    }
    else
    {
      RCLCPP_WARN(LOGGER,
                  "The joints for controller '%s' are not known. Perhaps the controller configuration is not loaded on "
                  "the param server?",
                  name.c_str());
      joints.clear();
    }
  }

  /*
   * Controllers are all active and default.
   */
  moveit_controller_manager::MoveItControllerManager::ControllerState
  getControllerState(const std::string& name) override
  {
    return controller_states_[name];
  }

  /* Cannot switch our controllers */
  bool switchControllers(const std::vector<std::string>& /*activate*/,
                         const std::vector<std::string>& /*deactivate*/) override
  {
    return false;
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  std::map<std::string, BaseFakeControllerPtr> controllers_;
  std::map<std::string, moveit_controller_manager::MoveItControllerManager::ControllerState> controller_states_;
};

}  // end namespace moveit_fake_controller_manager

PLUGINLIB_EXPORT_CLASS(moveit_fake_controller_manager::MoveItFakeControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
