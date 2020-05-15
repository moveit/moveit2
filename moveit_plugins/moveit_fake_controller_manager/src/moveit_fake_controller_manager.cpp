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
#include <iterator>

namespace moveit_fake_controller_manager
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.plugins.moveit_fake_controller_manager");
static const std::string DEFAULT_TYPE = "interpolate";
static const std::string ROBOT_DESCRIPTION = "robot_description";

class MoveItFakeControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  MoveItFakeControllerManager()
    :MoveItControllerManager()
  {
  } 
  
  void initialize(const rclcpp::Node::SharedPtr& node)
  {
    // TODO(henningkayser): use flexible base
    const std::string PARAM_BASE_NAME = "moveit_fake_controller_manager";
    node_ = node;
    if (!node_->has_parameter(PARAM_BASE_NAME + ".controller_names"))
    {
      RCLCPP_ERROR(LOGGER, "No controller_names specified.");
      return;
    }

    rclcpp::Parameter controller_names_param;
    node_->get_parameter(PARAM_BASE_NAME + ".controller_names", controller_names_param);
    if (controller_names_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
      RCLCPP_ERROR(LOGGER, "Parameter controller_names should be specified as a string array");
      return;
    }
    
    /* by setting latch to true we preserve the initial joint state while other nodes launch */
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("fake_controller_joint_states", 100);

    /* publish initial pose */
    // TODO (ddengster) : not sure if we still need loadInitialJointValues since joint_states is being published?
    // XmlRpc::XmlRpcValue initial;
    // if (node_handle_.getParam("initial", initial))
    // {
    //   sensor_msgs::msg::JointState js = loadInitialJointValues(initial);
    //   js.header.stamp = ros::Time::now();
    //   pub_.publish(js);
    // }

    std::vector<std::string> controller_names = controller_names_param.as_string_array();
    /* actually create each controller */
    for (const std::string& controller_name : controller_names)
    {
      try
      {
        std::vector<std::string> controller_joints;
        if (!node_->get_parameter(PARAM_BASE_NAME + "." + controller_name + ".joints", controller_joints))
        {
          RCLCPP_ERROR_STREAM(LOGGER, "No joints specified for controller " << controller_name);
          continue;
        }

        std::string type = DEFAULT_TYPE;
        if (node_->has_parameter(PARAM_BASE_NAME + "." + controller_name + ".type"))
          node_->get_parameter(PARAM_BASE_NAME + "." + controller_name + ".type", type);
        
        if (type == "last point")
          controllers_[controller_name].reset(new LastPointController(controller_name, controller_joints, pub_));
        else if (type == "via points")
          controllers_[controller_name].reset(new ViaPointController(controller_name, controller_joints, pub_));
        else if (type == "interpolate")
        {
          //@note: would put this in InterpolatingController's constructor 
          //but they disabled WallRate::operator= for some reason and that complicates things
          double rate = 10.0;
          if (node_->has_parameter("fake_interpolating_controller_rate"))
            node_->get_parameter("fake_interpolating_controller_rate", rate);
          controllers_[controller_name].reset(new InterpolatingController(controller_name, controller_joints, pub_, rate));
        }
        else
          RCLCPP_ERROR_STREAM(LOGGER, "Unknown fake controller type: " << type);
        moveit_controller_manager::MoveItControllerManager::ControllerState state;
        state.default_ = controller_list[i].hasMember("default") ? (bool)controller_list[i]["default"] : false;
        state.active_ = true;

        controller_states_[name] = state;
      }
      catch (...)
      {
        RCLCPP_ERROR(LOGGER, "Caught unknown exception while parsing controller information");
      }
    }
  }

  // TODO (ddengster) : not sure if we still need loadInitialJointValues since joint_states is being published?
  // sensor_msgs::msg::JointState loadInitialJointValues(XmlRpc::XmlRpcValue& param) const
  // {
  //   sensor_msgs::msg::JointState js;

  //   if (param.getType() != XmlRpc::XmlRpcValue::TypeArray || param.size() == 0)
  //   {
  //     RCLCPP_ERROR_ONCE(LOGGER, "Parameter 'initial' should be an array of (group, pose) "
  //                                 "structs.");
  //     return js;
  //   }

  //   robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
  //   const robot_model::RobotModelPtr& robot_model = robot_model_loader.getModel();
  //   moveit::core::RobotState robot_state(robot_model);
  //   typedef std::map<std::string, double> JointPoseMap;
  //   JointPoseMap joints;

  //   robot_state.setToDefaultValues();  // initialize all joint values (just in case...)
  //   for (int i = 0, end = param.size(); i != end; ++i)
  //   {
  //     try
  //     {
  //       std::string group_name = std::string(param[i]["group"]);
  //       std::string pose_name = std::string(param[i]["pose"]);
  //       if (!robot_model->hasJointModelGroup(group_name))
  //       {
  //         RCLCPP_ERROR_STREAM(LOGGER, "Unknown joint model group: " << group_name);
  //         continue;
  //       }
  //       moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
  //       const std::vector<std::string>& joint_names = jmg->getActiveJointModelNames();

  //       if (!robot_state.setToDefaultValues(jmg, pose_name))
  //       {
  //         RCLCPP_WARN(LOGGER, "Unknown pose '%s' for group '%s'.", pose_name.c_str(),
  //                        group_name.c_str());
  //         continue;
  //       }
  //       RCLCPP_WARN(LOGGER, "Set joints of group '%s' to pose '%s'.", group_name.c_str(),
  //                      pose_name.c_str());

  //       for (const std::string& joint_name : joint_names)
  //       {
  //         const moveit::core::JointModel* jm = robot_state.getJointModel(joint_name);
  //         if (!jm)
  //         {
  //           RCLCPP_WARN_STREAM(LOGGER, "Unknown joint: " << joint_name);
  //           continue;
  //         }
  //         if (jm->getVariableCount() != 1)
  //         {
  //           RCLCPP_WARN_STREAM(LOGGER, "Cannot handle multi-variable joint: " << joint_name);
  //           continue;
  //         }

  //         joints[joint_name] = robot_state.getJointPositions(jm)[0];
  //       }
  //     }
  //     catch (...)
  //     {
  //       RCLCPP_ERROR_ONCE(LOGGER, "Caught unknown exception while reading initial pose "
  //                                 "information.");
  //     }
  //   }

  //   // fill the joint state
  //   for (JointPoseMap::const_iterator it = joints.begin(), end = joints.end(); it != end; ++it)
  //   {
  //     js.name.push_back(it->first);
  //     js.position.push_back(it->second);
  //   }
  //   return js;
  // }

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
      RCLCPP_WARN(LOGGER, "The joints for controller '%s' are not known. Perhaps the controller configuration is not loaded on "
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
