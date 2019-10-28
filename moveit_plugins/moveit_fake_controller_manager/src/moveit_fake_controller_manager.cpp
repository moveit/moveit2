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
#include "rclcpp/rclcpp.hpp"
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <pluginlib/class_list_macros.hpp>
#include <map>
#include <iterator>

namespace moveit_fake_controller_manager
{
static const std::string DEFAULT_TYPE = "interpolate";
static const std::string ROBOT_DESCRIPTION = "robot_description";

struct Controller_list {
  std::string name;
  std::string action_ns;
  std::string type;
  std::string def;
  std::vector <std::string> joints;
} ;

struct Initial {
  std::string group;
  std::string pose;
} ;

class MoveItFakeControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:

  void initialize(std::shared_ptr<rclcpp::Node>& node)
  {
    this->node_ = node;
    auto initial_parameters = std::make_shared<rclcpp::SyncParametersClient>(node_);
    auto list_controller_params = initial_parameters->list_parameters({"controller_list"},10);

    if (list_controller_params.prefixes.size() == 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "No controller_list specified.");
      return;
    }

    std::vector<Controller_list> controller_list;
    for (auto & prefix : list_controller_params.prefixes) {
    Controller_list ct;
    ct.name = prefix;
    for (auto & name : list_controller_params.names) {

      if(name.find(".action_ns") != std::string::npos){
        ct.action_ns = node_->get_parameter(name).as_string();
       }
      if(name.find(".type") != std::string::npos){
        ct.type = node_->get_parameter(name).as_string();
       }
      if(name.find(".default") != std::string::npos){
        ct.def = node_->get_parameter(name).as_bool();
       }
      if(name.find(".joints") != std::string::npos){
        ct.joints = node_->get_parameter(name).as_string_array();
       }
      }
        controller_list.push_back(ct);
    }

    /* by setting latch to true we preserve the initial joint state while other nodes launch */
    bool latch = true;
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("fake_controller_joint_states", 100);
    /* publish initial pose */
    std::vector <Initial> initials;
    std::string group;
    Initial initial;

    if (initial_parameters->has_parameter({"initial.group"}))
    {
      initial.group = node_->get_parameter("initial.group").as_string();
    }
    if (initial_parameters->has_parameter({"initial.group"}))
    {
      initial.pose = node_->get_parameter("initial.pose").as_string();
    }
    initials.push_back(initial);

    //TODO (anasarrak) The vector only will have 1 initial value (fix it)
    if (!initials.empty())
    {
      sensor_msgs::msg::JointState js = loadInitialJointValues(initials);
      js.header.stamp = rclcpp::Clock().now();
      pub_->publish(js);
    }

    /* actually create each controller */
    for (int i = 0; i < controller_list.size(); ++i)
    {
      if (controller_list[i].name.empty() || controller_list[i].joints.empty())
      {
        RCLCPP_ERROR(node_->get_logger(), "Name and joints must be specified for each controller");
        continue;
      }

      try
      {
        const std::string name = std::string(controller_list[i].name);

        std::vector<std::string> joints;
        joints.reserve(controller_list[i].joints.size());
        for (int j = 0; j < controller_list[i].joints.size(); ++j)
          joints.emplace_back(std::string(controller_list[i].joints[j]));

        const std::string& type =
            controller_list[i].type.empty() ? std::string(controller_list[i].type) : DEFAULT_TYPE;
        if (type == "last point")
          controllers_[name].reset(new LastPointController(name, joints, pub_));
        else if (type == "via points")
          controllers_[name].reset(new ViaPointController(name, joints, pub_));
        else if (type == "interpolate")
          controllers_[name].reset(new InterpolatingController(name, joints, pub_,node_));
        else
          RCLCPP_ERROR(node_->get_logger(),"Unknown fake controller type: %s", type.c_str());
      }
      catch (...)
      {
        RCLCPP_ERROR(node_->get_logger(), "Caught unknown exception while parsing controller information");
      }
    }
  }

  MoveItFakeControllerManager()
  {
  }

  sensor_msgs::msg::JointState loadInitialJointValues(std::vector <moveit_fake_controller_manager::Initial>& param) const
  {
    sensor_msgs::msg::JointState js;
    robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
    const robot_model::RobotModelPtr& robot_model = robot_model_loader.getModel();
    typedef std::map<std::string, double> JointPoseMap;
    JointPoseMap joints;

    for (int i = 0, end = param.size(); i != end; ++i)
    {
      try
      {
        std::string group_name = std::string(param[i].group);
        std::string pose_name = std::string(param[i].pose);
        if (!robot_model->hasJointModelGroup(group_name))
        {
          RCLCPP_WARN(node_->get_logger(), "Unknown joint model group: %s", group_name.c_str());
          continue;
        }
        moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
        moveit::core::RobotState robot_state(robot_model);
        const std::vector<std::string>& joint_names = jmg->getActiveJointModelNames();

        if (!robot_state.setToDefaultValues(jmg, pose_name))
        {
          RCLCPP_WARN(node_->get_logger(), "Unknown pose '%s' for group '%s'.", pose_name.c_str(),
                         group_name.c_str());
          continue;
        }
        RCLCPP_INFO(node_->get_logger(), "Set joints of group '%s' to pose '%s'.", group_name.c_str(),
                       pose_name.c_str());

        for (std::vector<std::string>::const_iterator jit = joint_names.begin(), end = joint_names.end(); jit != end;
             ++jit)
        {
          const moveit::core::JointModel* jm = robot_state.getJointModel(*jit);
          if (!jm)
          {
            RCLCPP_WARN(node_->get_logger(), "Unknown joint: %s", (*jit).c_str());
            continue;
          }
          if (jm->getVariableCount() != 1)
          {
            RCLCPP_WARN(node_->get_logger(), "Cannot handle multi-variable joint: %s", (*jit).c_str());
            continue;
          }

          joints[*jit] = robot_state.getJointPositions(jm)[0];
        }
      }
      catch (...)
      {
        RCLCPP_ERROR_ONCE(node_->get_logger(), "Caught unknown exception while reading initial pose "
                                                       "information.");
      }
    }

    // fill the joint state
    for (JointPoseMap::const_iterator it = joints.begin(), end = joints.end(); it != end; ++it)
    {
      js.name.push_back(it->first);
      js.position.push_back(it->second);
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
      RCLCPP_FATAL(node_->get_logger(),"No such controller: %s", name.c_str());
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
    RCLCPP_INFO(node_->get_logger(),"Returned %d controllers in list", names.size());
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
      RCLCPP_WARN(node_->get_logger(),"The joints for controller '%s' are not known. Perhaps the controller configuration is not loaded on "
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
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  /* Cannot switch our controllers */
  bool switchControllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate) override
  {
    return false;
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  std::map<std::string, BaseFakeControllerPtr> controllers_;
};

}  // end namespace moveit_fake_controller_manager

PLUGINLIB_EXPORT_CLASS(moveit_fake_controller_manager::MoveItFakeControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
