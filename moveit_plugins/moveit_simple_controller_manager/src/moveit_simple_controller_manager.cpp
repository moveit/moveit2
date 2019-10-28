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
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <map>

// using namespace moveit::core;

namespace moveit_simple_controller_manager
{
class MoveItSimpleControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:

  void initialize(std::shared_ptr<rclcpp::Node>& node)
  {
    this->node_ = node;

    printf("MoveItSimpleControllerManager initialize\n");
    //
    // auto controller_list_parameters = std::make_shared<rclcpp::SyncParametersClient>(node_);
    // auto list_controller_params = controller_list_parameters->list_parameters({ "controller_list" }, 10);
    // //TODO (anasarrak) New controller_list yaml structure  https://gist.github.com/anasarrak/897a7fc2bc6fbb82777726e76aa6357e
    // if (!controller_list_parameters->has_parameter("controller_list"))
    // {
    //   RCLCPP_ERROR(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER, "No controller_list specified.");
    //   return;
    // }
    //
    // // XmlRpc::XmlRpcValue controller_list;
    // // node_handle_.getParam("controller_list", controller_list);
    // // if (!isArray(controller_list))
    // // {
    // //   RCLCPP_ERROR(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER, "Parameter controller_list should be specified as an
    // //   array");
    // //   return;
    // // }
    //
    // /* actually create each controller */
    // for (int i = 0; i < list_controller_params.prefixes.size(); ++i)
    // {
    //   // if (!isStruct(controller_list[i], { "name", "joints", "action_ns", "type" }))
    //   // {
    //   //   RCLCPP_ERROR(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER,"name, joints, action_ns, and type must be specifed for
    //   //   each controller");
    //   //   continue;
    //   // }
    //
    //   try
    //   {
    //     std::string name_prefix = list_controller_params.prefixes[i];
    //     std::istringstream iss(name_prefix);
    //     std::vector<std::string> indexes;
    //     std::string index;
    //     while (std::getline(iss, index, '.'))
    //     {
    //       if (!index.empty())
    //         indexes.push_back(index);
    //     }
    //
    //     std::string name = indexes[1];
    //     std::vector<std::string> joints;
    //
    //     for (auto& n : list_controller_params.names)
    //     {
    //       std::istringstream iss(n);
    //       indexes.clear();
    //       while (std::getline(iss, index, '.'))
    //       {
    //         if (!index.empty())
    //           indexes.push_back(index);
    //       }
    //
    //       const std::string action_ns_str = name_prefix + ".action_ns";
    //       const std::string type_str = name_prefix + ".type";
    //       const std::string joints_str = name_prefix + ".joints";
    //
    //       printf("action_ns_str : %s", action_ns_str.c_str());
    //       printf("type_str : %s", type_str.c_str());
    //       printf("joints_str : %s", joints_str.c_str());
    //
    //       std::string action_ns;
    //       std::string type;
    //
    //       if (action_ns_str.compare(name) == 0)
    //       {
    //         action_ns = node_->get_parameter(action_ns_str).as_string();
    //       }
    //       if (type_str.compare(name) == 0)
    //       {
    //         type = node_->get_parameter(type_str).as_string();
    //       }
    //
    //       if (joints_str.compare(name) == 0)
    //       {
    //         joints = node_->get_parameter(joints_str).as_string_array();
    //       }
    //
    //       // TODO (anasarrak)
    //       // if (!isArray(controller_list[i]["joints"]))
    //       // {
    //       //   RCLCPP_ERROR(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER,"The list of joints for controller %s is not
    //       //   specified as an array", name.c_str());
    //       //   continue;
    //       // }
    //
    //       ActionBasedControllerHandleBasePtr new_handle;
    //       if (type == "GripperCommand")
    //       {
    //         new_handle.reset(new GripperControllerHandle(name, node_));
    //         if (static_cast<GripperControllerHandle*>(new_handle.get())->isConnected())
    //         {
    //           std::string parallel_str = name_prefix + ".parallel";
    //           if (parallel_str.compare(name) == 0)
    //           {
    //             if (joints.size() != 2)
    //             {
    //               RCLCPP_ERROR(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER, "Parallel Gripper requires exactly two joints");
    //               continue;
    //             }
    //             static_cast<GripperControllerHandle*>(new_handle.get())
    //                 ->setParallelJawGripper(joints[0], joints[1]);
    //           }
    //           else
    //           {
    //             std::string command_joint_str = name_prefix + ".command_joint";
    //             std::string command_joint = node_->get_parameter(command_joint_str).as_string();
    //             if (command_joint_str.compare(name) == 0)
    //               static_cast<GripperControllerHandle*>(new_handle.get())
    //                   ->setCommandJoint(command_joint);
    //             else
    //               static_cast<GripperControllerHandle*>(new_handle.get())
    //                   ->setCommandJoint(joints[0]);
    //           }
    //           std::string allow_failure_str = name_prefix + ".allow_failure";
    //           std::vector <std::string> allow_failure = node_->get_parameter(allow_failure_str).as_string_array();
    //           if (allow_failure_str.compare(name) == 0)
    //             static_cast<GripperControllerHandle*>(new_handle.get())->allowFailure(true);
    //
    //           RCLCPP_INFO(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER, "Added GripperCommand controller for %s",
    //                       name.c_str());
    //           controllers_[name] = new_handle;
    //         }
    //       }
    //       else if (type == "FollowJointTrajectory")
    //       {
    //         auto h = new FollowJointTrajectoryControllerHandle(name, node_);
    //         new_handle.reset(h);
    //         if (h->isConnected())
    //         {
    //           RCLCPP_INFO(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER, "Added FollowJointTrajectory controller for %s",
    //                       name.c_str());
    //           controllers_[name] = new_handle;
    //         }
    //       }
    //       else
    //       {
    //         RCLCPP_ERROR(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER, "Unknown controller type: %s", type.c_str());
    //         continue;
    //       }
    //       if (!controllers_[name])
    //       {
    //         controllers_.erase(name);
    //         continue;
    //       }
    //
    //       /* add list of joints, used by controller manager and MoveIt! */
    //       for (int j = 0; j < joints.size(); ++j)
    //         new_handle->addJoint(std::string(joints[j]));
    //       //TODO (anasarrak)
    //       // new_handle->configure(joints);
    //       joints.clear();
    //     }
    //   }
    //   catch (...)
    //   {
    //     RCLCPP_ERROR(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER,
    //                  "Caught unknown exception while parsing controller information");
    //   }
    // }
    std::string action_ns = std::string("follow_joint_trajectory");
    std::string type = std::string("FollowJointTrajectory");
    std::vector<std::string> joints;
    std::string name = "follow_joint_trajectory";
    for(int i = 0; i < 6; i++){
      joints.push_back(std::string("motor") + std::to_string(i+1));
    }
    ActionBasedControllerHandleBasePtr new_handle;
    RCLCPP_INFO(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER, "FollowJointTrajectoryControllerHandle");
    auto h = new FollowJointTrajectoryControllerHandle(name, node_);
    RCLCPP_INFO(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER, "FollowJointTrajectoryControllerHandle end");
    new_handle.reset(h);
    if (h->isConnected())
    {
      RCLCPP_INFO(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER, "Added FollowJointTrajectory controller for %s",
                  name.c_str());
      controllers_[name] = new_handle;
    }

    for (int j = 0; j < joints.size(); ++j)
      new_handle->addJoint(std::string(joints[j]));
    //TODO (anasarrak)
    // new_handle->configure(joints);

  }

  MoveItSimpleControllerManager()
  {

  }

  ~MoveItSimpleControllerManager() override = default;

  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      return static_cast<moveit_controller_manager::MoveItControllerHandlePtr>(it->second);
    else
      RCLCPP_FATAL(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER, "No such controller: %s", name.c_str());
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
    RCLCPP_INFO(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER, "Returned %d controllers in list", names.size());
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
      RCLCPP_ERROR(LOGGER_MOVEIT_SIMPLE_CONTROLLER_MANAGER, "The joints for controller '%s' are not known. Perhaps the "
                                                            "controller configuration is not loaded on the param "
                                                            "server?",
                   name.c_str());
      joints.clear();
    }
  }

  /*
   * Controllers are all active and default -- that's what makes this thing simple.
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
  std::map<std::string, ActionBasedControllerHandleBasePtr> controllers_;
};

}  // end namespace moveit_simple_controller_manager

PLUGINLIB_EXPORT_CLASS(moveit_simple_controller_manager::MoveItSimpleControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
