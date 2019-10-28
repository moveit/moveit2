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

#ifndef MOVEIT_PLUGINS_ACTION_BASED_CONTROLLER_HANDLE
#define MOVEIT_PLUGINS_ACTION_BASED_CONTROLLER_HANDLE

// #include <actionlib/client/simple_action_client.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <moveit/controller_manager/controller_manager.h>
#include <moveit/macros/class_forward.h>
#include <memory>

namespace moveit_simple_controller_manager
{
static rclcpp::Logger LOGGER_ACTION_BASED_CONTROLLER = rclcpp::get_logger("moveit_simple_controller_manager").get_child("action_based_controller");
/*
 * This exist solely to inject addJoint/getJoints into base non-templated class.
 */
class ActionBasedControllerHandleBase : public moveit_controller_manager::MoveItControllerHandle
{
public:
  ActionBasedControllerHandleBase(const std::string& name) : moveit_controller_manager::MoveItControllerHandle(name)
  {
  }

  virtual void addJoint(const std::string& name) = 0;
  virtual void getJoints(std::vector<std::string>& joints) = 0;
  //TODO (anasarrak)
  // virtual void configure(XmlRpc::XmlRpcValue& config)
  // {
  // }
};

MOVEIT_CLASS_FORWARD(ActionBasedControllerHandleBase)

/*
 * This is a simple base class, which handles all of the action creation/etc
 */
template <typename T>
class ActionBasedControllerHandle : public ActionBasedControllerHandleBase
{
public:
  ActionBasedControllerHandle(const std::string& name, std::shared_ptr<rclcpp::Node>& node)
    : ActionBasedControllerHandleBase(name), node_(node), done_(true)
  {
    auto trajectory_execution_params = std::make_shared<rclcpp::SyncParametersClient>(node_);
    controller_action_client_.reset();
    controller_action_client_ = rclcpp_action::create_client<T>(node_, getActionName());

    unsigned int attempts = 0;
    rclcpp::Rate rate(std::chrono::milliseconds(1000));
    double timeout = 0.0;
    if (trajectory_execution_params->has_parameter({"trajectory_execution/controller_connection_timeout"}))
    {
      timeout = trajectory_execution_params->get_parameter("trajectory_execution/controller_connection_timeout", 15.0);
    }

    if (timeout == 0.0)
    {
      while (rclcpp::ok() && !controller_action_client_->wait_for_action_server(std::chrono::seconds(5)))
      {
        RCLCPP_WARN(LOGGER_ACTION_BASED_CONTROLLER, "Waiting for %s to come up", getActionName().c_str());
        rate.sleep();
      }
    }
    else
    {
      while (rclcpp::ok() && !controller_action_client_->wait_for_action_server(std::chrono::seconds( (int) timeout / 3)) && ++attempts < 3)
      {
        RCLCPP_WARN(LOGGER_ACTION_BASED_CONTROLLER, "Waiting for %s to come up",getActionName().c_str());
        rate.sleep();
      }
    }
    if (!controller_action_client_->action_server_is_ready())
    {
      RCLCPP_ERROR(LOGGER_ACTION_BASED_CONTROLLER, "Action client not connected: %s", getActionName().c_str());
      controller_action_client_.reset();
    }

    last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  }

  bool isConnected() const
  {
    return static_cast<bool>(controller_action_client_);
  }

  bool cancelExecution() override
  {
    //TODO (anasarrak)
    // typename goal_msg = T::Goal();
    // auto goal_handle_future = controller_action_client_->async_send_goal(T);
    // typename rclcpp_action::ClientGoalHandle<T>::SharedPtr goal_handle = goal_handle_future.get();
    // if (!controller_action_client_)
    //   return false;
    // if (!done_)
    // {
    //   auto cancel_result_future = controller_action_client_->async_cancel_goal(goal_handle);
    //   if (rclcpp::spin_until_future_complete(node_, cancel_result_future) !=
    //   rclcpp::executor::FutureReturnCode::SUCCESS)
    //   {
    //     RCLCPP_INFO(LOGGER_ACTION_BASED_CONTROLLER, "Cancelling execution for %s", name_.c_str());
    //     last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
    //   }
    //   done_ = true;
    // }
    return true;
  }

  bool waitForExecution(const rclcpp::Duration& timeout = rclcpp::Duration(0,0)) override
  {
    if (controller_action_client_ && !done_)
      // return controller_action_client_->waitForResult(timeout);
      return controller_action_client_->wait_for_action_server(std::chrono::seconds((int) timeout.seconds()));

    return true;
  }

  moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override
  {
    return last_exec_;
  }

  void addJoint(const std::string& name) override
  {
    joints_.push_back(name);
  }

  void getJoints(std::vector<std::string>& joints) override
  {
    joints = joints_;
  }

protected:
  rclcpp::Node::SharedPtr node_;
  std::string getActionName(void) const
  {
    if (namespace_.empty())
      return name_;
    else
      return name_ + "/" + namespace_;
  }

  void finishControllerExecution(const rclcpp_action::ResultCode& state)
  {
    //TODO(anasarrak)
    // RCLCPP_DEBUG(LOGGER_ACTION_BASED_CONTROLLER, "Controller %s is done with state %s:%s ",name_.c_str(), state.toString(), state.getText());
    if (state == rclcpp_action::ResultCode::SUCCEEDED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    else if (state == rclcpp_action::ResultCode::ABORTED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    //TODO(anasarrak) No preempt on ros2 resultCode
    // else if (state == actionlib::SimpleClientGoalState::PREEMPTED)
    //   last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
    else
      last_exec_ = moveit_controller_manager::ExecutionStatus::FAILED;
    done_ = true;
  }

  /* execution status */
  moveit_controller_manager::ExecutionStatus last_exec_;
  bool done_;

  /* the controller namespace, for instance, topics will map to name/ns/goal,
   * name/ns/result, etc */
  std::string namespace_;

  /* the joints controlled by this controller */
  std::vector<std::string> joints_;

  /* action client */
  std::shared_ptr<rclcpp_action::Client<T>> controller_action_client_;
};

}  // end namespace moveit_simple_controller_manager

#endif  // MOVEIT_PLUGINS_ACTION_BASED_CONTROLLER_HANDLE
