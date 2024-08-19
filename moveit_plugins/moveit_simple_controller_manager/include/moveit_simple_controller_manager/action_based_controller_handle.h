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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/controller_manager/controller_manager.h>
#include <moveit/macros/class_forward.h>
#include <memory>

namespace moveit_simple_controller_manager
{
using namespace std::chrono_literals;

/*
 * This exist solely to inject addJoint/getJoints into base non-templated class.
 */
class ActionBasedControllerHandleBase : public moveit_controller_manager::MoveItControllerHandle
{
public:
  ActionBasedControllerHandleBase(const std::string& name, const std::string& logger_name)
    : moveit_controller_manager::MoveItControllerHandle(name), LOGGER(rclcpp::get_logger(logger_name))
  {
  }

  virtual void addJoint(const std::string& name) = 0;
  virtual void getJoints(std::vector<std::string>& joints) = 0;
  // TODO(JafarAbdi): Revise parameter lookup
  //  virtual void configure(XmlRpc::XmlRpcValue& /* config */)
  //  {
  //  }

protected:
  const rclcpp::Logger LOGGER;
};

MOVEIT_CLASS_FORWARD(
    ActionBasedControllerHandleBase);  // Defines ActionBasedControllerHandleBasePtr, ConstPtr, WeakPtr... etc

/**
 * @brief Base class for controller handles that interact with a controller through a ROS action server.
 */
template <typename T>
class ActionBasedControllerHandle : public ActionBasedControllerHandleBase
{
public:
  ActionBasedControllerHandle(const rclcpp::Node::SharedPtr& node, const std::string& name, const std::string& ns,
                              const std::string& logger_name)
    : ActionBasedControllerHandleBase(name, logger_name), node_(node), done_(true), namespace_(ns)
  {
    // Creating the action client does not ensure that the action server is actually running. Executing trajectories
    // through the controller handle will fail if the server is not running when an action goal message is sent.
    controller_action_client_ = rclcpp_action::create_client<T>(node, getActionName());
    last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  }

  /**
   * @brief Cancels trajectory execution by triggering the controller action server's cancellation callback.
   * @return True if cancellation was accepted, false if cancellation failed.
   */
  bool cancelExecution() override
  {
    if (!controller_action_client_)
      return false;
    if (!done_)
    {
      RCLCPP_INFO_STREAM(LOGGER, "Cancelling execution for " << name_);
      auto cancel_result_future = controller_action_client_->async_cancel_goal(current_goal_);

      const auto& result = cancel_result_future.get();
      if (!result)
        RCLCPP_ERROR(LOGGER, "Failed to cancel goal");

      last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
      done_ = true;
    }
    return true;
  }

  /**
   * @brief Callback function to call when the result is received from the controller action server.
   * @param wrapped_result
   */
  virtual void
  controllerDoneCallback(const typename rclcpp_action::ClientGoalHandle<T>::WrappedResult& wrapped_result) = 0;

  /**
   * @brief Blocks waiting for the action result to be received.
   * @param timeout Duration to wait for a result before failing. Default value indicates no timeout.
   * @return True if a result was received, false on timeout.
   */
  bool waitForExecution(const rclcpp::Duration& timeout = rclcpp::Duration::from_seconds(-1.0)) override
  {
    auto result_callback_done = std::make_shared<std::promise<bool>>();
    auto result_future = controller_action_client_->async_get_result(
        current_goal_, [this, result_callback_done](const auto& wrapped_result) {
          controllerDoneCallback(wrapped_result);
          result_callback_done->set_value(true);
        });
    if (timeout < std::chrono::nanoseconds(0))
    {
      result_future.wait();
    }
    else
    {
      std::future_status status;
      if (node_->get_parameter("use_sim_time").as_bool())
      {
        const auto start = node_->now();
        do
        {
          status = result_future.wait_for(50ms);
          if ((status == std::future_status::timeout) && ((node_->now() - start) > timeout))
          {
            RCLCPP_WARN(LOGGER, "waitForExecution timed out");
            return false;
          }
        } while (status == std::future_status::timeout);
      }
      else
      {
        status = result_future.wait_for(timeout.to_chrono<std::chrono::duration<double>>());
        if (status == std::future_status::timeout)
        {
          RCLCPP_WARN(LOGGER, "waitForExecution timed out");
          return false;
        }
      }
    }
    // To accommodate for the delay after the future for the result is ready and the time controllerDoneCallback takes to finish
    result_callback_done->get_future().wait();
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
  /**
   * @brief A pointer to the node, required to read parameters and get the time.
   */
  const rclcpp::Node::SharedPtr node_;

  /**
   * @brief Check if the controller's action server is ready to receive action goals.
   * @return True if the action server is ready, false if it is not ready or does not exist.
   */
  bool isConnected() const
  {
    return controller_action_client_->action_server_is_ready();
  }

  /**
   * @brief Get the full name of the action using the action namespace and base name.
   * @return The action name.
   */
  std::string getActionName() const
  {
    if (namespace_.empty())
      return name_;
    else
      return name_ + "/" + namespace_;
  }

  /**
   * @brief Indicate that the controller handle is done executing the trajectory and set the controller manager handle's
   * ExecutionStatus based on the received action ResultCode.
   * @param rclcpp_action::ResultCode to convert to moveit_controller_manager::ExecutionStatus.
   */
  void finishControllerExecution(const rclcpp_action::ResultCode& state)
  {
    RCLCPP_DEBUG_STREAM(LOGGER, "Controller " << name_ << " is done with state " << static_cast<int>(state));
    if (state == rclcpp_action::ResultCode::SUCCEEDED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    else if (state == rclcpp_action::ResultCode::ABORTED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::ABORTED;
    else if (state == rclcpp_action::ResultCode::CANCELED)
      last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
    else if (state == rclcpp_action::ResultCode::UNKNOWN)
      last_exec_ = moveit_controller_manager::ExecutionStatus::UNKNOWN;
    else
      last_exec_ = moveit_controller_manager::ExecutionStatus::FAILED;
    done_ = true;
  }

  /**
   * @brief Status after last trajectory execution.
   */
  moveit_controller_manager::ExecutionStatus last_exec_;

  /**
   * @brief Indicates whether the controller handle is done executing its current trajectory.
   */
  bool done_;

  /**
   * @brief The controller namespace. The controller action server's topics will map to name/ns/goal, name/ns/result, etc.
   */
  std::string namespace_;

  /**
   * @brief The joints controlled by this controller.
   */
  std::vector<std::string> joints_;

  /**
   * @brief Action client to send trajectories to the controller's action server.
   */
  typename rclcpp_action::Client<T>::SharedPtr controller_action_client_;

  /**
   * @brief Current goal that has been sent to the action server.
   */
  typename rclcpp_action::ClientGoalHandle<T>::SharedPtr current_goal_;
};

}  // namespace moveit_simple_controller_manager
