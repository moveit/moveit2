/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik, Inc.
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
 *   * Neither the name of PickNik nor the names of its
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

/* Author: Tyler Weaver */

#pragma once

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/planning_scene_monitor/current_state_monitor.h>

namespace planning_scene_monitor
{
/**
 * @brief      This class contains the ros interfaces for CurrentStateMontior
 */
class CurrentStateMonitorMiddlewareHandle : public CurrentStateMonitor::MiddlewareHandle
{
public:
  /**
   * @brief      Constructor
   *
   * @param[in]  node  The ros node
   */
  CurrentStateMonitorMiddlewareHandle(const rclcpp::Node::SharedPtr& node);

  /**
   * @brief      Get the current time
   *
   * @return     Time object representing the time when this is called
   */
  rclcpp::Time now() const override;

  /**
   * @brief      Creates a joint state subscription
   *
   * @param[in]  topic     The topic
   * @param[in]  callback  The callback
   */
  void createJointStateSubscription(const std::string& topic, JointStateUpdateCallback callback) override;

  /**
   * @brief      Creates a static transform message subscription
   *
   * @param[in]  callback  The callback
   */
  void createStaticTfSubscription(std::function<void(const tf2_msgs::msg::TFMessage::ConstSharedPtr)> callback) override;

  /**
   * @brief      Creates a dynamic transform message subscription
   *
   * @param[in]  callback  The callback
   */
  void
  createDynamicTfSubscription(std::function<void(const tf2_msgs::msg::TFMessage::ConstSharedPtr)> callback) override;

  /**
   * @brief      Reset the joint state subscription
   */
  void resetJointStateSubscription() override;

  /**
   * @brief      Get the joint state topic name
   *
   * @return     The joint state topic name.
   */
  std::string getJointStateTopicName() const override;

  /**
   * @brief      Uses rclcpp::sleep_for to sleep
   *
   * @param[in]  nanoseconds  The nanoseconds to sleep for
   *
   * @return     Return of rclcpp::sleep_for
   */
  bool sleepFor(const std::chrono::nanoseconds& nanoseconds) const override;

  /**
   * @brief      Get the static transform topic name
   *
   * @return     The static transform topic name.
   */
  std::string getStaticTfTopicName() const override;

  /**
   * @brief      Get the dynamic transform topic name
   *
   * @return     The dynamic transform topic name.
   */
  std::string getDynamicTfTopicName() const override;

  /**
   * @brief      Reset the static & dynamic transform subscriptions
   */
  void resetTfSubscriptions() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr transform_subscriber_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_transform_subscriber_;
};

}  // namespace planning_scene_monitor
