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

#include <moveit/planning_scene_monitor/current_state_monitor_middleware_handle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace planning_scene_monitor
{
namespace
{
static const auto SUBSCRIPTION_QOS = rclcpp::QoS(25);
}

CurrentStateMonitorMiddlewareHandle::CurrentStateMonitorMiddlewareHandle(const rclcpp::Node::SharedPtr& node)
  : node_(node)
{
}

rclcpp::Time CurrentStateMonitorMiddlewareHandle::now() const
{
  return node_->now();
}

void CurrentStateMonitorMiddlewareHandle::createJointStateSubscription(const std::string& topic,
                                                                       JointStateUpdateCallback callback)
{
  joint_state_subscription_ =
      node_->create_subscription<sensor_msgs::msg::JointState>(topic, SUBSCRIPTION_QOS, callback);
}

void CurrentStateMonitorMiddlewareHandle::resetJointStateSubscription()
{
  joint_state_subscription_.reset();
}

std::string CurrentStateMonitorMiddlewareHandle::getJointStateTopicName() const
{
  if (joint_state_subscription_)
  {
    return joint_state_subscription_->get_topic_name();
  }
  else
  {
    return "";
  }
}

}  // namespace planning_scene_monitor
