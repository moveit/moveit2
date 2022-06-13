/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, PickNik Robotics
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
 *   * Neither the name of PickNik Robotics nor the names of its
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

/* Author: Paul Gesel */

#include "moveit_node_interface/moveit_node_interface.hpp"

namespace moveit::node_interface
{
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr NodeInterface::get_node_base_interface()
{
  return node_handle_->get_node_base_interface();
}

rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr NodeInterface::get_node_topics_interface()
{
  return node_handle_->get_node_topics_interface();
}

rclcpp::node_interfaces::NodeParametersInterface::SharedPtr NodeInterface::get_node_parameters_interface()
{
  return node_handle_->get_node_parameters_interface();
}

std::optional<std::shared_ptr<rclcpp::Node>> NodeInterface::get_rcl_node() const
{
  return node_handle_->get_rcl_node();
}

std::optional<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> NodeInterface::get_lifecycle_node() const
{
  return node_handle_->get_lifecycle_node();
}

template <>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
NodeInterface::Wrapper<std::shared_ptr<rclcpp::Node>>::get_node_base_interface()
{
  return node->get_node_base_interface();
}

template <>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
NodeInterface::Wrapper<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>::get_node_base_interface()
{
  return node->get_node_base_interface();
}

template <>
rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
NodeInterface::Wrapper<std::shared_ptr<rclcpp::Node>>::get_node_topics_interface()
{
  return node->get_node_topics_interface();
}

template <>
rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
NodeInterface::Wrapper<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>::get_node_topics_interface()
{
  return node->get_node_topics_interface();
}

template <>
rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
NodeInterface::Wrapper<std::shared_ptr<rclcpp::Node>>::get_node_parameters_interface()
{
  return node->get_node_parameters_interface();
}

template <>
rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
NodeInterface::Wrapper<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>::get_node_parameters_interface()
{
  return node->get_node_parameters_interface();
}

template <>
std::optional<std::shared_ptr<rclcpp::Node>> NodeInterface::Wrapper<std::shared_ptr<rclcpp::Node>>::get_rcl_node() const
{
  return node;
}

template <>
std::optional<std::shared_ptr<rclcpp::Node>>
NodeInterface::Wrapper<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>::get_rcl_node() const
{
  return {};
}

template <>
std::optional<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>
NodeInterface::Wrapper<std::shared_ptr<rclcpp::Node>>::get_lifecycle_node() const
{
  return {};
}

template <>
std::optional<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>
NodeInterface::Wrapper<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>::get_lifecycle_node() const
{
  return node;
}

}  // namespace moveit::node_interface
