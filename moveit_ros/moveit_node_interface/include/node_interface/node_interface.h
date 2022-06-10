/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace node_interface
{
// alias
using BaseNodeInterfaceSharedPtr = std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>;
using TopicsInterfaceSharedPtr = std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>;
using ParametersInterfaceSharedPtr = std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>;

struct NodeInterface
{
  BaseNodeInterfaceSharedPtr base_node_interface_ = nullptr;
  TopicsInterfaceSharedPtr topic_interface_ = nullptr;
  ParametersInterfaceSharedPtr parameter_interface_ = nullptr;

  NodeInterface(const std::shared_ptr<rclcpp::Node>& node)
  {
    base_node_interface_ = node->get_node_base_interface();
    topic_interface_ = node->get_node_topics_interface();
    parameter_interface_ = node->get_node_parameters_interface();
  }
  NodeInterface(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node)
  {
    base_node_interface_ = node->get_node_base_interface();
    topic_interface_ = node->get_node_topics_interface();
    parameter_interface_ = node->get_node_parameters_interface();
  }
  NodeInterface(const BaseNodeInterfaceSharedPtr& base_node_interface, const TopicsInterfaceSharedPtr& topic_interface,
                const ParametersInterfaceSharedPtr& parameter_interface)
  {
    base_node_interface_ = base_node_interface;
    topic_interface_ = topic_interface;
    parameter_interface_ = parameter_interface;
  }
};

typedef std::shared_ptr<NodeInterface> NodeInterfaceSharedPtr;

}  // namespace node_interface