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

/* Author: Paul Gesel
   Description: The NodeInterface struct is a wrapper for rclcpp::Node and rclcpp_lifecycle::LifecycleNode types.
*/

#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <memory>

namespace moveit::node_interface
{
struct NodeInterface
{
private:
  struct NodeBase
  {
    virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() = 0;

    virtual rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface() = 0;

    virtual rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface() = 0;

    virtual std::optional<std::shared_ptr<rclcpp::Node>> get_rcl_node() const = 0;

    virtual std::optional<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> get_lifecycle_node() const = 0;
  };

  template <typename T>
  struct Wrapper : public NodeBase
  {
    Wrapper(const T& t) : node(t)
    {
      static_assert(
          std::is_base_of_v<std::shared_ptr<rclcpp::Node>, T> ||
              std::is_base_of_v<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>, T>,
          "NodeInterface can only wrap std::shared_ptr<rclcpp::Node> and rclcpp_lifecycle::LifecycleNode type");
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override;

    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface() override;

    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface() override;

    std::optional<std::shared_ptr<rclcpp::Node>> get_rcl_node() const override;

    std::optional<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> get_lifecycle_node() const override;

    T node;
  };

public:
  NodeInterface(const std::shared_ptr<rclcpp::Node>& node_ptr)
    : node_handle_(std::make_shared<Wrapper<std::shared_ptr<rclcpp::Node>>>(node_ptr))
  {
  }

  NodeInterface(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node_ptr)
    : node_handle_(std::make_shared<Wrapper<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>>>(node_ptr))
  {
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr get_node_topics_interface();

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr get_node_parameters_interface();

  std::optional<std::shared_ptr<rclcpp::Node>> get_rcl_node() const;

  std::shared_ptr<NodeBase> node_handle_;

  std::optional<std::shared_ptr<rclcpp_lifecycle::LifecycleNode>> get_lifecycle_node() const;
};

using NodeInterfaceSharedPtr = std::shared_ptr<NodeInterface>;

}  // namespace moveit::node_interface
