/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Robotics
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

/* Author: David V. Lu!! */

#pragma once
#include <rclcpp/node.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>

namespace moveit_setup
{
/**
 * @brief Contains all of the non-GUI code necessary for doing one "screen" worth of setup.
 */
class SetupStep
{
public:
  /**
   * @brief Called after construction to initialize the step
   * @param parent_node Shared pointer to the parent node
   */
  void initialize(const rclcpp::Node::SharedPtr& parent_node, const DataWarehousePtr& config_data)
  {
    parent_node_ = parent_node;
    config_data_ = config_data;
    logger_ = std::make_shared<rclcpp::Logger>(parent_node->get_logger().get_child(getName()));
    onInit();
  }

  /**
   * @brief Overridable initialization method
   */
  virtual void onInit()
  {
  }

  /**
   * @brief Returns the name of the setup step
   */
  virtual std::string getName() const = 0;

  /**
   * @brief Return true if the data necessary to proceed with this step has been configured
   */
  virtual bool isReady() const
  {
    return true;
  }

  /**
   * @brief Makes a namespaced logger for this step available to the widget
   */
  const rclcpp::Logger& getLogger() const
  {
    return *logger_;
  }

protected:
  DataWarehousePtr config_data_;
  rclcpp::Node::SharedPtr parent_node_;
  std::shared_ptr<rclcpp::Logger> logger_;
};
}  // namespace moveit_setup
