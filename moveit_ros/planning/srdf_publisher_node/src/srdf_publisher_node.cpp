/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, PickNik Robotics Inc.
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
 *   * Neither the name of PickNik Robotics Inc. nor the names of its
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

// SrdfPublisher Node
// Author: Tyler Weaver
//
// Node with a single parameter and single publisher using transient local QoS.
// Publishes string set on parameter `robot_description_semantic` to topic
// `/robot_description_semantic`.
//
// This is similar to what [robot_state_publisher](https://github.com/ros/robot_state_publisher)
// does for the URDF using parameter `robot_description` and topic `/robot_description`.
//
// As MoveIt subscribes to the topic `/robot_description_semantic` for the srdf
// this node can be used when you want to set the SRDF parameter on only one node
// and have the rest of your system use a subscriber to that topic to get the
// SRDF.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

namespace moveit_ros_planning
{

class SrdfPublisher : public rclcpp::Node
{
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr srdf_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_handle_;

public:
  SrdfPublisher(const rclcpp::NodeOptions& options) : rclcpp::Node("srdf_publisher", options)
  {
    srdf_publisher_ = this->create_publisher<std_msgs::msg::String>("robot_description_semantic",
                                                                    rclcpp::QoS(1).transient_local().reliable());

    // TODO: Update the callback used here once Humble is EOL
    // Using add_on_set_parameters_callback as it is the only parameter callback available in Humble.
    // This is also why we have to return an always success validation.
    // Once Humble is EOL use add_post_set_parameters_callback.
    on_set_parameters_handle_ =
        this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
          for (auto const& parameter : parameters)
          {
            if (parameter.get_name() == "robot_description_semantic")
            {
              std_msgs::msg::String msg;
              msg.data = parameter.get_value<std::string>();
              srdf_publisher_->publish(msg);
              break;
            }
          }

          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          return result;
        });

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.description = "Semantic Robot Description Format";
    this->declare_parameter("robot_description_semantic", rclcpp::ParameterType::PARAMETER_STRING, descriptor);
  }
};

}  // namespace moveit_ros_planning

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_ros_planning::SrdfPublisher)
