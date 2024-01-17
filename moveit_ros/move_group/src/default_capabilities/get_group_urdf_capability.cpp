/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

/* Author: Sebastian Jahr */

#include "get_group_urdf_capability.h"

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/utils/logger.hpp>
#include <urdf_parser/urdf_parser.h>

namespace move_group
{

namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("get_urdf_service");
}
const auto JOINT_ELEMENT_CLOSING = std::string("</joint>");
const auto LINK_ELEMENT_CLOSING = std::string("</link>");
const auto ROBOT_ELEMENT_CLOSING = std::string("</robot>");
}  // namespace

GetUrdfService::GetUrdfService() : MoveGroupCapability("get_group_urdf")
{
}

void GetUrdfService::initialize()
{
  get_urdf_service_ = context_->moveit_cpp_->getNode()->create_service<moveit_msgs::srv::GetGroupUrdf>(
      GET_URDF_SERVICE_NAME,
      [this](const std::shared_ptr<moveit_msgs::srv::GetGroupUrdf::Request>& req,
             const std::shared_ptr<moveit_msgs::srv::GetGroupUrdf::Response>& res) {
        res->error_code.source = std::string("GetUrdfService");
        const auto subgroup = context_->moveit_cpp_->getRobotModel()->getJointModelGroup(req->group_name);
        // Check if group exists in loaded robot model
        if (!subgroup)
        {
          const auto error_string = std::string("Cannot create URDF because planning group '") + req->group_name +
                                    std::string("' does not exist");
          RCLCPP_ERROR(getLogger(), "%s", error_string.c_str());
          res->error_code.message = error_string;
          res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
          return;
        }
        // Get robot description string
        std::string full_urdf_string;
        context_->moveit_cpp_->getNode()->get_parameter_or("robot_description", full_urdf_string, std::string(""));

        // Check if string is empty
        if (full_urdf_string.empty())
        {
          const auto error_string =
              std::string("Couldn't load the urdf from parameter server. Is the '/robot_description' parameter "
                          "initialized?");
          RCLCPP_ERROR(getLogger(), "%s", error_string.c_str());
          res->error_code.message = error_string;
          res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
          return;
        }

        // Create subgroup urdf
        // Create header
        res->urdf_string = std::string("<?xml version=\"1.0\" ?>\n<robot name=\"") + req->group_name +
                           std::string("\" xmlns:xacro=\"http://ros.org/wiki/xacro\">");

        // Create links
        const auto& link_names = subgroup->getLinkModelNames();
        for (const auto& link_name : link_names)
        {
          const auto start = full_urdf_string.find("<link name=\"" + link_name);
          auto substring = full_urdf_string.substr(start, full_urdf_string.size() - start);
          res->urdf_string += substring.substr(0, substring.find(LINK_ELEMENT_CLOSING) + LINK_ELEMENT_CLOSING.size());
        }
        // Create joints
        const auto& joint_names = subgroup->getJointModelNames();
        for (const auto& joint_name : joint_names)
        {
          const auto start = full_urdf_string.find("<joint name=\"" + joint_name);
          auto substring = full_urdf_string.substr(start, full_urdf_string.size() - start);
          res->urdf_string += substring.substr(0, substring.find(JOINT_ELEMENT_CLOSING) + JOINT_ELEMENT_CLOSING.size());
        }
        // Create closing
        res->urdf_string += ROBOT_ELEMENT_CLOSING;

        // Validate urdf file
        if (!urdf::parseURDF(full_urdf_string))
        {
          const std::string error_string = std::string("Failed to create valid urdf");
          RCLCPP_ERROR(getLogger(), "%s", error_string.c_str());
          res->error_code.message = error_string;
          res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
          return;
        }
        res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      }  // End of callback function
  );
}
}  // namespace move_group

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(move_group::GetUrdfService, move_group::MoveGroupCapability)
