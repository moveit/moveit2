/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
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
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/utils/logger.hpp>
#include <tinyxml2.h>

namespace move_group
{

namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("GetUrdfService");
}
}  // namespace

GetUrdfService::GetUrdfService() : MoveGroupCapability("get_group_urdf")
{
}

void GetUrdfService::initialize()
{
  get_urdf_service_ = context_->moveit_cpp_->getNode()->create_service<moveit_msgs::srv::GetGroupUrdf>(
      GET_URDF_SERVICE_NAME, [this](const std::shared_ptr<moveit_msgs::srv::GetGroupUrdf::Request>& req,
                                    const std::shared_ptr<moveit_msgs::srv::GetGroupUrdf::Response>& res) {
        auto const robot_model = context_->moveit_cpp_->getRobotModel();
        auto const subgroup = robot_model->getJointModelGroup(req->group_name);
        // Check if group exists in loaded robot model
        if (!subgroup)
        {
          RCLCPP_ERROR(getLogger(), "Cannot create URDF because planning group %s does not exist",
                       req->group_name.c_str());
          res->success = false;
          return;
        }
        // Get robot description string
        auto full_urdf_string = std::string("");
        context_->moveit_cpp_->getNode()->get_parameter_or("robot_description", full_urdf_string, std::string(""));

        if (full_urdf_string.empty())
        {
          RCLCPP_ERROR(getLogger(), "Couldn't load the urdf from parameter server. Is the /robot_description parameter "
                                    "initialized?");
          res->success = false;
          return;
        }

        // Parse URDF
        tinyxml2::XMLDocument full_urdf_xml;
        full_urdf_xml.Parse(full_urdf_string.c_str());
        if (full_urdf_xml.Error())
        {
          RCLCPP_ERROR(getLogger(), "Failed to parse urdf. tinyxml returned '%s'", full_urdf_xml.ErrorStr());
          res->success = false;
          return;
        }
        res->urdf_string = full_urdf_string;

        // Create link list
        auto const link_names = subgroup->getLinkModelNames();
        // Create joint list
        auto const joint_names = subgroup->getJointModelNames();
        // Create closing
        auto const closing = "</robot>";
      });
}
}  // namespace move_group

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(move_group::GetUrdfService, move_group::MoveGroupCapability)
