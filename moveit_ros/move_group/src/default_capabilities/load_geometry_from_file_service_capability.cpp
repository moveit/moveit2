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

/* Author: Bilal Gill */

#include "load_geometry_from_file_service_capability.h"

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/utils/logger.hpp>

#include <fstream>

namespace move_group
{

namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("load_geometry_from_file_service");
}
}  // namespace

LoadGeometryFromFileService::LoadGeometryFromFileService() : MoveGroupCapability(LOAD_GEOMETRY_FROM_FILE_SERVICE_NAME)
{
}

void LoadGeometryFromFileService::initialize()
{
  load_geometry_service_ = context_->moveit_cpp_->getNode()->create_service<moveit_msgs::srv::LoadGeometryFromFile>(
      LOAD_GEOMETRY_FROM_FILE_SERVICE_NAME,
      [this](const std::shared_ptr<moveit_msgs::srv::LoadGeometryFromFile::Request>& req,
             const std::shared_ptr<moveit_msgs::srv::LoadGeometryFromFile::Response>& res) {
        std::ifstream file(req->file_path_and_name);
        if (!file.is_open())
        {
          RCLCPP_ERROR(getLogger(), "Unable to open file %s for loading CollisionObjects",
                       req->file_path_and_name.c_str());
          res->success = false;
          return;
        }
        planning_scene_monitor::LockedPlanningSceneRW locked_ps(context_->planning_scene_monitor_);
        locked_ps->loadGeometryFromStream(file);
        file.close();
        res->success = true;
      }  // End of callback function
  );
}
}  // namespace move_group

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(move_group::LoadGeometryFromFileService, move_group::MoveGroupCapability)
