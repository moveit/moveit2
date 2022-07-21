/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Larry Lu, Inc.
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
 *   * Neither the name of Larry Lu nor the names of its contributors
 *     may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
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

/* Author: Larry Lu */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <moveit_msgs/msg/robot_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace moveit
{
namespace planning_interface
{

class RobotInterface
{
public:

  RobotInterface();

  RobotInterface(const rclcpp::Node::SharedPtr& node, const std::string& robot_description);

  ~RobotInterface();

  const char* getRobotName() const;

  const std::vector<std::string> getActiveJointNames() const;

  const std::vector<std::string> getGroupActiveJointNames(const std::string&) const;

  const std::vector<std::string> getJointNames() const;

  const std::vector<std::string> getGroupJointNames(const std::string&) const;

  const std::vector<std::string> getGroupJointTips(const std::string&) const;

  const std::vector<std::string> getLinkNames() const;

  const std::vector<std::string> getGroupLinkNames(const std::string&) const;

  const std::vector<std::string> getGroupNames() const;

  const std::vector<std::vector<double> > getJointLimits(const std::string&) const;

  const char* getPlanningFrame() const;

  const std::vector<std::string> getDefaultStateNames(const std::string&);

  const std::map<std::string, double> getJointValues(const std::string&, const std::string&);

  const char* getRobotRootLink() const;

  bool hasGroup(const std::string&) const;

  const std::pair<std::string, std::string> getEndEffectorParentGroup(const std::string&);

  const std::vector<double> getLinkPose(const std::string&);

  std::vector<double> getCurrentJointValues(const std::string&);

  const std::map<std::string, double> getCurrentVariableValues();

  moveit_msgs::msg::RobotState getCurrentState();

  visualization_msgs::msg::MarkerArray getRobotMarkers();

  visualization_msgs::msg::MarkerArray getRobotMarkers(const std::vector<std::string>&);

  visualization_msgs::msg::MarkerArray getRobotMarkers(const std::map<std::string, double>&,
                                                       const std::vector<std::string>&);

  visualization_msgs::msg::MarkerArray getRobotMarkers(const std::map<std::string, double>&);

  visualization_msgs::msg::MarkerArray getRobotMarkers(const moveit_msgs::msg::RobotState);

  visualization_msgs::msg::MarkerArray getRobotMarkersGroup(const std::string&);

  visualization_msgs::msg::MarkerArray getRobotMarkersGroup(const std::string&,
                                                            const std::map<std::string, double>&);

private:
  class RobotInterfaceImpl;
  RobotInterfaceImpl* impl_;
};

}  // namespace planning_interface
}  // namespace moveit
