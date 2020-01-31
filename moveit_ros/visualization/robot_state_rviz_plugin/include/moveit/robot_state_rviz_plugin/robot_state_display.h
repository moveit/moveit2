/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#pragma once

#include <rviz_common/display.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>

#ifndef Q_MOC_RUN
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

namespace moveit_rviz_plugin
{
class RobotStateVisualization;

class RobotStateDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  RobotStateDisplay();
  ~RobotStateDisplay() override;

  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  const robot_model::RobotModelConstPtr& getRobotModel() const
  {
    return robot_model_;
  }

  void setLinkColor(const std::string& link_name, const QColor& color);
  void unsetLinkColor(const std::string& link_name);

public Q_SLOTS:
  void setVisible(bool visible);

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************
  void changedRobotDescription();
  void changedRootLinkName();
  void changedRobotSceneAlpha();
  void changedAttachedBodyColor();
  void changedRobotStateTopic();
  void changedEnableLinkHighlight();
  void changedEnableVisualVisible();
  void changedEnableCollisionVisible();
  void changedAllLinks();

protected:
  void loadRobotModel();

  /**
   * \brief Set the scene node's position, given the target frame and the planning frame
   */
  void calculateOffsetPosition();

  void setLinkColor(rviz_default_plugins::robot::Robot* robot, const std::string& link_name, const QColor& color);
  void unsetLinkColor(rviz_default_plugins::robot::Robot* robot, const std::string& link_name);

  void newRobotStateCallback(const moveit_msgs::msg::DisplayRobotState::ConstSharedPtr state);

  void setRobotHighlights(const moveit_msgs::msg::DisplayRobotState::_highlight_links_type& highlight_links);
  void setHighlight(const std::string& link_name, const std_msgs::msg::ColorRGBA& color);
  void unsetHighlight(const std::string& link_name);

  // overrides from Display
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void fixedFrameChanged() override;

  // render the robot
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_subscriber_;

  RobotStateVisualizationPtr robot_;
  rdf_loader::RDFLoaderPtr rdf_loader_;
  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  std::map<std::string, std_msgs::msg::ColorRGBA> highlights_;
  bool update_state_;
  bool load_robot_model_;  // for delayed robot initialization

  rviz_common::properties::StringProperty* robot_description_property_;
  rviz_common::properties::StringProperty* root_link_name_property_;
  rviz_common::properties::RosTopicProperty* robot_state_topic_property_;
  rviz_common::properties::FloatProperty* robot_alpha_property_;
  rviz_common::properties::ColorProperty* attached_body_color_property_;
  rviz_common::properties::BoolProperty* enable_link_highlight_;
  rviz_common::properties::BoolProperty* enable_visual_visible_;
  rviz_common::properties::BoolProperty* enable_collision_visible_;
  rviz_common::properties::BoolProperty* show_all_links_;
};

}  // namespace moveit_rviz_plugin
