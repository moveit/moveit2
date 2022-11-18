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

#include <moveit_setup_framework/data_warehouse.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <moveit/robot_state_rviz_plugin/robot_state_display.h>
#include <QWidget>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace moveit_setup
{
// Used for loading kinematic model
static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string MOVEIT_ROBOT_STATE = "moveit_robot_state";

class RVizPanel : public QWidget, public rviz_common::WindowManagerInterface
{
  Q_OBJECT
public:
  RVizPanel(QWidget* parent, const rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr& node_abstraction,
            const DataWarehousePtr& config_data);
  ~RVizPanel() override;

  bool isReadyForInitialization() const
  {
    return rviz_render_panel_ == nullptr && getRobotModel() != nullptr;
  }
  void initialize();
  void updateFixedFrame();

  QWidget* getParentWindow() override
  {
    return parent_;
  }

  rviz_common::PanelDockWidget* addPane(const QString& /*name*/, QWidget* /*pane*/,
                                        Qt::DockWidgetArea /*area*/ = Qt::LeftDockWidgetArea,
                                        bool /*floating*/ = true) override
  {
    // Stub for now...just to define the WindowManagerInterface methods
    return nullptr;
  }

  void setStatus(const QString& /*message*/) override
  {
    // Stub for now...just to define the WindowManagerInterface methods
  }

public Q_SLOTS:
  /**
   * Highlight a link of the robot
   *
   * @param link_name name of link to highlight
   */
  void highlightLink(const std::string& link_name, const QColor& color)
  {
    Q_EMIT highlightLinkSignal(link_name, color);
  }

  /**
   * Highlight a robot group
   */
  void highlightGroup(const std::string& group_name)
  {
    Q_EMIT highlightGroupSignal(group_name);
  }

  /**
   * Unhighlight all links of a robot
   */
  void unhighlightAll()
  {
    Q_EMIT unhighlightAllSignal();
  }

Q_SIGNALS:
  // Protected event handlers
  void highlightLinkSignal(const std::string& link_name, const QColor& color);
  void highlightGroupSignal(const std::string& group_name);
  void unhighlightAllSignal();

protected Q_SLOTS:
  void highlightLinkEvent(const std::string& link_name, const QColor& color);
  void highlightGroupEvent(const std::string& group_name);
  void unhighlightAllEvent();

protected:
  moveit::core::RobotModelPtr getRobotModel() const;

  QWidget* parent_;
  rviz_common::RenderPanel* rviz_render_panel_{ nullptr };
  rviz_common::VisualizationManager* rviz_manager_{ nullptr };
  moveit_rviz_plugin::RobotStateDisplay* robot_state_display_{ nullptr };
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node_abstraction_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Logger> logger_;

  DataWarehousePtr config_data_;
};
}  // namespace moveit_setup
