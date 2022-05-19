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

/* Author: Dave Coleman */

#pragma once
#include <rclcpp/node.hpp>
#include <moveit_setup_framework/setup_step.hpp>
#include <moveit_setup_framework/qt/rviz_panel.hpp>
#include <QWidget>

namespace moveit_setup
{
/**
 * @brief The GUI code for one SetupStep
 */
class SetupStepWidget : public QWidget
{
  Q_OBJECT
public:
  /**
   * @brief Called after construction to initialize the step
   * @param parent_node Shared pointer to the parent node
   * @param parent_widget Pointer to the parent gui element
   * @param rviz_panel Pointer to the shared rviz panel
   * @param config_data All the data
   */
  void initialize(const rclcpp::Node::SharedPtr& parent_node, QWidget* parent_widget, RVizPanel* rviz_panel,
                  const DataWarehousePtr& config_data)
  {
    getSetupStep().initialize(parent_node, config_data);
    setParent(parent_widget);
    rviz_panel_ = rviz_panel;
    debug_ = config_data->debug;
    onInit();
  }

  virtual void onInit()
  {
  }

  /**
   * @brief function called when widget is activated, allows to update/initialize GUI
   */
  virtual void focusGiven()
  {
  }

  /**
   * @brief function called when widget loses focus, although switching away can be rejected
   *
   * @return If the widget should not be switched away from, return false
   */
  virtual bool focusLost()
  {
    return true;  // accept switching by default
  }

  /**
   * @brief Return a reference to the SetupStep object
   */
  virtual SetupStep& getSetupStep() = 0;

  bool isReady()
  {
    return getSetupStep().isReady();
  }

  // ******************************************************************************************
  // Emitted Signal Functions
  // ******************************************************************************************

Q_SIGNALS:
  /// When the underlying data has been updated (which can cause other steps to become "Ready")
  void dataUpdated();

  /// When this signal is received, the GUI should attempt to advance to the next step.
  void advanceRequest();

  /// Event for when the current screen is in modal view. Disables the left navigation
  void setModalMode(bool isModal);

protected:
  RVizPanel* rviz_panel_;
  bool debug_;
};
}  // namespace moveit_setup
