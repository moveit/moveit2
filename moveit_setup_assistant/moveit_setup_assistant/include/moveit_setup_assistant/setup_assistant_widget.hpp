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

// ROS
#include <pluginlib/class_loader.hpp>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>

// Qt
#include <QWidget>
#include <QStackedWidget>
#include <QAbstractTableModel>
class QSplitter;

// Setup Assistant
#include <moveit_setup_framework/qt/setup_step_widget.hpp>
#include <moveit_setup_framework/qt/rviz_panel.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>
#include <moveit_setup_assistant/navigation_widget.hpp>

#ifndef Q_MOC_RUN
// Other
#include <boost/program_options/variables_map.hpp>  // for parsing input arguments
#endif

namespace moveit_setup
{
namespace assistant
{
class SetupAssistantWidget : public QWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /**
   * Construct the setup assistant widget, the primary window for this application
   * @param parent - used by Qt for destructing all elements
   * @return
   */
  SetupAssistantWidget(const rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr& node, QWidget* parent,
                       const boost::program_options::variables_map& args);

  /**
   * Changes viewable screen
   * @param index screen index to switch to
   */

  void moveToScreen(const int index);

  /**
   * Qt close event function for reminding user to save
   * @param event A Qt paramenter
   */
  void closeEvent(QCloseEvent* event) override;

  /**
   * Qt error handling function
   *
   * @param rec
   * @param ev
   * @return bool
   */
  virtual bool notify(QObject* rec, QEvent* ev);

  /**
   * Show/hide the Rviz right panel
   * @param show bool - whether to show
   */
  // void showRviz( bool show );

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

private Q_SLOTS:
  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /**
   * Event for changing screens by user clicking
   * @param index screen id
   */
  void navigationClicked(const QModelIndex& index);

  /**
   * Event for spinning the ros node
   */
  void updateTimer();

  /**
   * Function for handling the dataUpdated event
   */
  void onDataUpdate();

  /**
   * Advance to the next step
   */
  void onAdvanceRequest();

  /**
   * Change the widget modal state based on subwidgets state
   *
   * @param isModal if true disable left navigation
   */
  void onModalModeUpdate(bool isModal);

private:
  // ******************************************************************************************
  // Variables
  // ******************************************************************************************
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node_abstraction_;
  rclcpp::Node::SharedPtr node_;
  QList<QString> nav_name_list_;
  NavigationWidget* navs_view_;

  RVizPanel* rviz_panel_;
  QSplitter* splitter_;
  QStackedWidget* main_content_;
  int current_index_;
  std::mutex change_screen_lock_;

  // Setup Steps
  pluginlib::ClassLoader<SetupStepWidget> widget_loader_;
  std::vector<std::shared_ptr<SetupStepWidget>> steps_;

  /// Contains all the configuration data for the setup assistant
  DataWarehousePtr config_data_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************
};
}  // namespace assistant
}  // namespace moveit_setup
