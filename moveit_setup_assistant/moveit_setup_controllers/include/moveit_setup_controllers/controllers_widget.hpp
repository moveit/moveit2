/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Mohamad Ayman.
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
 *   * The name of Mohamad Ayman may be used to endorse or promote products derived
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

/* Author: Mohamad Ayman */

#pragma once

// Qt
#include <QHBoxLayout>
#include <QPushButton>
#include <QStackedWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>

// SA
#include <moveit_setup_framework/qt/setup_step_widget.hpp>
#include <moveit_setup_framework/qt/double_list_widget.hpp>
#include <moveit_setup_controllers/controllers.hpp>
#include <moveit_setup_controllers/controller_edit_widget.hpp>

namespace moveit_setup
{
namespace controllers
{
class ControllersWidget : public SetupStepWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  void onInit() override;

  void changeScreen(int index);

  /// Received when this widget is chosen from the navigation menu
  void focusGiven() override;

  SetupStep& getSetupStep() override
  {
    return *setup_step_;
  }

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  // Expand/Collapse Tree
  void alterTree(const QString& link);

  /// Create a new controller
  void addController();

  /// Edit an existing controller
  void editController();

  /// Delete an existing controller
  void deleteController();

  // Add a Follow Joint Trajectory action Controller for each Planning Group
  void addDefaultControllers();

  /// Call when screen is done being edited
  void saveControllerScreenJoints();
  void saveJointsScreen();
  bool saveControllerScreen();
  void saveControllerScreenEdit();
  void saveControllerScreenGroups();
  void saveJointsGroupsScreen();
  void cancelEditing();

  /// Called whenever element is selected in the controllers tree view
  void editSelected();

  /// Called from Double List widget to highlight a joint
  void previewSelectedJoints(const std::vector<std::string>& joints);

  /// Called from Double List widget to highlight a group
  void previewSelectedGroup(const std::vector<std::string>& groups);

  /// Called when an item is seleceted from the controllers tree
  void previewSelected(QTreeWidgetItem* selected_item, int column);

  /// Called sleceted item changed
  void itemSelectionChanged();

protected:
  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

  /// Main table for holding controllers
  QTreeWidget* controllers_tree_;
  QWidget* controllers_tree_widget_;

  /// For changing between table and different add/edit views
  QStackedWidget* stacked_widget_;
  ControllerEditWidget* controller_edit_widget_;

  QPushButton* btn_delete_;
  QPushButton* btn_add_;
  QPushButton* btn_edit_;
  QHBoxLayout* controls_layout_;
  DoubleListWidget* joints_widget_;
  DoubleListWidget* joint_groups_widget_;

  /// Remember what controller we are editing when an edit screen is being shown
  std::string current_edit_controller_;

  /// Remember whethere we're editing a controller or adding a new one
  bool adding_new_controller_;

  std::shared_ptr<Controllers> setup_step_;

  /// Builds the main screen list widget
  QWidget* createContentsWidget();

  void loadControllersTree();
  void loadToControllersTree(const ControllerInfo& controller_it);
  void showMainScreen();
  void loadJointsScreen(ControllerInfo* this_controller);
  void loadGroupsScreen(ControllerInfo* this_controller);
  void loadControllerScreen(ControllerInfo* this_controller);
};

}  // namespace controllers
}  // namespace moveit_setup
