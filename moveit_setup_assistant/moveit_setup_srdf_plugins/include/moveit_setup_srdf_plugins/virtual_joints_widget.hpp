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

#include <moveit_setup_framework/qt/setup_step_widget.hpp>
#include <moveit_setup_srdf_plugins/virtual_joints.hpp>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStackedWidget>
#include <QComboBox>
#include <QTableWidget>

namespace moveit_setup
{
namespace srdf_setup
{
class VirtualJointsWidget : public SetupStepWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************
  SetupStep& getSetupStep() override
  {
    return setup_step_;
  }

  void onInit() override;

  /// Received when this widget is chosen from the navigation menu
  void focusGiven() override;

protected:
  VirtualJoints setup_step_;

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  QTableWidget* data_table_;
  QPushButton* btn_edit_;
  QPushButton* btn_delete_;
  QPushButton* btn_save_;
  QPushButton* btn_cancel_;
  QStackedWidget* stacked_widget_;
  QLineEdit* vjoint_name_field_;
  QLineEdit* parent_name_field_;
  QComboBox* child_link_field_;
  QComboBox* joint_type_field_;
  QWidget* vjoint_list_widget_;
  QWidget* vjoint_edit_widget_;

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Show edit screen
  void showNewScreen();

  /// Edit whatever element is selected
  void editSelected();

  /// Edit the double clicked element
  void editDoubleClicked(int row, int column);

  /// Preview whatever element is selected
  void previewClicked(int row, int column);

  /// Delete currently editing ite
  void deleteSelected();

  /// Save editing changes
  void doneEditing();

  /// Cancel changes
  void cancelEditing();

private:
  // ******************************************************************************************
  // Variables
  // ******************************************************************************************
  /// Original name of vjoint currently being edited. This is used to find the element in the vector
  std::string current_edit_vjoint_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /**
   * Create the main list view of vjoints for robot
   *
   * @return the widget
   */
  QWidget* createContentsWidget();

  /**
   * Create the screen for editing vjoints
   *
   * @return the widget
   */
  QWidget* createEditWidget();

  /**
   * Load the robot vjoints into the table
   *
   */
  void loadDataTable();

  /**
   * Populate the combo dropdown box with avail group names
   *
   */
  void loadJointTypesComboBox();

  /**
   * Populate the combo dropdown box with avail parent links
   *
   */
  void loadChildLinksComboBox();

  /**
   * Edit the vjoint with the input name
   *
   * @param name name of vjoint
   */
  void edit(const std::string& name);
};

}  // namespace srdf_setup
}  // namespace moveit_setup
