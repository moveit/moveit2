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

#include <moveit_setup_controllers/controllers.hpp>
#include <QWidget>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

namespace moveit_setup
{
namespace controllers
{
class ControllerEditWidget : public QWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /// Constructor
  ControllerEditWidget(QWidget* parent, const FieldPointers& additional_fields);

  /// Set the previous data
  void setSelected(const std::string& controller_name, const ControllerInfo* info);

  /// Populate the combo dropdown box with controllers types
  void loadControllersTypesComboBox(const std::vector<std::string>& controller_types);

  /// Hide delete controller button
  void hideDelete();

  /// Hide save controller button
  void hideSave();

  /// Hide new buttons widget
  void hideNewButtonsWidget();

  /// Show delete controller button
  void showDelete();

  /// Show save controller button
  void showSave();

  /// Show new buttons widget
  void showNewButtonsWidget();

  /// Set widget title
  void setTitle(const QString& title);

  /// Get controller name
  std::string getControllerName();

  /// Get controller type
  std::string getControllerType();

  /// Get the names and values for any additional parameters
  std::map<std::string, std::string> getAdditionalParameters();

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************
  /// Called when the selected item in the controller_type_field_ combobox is changed
  void typeChanged(int index);

Q_SIGNALS:

  // ******************************************************************************************
  // Emitted Signals
  // ******************************************************************************************

  /// Button event for new groups, progressing to adding joints
  void saveJoints();

  /// Button event for new groups, progressing to adding subgroups
  void saveJointsGroups();

  /// Button event for just saving, when in edit mode
  void save();

  /// Event sent when user presses cancel button
  void cancelEditing();

  /// Event sent when delete is being requested for controller
  void deleteController();

private:
  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

  QLabel* title_;  // specify the title from the parent widget
  QLineEdit* controller_name_field_;
  QComboBox* controller_type_field_;
  QPushButton* btn_delete_;      // this button is hidden for new controllers
  QPushButton* btn_save_;        // this button is hidden for new controllers
  QWidget* new_buttons_widget_;  // for showing/hiding the new controllers buttons

  FieldPointers additional_fields_;
  std::vector<QLineEdit*> additional_fields_inputs_;  // one QLineEdit for each additional field

  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  // For loading default types combo box just once
  bool has_loaded_ = false;
};
}  // namespace controllers
}  // namespace moveit_setup
