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

#include <moveit_setup_core_plugins/configuration_files.hpp>
#include "moveit_setup_framework/qt/setup_step_widget.hpp"
#include "moveit_setup_framework/qt/helper_widgets.hpp"

#include <QList>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QProgressBar>
#include <QPushButton>

namespace moveit_setup
{
namespace core
{
// Class
class ConfigurationFilesWidget : public SetupStepWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  void onInit() override;

  SetupStep& getSetupStep() override
  {
    return setup_step_;
  }

  /// Received when this widget is chosen from the navigation menu
  void focusGiven() override;

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  QPushButton* btn_save_;
  LoadPathWidget* stack_path_;
  QProgressBar* progress_bar_;
  QListWidget* action_list_;
  QLabel* action_label_;
  QLabel* success_label_;
  QList<QString> action_desc_;  // Holds the descriptions explaining all performed actions

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Save package click event
  void savePackage();

  /// Generate the package
  bool generatePackage();

  /// Quit the program because we are done
  void exitSetupAssistant();

  /// Display the selected action in the desc box
  void changeActionDesc(int id);

  /// Disable or enable item in gen_files_ array
  void changeCheckedState(QListWidgetItem* item);

  /// Set checked state of all selected items
  void setCheckSelected(bool checked);

  // When the configuration path changes
  void onPackagePathChanged(const QString& path);

private:
  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  ConfigurationFiles setup_step_;

  /// Track progress
  unsigned int action_num_;

  /// Has the package been generated yet this program execution? Used for popping up exit warning
  bool has_generated_pkg_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /// Show the list of files to be generated
  void showGenFiles();

  /// Verify with user if certain screens have not been completed
  bool checkDependencies();

  /// A function for showing progress and user feedback about what happened
  void updateProgress();

  /// Check that no group is empty (without links/joints/etc)
  bool noGroupsEmpty();
};

}  // namespace core
}  // namespace moveit_setup
