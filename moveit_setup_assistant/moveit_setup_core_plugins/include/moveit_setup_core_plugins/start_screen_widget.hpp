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

#include "moveit_setup_framework/qt/setup_step_widget.hpp"
#include "moveit_setup_framework/qt/helper_widgets.hpp"
#include <rclcpp/node.hpp>
#include <QWidget>
#include <QFrame>
class QLabel;
class QProgressBar;
class QPushButton;

#ifndef Q_MOC_RUN
#include <moveit_setup_core_plugins/start_screen.hpp>
#endif

namespace moveit_setup
{
namespace core
{
// Class Prototypes
class SelectModeWidget;

/**
 * \brief Start screen user interface for MoveIt Configuration Assistant
 */
class StartScreenWidget : public SetupStepWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  void onInit() override;

  ~StartScreenWidget() override;

  void focusGiven() override;

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  SelectModeWidget* select_mode_;
  LoadPathArgsWidget* stack_path_;
  LoadPathArgsWidget* urdf_file_;
  QPushButton* btn_load_;
  QLabel* next_label_;
  QProgressBar* progress_bar_;  // TODO: Note that since the refactoring, the progress bar is less useful than before
  QImage* right_image_;
  QLabel* right_image_label_;

  SetupStep& getSetupStep() override
  {
    return setup_step_;
  }

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// User has chosen to show new options
  void showNewOptions();

  /// User has chosen to show edit options
  void showExistingOptions();

  /// Button event for loading user chosen files
  void loadFilesClick();

  /// load package settings
  void onPackagePathChanged(const QString& path);

  /// enable xacro arguments
  void onUrdfPathChanged(const QString& path);

private:
  // ******************************************************************************************
  // Variables
  // ******************************************************************************************
  rclcpp::Node::SharedPtr node_;
  StartScreen setup_step_;

  /// Create new config files, or load existing one?
  bool create_new_package_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /// load package settings from .setup_assistant file
  bool loadPackageSettings(bool show_warnings);

  /// Load chosen files for creating new package
  bool loadNewFiles();

  /// Load existing package files
  bool loadExistingFiles();
};

// ******************************************************************************************
// ******************************************************************************************
// Class for selecting which mode
// ******************************************************************************************
// ******************************************************************************************

class SelectModeWidget : public QFrame
{
  Q_OBJECT

private:
private Q_SLOTS:

public:
  SelectModeWidget(QWidget* parent);

  // Load file button
  QPushButton* btn_new_;
  QPushButton* btn_exist_;
  QLabel* widget_instructions_;
};
}  // namespace core
}  // namespace moveit_setup
