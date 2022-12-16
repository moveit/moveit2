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

// Qt
#include <QApplication>
#include <QFileDialog>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QString>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

// SA
#include <moveit_setup_core_plugins/start_screen_widget.hpp>
// C
#include <fstream>  // for reading in urdf
#include <streambuf>

namespace moveit_setup
{
namespace core
{
// ******************************************************************************************
// Start screen user interface for MoveIt Configuration Assistant
// ******************************************************************************************
void StartScreenWidget::onInit()
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Horizontal layout splitter
  QHBoxLayout* hlayout = new QHBoxLayout();
  // Left side of screen
  QVBoxLayout* left_layout = new QVBoxLayout();
  // Right side of screen
  QVBoxLayout* right_layout = new QVBoxLayout();

  // Right Image Area ----------------------------------------------
  right_image_ = new QImage();
  right_image_label_ = new QLabel(this);
  auto image_path = getSharePath("moveit_setup_assistant") / "resources/MoveIt_Setup_Assistant2.png";

  if (right_image_->load(image_path.c_str()))
  {
    right_image_label_->setPixmap(QPixmap::fromImage(*right_image_));
    right_image_label_->setMinimumHeight(384);  // size of right_image_label_
  }
  else
  {
    RCLCPP_ERROR_STREAM(setup_step_.getLogger(), "FAILED TO LOAD " << image_path);
  }

  right_layout->addWidget(right_image_label_);
  right_layout->setAlignment(right_image_label_, Qt::AlignRight | Qt::AlignTop);

  // Top Label Area ---------------------------------------------------
  HeaderWidget* header = new HeaderWidget(
      "MoveIt Setup Assistant",
      "These tools will assist you in creating a Semantic Robot Description Format (SRDF) file, various yaml "
      "configuration and many roslaunch files for utilizing all aspects of MoveIt functionality.",
      this);
  layout->addWidget(header);

  // Select Mode Area -------------------------------------------------
  select_mode_ = new SelectModeWidget(this);
  connect(select_mode_->btn_new_, SIGNAL(clicked()), this, SLOT(showNewOptions()));
  connect(select_mode_->btn_exist_, SIGNAL(clicked()), this, SLOT(showExistingOptions()));
  left_layout->addWidget(select_mode_);

  // Path Box Area ----------------------------------------------------

  // Stack Path Dialog
  stack_path_ = new LoadPathArgsWidget("Load MoveIt Configuration Package",
                                       "Specify the package name or path of an existing MoveIt configuration package "
                                       "to be edited for your robot. Example package name: <i>panda_moveit_config</i>",
                                       "optional xacro arguments:", this, true);  // directory
  // user needs to select option before this is shown
  stack_path_->hide();
  stack_path_->setArgs("");
  connect(stack_path_, SIGNAL(pathChanged(QString)), this, SLOT(onPackagePathChanged(QString)));
  left_layout->addWidget(stack_path_);

  // URDF File Dialog
  urdf_file_ = new LoadPathArgsWidget(
      "Load a URDF or COLLADA Robot Model",
      "Specify the location of an existing Universal Robot Description Format or COLLADA file for your robot",
      "optional xacro arguments:", this, false, true);  // no directory, load only
  // user needs to select option before this is shown
  urdf_file_->hide();
  urdf_file_->setArgs("");
  connect(urdf_file_, SIGNAL(pathChanged(QString)), this, SLOT(onUrdfPathChanged(QString)));
  left_layout->addWidget(urdf_file_);

  // Load settings box ---------------------------------------------
  QHBoxLayout* load_files_layout = new QHBoxLayout();

  progress_bar_ = new QProgressBar(this);
  progress_bar_->setMaximum(100);
  progress_bar_->setMinimum(0);
  progress_bar_->hide();
  load_files_layout->addWidget(progress_bar_);

  btn_load_ = new QPushButton("&Load Files", this);
  btn_load_->setMinimumWidth(180);
  btn_load_->setMinimumHeight(40);
  btn_load_->hide();
  load_files_layout->addWidget(btn_load_);
  load_files_layout->setAlignment(btn_load_, Qt::AlignRight);
  connect(btn_load_, SIGNAL(clicked()), this, SLOT(loadFilesClick()));

  // Next step instructions
  next_label_ = new QLabel(this);
  QFont next_label_font(QFont().defaultFamily(), 11, QFont::Bold);
  next_label_->setFont(next_label_font);
  next_label_->setText("Success! Use the left navigation pane to continue.");
  next_label_->hide();  // only show once the files have been loaded.

  // Final Layout Setup ---------------------------------------------
  // Alignment
  layout->setAlignment(Qt::AlignTop);
  hlayout->setAlignment(Qt::AlignTop);
  left_layout->setAlignment(Qt::AlignTop);
  right_layout->setAlignment(Qt::AlignTop);

  // Stretch
  left_layout->setSpacing(10);

  // Attach Layouts
  hlayout->addLayout(left_layout);
  hlayout->addLayout(right_layout);
  layout->addLayout(hlayout);

  // Vertical Spacer
  layout->addItem(new QSpacerItem(20, 20, QSizePolicy::Minimum, QSizePolicy::Expanding));

  // Attach bottom layout
  layout->addWidget(next_label_);
  layout->setAlignment(next_label_, Qt::AlignRight);
  layout->addLayout(load_files_layout);

  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  setLayout(layout);

  // Debug mode: auto load the configuration file by clicking button after a timeout
  if (debug_)
  {
    // select_mode_->btn_exist_->click();

    QTimer* update_timer = new QTimer(this);
    update_timer->setSingleShot(true);  // only run once
    connect(update_timer, SIGNAL(timeout()), btn_load_, SLOT(click()));
    update_timer->start(100);
  }
}

// ******************************************************************************************
// Destructor
// ******************************************************************************************
StartScreenWidget::~StartScreenWidget()
{
  delete right_image_;  // does not have a parent passed to it
}

void StartScreenWidget::focusGiven()
{
  std::filesystem::path pkg_path = setup_step_.getPackagePath();
  if (!pkg_path.empty())
  {
    stack_path_->setPath(pkg_path);
    select_mode_->btn_exist_->click();
    return;
  }

  std::filesystem::path urdf_path = setup_step_.getURDFPath();
  if (!urdf_path.empty())
  {
    urdf_file_->setPath(urdf_path);
    select_mode_->btn_new_->click();
  }
}

// ******************************************************************************************
// Show options for creating a new configuration package
// ******************************************************************************************
void StartScreenWidget::showNewOptions()
{
  // Do GUI stuff
  select_mode_->btn_exist_->setChecked(false);
  select_mode_->btn_new_->setChecked(true);
  select_mode_->widget_instructions_->hide();
  urdf_file_->show();
  stack_path_->hide();
  btn_load_->show();

  // Remember choice
  create_new_package_ = true;
}

// ******************************************************************************************
// Show options for editing an existing configuration package
// ******************************************************************************************
void StartScreenWidget::showExistingOptions()
{
  // Do GUI stuff
  select_mode_->btn_exist_->setChecked(true);
  select_mode_->btn_new_->setChecked(false);
  select_mode_->widget_instructions_->hide();
  urdf_file_->hide();
  stack_path_->show();
  btn_load_->show();

  // Remember choice
  create_new_package_ = false;
}

// ******************************************************************************************
// Load files to parameter server - CLICK
// ******************************************************************************************
void StartScreenWidget::loadFilesClick()
{
  // Disable start screen GUI components from being changed
  urdf_file_->setDisabled(true);
  // srdf_file_->setDisabled(true);
  stack_path_->setDisabled(true);
  select_mode_->setDisabled(true);
  btn_load_->setDisabled(true);
  progress_bar_->show();

  bool result;

  // Decide if this is a new config package, or loading an old one
  if (create_new_package_)
  {
    result = loadNewFiles();
  }
  else
  {
    result = loadExistingFiles();
  }

  // Check if there was a failure loading files
  if (!result)
  {
    // Renable components
    urdf_file_->setDisabled(false);
    // srdf_file_->setDisabled(false);
    stack_path_->setDisabled(false);
    select_mode_->setDisabled(false);
    btn_load_->setDisabled(false);
    progress_bar_->hide();
  }
  else
  {
    // Hide the logo image so that other screens can resize the rviz thing properly
    right_image_label_->hide();
  }
}

void StartScreenWidget::onPackagePathChanged(const QString& /*path*/)
{
  if (!loadPackageSettings(false))
    return;
  // set xacro args from loaded settings
  stack_path_->setArgs(QString::fromStdString(setup_step_.getXacroArgs()));
}

void StartScreenWidget::onUrdfPathChanged(const QString& path)
{
  setup_step_.loadURDFFile(path.toStdString(), urdf_file_->getArgs().toStdString());
  urdf_file_->setArgsEnabled(setup_step_.isXacroFile());
}

bool StartScreenWidget::loadPackageSettings(bool show_warnings)
{
  // Get the package path
  std::filesystem::path package_path_input = stack_path_->getPath();

  try
  {
    setup_step_.loadExisting(package_path_input);
    return true;
  }
  catch (const std::runtime_error& e)
  {
    if (show_warnings)
      QMessageBox::warning(this, "Error Loading Files", e.what());
    return false;
  }
}

// ******************************************************************************************
// Load existing package files
// ******************************************************************************************
bool StartScreenWidget::loadExistingFiles()
{
  try
  {
    // Progress Indicator
    progress_bar_->setValue(10);
    QApplication::processEvents();

    if (!loadPackageSettings(true))
      return false;
  }
  catch (const std::runtime_error& e)
  {
    QMessageBox::warning(this, "Error Loading SRDF", QString(e.what()));
    RCLCPP_ERROR(setup_step_.getLogger(), "%s", e.what());
    return false;
  }

  // Progress Indicator
  progress_bar_->setValue(100);
  QApplication::processEvents();

  next_label_->show();  // only show once the files have been loaded
  Q_EMIT dataUpdated();

  RCLCPP_INFO(setup_step_.getLogger(), "Loading Setup Assistant Complete");

  Q_EMIT advanceRequest();
  return true;  // success!
}

// ******************************************************************************************
// Load chosen files for creating new package
// ******************************************************************************************
bool StartScreenWidget::loadNewFiles()
{
  // Get URDF file path
  std::filesystem::path urdf_path = urdf_file_->getPath();

  // Check that box is filled out
  if (urdf_path.empty())
  {
    QMessageBox::warning(this, "Error Loading Files", "No robot model file specified");
    return false;
  }

  // Check that this file exits
  if (!std::filesystem::is_regular_file(urdf_path))
  {
    QMessageBox::warning(this, "Error Loading Files",
                         QString("Unable to locate the URDF file: ").append(urdf_path.c_str()));
    return false;
  }

  // Progress Indicator
  progress_bar_->setValue(20);
  QApplication::processEvents();

  // use xacro args from GUI
  std::string xacro_args = urdf_file_->getArgs().toStdString();
  try
  {
    setup_step_.loadURDFFile(urdf_path, xacro_args);
  }
  catch (const std::runtime_error& e)
  {
    QMessageBox::warning(this, "Error Loading URDF", QString(e.what()));
    return false;
  }

  // Progress Indicator
  progress_bar_->setValue(50);
  QApplication::processEvents();

  // DONE LOADING --------------------------------------------------------------------------

  // Call a function that enables navigation
  Q_EMIT dataUpdated();

  // Progress Indicator
  progress_bar_->setValue(70);
  QApplication::processEvents();

  // Progress Indicator
  progress_bar_->setValue(100);
  QApplication::processEvents();

  next_label_->show();  // only show once the files have been loaded

  RCLCPP_INFO(setup_step_.getLogger(), "Loading Setup Assistant Complete");
  return true;  // success!
}

// ******************************************************************************************
// ******************************************************************************************
// Class for selecting which mode
// ******************************************************************************************
// ******************************************************************************************

// ******************************************************************************************
// Create the widget
// ******************************************************************************************
SelectModeWidget::SelectModeWidget(QWidget* parent) : QFrame(parent)
{
  // Set frame graphics
  setFrameShape(QFrame::StyledPanel);
  setFrameShadow(QFrame::Raised);
  setLineWidth(1);
  setMidLineWidth(0);

  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout(this);

  // Horizontal layout splitter
  QHBoxLayout* hlayout = new QHBoxLayout();

  // Widget Title
  QLabel* widget_title = new QLabel(this);
  widget_title->setText("Create new or edit existing?");
  QFont widget_title_font(QFont().defaultFamily(), 12, QFont::Bold);
  widget_title->setFont(widget_title_font);
  layout->addWidget(widget_title);
  layout->setAlignment(widget_title, Qt::AlignTop);

  // Widget Instructions
  widget_instructions_ = new QLabel(this);
  widget_instructions_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  widget_instructions_->setWordWrap(true);
  widget_instructions_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  widget_instructions_->setText(
      "All settings for MoveIt are stored in the MoveIt configuration package. Here you have the option to create a "
      "new configuration package or load an existing one. Note: changes to a MoveIt configuration package outside this "
      "Setup Assistant are likely to be overwritten by this tool.");

  layout->addWidget(widget_instructions_);
  layout->setAlignment(widget_instructions_, Qt::AlignTop);

  // New Button
  btn_new_ = new QPushButton(this);
  btn_new_->setText("Create &New MoveIt\nConfiguration Package");
  hlayout->addWidget(btn_new_);

  // Exist Button
  btn_exist_ = new QPushButton(this);
  btn_exist_->setText("&Edit Existing MoveIt\nConfiguration Package");
  btn_exist_->setCheckable(true);
  hlayout->addWidget(btn_exist_);

  // Add horizontal layer to vertical layer
  layout->addLayout(hlayout);
  setLayout(layout);
  btn_new_->setCheckable(true);
}

}  // namespace core
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::core::StartScreenWidget, moveit_setup::SetupStepWidget)
