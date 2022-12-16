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
#include <QAction>
#include <QApplication>
#include <QLabel>
#include <QList>
#include <QListWidget>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QPushButton>
#include <QRegExp>
#include <QSplitter>
#include <QVBoxLayout>

#include <moveit_setup_core_plugins/configuration_files_widget.hpp>

// Boost
#include <boost/algorithm/string.hpp>  // string trim
// Read write files
#include <iostream>  // For writing yaml and launch files
#include <fstream>

namespace moveit_setup
{
namespace core
{
// ******************************************************************************************
// Outer User Interface for MoveIt Configuration Assistant
// ******************************************************************************************
void ConfigurationFilesWidget::onInit()
{
  has_generated_pkg_ = false;

  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();

  // Top Header Area ------------------------------------------------

  auto header =
      new HeaderWidget("Generate Configuration Files",
                       "Create or update the configuration files package needed to run your robot with MoveIt. Uncheck "
                       "files to disable them from being generated - this is useful if you have made custom changes to "
                       "them. Files in orange have been automatically detected as changed.",
                       this);
  layout->addWidget(header);

  // Path Widget ----------------------------------------------------

  // Stack Path Dialog
  stack_path_ = new LoadPathWidget("Configuration Package Save Path",
                                   "Specify the desired directory for the MoveIt configuration package to be "
                                   "generated. Overwriting an existing configuration package directory is acceptable. "
                                   "Example: <i>/u/robot/ros/panda_moveit_config</i>",
                                   this, true);  // is directory
  layout->addWidget(stack_path_);
  connect(stack_path_, SIGNAL(pathChanged(QString)), this, SLOT(onPackagePathChanged(QString)));

  // Generated Files List -------------------------------------------
  QLabel* generated_list = new QLabel("Check files you want to be generated:", this);
  layout->addWidget(generated_list);

  QSplitter* splitter = new QSplitter(Qt::Horizontal, this);
  splitter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  // List Box
  action_list_ = new QListWidget(this);
  action_list_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  action_list_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  connect(action_list_, SIGNAL(currentRowChanged(int)), this, SLOT(changeActionDesc(int)));
  // Allow checking / unchecking of multiple items
  action_list_->setContextMenuPolicy(Qt::ActionsContextMenu);
  QAction* action = new QAction("Check all selected files", this);
  connect(action, &QAction::triggered, [this]() { setCheckSelected(true); });
  action_list_->addAction(action);
  action = new QAction("Uncheck all selected files", this);
  connect(action, &QAction::triggered, [this]() { setCheckSelected(false); });
  action_list_->addAction(action);

  // Description
  action_label_ = new QLabel(this);
  action_label_->setFrameShape(QFrame::StyledPanel);
  action_label_->setFrameShadow(QFrame::Raised);
  action_label_->setLineWidth(1);
  action_label_->setMidLineWidth(0);
  action_label_->setWordWrap(true);
  action_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  action_label_->setMinimumWidth(100);
  action_label_->setAlignment(Qt::AlignTop);
  action_label_->setOpenExternalLinks(true);  // open with web browser

  // Add to splitter
  splitter->addWidget(action_list_);
  splitter->addWidget(action_label_);

  // Add Layout
  layout->addWidget(splitter);

  // Progress bar and generate buttons ---------------------------------------------------
  QHBoxLayout* hlayout1 = new QHBoxLayout();

  // Progress Bar
  progress_bar_ = new QProgressBar(this);
  progress_bar_->setMaximum(100);
  progress_bar_->setMinimum(0);
  hlayout1->addWidget(progress_bar_);
  // hlayout1->setContentsMargins( 20, 30, 20, 30 );

  // Generate Package Button
  btn_save_ = new QPushButton("&Generate Package", this);
  // btn_save_->setMinimumWidth(180);
  btn_save_->setMinimumHeight(40);
  connect(btn_save_, SIGNAL(clicked()), this, SLOT(savePackage()));
  hlayout1->addWidget(btn_save_);

  // Add Layout
  layout->addLayout(hlayout1);

  // Bottom row --------------------------------------------------

  QHBoxLayout* hlayout3 = new QHBoxLayout();

  // Success label
  success_label_ = new QLabel(this);
  QFont success_label_font(QFont().defaultFamily(), 12, QFont::Bold);
  success_label_->setFont(success_label_font);
  success_label_->hide();  // only show once the files have been generated
  success_label_->setText("Configuration package generated successfully!");
  hlayout3->addWidget(success_label_);
  hlayout3->setAlignment(success_label_, Qt::AlignRight);

  // Exit button
  QPushButton* btn_exit = new QPushButton("E&xit Setup Assistant", this);
  btn_exit->setMinimumWidth(180);
  connect(btn_exit, SIGNAL(clicked()), this, SLOT(exitSetupAssistant()));
  hlayout3->addWidget(btn_exit);
  hlayout3->setAlignment(btn_exit, Qt::AlignRight);

  layout->addLayout(hlayout3);

  // Finish Layout --------------------------------------------------
  setLayout(layout);
}

void ConfigurationFilesWidget::setCheckSelected(bool checked)
{
  for (const QModelIndex& row : action_list_->selectionModel()->selectedRows())
    action_list_->model()->setData(row, checked ? Qt::Checked : Qt::Unchecked, Qt::CheckStateRole);
}

void ConfigurationFilesWidget::onPackagePathChanged(const QString& path)
{
  std::filesystem::path package_path = path.toStdString();

  if (package_path == setup_step_.getPackagePath())
  {
    return;
  }
  setup_step_.setPackagePath(package_path);
  setup_step_.setPackageName(package_path.filename().string());

  focusGiven();
}

// ******************************************************************************************
// Verify with user if certain screens have not been completed
// ******************************************************************************************
bool ConfigurationFilesWidget::checkDependencies()
{
  std::vector<std::string> dependencies = setup_step_.getIncompleteWarnings();
  bool required_actions = false;

  // Note that MSA 1.0 required that you have valid author information before proceedings, and if not, would
  // set required_actions to true here. We ignore this for now.

  // Display all accumumlated errors:
  if (!dependencies.empty())
  {
    // Create a dependency message
    QString dep_message;
    if (!required_actions)
    {
      dep_message = "Some setup steps have not been completed. None of the steps are required, but here is a reminder "
                    "of what was not filled in, just in case something was forgotten:<br /><ul>";
    }
    else
    {
      dep_message = "Some setup steps have not been completed. Please fix the required steps (printed in bold), "
                    "otherwise the setup cannot be completed:<br /><ul>";
    }

    for (const auto& dependency : dependencies)
    {
      dep_message.append("<li>").append(QString::fromStdString(dependency)).append("</li>");
    }

    if (!required_actions)
    {
      dep_message.append("</ul><br/>Press Ok to continue generating files.");
      if (QMessageBox::question(this, "Incomplete MoveIt Setup Assistant Steps", dep_message,
                                QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Cancel)
      {
        return false;  // abort
      }
    }
    else
    {
      QMessageBox::warning(this, "Incomplete MoveIt Setup Assistant Steps", dep_message);
      return false;
    }
  }

  return true;
}

// ******************************************************************************************
// A function for showing progress and user feedback about what happened
// ******************************************************************************************
void ConfigurationFilesWidget::updateProgress()
{
  action_num_++;

  // Calc percentage
  progress_bar_->setValue(double(action_num_) / setup_step_.getNumFiles() * 100);

  // allow the progress bar to be shown
  QApplication::processEvents();
}

// ******************************************************************************************
// Display the selected action in the desc box
// ******************************************************************************************
void ConfigurationFilesWidget::changeActionDesc(int id)
{
  // Only allow event if list is not empty
  if (id >= 0)
  {
    // Show the selected text
    action_label_->setText(action_desc_.at(id));
  }
}

// ******************************************************************************************
// Disable or enable item in gen_files_ array
// ******************************************************************************************
void ConfigurationFilesWidget::changeCheckedState(QListWidgetItem* item)
{
  std::size_t index = item->data(Qt::UserRole).toUInt();

  auto gen_file = setup_step_.getGeneratedFiles()[index];

  bool generate = (item->checkState() == Qt::Checked);

  if (!generate && gen_file->hasChanges())
  {
    QMessageBox::warning(this, "Package Generation",
                         "You should generate this file to ensure your changes will take "
                         "effect.");
  }

  // Enable/disable file
  setup_step_.setShouldGenerate(gen_file->getRelativePath(), generate);
}

// ******************************************************************************************
// Called when setup assistant navigation switches to this screen
// ******************************************************************************************
void ConfigurationFilesWidget::focusGiven()
{
  // Pass the package path from start screen to configuration files screen
  stack_path_->setPath(setup_step_.getPackagePath());

  setup_step_.loadFiles();

  // disable reaction to checkbox changes
  disconnect(action_list_, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(changeCheckedState(QListWidgetItem*)));

  // Show files in GUI
  showGenFiles();

  // react to manual changes only (not programmatic ones)
  connect(action_list_, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(changeCheckedState(QListWidgetItem*)));

  // Allow list box to populate
  QApplication::processEvents();

  // Which files have been modified outside the Setup Assistant?
  if (setup_step_.hasModifiedFiles())
  {
    // Some were found to be modified
    QString msg("Some files have been modified outside of the Setup Assistant (according to timestamp). "
                "The Setup Assistant will not overwrite these changes by default because often changing configuration "
                "files manually is necessary, "
                "but we recommend you check the list and enable the checkbox next to files you would like to "
                "overwrite. ");
    if (setup_step_.hasConflictingFiles())
    {
      msg += "<br/><font color='red'>Attention:</font> Some files (<font color='red'>marked red</font>) are changed "
             "both, externally and in Setup Assistant.";
    }
    QMessageBox::information(this, "Files Modified", msg);
  }
}

// ******************************************************************************************
// Show the list of files to be generated
// ******************************************************************************************
void ConfigurationFilesWidget::showGenFiles()
{
  action_list_->clear();

  auto gen_files = setup_step_.getGeneratedFiles();

  // Display this list in the GUI
  for (std::size_t i = 0; i < gen_files.size(); ++i)
  {
    auto gen_file = gen_files[i];

    // Create a formatted row
    QListWidgetItem* item = new QListWidgetItem(QString(gen_file->getRelativePath().c_str()), action_list_, 0);

    // Checkbox
    item->setCheckState(setup_step_.shouldGenerate(gen_file) ? Qt::Checked : Qt::Unchecked);

    auto status = gen_file->getStatus();
    if (status == FileStatus::CONFLICTED)
    {
      item->setForeground(QBrush(QColor(255, 0, 0)));
    }
    else if (status == FileStatus::EXTERNALLY_MODIFIED)
    {
      item->setForeground(QBrush(QColor(255, 135, 0)));
    }

    // Link the gen_files_ index to this item
    item->setData(Qt::UserRole, QVariant(static_cast<qulonglong>(i)));

    // Add actions to list
    action_list_->addItem(item);
    action_desc_.append(QString(gen_file->getDescription().c_str()));
  }

  // Select the first item in the list so that a description is visible
  action_list_->setCurrentRow(0);
}

// ******************************************************************************************
// Save configuration click event
// ******************************************************************************************
void ConfigurationFilesWidget::savePackage()
{
  // Feedback
  success_label_->hide();

  // Reset the progress bar counter and GUI stuff
  action_num_ = 0;
  progress_bar_->setValue(0);

  if (!generatePackage())
  {
    RCLCPP_ERROR_STREAM(setup_step_.getLogger(), "Failed to generate entire configuration package");
    return;
  }

  // Alert user it completed successfully --------------------------------------------------
  progress_bar_->setValue(100);
  success_label_->show();
  has_generated_pkg_ = true;
}

// ******************************************************************************************
// Save package using default path
// ******************************************************************************************
bool ConfigurationFilesWidget::generatePackage()
{
  // Get path name
  std::string package_path_s = stack_path_->getPath();
  // Trim whitespace from user input
  boost::trim(package_path_s);

  // Check that a valid stack package name has been given
  if (package_path_s.empty())
  {
    QMessageBox::warning(this, "Error Generating",
                         "No package path provided. Please choose a directory location to "
                         "generate the MoveIt configuration files.");
    return false;
  }

  // Check setup assist deps
  if (!checkDependencies())
    return false;  // canceled

  // Check that all groups have components
  if (!noGroupsEmpty())
    return false;  // not ready

  // Make sure old package is correct package type and verify over write
  if (setup_step_.isExistingConfig())
  {
    // Check if the old package is a setup assistant package. If it is not, quit
    if (!setup_step_.hasSetupAssistantFile())
    {
      QMessageBox::warning(
          this, "Incorrect Folder/Package",
          QString("The chosen package location already exists but was not previously created using this MoveIt Setup "
                  "Assistant. "
                  "If this is a mistake, add the missing file: ")
              .append(SETUP_ASSISTANT_FILE.c_str()));
      return false;
    }

    // Confirm overwrite
    if (QMessageBox::question(this, "Confirm Package Update",
                              QString("Are you sure you want to overwrite this existing package with updated "
                                      "configurations?<br /><i>")
                                  .append(package_path_s.c_str())
                                  .append("</i>"),
                              QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Cancel)
    {
      return false;  // abort
    }
  }

  setup_step_.setGenerationTime();

  // Begin to create files and folders ----------------------------------------------------------------------
  std::filesystem::path absolute_path;

  for (auto& gen_file : setup_step_.getGeneratedFiles())
  {
    // Check if we should skip this file
    if (!setup_step_.shouldGenerate(gen_file))
    {
      continue;
    }
    absolute_path = gen_file->getPath();

    // Create the absolute path
    RCLCPP_DEBUG_STREAM(setup_step_.getLogger(), "Creating file " << absolute_path.string());

    // Run the generate function
    if (!gen_file->write())
    {
      // Error occurred
      QMessageBox::critical(this, "Error Generating File",
                            QString("Failed to generate folder or file: '")
                                .append(gen_file->getRelativePath().c_str())
                                .append("' at location:\n")
                                .append(absolute_path.c_str()));
      return false;
    }
    updateProgress();  // Increment and update GUI
  }

  return true;
}

// ******************************************************************************************
// Quit the program because we are done
// ******************************************************************************************
void ConfigurationFilesWidget::exitSetupAssistant()
{
  if (has_generated_pkg_ || QMessageBox::question(this, "Exit Setup Assistant",
                                                  QString("Are you sure you want to exit the MoveIt Setup Assistant?"),
                                                  QMessageBox::Ok | QMessageBox::Cancel) == QMessageBox::Ok)
  {
    QApplication::quit();
  }
}

// ******************************************************************************************
// Check that no group is empty (without links/joints/etc)
// ******************************************************************************************
bool ConfigurationFilesWidget::noGroupsEmpty()
{
  // Loop through all groups
  std::string invalid_group = setup_step_.getInvalidGroupName();
  if (!invalid_group.empty())
  {
    // This group has no contents, bad
    QMessageBox::warning(
        this, "Empty Group",
        QString("The planning group '")
            .append(invalid_group.c_str())
            .append("' is empty and has no subcomponents associated with it (joints/links/chains/subgroups). You must "
                    "edit or remove this planning group before this configuration package can be saved."));
    return false;
  }

  return true;  // good
}
}  // namespace core
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::core::ConfigurationFilesWidget, moveit_setup::SetupStepWidget)
