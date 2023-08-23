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
 *   * The name of Mohamad Ayman may not be used to endorse or promote products derived
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

// SA
#include <moveit_setup_simulation/simulation_widget.hpp>
#include <moveit_setup_framework/qt/helper_widgets.hpp>
#include <moveit_setup_framework/qt/xml_syntax_highlighter.hpp>

// Qt
#include <QColor>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QProcess>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>

#include <regex>

namespace moveit_setup
{
namespace simulation
{
// ******************************************************************************************
// Constructor
// ******************************************************************************************
void SimulationWidget::onInit()
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();
  layout->setAlignment(Qt::AlignTop);

  // Top Header Area ------------------------------------------------

  auto header = new HeaderWidget("Simulate With Gazebo",
                                 "The following tool will auto-generate the URDF changes needed for Gazebo "
                                 "compatibility with ROSControl and MoveIt. The needed changes are shown in green.",
                                 this);
  layout->addWidget(header);
  layout->addSpacerItem(new QSpacerItem(1, 8, QSizePolicy::Fixed, QSizePolicy::Fixed));

  // Top Buttons --------------------------------------------------
  QHBoxLayout* controls_layout = new QHBoxLayout();

  // Used to overwrite the original URDF
  btn_overwrite_ = new QPushButton("Over&write original URDF", this);
  btn_overwrite_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  connect(btn_overwrite_, SIGNAL(clicked()), this, SLOT(overwriteURDF()));
  controls_layout->addWidget(btn_overwrite_);

  btn_open_ = new QPushButton("&Open original URDF", this);
  btn_open_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  btn_open_->setToolTip("Open original URDF file in editor");
  connect(btn_open_, SIGNAL(clicked()), this, SLOT(openURDF()));
  controls_layout->addWidget(btn_open_);

  // Align buttons to the left
  controls_layout->addItem(new QSpacerItem(20, 20, QSizePolicy::Expanding, QSizePolicy::Fixed));

  // Add layout
  layout->addLayout(controls_layout);

  // When there are no changes to be made
  no_changes_label_ = new QLabel(this);
  no_changes_label_->setText("URDF is ready for Gazebo. No changes required.");
  no_changes_label_->setFont(QFont(QFont().defaultFamily(), 18));
  no_changes_label_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  no_changes_label_->setAlignment(Qt::AlignTop);
  layout->addWidget(no_changes_label_);

  // URDF text
  simulation_text_ = new QTextEdit(this);
  simulation_text_->setLineWrapMode(QTextEdit::NoWrap);
  layout->addWidget(simulation_text_);
  // Configure highlighter
  auto highlighter = new XmlSyntaxHighlighter(simulation_text_->document());
  QTextCharFormat format;
  format.setForeground(Qt::darkGreen);
  highlighter->addTag("inertial", format);
  highlighter->addTag("transmission", format);
  highlighter->addTag("gazebo", format);

  // Copy URDF link, hidden initially
  copy_urdf_ = new QLabel(this);
  copy_urdf_->setText("<a href='contract'>Copy to Clipboard</a>");
  connect(copy_urdf_, &QLabel::linkActivated, this, &SimulationWidget::copyURDF);
  layout->addWidget(copy_urdf_);

  // Finish Layout --------------------------------------------------
  setLayout(layout);
}

void SimulationWidget::focusGiven()
{
  if (!simulation_text_->document()->isEmpty())
    return;  // nothing to do

  simulation_text_->setVisible(true);
  std::string text = setup_step_.getGazeboCompatibleURDF();

  simulation_text_->document()->setPlainText(QString::fromStdString(text));

  // Add generated Gazebo URDF to config file if not empty
  bool have_changes = !text.empty();
  config_data_->save_gazebo_urdf_ = have_changes;

  // GUI elements are visible only if there are URDF changes to display/edit
  simulation_text_->setVisible(have_changes);
  btn_overwrite_->setVisible(have_changes);
  btn_open_->setVisible(have_changes && !qgetenv("EDITOR").isEmpty());
  copy_urdf_->setVisible(have_changes);
  no_changes_label_->setVisible(!have_changes);

  // Disable overwrite button if URDF originates from xacro
  btn_overwrite_->setDisabled(config_data_->urdf_from_xacro_);
  QString tooltip;
  if (btn_overwrite_->isEnabled())
    tooltip = tr("Overwrite URDF in original location:\n").append(setup_step_.getURDFPath().c_str());
  else
    tooltip = tr("Cannot overwrite original, <i>xacro-based</i> URDF");
  btn_overwrite_->setToolTip(tooltip);

  if (have_changes)
    config_data_->changes |= MoveItConfigData::SIMULATION;
  else
    config_data_->changes &= ~MoveItConfigData::SIMULATION;
}

bool SimulationWidget::focusLost()
{
  if (!config_data_->save_gazebo_urdf_)
    return true;  // saving is disabled anyway

  // validate XML
  std::string urdf = simulation_text_->document()->toPlainText().toStdString();
  int error_row;
  std::string error_description;

  if (setup_step_.isValidXML(urdf, error_row, error_description))
  {
    config_data_->gazebo_urdf_string_ = std::move(urdf);
    return true;
  }
  else
  {
    QTextCursor cursor = simulation_text_->textCursor();
    cursor.movePosition(QTextCursor::Start);
    cursor.movePosition(QTextCursor::Down, QTextCursor::MoveAnchor, error_row);
    simulation_text_->setTextCursor(cursor);
    QMessageBox::warning(this, tr("Gazebo URDF"), tr("Error parsing XML:\n").append(error_description.c_str()));
    simulation_text_->setFocus(Qt::OtherFocusReason);
    return false;  // reject switching
  }
}

// ******************************************************************************************
// Called when save URDF button is clicked
// ******************************************************************************************
void SimulationWidget::overwriteURDF()
{
  if (!focusLost())  // validate XML
    return;

  if (!setup_step_.outputGazeboURDFFile(config_data_->urdf_path_))
    QMessageBox::warning(this, "Gazebo URDF", tr("Failed to save to ").append(config_data_->urdf_path_.c_str()));
  else  // Display success message
    QMessageBox::information(this, "Overwriting Successful",
                             "Original robot description URDF was successfully overwritten.");

  // Remove Gazebo URDF file from list of to-be-written config files
  config_data_->save_gazebo_urdf_ = false;
  config_data_->changes &= ~MoveItConfigData::SIMULATION;
}

void SimulationWidget::openURDF()
{
  QProcess::startDetached(qgetenv("EDITOR"), { setup_step_.getURDFPath().c_str() });
}

// ******************************************************************************************
// Called the copy to clipboard button is clicked
// ******************************************************************************************
void SimulationWidget::copyURDF()
{
  simulation_text_->selectAll();
  simulation_text_->copy();
}
}  // namespace simulation
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::simulation::SimulationWidget, moveit_setup::SetupStepWidget)
