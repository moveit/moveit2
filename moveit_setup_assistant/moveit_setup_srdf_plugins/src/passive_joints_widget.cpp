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

// SA
#include <moveit_setup_srdf_plugins/passive_joints_widget.hpp>
#include <moveit_setup_framework/qt/helper_widgets.hpp>

// Qt
#include <QFormLayout>
#include <QLabel>
#include <QMessageBox>
#include <QTableWidget>

namespace moveit_setup
{
namespace srdf_setup
{
// ******************************************************************************************
// Constructor
// ******************************************************************************************
void PassiveJointsWidget::onInit()
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();

  // Top Header Area ------------------------------------------------

  auto header = new HeaderWidget(
      "Define Passive Joints",
      "Specify the set of passive joints (not actuated). Joint state is not expected to be published for these joints.",
      this);
  layout->addWidget(header);

  // Joints edit widget
  joints_widget_ = new DoubleListWidget(this, "Joint Collection", "Joint", false);
  connect(joints_widget_, SIGNAL(selectionUpdated()), this, SLOT(selectionUpdated()));
  connect(joints_widget_, SIGNAL(previewSelected(std::vector<std::string>)), this,
          SLOT(previewSelectedJoints(std::vector<std::string>)));

  // Set the title
  joints_widget_->title_->setText("");

  joints_widget_->setColumnNames("Active Joints", "Passive Joints");

  layout->addWidget(joints_widget_);

  // Finish Layout --------------------------------------------------
  setLayout(layout);
}

// ******************************************************************************************
//
// ******************************************************************************************
void PassiveJointsWidget::focusGiven()
{
  joints_widget_->clearContents();

  std::vector<std::string> active_joints = setup_step_.getActiveJoints();

  if (active_joints.empty())
  {
    QMessageBox::critical(this, "Error Loading", "No joints found for robot model");
    return;
  }

  // Set the available joints (left box)
  joints_widget_->setAvailable(active_joints);

  joints_widget_->setSelected(setup_step_.getPassiveJoints());
}

// ******************************************************************************************
//
// ******************************************************************************************
void PassiveJointsWidget::selectionUpdated()
{
  setup_step_.setPassiveJoints(joints_widget_->getSelectedValues());
}

// ******************************************************************************************
// Called from Double List widget to highlight joints
// ******************************************************************************************
void PassiveJointsWidget::previewSelectedJoints(const std::vector<std::string>& joints)
{
  // Unhighlight all links
  rviz_panel_->unhighlightAll();

  for (const std::string& joint : joints)
  {
    std::string link = setup_step_.getChildOfJoint(joint);
    // Check that a joint model/link was found
    if (link.empty())
    {
      continue;
    }

    // Highlight link
    rviz_panel_->highlightLink(link, QColor(255, 0, 0));
  }
}

}  // namespace srdf_setup
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::srdf_setup::PassiveJointsWidget, moveit_setup::SetupStepWidget)
