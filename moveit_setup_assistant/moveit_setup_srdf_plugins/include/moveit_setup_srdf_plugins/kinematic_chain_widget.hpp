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

#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include <moveit_setup_framework/qt/rviz_panel.hpp>
#include <moveit_setup_srdf_plugins/planning_groups.hpp>

namespace moveit_setup
{
namespace srdf_setup
{
class KinematicChainWidget : public QWidget
{
  Q_OBJECT
public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  /// Constructor
  KinematicChainWidget(QWidget* parent, RVizPanel* rviz_panel);

  /// Loads the available data list
  void setAvailable(const LinkNameTree& link_name_tree);

  /// Set the link field with previous value
  void setSelected(const std::string& base_link, const std::string& tip_link);

  QTreeWidgetItem* addLinkChildRecursive(QTreeWidgetItem* parent, const LinkNameTree& link);

  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

  QLabel* title_;  // specify the title from the parent widget
  QTreeWidget* link_tree_;
  QLineEdit* base_link_field_;
  QLineEdit* tip_link_field_;

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Choose the base link
  void baseLinkTreeClick();

  /// Choose the tip link
  void tipLinkTreeClick();

  /// Expand/Collapse Tree
  void alterTree(const QString& link);

  /// Highlight the selected link in the kinematic chain
  void itemSelected();

Q_SIGNALS:

  // ******************************************************************************************
  // Emitted Signals
  // ******************************************************************************************

  /// Event sent when this widget is done making data changes and parent widget can save
  void doneEditing();

  /// Event sent when user presses cancel button
  void cancelEditing();

private:
  // ******************************************************************************************
  // Variables
  // ******************************************************************************************

  /// Remember if the chain tree has been loaded
  bool kinematic_chain_loaded_;
  RVizPanel* rviz_panel_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************
};
}  // namespace srdf_setup
}  // namespace moveit_setup
