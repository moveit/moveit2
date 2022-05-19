/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Robotics
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
 *   * Neither the name of PickNik Robotics nor the names of its
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

/* Author: David V. Lu!! */
#include <moveit_setup_framework/qt/rviz_panel.hpp>
#include <moveit_setup_framework/data/urdf_config.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>
#include <QApplication>
#include "rviz_rendering/render_window.hpp"

namespace moveit_setup
{
RVizPanel::RVizPanel(QWidget* parent, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr node_abstraction,
                     DataWarehousePtr config_data)
  : QWidget(parent)
  , parent_(parent)
  , node_abstraction_(node_abstraction)
  , node_(node_abstraction_.lock()->get_raw_node())
  , config_data_(config_data)
{
  logger_ = std::make_shared<rclcpp::Logger>(node_->get_logger().get_child("RVizPanel"));

  connect(this, SIGNAL(highlightLinkSignal(const std::string&, const QColor&)), this,
          SLOT(highlightLinkEvent(const std::string&, const QColor&)));
  connect(this, SIGNAL(highlightGroupSignal(const std::string&)), this, SLOT(highlightGroupEvent(const std::string&)));
  connect(this, SIGNAL(unhighlightAllSignal()), this, SLOT(unhighlightAllEvent()));
}

void RVizPanel::initialize()
{
  // Create rviz frame
  rviz_render_panel_ = new rviz_common::RenderPanel();
  rviz_render_panel_->setMinimumWidth(200);
  rviz_render_panel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  QApplication::processEvents();
  rviz_render_panel_->getRenderWindow()->initialize();

  rviz_manager_ =
      new rviz_common::VisualizationManager(rviz_render_panel_, node_abstraction_, this, node_->get_clock());
  rviz_render_panel_->initialize(rviz_manager_);
  rviz_manager_->initialize();
  rviz_manager_->startUpdate();

  // Create the MoveIt Rviz Plugin and attach to display
  robot_state_display_ = new moveit_rviz_plugin::RobotStateDisplay();
  robot_state_display_->setName("Robot State");

  rviz_manager_->addDisplay(robot_state_display_, true);

  // Set the fixed and target frame
  updateFixedFrame();

  // Set the topic on which the moveit_msgs::msg::PlanningScene messages are received
  robot_state_display_->subProp("Robot State Topic")->setValue(QString::fromStdString(MOVEIT_ROBOT_STATE));

  // Set robot description
  robot_state_display_->subProp("Robot Description")->setValue(QString::fromStdString(ROBOT_DESCRIPTION));
  robot_state_display_->setVisible(true);
  // Zoom into robot
  rviz_common::ViewController* view = rviz_manager_->getViewManager()->getCurrent();
  view->subProp("Distance")->setValue(4.0f);

  // Add Rviz to Planning Groups Widget
  QVBoxLayout* rviz_layout = new QVBoxLayout();
  rviz_layout->addWidget(rviz_render_panel_);
  setLayout(rviz_layout);

  // visual / collision buttons
  auto btn_layout = new QHBoxLayout();
  rviz_layout->addLayout(btn_layout);

  QCheckBox* btn;
  btn_layout->addWidget(btn = new QCheckBox("visual"), 0);
  btn->setChecked(true);
  connect(btn, &QCheckBox::toggled,
          [this](bool checked) { robot_state_display_->subProp("Visual Enabled")->setValue(checked); });

  btn_layout->addWidget(btn = new QCheckBox("collision"), 1);
  btn->setChecked(false);
  connect(btn, &QCheckBox::toggled,
          [this](bool checked) { robot_state_display_->subProp("Collision Enabled")->setValue(checked); });
}
RVizPanel::~RVizPanel()
{
  if (rviz_manager_ != nullptr)
    rviz_manager_->removeAllDisplays();
  if (rviz_render_panel_ != nullptr)
    delete rviz_render_panel_;
  if (rviz_manager_ != nullptr)
    delete rviz_manager_;
}

moveit::core::RobotModelPtr RVizPanel::getRobotModel() const
{
  auto urdf = config_data_->get<moveit_setup::URDFConfig>("urdf");

  if (!urdf->isConfigured())
  {
    return nullptr;
  }

  auto srdf = config_data_->get<moveit_setup::SRDFConfig>("srdf");

  return srdf->getRobotModel();
}

void RVizPanel::updateFixedFrame()
{
  auto rm = getRobotModel();
  if (rm && rviz_manager_ && robot_state_display_)
  {
    std::string frame = rm->getModelFrame();
    rviz_manager_->setFixedFrame(QString::fromStdString(frame));
    robot_state_display_->reset();
    robot_state_display_->setVisible(true);
  }
}

// ******************************************************************************************
// Highlight a robot link
// ******************************************************************************************
void RVizPanel::highlightLinkEvent(const std::string& link_name, const QColor& color)
{
  auto rm = getRobotModel();
  if (!rm)
    return;
  const moveit::core::LinkModel* lm = rm->getLinkModel(link_name);
  if (!lm->getShapes().empty())  // skip links with no geometry
    robot_state_display_->setLinkColor(link_name, color);
}

// ******************************************************************************************
// Highlight a robot group
// ******************************************************************************************
void RVizPanel::highlightGroupEvent(const std::string& group_name)
{
  auto rm = getRobotModel();
  if (!rm)
    return;
  // Highlight the selected planning group by looping through the links
  if (!rm->hasJointModelGroup(group_name))
    return;

  const moveit::core::JointModelGroup* joint_model_group = rm->getJointModelGroup(group_name);
  if (joint_model_group)
  {
    const std::vector<const moveit::core::LinkModel*>& link_models = joint_model_group->getLinkModels();
    // Iterate through the links
    for (std::vector<const moveit::core::LinkModel*>::const_iterator link_it = link_models.begin();
         link_it < link_models.end(); ++link_it)
      highlightLink((*link_it)->getName(), QColor(255, 0, 0));
  }
}

// ******************************************************************************************
// Unhighlight all robot links
// ******************************************************************************************
void RVizPanel::unhighlightAllEvent()
{
  auto rm = getRobotModel();
  if (!rm)
    return;
  // Get the names of the all links robot
  const std::vector<std::string>& links = rm->getLinkModelNamesWithCollisionGeometry();

  // Quit if no links found
  if (links.empty())
  {
    return;
  }

  // check if rviz is ready
  if (!rviz_manager_ || !robot_state_display_)
  {
    return;
  }

  // Iterate through the links
  for (std::vector<std::string>::const_iterator link_it = links.begin(); link_it < links.end(); ++link_it)
  {
    if ((*link_it).empty())
      continue;

    robot_state_display_->unsetLinkColor(*link_it);
  }
}

}  // namespace moveit_setup
