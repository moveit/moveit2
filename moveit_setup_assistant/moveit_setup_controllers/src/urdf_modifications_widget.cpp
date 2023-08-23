/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Metro Robots
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
 *   * Neither the name of Metro Robots nor the names of its
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

#include <moveit_setup_controllers/urdf_modifications_widget.hpp>
#include <moveit_setup_framework/qt/helper_widgets.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>

namespace moveit_setup
{
namespace controllers
{
void UrdfModificationsWidget::onInit()
{
  QVBoxLayout* layout = new QVBoxLayout();

  layout->setAlignment(Qt::AlignTop);

  // Top Header Area ------------------------------------------------
  auto header = new HeaderWidget("ros2_control URDF Modifications",
                                 "This step can add the tags to the URDF required for interfacing with ros2_control. "
                                 "See https://control.ros.org/ for more info.",
                                 this);
  layout->addWidget(header);
  content_widget_ = new QWidget(this);
  layout->addWidget(content_widget_);

  setLayout(layout);
}

QWidget* UrdfModificationsWidget::makeInterfacesBox(const std::string& interface_type,
                                                    const std::vector<std::string>& available_interfaces,
                                                    const std::vector<std::string>& selected_interfaces,
                                                    QWidget* parent)
{
  QGroupBox* box = new QGroupBox((interface_type + " Interfaces").c_str(), parent);
  QVBoxLayout* box_layout = new QVBoxLayout(parent);
  for (const std::string& interface_name : available_interfaces)
  {
    QCheckBox* check = new QCheckBox(interface_name.c_str(), parent);
    box_layout->addWidget(check);

    std::string key = interface_type[0] + interface_name;
    check_boxes_[key] = check;
  }
  for (const std::string& interface_name : selected_interfaces)
  {
    std::string key = interface_type[0] + interface_name;
    check_boxes_[key]->setChecked(true);
  }
  box->setLayout(box_layout);
  return box;
}

void UrdfModificationsWidget::focusGiven()
{
  setup_step_.refresh();
  qDeleteAll(content_widget_->children());
  check_boxes_.clear();

  QVBoxLayout* layout = new QVBoxLayout();
  if (!setup_step_.needsModification())
  {
    layout->addWidget(new QLabel("All of the joints used by this MoveIt config have already been configured using\n"
                                 "ros2_control, so there is no need to modify the URDF with ros2_control tags."));
    content_widget_->setLayout(layout);
    return;
  }

  QWidget* interface_widget = new QWidget(this);
  QHBoxLayout* interface_layout = new QHBoxLayout();

  auto available_interfaces = setup_step_.getAvailableControlInterfaces(),
       selected_interfaces = setup_step_.getDefaultControlInterfaces();

  interface_layout->addWidget(makeInterfacesBox("Command", available_interfaces.command_interfaces,
                                                selected_interfaces.command_interfaces, interface_widget));
  interface_layout->addWidget(makeInterfacesBox("State", available_interfaces.state_interfaces,
                                                selected_interfaces.state_interfaces, interface_widget));

  interface_widget->setLayout(interface_layout);
  layout->addWidget(interface_widget);

  btn_add_interfaces_ = new QPushButton("Add interfaces");
  connect(btn_add_interfaces_, SIGNAL(clicked()), this, SLOT(addInterfaces()));
  layout->addWidget(btn_add_interfaces_);

  generated_text_widget_ = new QTextEdit();
  generated_text_widget_->setReadOnly(true);
  generated_text_widget_->setText(setup_step_.getJointsXML().c_str());
  layout->addWidget(generated_text_widget_);
  content_widget_->setLayout(layout);
}

std::vector<std::string> UrdfModificationsWidget::getInterfaces(const char first_letter,
                                                                const std::vector<std::string>& available_interfaces)
{
  std::vector<std::string> interface_names;
  for (const std::string& interface_name : available_interfaces)
  {
    std::string key = first_letter + interface_name;
    if (check_boxes_[key]->isChecked())
    {
      interface_names.push_back(interface_name);
    }
  }
  return interface_names;
}

void UrdfModificationsWidget::addInterfaces()
{
  auto available_interfaces = setup_step_.getAvailableControlInterfaces();
  setup_step_.setInterfaces(getInterfaces('C', available_interfaces.command_interfaces),
                            getInterfaces('S', available_interfaces.state_interfaces));
  generated_text_widget_->setText(setup_step_.getJointsXML().c_str());
}

}  // namespace controllers
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::controllers::UrdfModificationsWidget, moveit_setup::SetupStepWidget)
