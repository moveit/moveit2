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

#pragma once

#include <moveit_setup_framework/qt/setup_step_widget.hpp>
#include <moveit_setup_controllers/urdf_modifications.hpp>

#include <QPushButton>
#include <QCheckBox>
#include <QTextEdit>

namespace moveit_setup
{
namespace controllers
{
class UrdfModificationsWidget : public SetupStepWidget
{
  Q_OBJECT

public:
  void onInit() override;
  void focusGiven() override;

  SetupStep& getSetupStep() override
  {
    return setup_step_;
  }
private Q_SLOTS:
  void addInterfaces();

private:
  QWidget* makeInterfacesBox(const std::string& interface_type, const std::vector<std::string>& available_interfaces,
                             const std::vector<std::string>& selected_interfaces, QWidget* parent = nullptr);
  std::vector<std::string> getInterfaces(const char first_letter, const std::vector<std::string>& available_interfaces);
  QWidget* content_widget_;
  QPushButton* btn_add_interfaces_;
  QTextEdit* generated_text_widget_;
  std::unordered_map<std::string, QCheckBox*> check_boxes_;

  UrdfModifications setup_step_;
};
}  // namespace controllers
}  // namespace moveit_setup
