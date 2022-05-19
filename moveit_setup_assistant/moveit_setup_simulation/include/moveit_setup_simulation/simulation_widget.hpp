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

#pragma once

// Qt
class QLabel;
class QTextEdit;
class QPushButton;

// SA
#include <moveit_setup_framework/qt/setup_step_widget.hpp>
#include <moveit_setup_simulation/simulation.hpp>

namespace moveit_setup
{
namespace simulation
{
// ******************************************************************************************
// ******************************************************************************************
// Class for showing changes needed to help user bring his robot into gazebo simulation
// ******************************************************************************************
// ******************************************************************************************
class SimulationWidget : public SetupStepWidget
{
  Q_OBJECT

public:
  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************

  void onInit() override;
  void focusGiven() override;
  bool focusLost() override;

  SetupStep& getSetupStep() override
  {
    return setup_step_;
  }

private Q_SLOTS:
  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /// Overwrite original URDF with content of document
  void overwriteURDF();
  /// Open original URDF with system editor
  void openURDF();
  /// Copy the content of the URDF document to the clipboard
  void copyURDF();

private:
  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************

  QTextEdit* simulation_text_;
  QLabel* no_changes_label_;
  QPushButton* btn_overwrite_;
  QPushButton* btn_open_;
  QLabel* copy_urdf_;

  Simulation setup_step_;
};

}  // namespace simulation
}  // namespace moveit_setup
