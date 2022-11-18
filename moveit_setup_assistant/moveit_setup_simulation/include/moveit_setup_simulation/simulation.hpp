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
#pragma once

#include <moveit_setup_framework/setup_step.hpp>
#include <moveit_setup_framework/data/urdf_config.hpp>

namespace moveit_setup
{
namespace simulation
{
class Simulation : public SetupStep
{
public:
  std::string getName() const override
  {
    return "Simulation";
  }

  void onInit() override;

  bool isReady() const override
  {
    return false;
  }

  std::filesystem::path getURDFPath() const
  {
    return urdf_config_->getURDFPath();
  }

  std::string getURDFPackageName() const
  {
    return urdf_config_->getURDFPackageName();
  }

  std::string getURDFContents() const
  {
    return urdf_config_->getURDFContents();
  }

  /**
   * \brief Helper function to get the controller that is controlling the joint
   * \return controller type
   */
  std::string getJointHardwareInterface(const std::string& joint_name);

  /**
   * \brief Parses the existing urdf and constructs a string from it with the elements required by gazebo simulator
   * added
   * \return gazebo compatible urdf or empty if error encountered
   */
  std::string getGazeboCompatibleURDF();

  bool outputGazeboURDFFile(const std::filesystem::path& file_path);

  /**
   * @brief Check if the given xml is valid
   * @param[in] new_urdf_contents The string of xml to check
   * @param[out] error_row The row of the error
   * @param[out] error_description The description
   * @returns True if valid, false otherwise
   */
  bool isValidXML(const std::string& new_urdf_contents, int& error_row, std::string& error_description) const;

protected:
  std::shared_ptr<URDFConfig> urdf_config_;

  /// Gazebo URDF robot model string
  // NOTE: Created when the robot urdf is not compatible with Gazebo.
  std::string gazebo_urdf_string_;

  /// Whether a new Gazebo URDF is created
  bool save_gazebo_urdf_;
};
}  // namespace simulation
}  // namespace moveit_setup
