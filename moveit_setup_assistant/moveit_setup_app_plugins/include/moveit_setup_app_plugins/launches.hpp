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

#include <moveit_setup_framework/setup_step.hpp>
#include <moveit_setup_app_plugins/launches_config.hpp>

namespace moveit_setup
{
namespace app
{
/**
 * @brief Setup step for generating launch files that are not otherwise associated with a specific step.
 */
class Launches : public SetupStep
{
public:
  std::string getName() const override
  {
    return "Launch Files";
  }

  void onInit() override;
  bool isReady() const override
  {
    return true;  // no generic dependencies
  }

  /**
   * @brief Get all available launch bundles
   */
  const std::vector<LaunchBundle>& getAvailableLaunchBundles() const
  {
    return available_launch_bundles_;
  }

  /**
   * @returns True if the LaunchBundle with the given id is currently included
   */
  bool getState(unsigned int id) const;

  /**
   * @brief Sets whether the LaunchBundle with the given id is included (true) or not (false)
   */
  void setState(unsigned int id, bool state);

  /**
   * @returns The description for the LaunchBundle with the given id
   */
  const std::string& getDescription(unsigned int id) const
  {
    return available_launch_bundles_[id].getDescription();
  }

protected:
  std::vector<LaunchBundle> available_launch_bundles_;
  std::shared_ptr<LaunchesConfig> launches_config_;
};
}  // namespace app
}  // namespace moveit_setup
