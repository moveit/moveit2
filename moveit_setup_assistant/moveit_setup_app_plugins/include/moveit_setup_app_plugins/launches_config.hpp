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

#include <moveit_setup_framework/config.hpp>
#include <moveit_setup_framework/templates.hpp>
#include <moveit_setup_app_plugins/launch_bundle.hpp>
#include <moveit_setup_framework/utilities.hpp>

namespace moveit_setup
{
namespace app
{
/**
 * @brief Stores which LaunchBundles are configured to be generated.
 *
 * Saved as a set in which the bundles to be generated are included in the set.
 */
class LaunchesConfig : public SetupConfig
{
public:
  bool isConfigured() const override
  {
    return !bundles_.empty();
  }

  /**
   * @returns True if bundle is currently included
   */
  bool isIncluded(const LaunchBundle& bundle) const;

  /**
   * @brief Add the given launch bundle to the set
   */
  void include(const LaunchBundle& bundle);

  /**
   * @brief Remove the given launch bundle from the set
   */
  void remove(const LaunchBundle& bundle);

  /**
   * @brief Add the dependencies from the launch bundles to the moveit config's dependencies
   */
  void collectDependencies(std::set<std::string>& packages) const override;

  /**
   * @brief Provide the files to be generated
   */
  void collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                    std::vector<GeneratedFilePtr>& files) override;

protected:
  std::set<LaunchBundle> bundles_;
};
}  // namespace app
}  // namespace moveit_setup
