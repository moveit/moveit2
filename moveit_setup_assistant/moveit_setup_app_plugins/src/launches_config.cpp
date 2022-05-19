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

#include <moveit_setup_app_plugins/launches_config.hpp>

namespace moveit_setup
{
namespace app
{
bool LaunchesConfig::isIncluded(const LaunchBundle& bundle) const
{
  return bundles_.count(bundle);
}

void LaunchesConfig::include(const LaunchBundle& bundle)
{
  bundles_.insert(bundle);
}

void LaunchesConfig::remove(const LaunchBundle& bundle)
{
  bundles_.erase(bundle);
}

void LaunchesConfig::collectDependencies(std::set<std::string>& packages) const
{
  packages.insert("moveit_configs_utils");
  for (const LaunchBundle& bundle : bundles_)
  {
    for (const std::string& dependency : bundle.getDependencies())
    {
      packages.insert(dependency);
    }
  }
}

void LaunchesConfig::collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                                  std::vector<GeneratedFilePtr>& files)
{
  for (const LaunchBundle& bundle : bundles_)
  {
    bundle.collectFiles(package_path, last_gen_time, files);
  }
}
}  // namespace app
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::app::LaunchesConfig, moveit_setup::SetupConfig)
