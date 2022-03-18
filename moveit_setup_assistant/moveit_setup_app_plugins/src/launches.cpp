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
#include <moveit_setup_app_plugins/launches.hpp>

namespace moveit_setup_app_plugins
{
void Launches::onInit()
{
  config_data_->registerType("launches", "moveit_setup_app_plugins::LaunchesConfig");
  launches_config_ = config_data_->get<moveit_setup_app_plugins::LaunchesConfig>("launches");

  for (unsigned int i = 0; i < launch_bundles_.size(); i++)
  {
    launch_bundles_[i].setID(i);

    // By default, we start with all the launch bundles included
    launches_config_->include(launch_bundles_[i]);
  }
}

bool Launches::getState(unsigned int id) const
{
  return launches_config_->isIncluded(launch_bundles_[id]);
}

void Launches::setState(unsigned int id, bool state)
{
  const LaunchBundle& bundle = launch_bundles_[id];
  if (state)
  {
    launches_config_->include(bundle);
  }
  else
  {
    launches_config_->remove(bundle);
  }
}
}  // namespace moveit_setup_app_plugins
