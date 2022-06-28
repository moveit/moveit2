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

#include <moveit_setup_framework/data/urdf_config.hpp>
#include <moveit_setup_framework/data_warehouse.hpp>

namespace moveit_setup
{
/**
 * @brief A virtual class that represents a xacro header that should be included in the modified urdf configuration
 */
class IncludedXacroConfig : public SetupConfig
{
public:
  void onInit() override
  {
    // Register the modified config and access it to ensure it is generated
    config_data_->registerType("modified_urdf", "moveit_setup::ModifiedUrdfConfig");
    config_data_->get("modified_urdf", "moveit_setup::ModifiedUrdfConfig");

    urdf_config_ = config_data_->get<URDFConfig>("urdf");
  }

  /**
   * @brief The file path to use in the <xacro:include> tag.
   *
   * If the file is in MoveIt config's config folder, just the file name is fine.
   * Alternatively you can return a package string a la
   * `$(find SOME_PACKAGE_NAME)/relative/path/to/header.xacro`
   */
  virtual std::string getFilepath() const = 0;

  /**
   * @brief Returns if the xacro and its properties have changed, resulting in the whole urdf needing regeneration
   */
  virtual bool hasChanges() const = 0;

  /**
   * @brief Returns a list of name/value pairs for arguments that the modified urdf should have
   *
   * Result will be <xacro:arg name="pair.first" default="pair.second" />
   */
  virtual std::vector<std::pair<std::string, std::string>> getArguments() const
  {
    return {};
  };

  /**
   * @brief Return a list of additional commands that need to be inserted after the xacro is included.
   *
   * e.g. If the included xacro includes a macro definition, the command to run the macro could be here.
   */
  virtual std::vector<std::string> getCommands() const
  {
    return {};
  }

  using Ptr = std::shared_ptr<IncludedXacroConfig>;

protected:
  std::shared_ptr<URDFConfig> urdf_config_;
};

}  // namespace moveit_setup
