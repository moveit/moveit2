/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Robotics, Inc.
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

#pragma once

#include <moveit_setup_framework/config.hpp>
#if __has_include(<urdf/model.hpp>)  // for testing a valid urdf is loaded
#include <urdf/model.hpp>
#else
#include <urdf/model.h>
#endif

namespace moveit_setup
{
class URDFConfig : public SetupConfig
{
public:
  URDFConfig()
  {
    urdf_model_ = std::make_shared<urdf::Model>();
  }

  void onInit() override;

  void loadPrevious(const std::filesystem::path& package_path, const YAML::Node& node) override;
  YAML::Node saveToYaml() const override;

  /// Load URDF File
  void loadFromPath(const std::filesystem::path& urdf_file_path, const std::string& xacro_args = "");
  void loadFromPath(const std::filesystem::path& urdf_file_path, const std::vector<std::string>& xacro_args);
  void loadFromPackage(const std::filesystem::path& package_name, const std::filesystem::path& relative_path,
                       const std::string& xacro_args = "");

  const urdf::Model& getModel() const
  {
    return *urdf_model_;
  }

  const std::shared_ptr<urdf::Model>& getModelPtr() const
  {
    return urdf_model_;
  }

  std::string getURDFPackageName() const
  {
    return urdf_pkg_name_;
  }

  std::string getURDFContents() const
  {
    return urdf_string_;
  }

  std::filesystem::path getURDFPath() const
  {
    return urdf_path_;
  }

  std::string getXacroArgs() const
  {
    return xacro_args_;
  }

  bool isConfigured() const override;

  bool isXacroFile() const;

  void collectDependencies(std::set<std::string>& packages) const override;

  void collectVariables(std::vector<TemplateVariable>& variables) override;

  std::string getRobotName() const
  {
    return urdf_model_->getName();
  }

protected:
  void setPackageName();
  void load();

  /// Full file-system path to urdf
  std::filesystem::path urdf_path_;

  /// Name of package containing urdf (note: this may be empty b/c user may not have urdf in pkg)
  std::string urdf_pkg_name_;

  /// Path relative to urdf package (note: this may be same as urdf_path_)
  std::filesystem::path urdf_pkg_relative_path_;

  /// Flag indicating whether the URDF was loaded from .xacro format
  bool urdf_from_xacro_;

  /// xacro arguments in two different formats
  std::string xacro_args_;
  std::vector<std::string> xacro_args_vec_;

  /// URDF robot model
  std::shared_ptr<urdf::Model> urdf_model_;

  /// URDF robot model string
  std::string urdf_string_;
};
}  // namespace moveit_setup
