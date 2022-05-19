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

#include <moveit_setup_framework/templates.hpp>
#include <moveit_setup_framework/utilities.hpp>

namespace moveit_setup
{
namespace app
{
/**
 * @brief One launch file and any other bonus files that get bundled with it, i.e. the RViz launch file and its config.
 *
 * Each launch bundle is presented to the user as option to generate. They are bundled because it would not (generally)
 * make sense to generate the RViz config without the launch file.
 */
class LaunchBundle
{
public:
  /**
   * @param title The display name for this LaunchBundle
   * @param description An English description of the files contained within
   * @param launch_name The identifier that signifies the name of the generated launch file (launch_name.launch.py)
   * @param dependencies Each string is the name of a package that should be added as a dependency if this bundle is generated
   */
  LaunchBundle(const std::string& title, const std::string& description, const std::string& launch_name,
               const std::set<std::string>& dependencies = {})
    : title_(title), description_(description), launch_name_(launch_name), dependencies_(dependencies), id_(-1)
  {
  }

  const std::string& getTitle() const
  {
    return title_;
  }

  const std::string& getDescription() const
  {
    return description_;
  }

  /**
   * @brief The ID is an index in a list, used for quick identification and argument passing
   */
  unsigned int getID() const
  {
    return id_;
  }

  void setID(unsigned int id)
  {
    id_ = id;
  }

  void addFile(const std::filesystem::path& relative_path, const std::string& description)
  {
    bonus_files_.push_back(BonusFile(relative_path, description));
  }

  const std::set<std::string> getDependencies() const
  {
    return dependencies_;
  }

  /**
   * @brief Defined so that LaunchBundles can be added to sets
   */
  bool operator<(const LaunchBundle& other) const
  {
    return title_ < other.title_;
  }

  class GenericLaunchTemplate : public TemplatedGeneratedFile
  {
  public:
    GenericLaunchTemplate(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                          const LaunchBundle& parent)
      : TemplatedGeneratedFile(package_path, last_gen_time), parent_(parent)
    {
      function_name_ = "generate_" + parent_.launch_name_ + "_launch";
      relative_path_ = std::filesystem::path("launch") / (parent_.launch_name_ + ".launch.py");
      template_path_ = getSharePath("moveit_setup_app_plugins") / "templates/launch/generic.launch.py.template";
    }

    std::filesystem::path getRelativePath() const override
    {
      return relative_path_;
    }

    std::string getDescription() const override
    {
      return parent_.description_;
    }

    std::filesystem::path getTemplatePath() const override
    {
      return template_path_;
    }

    bool hasChanges() const override
    {
      return false;
    }

    bool write() override
    {
      // Add function name as a TemplateVariable, then remove it
      variables_.push_back(TemplateVariable("FUNCTION_NAME", function_name_));
      bool ret = TemplatedGeneratedFile::write();
      variables_.pop_back();
      return ret;
    }

  protected:
    const LaunchBundle& parent_;
    std::string function_name_;
    std::filesystem::path relative_path_, template_path_;
  };

  struct BonusFile  // basically a std::pair<std::filesystem::path, std::string>
  {
    BonusFile(const std::filesystem::path& path, const std::string& description) : path(path), description(description)
    {
    }
    std::filesystem::path path;
    std::string description;
  };

  class BonusTemplatedFile : public TemplatedGeneratedFile
  {
  public:
    BonusTemplatedFile(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                       const std::filesystem::path& relative_path, const std::string& description)
      : TemplatedGeneratedFile(package_path, last_gen_time), relative_path_(relative_path), description_(description)
    {
    }

    std::filesystem::path getRelativePath() const override
    {
      return relative_path_;
    }

    std::string getDescription() const override
    {
      return description_;
    }

    std::filesystem::path getTemplatePath() const override
    {
      return getSharePath("moveit_setup_app_plugins") / "templates" / relative_path_;
    }

    bool hasChanges() const override
    {
      return false;
    }

  protected:
    std::filesystem::path relative_path_;
    std::string description_;
  };

  void collectFiles(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time,
                    std::vector<GeneratedFilePtr>& files) const
  {
    files.push_back(std::make_shared<GenericLaunchTemplate>(package_path, last_gen_time, *this));

    for (const BonusFile& bonus_file : bonus_files_)
    {
      files.push_back(
          std::make_shared<BonusTemplatedFile>(package_path, last_gen_time, bonus_file.path, bonus_file.description));
    }
  }

protected:
  std::string title_, description_, launch_name_;
  std::set<std::string> dependencies_;
  unsigned int id_;

  std::vector<BonusFile> bonus_files_;
};
}  // namespace app
}  // namespace moveit_setup
