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
#include <moveit_setup_framework/utilities.hpp>
#include <moveit_setup_framework/generated_time.hpp>
#include <moveit/macros/class_forward.h>
#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

namespace moveit_setup
{
/**
 * @brief Status associated with each GeneratedFile
 */
enum class FileStatus
{
  NEW,                  // The file does not exist in the configuration package
  UNCHANGED,            // The file exists and would be the same as the generated file
  CHANGED,              // The file exists, but a new version will be written
  EXTERNALLY_MODIFIED,  // The file exists and was externally modified
  CONFLICTED            // The file exists, was externally modified and there are changes to be written
};

MOVEIT_CLASS_FORWARD(GeneratedFile);  // Defines GeneratedFilePtr, ConstPtr, WeakPtr... etc

/**
 * @brief Container for the logic for a single file to appear in MoveIt configuration package.
 */
class GeneratedFile : public std::enable_shared_from_this<GeneratedFile>
{
public:
  GeneratedFile(const GeneratedFile&) = default;
  GeneratedFile(GeneratedFile&&) = default;
  virtual ~GeneratedFile() = default;

  GeneratedFile(const std::filesystem::path& package_path, const GeneratedTime& last_gen_time)
    : package_path_(package_path), last_gen_time_(last_gen_time)
  {
  }

  /**
   * @brief Returns the path relative to the configuration package root
   */
  virtual std::filesystem::path getRelativePath() const = 0;

  /**
   * @brief Returns an English description of this file's purpose.
   */
  virtual std::string getDescription() const = 0;

  /**
   * @brief Returns true if this file will have changes when it is written to file
   */
  virtual bool hasChanges() const = 0;

  /**
   * @brief Writes the file to disk
   */
  virtual bool write() = 0;

  /**
   * @brief Returns the fully qualified path to this file
   */
  std::filesystem::path getPath() const
  {
    return package_path_ / getRelativePath();
  }

  FileStatus getStatus() const
  {
    std::filesystem::path full_path = getPath();
    if (!std::filesystem::is_regular_file(full_path) || last_gen_time_ == GeneratedTime())
    {
      return FileStatus::NEW;
    }
    GeneratedTime mod_time = std::filesystem::last_write_time(full_path);
    if (mod_time > last_gen_time_ + TIME_MOD_TOLERANCE || mod_time < last_gen_time_ - TIME_MOD_TOLERANCE)
    {
      return hasChanges() ? FileStatus::CONFLICTED : FileStatus::EXTERNALLY_MODIFIED;
    }
    else
    {
      return hasChanges() ? FileStatus::CHANGED : FileStatus::UNCHANGED;
    }
  }

protected:
  static constexpr GeneratedTime::duration TIME_MOD_TOLERANCE = std::chrono::seconds(10);

  std::filesystem::path package_path_;
  const GeneratedTime& last_gen_time_;
};

class YamlGeneratedFile : public GeneratedFile
{
public:
  using GeneratedFile::GeneratedFile;

  bool write() override
  {
    YAML::Emitter emitter;
    bool ret = writeYaml(emitter);
    if (!ret)
    {
      return false;
    }

    std::filesystem::path file_path = getPath();
    createParentFolders(file_path);
    std::ofstream output_stream(file_path, std::ios_base::trunc);
    if (!output_stream.good())
    {
      return false;
    }

    output_stream << emitter.c_str();
    output_stream.close();

    return true;  // file created successfully
  }

  virtual bool writeYaml(YAML::Emitter& emitter) = 0;
};

}  // namespace moveit_setup
