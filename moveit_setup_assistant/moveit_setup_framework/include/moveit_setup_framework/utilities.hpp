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

#include <boost/filesystem/path.hpp>        // for creating folders/files
#include <boost/filesystem/operations.hpp>  // is_regular_file, is_directory, etc.
#include <string>
#include <yaml-cpp/yaml.h>

namespace moveit_setup_framework
{
// ******************************************************************************************
// Helper Function for joining a file path and a file name, or two file paths, etc, in a cross-platform way
// ******************************************************************************************
inline std::string appendPaths(const std::string& path1, const std::string& path2)
{
  boost::filesystem::path result = path1;
  result /= path2;
  return result.make_preferred().string();
}

/**
 * @brief Create folders (recursively)
 * @return false if the path was not a directory and was unable to create the directory(s)
 */
inline bool createFolders(const std::string& output_path)
{
  return boost::filesystem::is_directory(output_path) || boost::filesystem::create_directories(output_path);
}

/**
 * @brief Create parent folders (recursively)
 * @return false if the path was not a directory and was unable to create the directory(s)
 */
inline bool createParentFolders(const std::string& file_path_string)
{
  boost::filesystem::path file_path(file_path_string);
  return createFolders(file_path.parent_path().string());
}

/**
 * determine the package name containing a given file path
 * @param path to a file
 * @param package_name holds the ros package name if found
 * @param relative_filepath holds the relative path of the file to the package root
 * @return whether the file belongs to a known package
 */
inline bool extractPackageNameFromPath(const std::string& path, std::string& package_name,
                                       std::string& relative_filepath)
{
  boost::filesystem::path sub_path = path;  // holds the directory less one folder
  boost::filesystem::path relative_path;    // holds the path after the sub_path

  // truncate path step by step and check if it contains a package.xml
  while (!sub_path.empty())
  {
    if (boost::filesystem::is_regular_file(sub_path / "package.xml"))
    {
      relative_filepath = relative_path.string();
      package_name = sub_path.leaf().string();
      return true;
    }
    relative_path = sub_path.leaf() / relative_path;
    sub_path.remove_leaf();
  }

  // No package name found, we must be outside ROS
  return false;
}

// Formerly "parse"
template <typename T>
inline bool getYamlProperty(const YAML::Node& node, const std::string& key, T& storage, const T& default_value = T())
{
  const YAML::Node& n = node[key];
  bool valid = n;
  storage = valid ? n.as<T>() : default_value;
  return valid;
}

}  // namespace moveit_setup_framework
