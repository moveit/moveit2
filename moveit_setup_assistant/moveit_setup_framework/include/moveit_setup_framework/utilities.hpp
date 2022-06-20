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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <string>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>

namespace moveit_setup
{
/**
 * @brief Return a path for the given package's share folder
 */
inline std::filesystem::path getSharePath(const std::string& package_name)
{
  return std::filesystem::path(ament_index_cpp::get_package_share_directory(package_name));
}

/**
 * @brief Create folders (recursively)
 * @return false if the path was not a directory and was unable to create the directory(s)
 */
inline bool createFolders(const std::filesystem::path& output_path)
{
  return std::filesystem::is_directory(output_path) || std::filesystem::create_directories(output_path);
}

/**
 * @brief Create parent folders (recursively)
 * @return false if the path was not a directory and was unable to create the directory(s)
 */
inline bool createParentFolders(const std::filesystem::path& file_path)
{
  return createFolders(file_path.parent_path());
}

/**
 * determine the package name containing a given file path
 * @param path to a file
 * @param package_name holds the ros package name if found
 * @param relative_filepath holds the relative path of the file to the package root
 * @return whether the file belongs to a known package
 */
bool extractPackageNameFromPath(const std::filesystem::path& path, std::string& package_name,
                                std::filesystem::path& relative_filepath);

/**
 * @brief Simple structure for easy xml creation
 */
struct XMLAttribute
{
  const char* name;
  const char* value;
  bool required = false;
};

/**
 * @brief Insert a new XML element with a given tag, attributes, and text
 *
 * If a corresponding element already exists (and has required attribute values), it is just reused.
 *
 * @param doc The XMLDocument, used for creating new elements
 * @param element The tag inside of which the new tag should be inserted
 * @param tag The name of the tag
 * @param attributes Attribute name/value pairs to be created/overwritten
 * @param text If not null, text value to insert inside the new tag
 * @returns The new or existing element
 */
tinyxml2::XMLElement* uniqueInsert(tinyxml2::XMLDocument& doc, tinyxml2::XMLElement& element, const char* tag,
                                   const std::vector<XMLAttribute>& attributes = {}, const char* text = nullptr);

// Formerly "parse"
template <typename T>
inline bool getYamlProperty(const YAML::Node& node, const std::string& key, T& storage, const T& default_value = T())
{
  const YAML::Node& n = node[key];
  bool valid = n.IsDefined();
  storage = valid ? n.as<T>() : default_value;
  return valid;
}

inline bool getYamlProperty(const YAML::Node& node, const std::string& key, std::filesystem::path& storage,
                            const std::string& default_value = "")
{
  std::string storage_s;
  bool ret = getYamlProperty(node, key, storage_s, default_value);
  if (ret)
  {
    storage = storage_s;
  }
  return ret;
}

}  // namespace moveit_setup
