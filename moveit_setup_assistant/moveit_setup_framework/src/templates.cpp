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

/* Author: David V. Lu!! */

#include <moveit_setup_framework/templates.hpp>
#include <rclcpp/rclcpp.hpp>
#include <boost/algorithm/string.hpp>  // for string find and replace in templates
#include <moveit/utils/logger.hpp>

namespace moveit_setup
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.setup_assistant.setup.templates");
}
}  // namespace
std::vector<TemplateVariable> TemplatedGeneratedFile::variables;

bool TemplatedGeneratedFile::write()
{
  std::filesystem::path template_path = getTemplatePath();

  // Error check file
  if (!std::filesystem::is_regular_file(template_path))
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Unable to find template file " << template_path.string());
    return false;
  }

  // Load file
  std::ifstream template_stream(template_path);
  if (!template_stream.good())  // File not found
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Unable to load file " << template_path.string());
    return false;
  }

  // Load the file to a string using an efficient memory allocation technique
  std::string template_string;
  template_stream.seekg(0, std::ios::end);
  template_string.reserve(template_stream.tellg());
  template_stream.seekg(0, std::ios::beg);
  template_string.assign((std::istreambuf_iterator<char>(template_stream)), std::istreambuf_iterator<char>());
  template_stream.close();

  // Replace keywords in string ------------------------------------------------------------
  for (const auto& variable : variables)
  {
    std::string key_with_brackets = "[" + variable.key + "]";
    boost::replace_all(template_string, key_with_brackets, variable.value);
  }

  // Save string to new location -----------------------------------------------------------
  std::filesystem::path file_path = getPath();
  createParentFolders(file_path);

  std::ofstream output_stream(file_path, std::ios_base::trunc);
  if (!output_stream.good())
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Unable to open file for writing " << file_path.string());
    return false;
  }

  output_stream << template_string.c_str();
  output_stream.close();

  return true;  // file created successfully
}

}  // namespace moveit_setup
