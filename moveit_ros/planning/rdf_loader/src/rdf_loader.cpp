/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Mathias Lüdtke, Dave Coleman */

// MoveIt
#include <moveit/rdf_loader/rdf_loader.h>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/utils/logger.hpp>

#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

// C++
#include <fstream>
#include <streambuf>
#include <algorithm>
#include <chrono>
#include <filesystem>

namespace rdf_loader
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("RDFLoader");
}
}  // namespace

RDFLoader::RDFLoader(const std::shared_ptr<rclcpp::Node>& node, const std::string& ros_name,
                     bool default_continuous_value, double default_timeout)
  : ros_name_(ros_name)
{
  auto start = node->now();

  urdf_string_ = urdf_ssp_.loadInitialValue(
      node, ros_name, [this](const std::string& new_urdf_string) { return urdfUpdateCallback(new_urdf_string); },
      default_continuous_value, default_timeout);

  const std::string srdf_name = ros_name + "_semantic";
  srdf_string_ = srdf_ssp_.loadInitialValue(
      node, srdf_name, [this](const std::string& new_srdf_string) { return srdfUpdateCallback(new_srdf_string); },
      default_continuous_value, default_timeout);

  if (!loadFromStrings())
  {
    return;
  }

  RCLCPP_INFO_STREAM(getLogger(), "Loaded robot model in " << (node->now() - start).seconds() << " seconds");
}

RDFLoader::RDFLoader(const std::string& urdf_string, const std::string& srdf_string)
  : urdf_string_(urdf_string), srdf_string_(srdf_string)
{
  if (!loadFromStrings())
  {
    return;
  }
}

bool RDFLoader::loadFromStrings()
{
  std::unique_ptr<urdf::Model> urdf = std::make_unique<urdf::Model>();
  if (!urdf->initString(urdf_string_))
  {
    RCLCPP_INFO(getLogger(), "Unable to parse URDF");
    return false;
  }

  srdf::ModelSharedPtr srdf = std::make_shared<srdf::Model>();
  if (!srdf->initString(*urdf, srdf_string_))
  {
    RCLCPP_ERROR(getLogger(), "Unable to parse SRDF");
    return false;
  }

  urdf_ = std::move(urdf);
  srdf_ = std::move(srdf);
  return true;
}

bool RDFLoader::isXacroFile(const std::string& path)
{
  std::string lower_path = path;
  std::transform(lower_path.begin(), lower_path.end(), lower_path.begin(), ::tolower);

  return lower_path.find(".xacro") != std::string::npos;
}

bool RDFLoader::loadFileToString(std::string& buffer, const std::string& path)
{
  if (path.empty())
  {
    RCLCPP_ERROR(getLogger(), "Path is empty");
    return false;
  }

  if (!std::filesystem::exists(path))
  {
    RCLCPP_ERROR(getLogger(), "File does not exist");
    return false;
  }

  std::ifstream stream(path.c_str());
  if (!stream.good())
  {
    RCLCPP_ERROR(getLogger(), "Unable to load path");
    return false;
  }

  // Load the file to a string using an efficient memory allocation technique
  stream.seekg(0, std::ios::end);
  buffer.reserve(stream.tellg());
  stream.seekg(0, std::ios::beg);
  buffer.assign((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  stream.close();

  return true;
}

bool RDFLoader::loadXacroFileToString(std::string& buffer, const std::string& path,
                                      const std::vector<std::string>& xacro_args)
{
  buffer.clear();
  if (path.empty())
  {
    RCLCPP_ERROR(getLogger(), "Path is empty");
    return false;
  }

  if (!std::filesystem::exists(path))
  {
    RCLCPP_ERROR(getLogger(), "File does not exist");
    return false;
  }

  std::string cmd = "ros2 run xacro xacro ";
  for (const std::string& xacro_arg : xacro_args)
    cmd += xacro_arg + " ";
  cmd += path;

#ifdef _WIN32
  FILE* pipe = _popen(cmd.c_str(), "r");
#else
  FILE* pipe = popen(cmd.c_str(), "r");
#endif
  if (!pipe)
  {
    RCLCPP_ERROR(getLogger(), "Unable to load path");
    return false;
  }

  char pipe_buffer[128];
  while (!feof(pipe))
  {
    if (fgets(pipe_buffer, 128, pipe) != nullptr)
      buffer += pipe_buffer;
  }
#ifdef _WIN32
  _pclose(pipe);
#else
  pclose(pipe);
#endif

  return true;
}

bool RDFLoader::loadXmlFileToString(std::string& buffer, const std::string& path,
                                    const std::vector<std::string>& xacro_args)
{
  if (isXacroFile(path))
  {
    return loadXacroFileToString(buffer, path, xacro_args);
  }

  return loadFileToString(buffer, path);
}

bool RDFLoader::loadPkgFileToString(std::string& buffer, const std::string& package_name,
                                    const std::string& relative_path, const std::vector<std::string>& xacro_args)
{
  std::string package_path;
  try
  {
    package_path = ament_index_cpp::get_package_share_directory(package_name);
  }
  catch (const ament_index_cpp::PackageNotFoundError& e)
  {
    RCLCPP_ERROR(getLogger(), "ament_index_cpp: %s", e.what());
    return false;
  }

  std::filesystem::path path(package_path);
  path = path / relative_path;

  return loadXmlFileToString(buffer, path.string(), xacro_args);
}

void RDFLoader::urdfUpdateCallback(const std::string& new_urdf_string)
{
  urdf_string_ = new_urdf_string;
  if (!loadFromStrings())
  {
    return;
  }
  if (new_model_cb_)
  {
    new_model_cb_();
  }
}

void RDFLoader::srdfUpdateCallback(const std::string& new_srdf_string)
{
  srdf_string_ = new_srdf_string;
  if (!loadFromStrings())
  {
    return;
  }
  if (new_model_cb_)
  {
    new_model_cb_();
  }
}
}  // namespace rdf_loader
