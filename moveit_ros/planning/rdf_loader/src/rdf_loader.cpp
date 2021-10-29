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
#include <moveit/profiler/profiler.h>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Boost
#include <boost/filesystem.hpp>

// C++
#include <fstream>
#include <streambuf>
#include <algorithm>
#include <chrono>

namespace rdf_loader
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_rdf_loader.rdf_loader");

RDFLoader::RDFLoader(const std::shared_ptr<rclcpp::Node>& node, const std::string& robot_description)
  : robot_description_(robot_description)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RDFLoader(robot_description)");

  auto start = node->now();

  std::string description_content = description_loader_.loadInitialValue(
      node, robot_description, std::bind(&RDFLoader::descriptionUpdateCallback, this, std::placeholders::_1));

  const std::string srdf_description = robot_description + "_semantic";
  std::string semantic_content = semantic_loader_.loadInitialValue(
      node, srdf_description, std::bind(&RDFLoader::semanticUpdateCallback, this, std::placeholders::_1));

  if (!loadURDFFromString(description_content))
  {
    return;
  }

  if (!loadSRDFFromString(semantic_content))
  {
    return;
  }

  RCLCPP_INFO_STREAM(LOGGER, "Loaded robot model in " << (node->now() - start).seconds() << " seconds");
}

RDFLoader::RDFLoader(const std::string& urdf_string, const std::string& srdf_string)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RDFLoader(string)");

  if (loadURDFFromString(urdf_string))
  {
    loadSRDFFromString(srdf_string);
  }
}

bool RDFLoader::loadURDFFromString(const std::string& content)
{
  std::unique_ptr<urdf::Model> urdf(new urdf::Model());
  if (!urdf->initString(content))
  {
    RCLCPP_INFO(LOGGER, "Unable to parse URDF");
    return false;
  }
  urdf_ = std::move(urdf);
  return true;
}

bool RDFLoader::loadSRDFFromString(const std::string& content)
{
  srdf::ModelSharedPtr srdf(new srdf::Model());
  if (!srdf->initString(*urdf_, content))
  {
    RCLCPP_ERROR(LOGGER, "Unable to parse SRDF");
    return false;
  }
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
    RCLCPP_ERROR(LOGGER, "Path is empty");
    return false;
  }

  if (!boost::filesystem::exists(path))
  {
    RCLCPP_ERROR(LOGGER, "File does not exist");
    return false;
  }

  std::ifstream stream(path.c_str());
  if (!stream.good())
  {
    RCLCPP_ERROR(LOGGER, "Unable to load path");
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
  if (path.empty())
  {
    RCLCPP_ERROR(LOGGER, "Path is empty");
    return false;
  }

  if (!boost::filesystem::exists(path))
  {
    RCLCPP_ERROR(LOGGER, "File does not exist");
    return false;
  }

  std::string cmd = "ros2 run xacro xacro";
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
    RCLCPP_ERROR(LOGGER, "Unable to load path");
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
    RCLCPP_ERROR(LOGGER, "ament_index_cpp: %s", e.what());
    return false;
  }

  boost::filesystem::path path(package_path);
  // Use boost to cross-platform combine paths
  path = path / relative_path;

  return loadXmlFileToString(buffer, path.string(), xacro_args);
}

void RDFLoader::descriptionUpdateCallback(const std::string& new_description)
{
  if (external_description_update_cb_)
  {
    external_description_update_cb_(new_description);
  }
}

void RDFLoader::semanticUpdateCallback(const std::string& new_semantic)
{
  if (external_semantic_update_cb_)
  {
    external_semantic_update_cb_(new_semantic);
  }
}
}  // namespace rdf_loader
