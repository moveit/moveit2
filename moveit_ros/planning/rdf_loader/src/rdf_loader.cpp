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

// MoveIt!
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/profiler/profiler.h>

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// Boost
#include <boost/filesystem.hpp>

// C++
#include <fstream>
#include <streambuf>
#include <algorithm>
#include <chrono>

rclcpp::Logger LOGGER_RDF_LOADER = rclcpp::get_logger("moveit").get_child("rdf_loader");

rdf_loader::RDFLoader::RDFLoader(const std::string& robot_description)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RDFLoader(robot_description)");

  auto start = std::chrono::system_clock::now();
  // TODO (anasarrak): Add the correct node_handle name
  auto node = rclcpp::Node::make_shared("/");

  std::string content;

  auto desc_parameters = std::make_shared<rclcpp::SyncParametersClient>(node);

  if (!desc_parameters->has_parameter(robot_description))
  {
    RCLCPP_ERROR(LOGGER, "Robot model parameter not found! Did you remap '%s'?", robot_description.c_str());
    return;
  }

  urdf::Model* umodel = new urdf::Model();
  if (!umodel->initString(content))
  {
    RCLCPP_INFO(LOGGER_RDF_LOADER, "Unable to parse URDF from parameter '%s'", robot_description_.c_str());
    return;
  }
  urdf_.reset(umodel);

  const std::string srdf_description(robot_description_ + "_semantic");
  std::string scontent;

  if (!desc_parameters->has_parameter(srdf_description))
  {
    RCLCPP_ERROR(LOGGER, "Robot semantic description not found. Did you forget to define or remap '%s'?",
                    srdf_description.c_str());
    return;
  }

  srdf_.reset(new srdf::Model());
  if (!srdf_->initString(*urdf_, scontent))
  {
    RCLCPP_ERROR(LOGGER_RDF_LOADER, "Unable to parse SRDF from parameter '%s'", srdf_description.c_str());
    srdf_.reset();
    return;
  }

  RCLCPP_INFO(LOGGER_RDF_LOADER, "Loaded robot model in %d seconds",(std::chrono::system_clock::now() - start));
}

rdf_loader::RDFLoader::RDFLoader(const std::string& urdf_string, const std::string& srdf_string)
{
  moveit::tools::Profiler::ScopedStart prof_start;
  moveit::tools::Profiler::ScopedBlock prof_block("RDFLoader(string)");

  urdf::Model* umodel = new urdf::Model();
  urdf_.reset(umodel);
  if (umodel->initString(urdf_string))
  {
    srdf_.reset(new srdf::Model());
    if (!srdf_->initString(*urdf_, srdf_string))
    {
      RCLCPP_ERROR(LOGGER_RDF_LOADER, "Unable to parse SRDF");
      srdf_.reset();
    }
  }
  else
  {
    RCLCPP_ERROR(LOGGER_RDF_LOADER, "Unable to parse URDF");
    urdf_.reset();
  }
}

bool rdf_loader::RDFLoader::isXacroFile(const std::string& path)
{
  std::string lower_path = path;
  std::transform(lower_path.begin(), lower_path.end(), lower_path.begin(), ::tolower);

  return lower_path.find(".xacro") != std::string::npos;
}

bool rdf_loader::RDFLoader::loadFileToString(std::string& buffer, const std::string& path)
{
  if (path.empty())
  {
    RCLCPP_ERROR(LOGGER_RDF_LOADER, "Path is empty");
    return false;
  }

  if (!boost::filesystem::exists(path))
  {
    RCLCPP_ERROR(LOGGER_RDF_LOADER, "File does not exist");
    return false;
  }

  std::ifstream stream(path.c_str());
  if (!stream.good())
  {
    RCLCPP_ERROR(LOGGER_RDF_LOADER, "Unable to load path");
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

bool rdf_loader::RDFLoader::loadXacroFileToString(std::string& buffer, const std::string& path,
                                                  const std::vector<std::string>& xacro_args)
{
  if (path.empty())
  {
    RCLCPP_ERROR(LOGGER_RDF_LOADER, "Path is empty");
    return false;
  }

  if (!boost::filesystem::exists(path))
  {
    RCLCPP_ERROR(LOGGER_RDF_LOADER, "File does not exist");
    return false;
  }
  //TODO (anasarrak): Test ros2 xacro https://github.com/ros/xacro/tree/ros2 // https://github.com/bponsler/xacro
  std::string cmd = "ros2 run xacro xacro";
  for (std::vector<std::string>::const_iterator it = xacro_args.begin(); it != xacro_args.end(); ++it)
    cmd += *it + " ";
  cmd += path;

  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe)
  {
    RCLCPP_ERROR(LOGGER_RDF_LOADER, "Unable to load path");
    return false;
  }

  char pipe_buffer[128];
  while (!feof(pipe))
  {
    if (fgets(pipe_buffer, 128, pipe) != nullptr)
      buffer += pipe_buffer;
  }
  pclose(pipe);

  return true;
}

bool rdf_loader::RDFLoader::loadXmlFileToString(std::string& buffer, const std::string& path,
                                                const std::vector<std::string>& xacro_args)
{
  if (isXacroFile(path))
  {
    return loadXacroFileToString(buffer, path, xacro_args);
  }

  return loadFileToString(buffer, path);
}

bool rdf_loader::RDFLoader::loadPkgFileToString(std::string& buffer, const std::string& package_name,
                                                const std::string& relative_path,
                                                const std::vector<std::string>& xacro_args)
{
  std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
  if (package_path.empty())
  {
    RCLCPP_ERROR(LOGGER_RDF_LOADER, "ROS2 was unable to find the package name '%s'", package_name.c_str());
    return false;
  }

  boost::filesystem::path path(package_path);
  // Use boost to cross-platform combine paths
  path = path / relative_path;

  return loadXmlFileToString(buffer, path.string(), xacro_args);
}
