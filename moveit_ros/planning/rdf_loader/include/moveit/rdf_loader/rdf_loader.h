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

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/rdf_loader/synchronized_string_parameter.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <rclcpp/rclcpp.hpp>

namespace rdf_loader
{
MOVEIT_CLASS_FORWARD(RDFLoader);  // Defines RDFLoaderPtr, ConstPtr, WeakPtr... etc

using NewModelCallback = std::function<void()>;

/** @class RDFLoader
 */
class RDFLoader
{
public:
  /** @brief Default constructor
   *
   *  Loads the URDF from a parameter given by the string argument,
   *  and the SRDF that has the same name + the "_semantic" suffix
   *
   *  If the parameter does not exist, attempt to subscribe to topics
   *  with the same name and type std_msgs::msg::String.
   *
   *  (specifying default_continuous_value/default_timeout allows users
   *   to specify values without setting ros parameters)
   *
   *  @param node ROS interface for parameters / topics
   *  @param ros_name The string name corresponding to the URDF
   *  @param default_continuous_value Default value for parameter with "_continuous" suffix.
   *  @param default_timeout Default value for parameter with "_timeout" suffix.
   */
  RDFLoader(const std::shared_ptr<rclcpp::Node>& node, const std::string& ros_name = "robot_description",
            bool default_continuous_value = false, double default_timeout = 10.0);

  /** @brief Initialize the robot model from a string representation of the URDF and SRDF documents */
  RDFLoader(const std::string& urdf_string, const std::string& srdf_string);

  /** @brief Get the resolved parameter name for the robot description */
  const std::string& getRobotDescription() const
  {
    return ros_name_;
  }

  /** @brief Get the parsed URDF model*/
  const urdf::ModelInterfaceSharedPtr& getURDF() const
  {
    return urdf_;
  }

  /** @brief Get the parsed SRDF model*/
  const srdf::ModelSharedPtr& getSRDF() const
  {
    return srdf_;
  }

  void setNewModelCallback(const NewModelCallback& cb)
  {
    new_model_cb_ = cb;
  }

  /** @brief determine if given path points to a xacro file */
  static bool isXacroFile(const std::string& path);

  /** @brief load file from given path into buffer */
  static bool loadFileToString(std::string& buffer, const std::string& path);

  /** @brief run xacro with the given args on the file, return result in buffer */
  static bool loadXacroFileToString(std::string& buffer, const std::string& path,
                                    const std::vector<std::string>& xacro_args);

  /** @brief helper that branches between loadFileToString() and loadXacroFileToString() based on result of
   * isXacroFile() */
  static bool loadXmlFileToString(std::string& buffer, const std::string& path,
                                  const std::vector<std::string>& xacro_args);

  /** @brief helper that generates a file path based on package name and relative file path to package */
  static bool loadPkgFileToString(std::string& buffer, const std::string& package_name,
                                  const std::string& relative_path, const std::vector<std::string>& xacro_args);

private:
  bool loadFromStrings();

  void urdfUpdateCallback(const std::string& new_urdf_string);
  void srdfUpdateCallback(const std::string& new_srdf_string);

  NewModelCallback new_model_cb_;

  std::string ros_name_;
  std::string urdf_string_, srdf_string_;

  SynchronizedStringParameter urdf_ssp_;
  SynchronizedStringParameter srdf_ssp_;

  srdf::ModelSharedPtr srdf_;
  urdf::ModelInterfaceSharedPtr urdf_;
};
}  // namespace rdf_loader
