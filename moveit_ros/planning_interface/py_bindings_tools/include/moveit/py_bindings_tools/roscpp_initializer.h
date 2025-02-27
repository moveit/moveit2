/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#pragma once

#include <boost/python.hpp>
#include <string>

namespace moveit
{
/** \brief Tools for creating python bindings for MoveIt */
namespace py_bindings_tools
{
/** \brief The constructor of this class ensures that ros::init() has
    been called.  Thread safety and multiple initialization is
    properly handled. When the process terminates, ros::shutdown() is
    also called, if needed. */
class ROScppInitializer
{
public:
  ROScppInitializer();
  ROScppInitializer(boost::python::list& argv);
  ROScppInitializer(const std::string& node_name, boost::python::list& argv);
};

/** \brief This function can be used to specify the ROS command line arguments for the internal ROScpp instance;
    Usually this function would also be exposed in the py module that uses ROScppInitializer. */
void roscpp_set_arguments(const std::string& node_name, boost::python::list& argv);

/** \brief Initialize ROScpp with specified command line args */
void roscpp_init(const std::string& node_name, boost::python::list& argv);

/** \brief Initialize ROScpp with specified command line args */
void roscpp_init(boost::python::list& argv);

<<<<<<< HEAD:moveit_ros/planning_interface/py_bindings_tools/include/moveit/py_bindings_tools/roscpp_initializer.h
/** \brief Initialize ROScpp with default command line args */
void roscpp_init();
=======
  /// An error code reflecting what went wrong (if anything)
  moveit_msgs::msg::MoveItErrorCodes error_code;

  planning_scene::PlanningScenePtr copyPlanningScene()
  {
    // planning_scene_ is based on the scene from this monitor
    // (either it's the monitored scene or a diff on top of it)
    // so in order to copy the scene, we must also lock the underlying monitor
    planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor);
    return planning_scene::PlanningScene::clone(planning_scene);
  }
};
>>>>>>> e2b24f5ac (Ports moveit1 #3689 (#3357)):moveit_ros/planning/plan_execution/include/moveit/plan_execution/plan_representation.hpp

void roscpp_shutdown();
}  // namespace py_bindings_tools
}  // namespace moveit
