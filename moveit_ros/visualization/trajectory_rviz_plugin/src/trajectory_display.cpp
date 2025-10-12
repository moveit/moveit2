/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Wraps a trajectory_visualization playback class for Rviz into a stand alone display
*/

#include <moveit/trajectory_rviz_plugin/trajectory_display.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <moveit/utils/logger.hpp>

#include <memory>

namespace moveit_rviz_plugin
{

TrajectoryDisplay::TrajectoryDisplay() : Display(), logger_(moveit::getLogger("moveit.ros.trajectory_display"))

{
  // The robot description property is only needed when using the trajectory playback standalone (not within motion
  // planning plugin)
  robot_description_property_ = new rviz_common::properties::StringProperty(
      "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded",
      this, SLOT(changedRobotDescription()), this);

  trajectory_visual_ = std::make_shared<TrajectoryVisualization>(this, this);
}

TrajectoryDisplay::~TrajectoryDisplay() = default;

void TrajectoryDisplay::onInitialize()
{
  Display::onInitialize();

  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction)
  {
    RCLCPP_INFO(logger_, "Unable to lock weak_ptr from DisplayContext in TrajectoryDisplay constructor");
    return;
  }
  node_ = ros_node_abstraction->get_raw_node();
  trajectory_visual_->onInitialize(node_, scene_node_, context_);
}

void TrajectoryDisplay::loadRobotModel()
{
  try
  {
    rdf_loader_ = std::make_shared<rdf_loader::RDFLoader>(node_, robot_description_property_->getStdString());

    if (!rdf_loader_->getURDF())
    {
      setStatus(rviz_common::properties::StatusProperty::Error, "Robot Model",
                "Failed to load from parameter " + robot_description_property_->getString());
      return;
    }
    setStatus(rviz_common::properties::StatusProperty::Ok, "Robot Model", "Successfully loaded");

    const srdf::ModelSharedPtr& srdf =
        rdf_loader_->getSRDF() ? rdf_loader_->getSRDF() : std::make_shared<srdf::Model>();
    robot_model_ = std::make_shared<moveit::core::RobotModel>(rdf_loader_->getURDF(), srdf);

    // Send to child class
    trajectory_visual_->onRobotModelLoaded(robot_model_);
  }
  catch (std::exception& e)
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "RobotModel", QString("Loading failed: %1").arg(e.what()));
  }
}

void TrajectoryDisplay::reset()
{
  Display::reset();
  loadRobotModel();
  trajectory_visual_->reset();
}

void TrajectoryDisplay::load(const rviz_common::Config& config)
{
  // This property needs to be loaded in onEnable() below, which is triggered
  // in the beginning of Display::load() before the other property would be available
  robot_description_property_->load(config.mapGetChild("Robot Description"));
  Display::load(config);
}

void TrajectoryDisplay::onEnable()
{
  Display::onEnable();
  if (!rdf_loader_)
    loadRobotModel();
  trajectory_visual_->onEnable();
}

void TrajectoryDisplay::onDisable()
{
  Display::onDisable();
  trajectory_visual_->onDisable();
}

// For Rolling, L-turtle, and newer
#if RCLCPP_VERSION_GTE(30, 0, 0)
void TrajectoryDisplay::update(std::chrono::nanoseconds wall_dt, std::chrono::nanoseconds ros_dt)
{
  Display::update(wall_dt, ros_dt);
  trajectory_visual_->update(wall_dt, ros_dt);
}
// For Kilted and older
#else
void TrajectoryDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
  trajectory_visual_->update(wall_dt, ros_dt);
}
#endif

void TrajectoryDisplay::changedRobotDescription()
{
  if (isEnabled())
  {
    reset();
  }
  else
  {
    loadRobotModel();
  }
}

}  // namespace moveit_rviz_plugin
