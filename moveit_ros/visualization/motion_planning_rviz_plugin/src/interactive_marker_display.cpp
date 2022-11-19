/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_common/display_context.hpp>

#include <rviz_default_plugins/displays/interactive_markers/interactive_marker_namespace_property.hpp>

#include <moveit/motion_planning_rviz_plugin/interactive_marker_display.h>

namespace rviz_default_plugins
{
namespace displays
{
bool validateFloats(const visualization_msgs::msg::InteractiveMarker& msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.pose);
  valid = valid && rviz_common::validateFloats(msg.scale);
  for (const auto& control : msg.controls)
  {
    valid = valid && rviz_common::validateFloats(control.orientation);
    for (const auto& marker : control.markers)
    {
      valid = valid && rviz_common::validateFloats(marker.pose);
      valid = valid && rviz_common::validateFloats(marker.scale);
      valid = valid && rviz_common::validateFloats(marker.color);
      valid = valid && rviz_common::validateFloats(marker.points);
    }
  }
  return valid;
}

InteractiveMarkerDisplay::InteractiveMarkerDisplay()
{
  interactive_marker_namespace_property_ =
      new InteractiveMarkerNamespaceProperty("Interactive Markers Namespace", "",
                                             "Namespace of the interactive marker server to connect to.", this,
                                             SLOT(namespaceChanged()));

  show_descriptions_property_ =
      new rviz_common::properties::BoolProperty("Show Descriptions", true,
                                                "Whether or not to show the descriptions of each Interactive Marker.",
                                                this, SLOT(updateShowDescriptions()));

  show_axes_property_ = new rviz_common::properties::BoolProperty(
      "Show Axes", false, "Whether or not to show the axes of each Interactive Marker.", this, SLOT(updateShowAxes()));

  show_visual_aids_property_ = new rviz_common::properties::BoolProperty(
      "Show Visual Aids", false, "Whether or not to show visual helpers while moving/rotating Interactive Markers.",
      this, SLOT(updateShowVisualAids()));

  enable_transparency_property_ = new rviz_common::properties::BoolProperty(
      "Enable Transparency", true,
      "Whether or not to allow transparency for auto-completed markers (e.g. rings and arrows).", this,
      SLOT(updateEnableTransparency()));
}

void InteractiveMarkerDisplay::onInitialize()
{
  //  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  //  if (!ros_node_abstraction) {
  //    return;
  //  }
  //
  //  interactive_marker_namespace_property_->initialize(ros_node_abstraction);
  //
  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r",
                      "__node:=" + std::string("interactive_marker_display_") +
                          std::to_string(reinterpret_cast<std::size_t>(this)) });
  pnode_ = rclcpp::Node::make_shared("_", "", options);
  private_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  private_executor_->add_node(pnode_);
  // start executor on a different thread now
  private_executor_thread_ = std::thread([this]() { private_executor_->spin(); });
  auto frame_transformer = context_->getFrameManager()->getTransformer();
  //  rclcpp::Node::SharedPtr node = ros_node_abstraction->get_raw_node();
  interactive_marker_client_ = std::make_unique<interactive_markers::InteractiveMarkerClient>(
      pnode_, frame_transformer, fixed_frame_.toStdString());

  interactive_marker_client_->setInitializeCallback(
      [this](const visualization_msgs::srv::GetInteractiveMarkers::Response::SharedPtr& msg) {
        return initializeCallback(msg);
      });
  interactive_marker_client_->setUpdateCallback(
      [this](const visualization_msgs::msg::InteractiveMarkerUpdate::ConstSharedPtr& msg) {
        return updateCallback(msg);
      });
  interactive_marker_client_->setResetCallback([this]() { return resetCallback(); });
  interactive_marker_client_->setStatusCallback(
      [this](interactive_markers::InteractiveMarkerClient::Status status, const std::string& message) {
        return statusCallback(status, message);
      });

  subscribe();
}

void InteractiveMarkerDisplay::onEnable()
{
  subscribe();
}

void InteractiveMarkerDisplay::onDisable()
{
  unsubscribe();
}

void InteractiveMarkerDisplay::namespaceChanged()
{
  unsubscribe();

  if (interactive_marker_namespace_property_->isEmpty())
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Interactive Marker Client",
              QString("Error connecting: empty namespace"));
    return;
  }

  subscribe();
}

void InteractiveMarkerDisplay::subscribe()
{
  const std::string topic_namespace = interactive_marker_namespace_property_->getNamespaceStd();
  if (isEnabled() && !topic_namespace.empty())
  {
    interactive_marker_client_->connect(topic_namespace);
  }
}

void InteractiveMarkerDisplay::publishFeedback(visualization_msgs::msg::InteractiveMarkerFeedback& feedback)
{
  interactive_marker_client_->publishFeedback(feedback);
}

void InteractiveMarkerDisplay::onStatusUpdate(rviz_common::properties::StatusProperty::Level level,
                                              const std::string& name, const std::string& text)
{
  setStatusStd(level, name, text);
}

void InteractiveMarkerDisplay::unsubscribe()
{
  if (interactive_marker_client_)
  {
    interactive_marker_client_->disconnect();
  }
  Display::reset();
}

void InteractiveMarkerDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  interactive_marker_client_->update();

  for (const auto& name_marker_pair : interactive_markers_map_)
  {
    name_marker_pair.second->update();
  }
}

void InteractiveMarkerDisplay::updateMarkers(const std::vector<visualization_msgs::msg::InteractiveMarker>& markers)
{
  for (const visualization_msgs::msg::InteractiveMarker& marker : markers)
  {
    if (!validateFloats(marker))
    {
      setStatusStd(rviz_common::properties::StatusProperty::Error, marker.name, "Marker contains invalid floats!");
      continue;
    }
    RVIZ_COMMON_LOG_DEBUG_STREAM("Processing interactive marker '" << marker.name << "'. " << marker.controls.size());

    auto int_marker_entry = interactive_markers_map_.find(marker.name);

    if (int_marker_entry == interactive_markers_map_.end())
    {
      int_marker_entry =
          interactive_markers_map_
              .insert(std::make_pair(marker.name, std::make_shared<InteractiveMarker>(getSceneNode(), context_)))
              .first;
      connect(int_marker_entry->second.get(), SIGNAL(userFeedback(visualization_msgs::msg::InteractiveMarkerFeedback&)),
              this, SLOT(publishFeedback(visualization_msgs::msg::InteractiveMarkerFeedback&)));
      connect(
          int_marker_entry->second.get(),
          SIGNAL(statusUpdate(rviz_common::properties::StatusProperty::Level, const std::string&, const std::string&)),
          this,
          SLOT(onStatusUpdate(rviz_common::properties::StatusProperty::Level, const std::string&, const std::string&)));
    }

    if (int_marker_entry->second->processMessage(marker))
    {
      int_marker_entry->second->setShowAxes(show_axes_property_->getBool());
      int_marker_entry->second->setShowVisualAids(show_visual_aids_property_->getBool());
      int_marker_entry->second->setShowDescription(show_descriptions_property_->getBool());
    }
    else
    {
      unsubscribe();
      return;
    }
  }
}

void InteractiveMarkerDisplay::eraseAllMarkers()
{
  interactive_markers_map_.clear();
  deleteStatusStd("Interactive Marker Client");
}

void InteractiveMarkerDisplay::eraseMarkers(const std::vector<std::string>& erases)
{
  for (const std::string& marker_name : erases)
  {
    interactive_markers_map_.erase(marker_name);
    deleteStatusStd(marker_name);
  }
}

void InteractiveMarkerDisplay::updatePoses(
    const std::vector<visualization_msgs::msg::InteractiveMarkerPose>& marker_poses)
{
  for (const visualization_msgs::msg::InteractiveMarkerPose& marker_pose : marker_poses)
  {
    if (!rviz_common::validateFloats(marker_pose.pose))
    {
      setStatusStd(rviz_common::properties::StatusProperty::Error, marker_pose.name,
                   "Pose message contains invalid floats!");
      return;
    }

    auto int_marker_entry = interactive_markers_map_.find(marker_pose.name);

    if (int_marker_entry != interactive_markers_map_.end())
    {
      int_marker_entry->second->processMessage(marker_pose);
    }
    else
    {
      setStatusStd(rviz_common::properties::StatusProperty::Error, marker_pose.name,
                   "Pose received for non-existing marker '" + marker_pose.name);
      unsubscribe();
      return;
    }
  }
}

void InteractiveMarkerDisplay::initializeCallback(
    const visualization_msgs::srv::GetInteractiveMarkers::Response::SharedPtr& msg)
{
  eraseAllMarkers();
  updateMarkers(msg->markers);
}

void InteractiveMarkerDisplay::updateCallback(const visualization_msgs::msg::InteractiveMarkerUpdate::ConstSharedPtr& msg)
{
  updateMarkers(msg->markers);
  updatePoses(msg->poses);
  eraseMarkers(msg->erases);
}

void InteractiveMarkerDisplay::resetCallback()
{
  eraseAllMarkers();
  deleteStatusStd("Interactive Marker Client");
}

void InteractiveMarkerDisplay::statusCallback(interactive_markers::InteractiveMarkerClient::Status status,
                                              const std::string& message)
{
  rviz_common::properties::StatusProperty::Level rviz_level;
  switch (status)
  {
    case interactive_markers::InteractiveMarkerClient::STATUS_DEBUG:
      rviz_level = rviz_common::properties::StatusProperty::Ok;
      break;
    case interactive_markers::InteractiveMarkerClient::STATUS_INFO:
      rviz_level = rviz_common::properties::StatusProperty::Ok;
      break;
    case interactive_markers::InteractiveMarkerClient::STATUS_WARN:
      rviz_level = rviz_common::properties::StatusProperty::Warn;
      break;
    case interactive_markers::InteractiveMarkerClient::STATUS_ERROR:
      rviz_level = rviz_common::properties::StatusProperty::Error;
      break;
    default:
      RVIZ_COMMON_LOG_WARNING("Unexpected status level from interactive marker client received");
      rviz_level = rviz_common::properties::StatusProperty::Error;
  }
  setStatusStd(rviz_level, "Interactive Marker Client", message);
}

void InteractiveMarkerDisplay::fixedFrameChanged()
{
  if (interactive_marker_client_)
  {
    interactive_marker_client_->setTargetFrame(fixed_frame_.toStdString());
  }
  Display::reset();
}

void InteractiveMarkerDisplay::reset()
{
  Display::reset();
  unsubscribe();
  subscribe();
}

void InteractiveMarkerDisplay::updateShowDescriptions()
{
  bool show = show_descriptions_property_->getBool();

  for (const auto& name_marker_pair : interactive_markers_map_)
  {
    name_marker_pair.second->setShowDescription(show);
  }
}

void InteractiveMarkerDisplay::updateShowAxes()
{
  bool show = show_axes_property_->getBool();

  for (const auto& name_marker_pair : interactive_markers_map_)
  {
    name_marker_pair.second->setShowAxes(show);
  }
}

void InteractiveMarkerDisplay::updateShowVisualAids()
{
  bool show = show_visual_aids_property_->getBool();

  for (const auto& name_marker_pair : interactive_markers_map_)
  {
    name_marker_pair.second->setShowVisualAids(show);
  }
}

void InteractiveMarkerDisplay::updateEnableTransparency()
{
  // This is not very efficient, but works
  unsubscribe();
  interactive_marker_client_->setEnableAutocompleteTransparency(enable_transparency_property_->getBool());
  subscribe();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT(build/include_order)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::InteractiveMarkerDisplay, rviz_common::Display)
