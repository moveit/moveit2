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

/* Author: Sachin Chitta */

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <object_recognition_msgs/action/object_recognition.hpp>

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros_visualization.motion_planning_frame_manipulation");

/////////////// Object Detection ///////////////////////
void MotionPlanningFrame::detectObjectsButtonClicked()
{
  // TODO (ddengster): Enable when moveit_ros_perception is ported
  //  if (!semantic_world_)
  //  {
  //    const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
  //    if (ps)
  //    {
  //      semantic_world_ = std::make_shared<moveit::semantic_world::SemanticWorld>(ps);
  //    }
  //    if (semantic_world_)
  //    {
  //      semantic_world_->addTableCallback([this] { updateTables(); });
  //    }
  //  }
  planning_display_->addBackgroundJob([this] { triggerObjectDetection(); }, "detect objects");
}

void MotionPlanningFrame::processDetectedObjects()
{
  pick_object_name_.clear();

  std::vector<std::string> object_ids;
  double min_x = ui_->roi_center_x->value() - ui_->roi_size_x->value() / 2.0;
  double min_y = ui_->roi_center_y->value() - ui_->roi_size_y->value() / 2.0;
  double min_z = ui_->roi_center_z->value() - ui_->roi_size_z->value() / 2.0;

  double max_x = ui_->roi_center_x->value() + ui_->roi_size_x->value() / 2.0;
  double max_y = ui_->roi_center_y->value() + ui_->roi_size_y->value() / 2.0;
  double max_z = ui_->roi_center_z->value() + ui_->roi_size_z->value() / 2.0;

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (object_ids.empty() && (rclcpp::Clock().now() - start_time) <= rclcpp::Duration::from_seconds(3))
  {
    // collect all objects in region of interest
    {
      const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
      const collision_detection::WorldConstPtr& world = ps->getWorld();
      object_ids.clear();
      for (const auto& object : *world)
      {
        const auto& position = object.second->pose_.translation();
        if (position.x() >= min_x && position.x() <= max_x && position.y() >= min_y && position.y() <= max_y &&
            position.z() >= min_z && position.z() <= max_z)
        {
          object_ids.push_back(object.first);
        }
      }
      if (!object_ids.empty())
        break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_DEBUG(LOGGER, "Found %d objects", static_cast<int>(object_ids.size()));
  updateDetectedObjectsList(object_ids);
}

void MotionPlanningFrame::selectedDetectedObjectChanged()
{
  QList<QListWidgetItem*> sel = ui_->detected_objects_list->selectedItems();
  if (sel.empty())
  {
    RCLCPP_INFO(LOGGER, "No objects to select");
    return;
  }
  planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
  std_msgs::msg::ColorRGBA pick_object_color;
  pick_object_color.r = 1.0;
  pick_object_color.g = 0.0;
  pick_object_color.b = 0.0;
  pick_object_color.a = 1.0;

  if (ps)
  {
    if (!selected_object_name_.empty())
      ps->removeObjectColor(selected_object_name_);
    selected_object_name_ = sel[0]->text().toStdString();
    ps->setObjectColor(selected_object_name_, pick_object_color);
  }
}

void MotionPlanningFrame::detectedObjectChanged(QListWidgetItem* /*item*/)
{
}

void MotionPlanningFrame::triggerObjectDetection()
{
  if (!object_recognition_client_)
  {
    object_recognition_client_ = rclcpp_action::create_client<object_recognition_msgs::action::ObjectRecognition>(
        node_, OBJECT_RECOGNITION_ACTION);
    if (!object_recognition_client_->wait_for_action_server(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(LOGGER, "Object recognition action server not responsive");
      return;
    }
  }

  object_recognition_msgs::action::ObjectRecognition::Goal goal;
  auto goal_handle_future = object_recognition_client_->async_send_goal(goal);
  goal_handle_future.wait();
  if (goal_handle_future.get()->get_status() != rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
  {
    RCLCPP_ERROR(LOGGER, "ObjectRecognition client: send goal call failed");
    return;
  }
}

void MotionPlanningFrame::listenDetectedObjects(
    const object_recognition_msgs::msg::RecognizedObjectArray::ConstSharedPtr /*msg*/)
{
  rclcpp::sleep_for(std::chrono::seconds(1));
  planning_display_->addMainLoopJob([this] { processDetectedObjects(); });
}

void MotionPlanningFrame::updateDetectedObjectsList(const std::vector<std::string>& object_ids)
{
  ui_->detected_objects_list->setUpdatesEnabled(false);
  bool old_state = ui_->detected_objects_list->blockSignals(true);
  ui_->detected_objects_list->clear();
  {
    for (std::size_t i = 0; i < object_ids.size(); ++i)
    {
      QListWidgetItem* item =
          new QListWidgetItem(QString::fromStdString(object_ids[i]), ui_->detected_objects_list, static_cast<int>(i));
      item->setToolTip(item->text());
      Qt::ItemFlags flags = item->flags();
      flags &= ~(Qt::ItemIsUserCheckable);
      item->setFlags(flags);
      ui_->detected_objects_list->addItem(item);
    }
  }
  ui_->detected_objects_list->blockSignals(old_state);
  ui_->detected_objects_list->setUpdatesEnabled(true);
  if (!object_ids.empty())
    ui_->pick_button->setEnabled(true);
}

/////////////////////// Support Surfaces ///////////////////////
void MotionPlanningFrame::updateTables()
{
  RCLCPP_DEBUG(LOGGER, "Update table callback");
  planning_display_->addBackgroundJob([this] { publishTables(); }, "publish tables");
}

void MotionPlanningFrame::publishTables()
{
  // TODO (ddengster): Enable when moveit_ros_perception is ported
  // semantic_world_->addTablesToCollisionWorld();

  planning_display_->addMainLoopJob([this] { updateSupportSurfacesList(); });
}

void MotionPlanningFrame::selectedSupportSurfaceChanged()
{
  QList<QListWidgetItem*> sel = ui_->support_surfaces_list->selectedItems();
  if (sel.empty())
  {
    RCLCPP_INFO(LOGGER, "No tables to select");
    return;
  }
  planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
  std_msgs::msg::ColorRGBA selected_support_surface_color;
  selected_support_surface_color.r = 0.0;
  selected_support_surface_color.g = 0.0;
  selected_support_surface_color.b = 1.0;
  selected_support_surface_color.a = 1.0;

  if (ps)
  {
    if (!selected_support_surface_name_.empty())
      ps->removeObjectColor(selected_support_surface_name_);
    selected_support_surface_name_ = sel[0]->text().toStdString();
    ps->setObjectColor(selected_support_surface_name_, selected_support_surface_color);
  }
}

void MotionPlanningFrame::updateSupportSurfacesList()
{
  // TODO (ddengster): Enable when moveit_ros_perception is ported
  // double min_x = ui_->roi_center_x->value() - ui_->roi_size_x->value() / 2.0;
  // double min_y = ui_->roi_center_y->value() - ui_->roi_size_y->value() / 2.0;
  // double min_z = ui_->roi_center_z->value() - ui_->roi_size_z->value() / 2.0;

  // double max_x = ui_->roi_center_x->value() + ui_->roi_size_x->value() / 2.0;
  // double max_y = ui_->roi_center_y->value() + ui_->roi_size_y->value() / 2.0;
  // double max_z = ui_->roi_center_z->value() + ui_->roi_size_z->value() / 2.0;
  // std::vector<std::string> support_ids = semantic_world_->getTableNamesInROI(min_x, min_y, min_z, max_x, max_y,
  // max_z);
  std::vector<std::string> support_ids;
  RCLCPP_INFO(LOGGER, "%d Tables in collision world", static_cast<int>(support_ids.size()));

  ui_->support_surfaces_list->setUpdatesEnabled(false);
  bool old_state = ui_->support_surfaces_list->blockSignals(true);
  ui_->support_surfaces_list->clear();
  {
    for (std::size_t i = 0; i < support_ids.size(); ++i)
    {
      QListWidgetItem* item =
          new QListWidgetItem(QString::fromStdString(support_ids[i]), ui_->support_surfaces_list, static_cast<int>(i));
      item->setToolTip(item->text());
      Qt::ItemFlags flags = item->flags();
      flags &= ~(Qt::ItemIsUserCheckable);
      item->setFlags(flags);
      ui_->support_surfaces_list->addItem(item);
    }
  }
  ui_->support_surfaces_list->blockSignals(old_state);
  ui_->support_surfaces_list->setUpdatesEnabled(true);
}

/////////////////////////////// Pick & Place /////////////////////////////////
void MotionPlanningFrame::pickObjectButtonClicked()
{
  RCLCPP_WARN_STREAM(LOGGER, "Pick & Place capability isn't supported yet");
  //  QList<QListWidgetItem*> sel = ui_->detected_objects_list->selectedItems();
  //  QList<QListWidgetItem*> sel_table = ui_->support_surfaces_list->selectedItems();
  //
  //  std::string group_name = planning_display_->getCurrentPlanningGroup();
  //  if (sel.empty())
  //  {
  //    RCLCPP_INFO(LOGGER, "No objects to pick");
  //    return;
  //  }
  //  pick_object_name_[group_name] = sel[0]->text().toStdString();
  //
  //  if (!sel_table.empty())
  //    support_surface_name_ = sel_table[0]->text().toStdString();
  //  else
  //  {
  //    if (semantic_world_)
  //    {
  //      const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
  //      if (ps->getWorld()->hasObject(pick_object_name_[group_name]))
  //      {
  //        geometry_msgs::msg::Pose
  //        object_pose(tf2::toMsg(ps->getWorld()->getTransform(pick_object_name_[group_name])));
  //        RCLCPP_DEBUG(LOGGER, "Finding current table for object: " << pick_object_name_[group_name]);
  //        support_surface_name_ = semantic_world_->findObjectTable(object_pose);
  //      }
  //      else
  //        support_surface_name_.clear();
  //    }
  //    else
  //      support_surface_name_.clear();
  //  }
  //  RCLCPP_INFO(LOGGER, "Trying to pick up object %s from support surface %s", pick_object_name_[group_name].c_str(),
  //           support_surface_name_.c_str());
  //  planning_display_->addBackgroundJob([this] { pickObject(); }, "pick");
}

void MotionPlanningFrame::placeObjectButtonClicked()
{
  QList<QListWidgetItem*> sel_table = ui_->support_surfaces_list->selectedItems();
  std::string group_name = planning_display_->getCurrentPlanningGroup();

  if (!sel_table.empty())
    support_surface_name_ = sel_table[0]->text().toStdString();
  else
  {
    support_surface_name_.clear();
    RCLCPP_ERROR(LOGGER, "Need to specify table to place object on");
    return;
  }

  ui_->pick_button->setEnabled(false);
  ui_->place_button->setEnabled(false);

  std::vector<const moveit::core::AttachedBody*> attached_bodies;
  const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
  if (!ps)
  {
    RCLCPP_ERROR(LOGGER, "No planning scene");
    return;
  }
  const moveit::core::JointModelGroup* jmg = ps->getCurrentState().getJointModelGroup(group_name);
  if (jmg)
    ps->getCurrentState().getAttachedBodies(attached_bodies, jmg);

  if (attached_bodies.empty())
  {
    RCLCPP_ERROR(LOGGER, "No bodies to place");
    return;
  }

  geometry_msgs::msg::Quaternion upright_orientation;
  upright_orientation.w = 1.0;

  // Else place the first one
  place_poses_.clear();
  // TODO (ddengster): Enable when moveit_ros_perception is ported
  //  place_poses_ = semantic_world_->generatePlacePoses(support_surface_name_, attached_bodies[0]->getShapes()[0],
  //                                                     upright_orientation, 0.1);
  //  planning_display_->visualizePlaceLocations(place_poses_);
  //  place_object_name_ = attached_bodies[0]->getName();
  //  planning_display_->addBackgroundJob([this] { placeObject(); }, "place");
}

// void MotionPlanningFrame::pickObject()
//{
//  std::string group_name = planning_display_->getCurrentPlanningGroup();
//  ui_->pick_button->setEnabled(false);
//  if (pick_object_name_.find(group_name) == pick_object_name_.end())
//  {
//    RCLCPP_ERROR(LOGGER, "No pick object set for this group");
//    return;
//  }
//  if (!support_surface_name_.empty())
//  {
//    move_group_->setSupportSurfaceName(support_surface_name_);
//  }
//  if (move_group_->pick(pick_object_name_[group_name]))
//  {
//    ui_->place_button->setEnabled(true);
//  }
//}
//
// void MotionPlanningFrame::placeObject()
//  move_group_->place(place_object_name_, place_poses_);
//  return;
//}
}  // namespace moveit_rviz_plugin
