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

/* Author: Ioan Sucan, Sachin Chitta */

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include "rclcpp/rclcpp.hpp"
#include <algorithm>

namespace moveit
{
namespace planning_interface
{
class PlanningSceneInterface::PlanningSceneInterfaceImpl
{
public:
  explicit PlanningSceneInterfaceImpl(std::shared_ptr<rclcpp::Node>& node, const std::string& ns = "")
  {
    node_ = node;
    planning_scene_service_ =
        node_->create_client<moveit_msgs::srv::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    apply_planning_scene_service_ =
        node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(move_group::APPLY_PLANNING_SCENE_SERVICE_NAME);

    planning_scene_diff_publisher_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
  }

  std::vector<std::string> getKnownObjectNames(bool with_type)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    auto response = std::make_shared<moveit_msgs::srv::GetPlanningScene::Response>();

    std::vector<std::string> result;
    request->components.components = request->components.WORLD_OBJECT_NAMES;
    auto res = planning_scene_service_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, res) != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      return result;
    }
    if (with_type)
    {
      for (std::size_t i = 0; i < response->scene.world.collision_objects.size(); ++i)
        if (!response->scene.world.collision_objects[i].type.key.empty())
          result.push_back(response->scene.world.collision_objects[i].id);
    }
    else
    {
      for (std::size_t i = 0; i < response->scene.world.collision_objects.size(); ++i)
        result.push_back(response->scene.world.collision_objects[i].id);
    }
    return result;
  }

  std::vector<std::string> getKnownObjectNamesInROI(double minx, double miny, double minz, double maxx, double maxy,
                                                    double maxz, bool with_type, std::vector<std::string>& types)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    auto response = std::make_shared<moveit_msgs::srv::GetPlanningScene::Response>();

    std::vector<std::string> result;
    request->components.components = request->components.WORLD_OBJECT_GEOMETRY;
    auto res = planning_scene_service_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, res) != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node_->get_logger(), "Could not call planning scene service to get object names");
      return result;
    }

    for (std::size_t i = 0; i < response->scene.world.collision_objects.size(); ++i)
    {
      if (with_type && response->scene.world.collision_objects[i].type.key.empty())
        continue;
      if (response->scene.world.collision_objects[i].mesh_poses.empty() &&
          response->scene.world.collision_objects[i].primitive_poses.empty())
        continue;
      bool good = true;
      for (std::size_t j = 0; j < response->scene.world.collision_objects[i].mesh_poses.size(); ++j)
        if (!(response->scene.world.collision_objects[i].mesh_poses[j].position.x >= minx &&
              response->scene.world.collision_objects[i].mesh_poses[j].position.x <= maxx &&
              response->scene.world.collision_objects[i].mesh_poses[j].position.y >= miny &&
              response->scene.world.collision_objects[i].mesh_poses[j].position.y <= maxy &&
              response->scene.world.collision_objects[i].mesh_poses[j].position.z >= minz &&
              response->scene.world.collision_objects[i].mesh_poses[j].position.z <= maxz))
        {
          good = false;
          break;
        }
      for (std::size_t j = 0; j < response->scene.world.collision_objects[i].primitive_poses.size(); ++j)
        if (!(response->scene.world.collision_objects[i].primitive_poses[j].position.x >= minx &&
              response->scene.world.collision_objects[i].primitive_poses[j].position.x <= maxx &&
              response->scene.world.collision_objects[i].primitive_poses[j].position.y >= miny &&
              response->scene.world.collision_objects[i].primitive_poses[j].position.y <= maxy &&
              response->scene.world.collision_objects[i].primitive_poses[j].position.z >= minz &&
              response->scene.world.collision_objects[i].primitive_poses[j].position.z <= maxz))
        {
          good = false;
          break;
        }
      if (good)
      {
        result.push_back(response->scene.world.collision_objects[i].id);
        if (with_type)
          types.push_back(response->scene.world.collision_objects[i].type.key);
      }
    }
    return result;
  }

  std::map<std::string, geometry_msgs::msg::Pose> getObjectPoses(const std::vector<std::string>& object_ids)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    auto response = std::make_shared<moveit_msgs::srv::GetPlanningScene::Response>();
    std::map<std::string, geometry_msgs::msg::Pose> result;
    request->components.components = request->components.WORLD_OBJECT_GEOMETRY;
    auto res = planning_scene_service_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, res) != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node_->get_logger(), "Could not call planning scene service to get object names");
      return result;
    }

    for (std::size_t i = 0; i < response->scene.world.collision_objects.size(); ++i)
    {
      if (std::find(object_ids.begin(), object_ids.end(), response->scene.world.collision_objects[i].id) !=
          object_ids.end())
      {
        if (response->scene.world.collision_objects[i].mesh_poses.empty() &&
            response->scene.world.collision_objects[i].primitive_poses.empty())
          continue;
        if (!response->scene.world.collision_objects[i].mesh_poses.empty())
          result[response->scene.world.collision_objects[i].id] =
              response->scene.world.collision_objects[i].mesh_poses[0];
        else
          result[response->scene.world.collision_objects[i].id] =
              response->scene.world.collision_objects[i].primitive_poses[0];
      }
    }
    return result;
  }

  std::map<std::string, moveit_msgs::msg::CollisionObject> getObjects(const std::vector<std::string>& object_ids)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    auto response = std::make_shared<moveit_msgs::srv::GetPlanningScene::Response>();
    std::map<std::string, moveit_msgs::msg::CollisionObject> result;
    request->components.components = request->components.WORLD_OBJECT_GEOMETRY;
    auto res = planning_scene_service_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, res) != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node_->get_logger(), "Could not call planning scene service to get object geometries");
      return result;
    }

    for (std::size_t i = 0; i < response->scene.world.collision_objects.size(); ++i)
    {
      if (object_ids.empty() ||
          std::find(object_ids.begin(), object_ids.end(), response->scene.world.collision_objects[i].id) !=
              object_ids.end())
      {
        result[response->scene.world.collision_objects[i].id] = response->scene.world.collision_objects[i];
      }
    }
    return result;
  }

  std::map<std::string, moveit_msgs::msg::AttachedCollisionObject>
  getAttachedObjects(const std::vector<std::string>& object_ids)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    auto response = std::make_shared<moveit_msgs::srv::GetPlanningScene::Response>();
    std::map<std::string, moveit_msgs::msg::AttachedCollisionObject> result;
    request->components.components = request->components.ROBOT_STATE_ATTACHED_OBJECTS;
    auto res = planning_scene_service_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, res) != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node_->get_logger(), "Could not call planning scene service to get attached object geometries");
      return result;
    }

    for (std::size_t i = 0; i < response->scene.robot_state.attached_collision_objects.size(); ++i)
    {
      if (object_ids.empty() ||
          std::find(object_ids.begin(), object_ids.end(),
                    response->scene.robot_state.attached_collision_objects[i].object.id) != object_ids.end())
      {
        result[response->scene.robot_state.attached_collision_objects[i].object.id] =
            response->scene.robot_state.attached_collision_objects[i];
      }
    }
    return result;
  }

  bool applyPlanningScene(const moveit_msgs::msg::PlanningScene& planning_scene)
  {
    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    auto response = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Response>();
    request->scene = planning_scene;
    auto res = apply_planning_scene_service_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, res) != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to call ApplyPlanningScene service");
      return false;
    }
    return response->success;
  }

  void addCollisionObjects(const std::vector<moveit_msgs::msg::CollisionObject>& collision_objects,
                           const std::vector<moveit_msgs::msg::ObjectColor>& object_colors) const
  {
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.world.collision_objects = collision_objects;
    planning_scene.object_colors = object_colors;

    for (size_t i = 0; i < planning_scene.object_colors.size(); ++i)
    {
      if (planning_scene.object_colors[i].id.empty() && i < collision_objects.size())
        planning_scene.object_colors[i].id = collision_objects[i].id;
      else
        break;
    }

    planning_scene.is_diff = true;
    planning_scene_diff_publisher_->publish(planning_scene);
  }

  void removeCollisionObjects(const std::vector<std::string>& object_ids) const
  {
    moveit_msgs::msg::PlanningScene planning_scene;
    moveit_msgs::msg::CollisionObject object;
    for (std::size_t i = 0; i < object_ids.size(); ++i)
    {
      object.id = object_ids[i];
      object.operation = object.REMOVE;
      planning_scene.world.collision_objects.push_back(object);
    }
    planning_scene.is_diff = true;
    planning_scene_diff_publisher_->publish(planning_scene);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr planning_scene_service_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_service_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
  robot_model::RobotModelConstPtr robot_model_;
};

PlanningSceneInterface::PlanningSceneInterface(std::shared_ptr<rclcpp::Node>& node, const std::string& ns)
{
  impl_ = new PlanningSceneInterfaceImpl(node, ns);
}

PlanningSceneInterface::~PlanningSceneInterface()
{
  delete impl_;
}

std::vector<std::string> PlanningSceneInterface::getKnownObjectNames(bool with_type)
{
  return impl_->getKnownObjectNames(with_type);
}

std::vector<std::string> PlanningSceneInterface::getKnownObjectNamesInROI(double minx, double miny, double minz,
                                                                          double maxx, double maxy, double maxz,
                                                                          bool with_type,
                                                                          std::vector<std::string>& types)
{
  return impl_->getKnownObjectNamesInROI(minx, miny, minz, maxx, maxy, maxz, with_type, types);
}

std::map<std::string, geometry_msgs::msg::Pose>
PlanningSceneInterface::getObjectPoses(const std::vector<std::string>& object_ids)
{
  return impl_->getObjectPoses(object_ids);
}

std::map<std::string, moveit_msgs::msg::CollisionObject>
PlanningSceneInterface::getObjects(const std::vector<std::string>& object_ids)
{
  return impl_->getObjects(object_ids);
}

std::map<std::string, moveit_msgs::msg::AttachedCollisionObject>
PlanningSceneInterface::getAttachedObjects(const std::vector<std::string>& object_ids)
{
  return impl_->getAttachedObjects(object_ids);
}

bool PlanningSceneInterface::applyCollisionObject(const moveit_msgs::msg::CollisionObject& collision_object)
{
  moveit_msgs::msg::PlanningScene ps;
  ps.robot_state.is_diff = true;
  ps.is_diff = true;
  ps.world.collision_objects.reserve(1);
  ps.world.collision_objects.push_back(collision_object);
  return applyPlanningScene(ps);
}

bool PlanningSceneInterface::applyCollisionObject(const moveit_msgs::msg::CollisionObject& collision_object,
                                                  const std_msgs::msg::ColorRGBA& object_color)
{
  moveit_msgs::msg::PlanningScene ps;
  ps.robot_state.is_diff = true;
  ps.is_diff = true;
  ps.world.collision_objects.reserve(1);
  ps.world.collision_objects.push_back(collision_object);
  moveit_msgs::msg::ObjectColor oc;
  oc.id = collision_object.id;
  oc.color = object_color;
  ps.object_colors.push_back(oc);
  return applyPlanningScene(ps);
}

bool PlanningSceneInterface::applyCollisionObjects(
    const std::vector<moveit_msgs::msg::CollisionObject>& collision_objects,
    const std::vector<moveit_msgs::msg::ObjectColor>& object_colors)
{
  moveit_msgs::msg::PlanningScene ps;
  ps.robot_state.is_diff = true;
  ps.is_diff = true;
  ps.world.collision_objects = collision_objects;
  ps.object_colors = object_colors;

  for (size_t i = 0; i < ps.object_colors.size(); ++i)
  {
    if (ps.object_colors[i].id.empty() && i < collision_objects.size())
      ps.object_colors[i].id = collision_objects[i].id;
    else
      break;
  }

  return applyPlanningScene(ps);
}

bool PlanningSceneInterface::applyAttachedCollisionObject(
    const moveit_msgs::msg::AttachedCollisionObject& collision_object)
{
  moveit_msgs::msg::PlanningScene ps;
  ps.robot_state.is_diff = true;
  ps.is_diff = true;
  ps.robot_state.attached_collision_objects.reserve(1);
  ps.robot_state.attached_collision_objects.push_back(collision_object);
  return applyPlanningScene(ps);
}

bool PlanningSceneInterface::applyAttachedCollisionObjects(
    const std::vector<moveit_msgs::msg::AttachedCollisionObject>& collision_objects)
{
  moveit_msgs::msg::PlanningScene ps;
  ps.robot_state.is_diff = true;
  ps.is_diff = true;
  ps.robot_state.attached_collision_objects = collision_objects;
  return applyPlanningScene(ps);
}

bool PlanningSceneInterface::applyPlanningScene(const moveit_msgs::msg::PlanningScene& ps)
{
  return impl_->applyPlanningScene(ps);
}

void PlanningSceneInterface::addCollisionObjects(
    const std::vector<moveit_msgs::msg::CollisionObject>& collision_objects,
    const std::vector<moveit_msgs::msg::ObjectColor>& object_colors) const
{
  impl_->addCollisionObjects(collision_objects, object_colors);
}

void PlanningSceneInterface::removeCollisionObjects(const std::vector<std::string>& object_ids) const
{
  impl_->removeCollisionObjects(object_ids);
}
}  // namespace planning_interface
}  // namespace moveit
