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
#include <algorithm>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <moveit/utils/logger.hpp>

namespace moveit
{
namespace planning_interface
{

class PlanningSceneInterface::PlanningSceneInterfaceImpl
{
public:
  explicit PlanningSceneInterfaceImpl(const std::string& ns = "", bool wait = true)
  {
    rclcpp::NodeOptions options;
    options.arguments({ "--ros-args", "-r",
                        "__node:=" + std::string("planning_scene_interface_") +
                            std::to_string(reinterpret_cast<std::size_t>(this)) });
    node_ = rclcpp::Node::make_shared("_", ns, options);
    moveit::setNodeLoggerName(node_->get_name());
    planning_scene_diff_publisher_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
    planning_scene_service_ =
        node_->create_client<moveit_msgs::srv::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    apply_planning_scene_service_ =
        node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(move_group::APPLY_PLANNING_SCENE_SERVICE_NAME);

    if (wait)
    {
      waitForService(std::static_pointer_cast<rclcpp::ClientBase>(planning_scene_service_));
      waitForService(std::static_pointer_cast<rclcpp::ClientBase>(apply_planning_scene_service_));
    }
  }

  std::vector<std::string> getKnownObjectNames(bool with_type)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    moveit_msgs::srv::GetPlanningScene::Response::SharedPtr response;
    std::vector<std::string> result;
    request->components.components = request->components.WORLD_OBJECT_NAMES;

    auto res = planning_scene_service_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, res) != rclcpp::FutureReturnCode::SUCCESS)
    {
      return result;
    }
    response = res.get();

    if (with_type)
    {
      for (const moveit_msgs::msg::CollisionObject& collision_object : response->scene.world.collision_objects)
      {
        if (!collision_object.type.key.empty())
          result.push_back(collision_object.id);
      }
    }
    else
    {
      for (const moveit_msgs::msg::CollisionObject& collision_object : response->scene.world.collision_objects)
        result.push_back(collision_object.id);
    }
    return result;
  }

  std::vector<std::string> getKnownObjectNamesInROI(double minx, double miny, double minz, double maxx, double maxy,
                                                    double maxz, bool with_type, std::vector<std::string>& types)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    moveit_msgs::srv::GetPlanningScene::Response::SharedPtr response;
    std::vector<std::string> result;
    request->components.components = request->components.WORLD_OBJECT_GEOMETRY;

    auto res = planning_scene_service_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, res) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node_->get_logger(), "Could not call planning scene service to get object names");
      return result;
    }
    response = res.get();

    for (const moveit_msgs::msg::CollisionObject& collision_object : response->scene.world.collision_objects)
    {
      if (with_type && collision_object.type.key.empty())
        continue;
      if (collision_object.mesh_poses.empty() && collision_object.primitive_poses.empty())
        continue;
      bool good = true;
      for (const geometry_msgs::msg::Pose& mesh_pose : collision_object.mesh_poses)
      {
        if (mesh_pose.position.x < minx || mesh_pose.position.x > maxx || mesh_pose.position.y < miny ||
            mesh_pose.position.y > maxy || mesh_pose.position.z < minz || mesh_pose.position.z > maxz)
        {
          good = false;
          break;
        }
      }
      for (const geometry_msgs::msg::Pose& primitive_pose : collision_object.primitive_poses)
      {
        if (primitive_pose.position.x < minx || primitive_pose.position.x > maxx || primitive_pose.position.y < miny ||
            primitive_pose.position.y > maxy || primitive_pose.position.z < minz || primitive_pose.position.z > maxz)
        {
          good = false;
          break;
        }
      }
      if (good)
      {
        result.push_back(collision_object.id);
        if (with_type)
          types.push_back(collision_object.type.key);
      }
    }
    return result;
  }

  std::map<std::string, geometry_msgs::msg::Pose> getObjectPoses(const std::vector<std::string>& object_ids)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    moveit_msgs::srv::GetPlanningScene::Response::SharedPtr response;
    std::map<std::string, geometry_msgs::msg::Pose> result;
    request->components.components = request->components.WORLD_OBJECT_GEOMETRY;

    auto res = planning_scene_service_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, res) == rclcpp::FutureReturnCode::SUCCESS)
    {
      response = res.get();
      for (const moveit_msgs::msg::CollisionObject& collision_object : response->scene.world.collision_objects)
      {
        if (std::find(object_ids.begin(), object_ids.end(), collision_object.id) != object_ids.end())
        {
          result[collision_object.id] = collision_object.pose;
        }
      }
    }
    return result;
  }

  std::map<std::string, moveit_msgs::msg::CollisionObject> getObjects(const std::vector<std::string>& object_ids)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    moveit_msgs::srv::GetPlanningScene::Response::SharedPtr response;
    std::map<std::string, moveit_msgs::msg::CollisionObject> result;
    request->components.components = request->components.WORLD_OBJECT_GEOMETRY;

    auto res = planning_scene_service_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, res) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node_->get_logger(), "Could not call planning scene service to get object geometries");
      return result;
    }
    response = res.get();

    for (const moveit_msgs::msg::CollisionObject& collision_object : response->scene.world.collision_objects)
    {
      if (object_ids.empty() || std::find(object_ids.begin(), object_ids.end(), collision_object.id) != object_ids.end())
      {
        result[collision_object.id] = collision_object;
      }
    }
    return result;
  }

  std::map<std::string, moveit_msgs::msg::AttachedCollisionObject>
  getAttachedObjects(const std::vector<std::string>& object_ids)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    moveit_msgs::srv::GetPlanningScene::Response::SharedPtr response;
    std::map<std::string, moveit_msgs::msg::AttachedCollisionObject> result;
    request->components.components = request->components.ROBOT_STATE_ATTACHED_OBJECTS;

    auto res = planning_scene_service_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, res) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node_->get_logger(), "Could not call planning scene service to get attached object geometries");
      return result;
    }
    response = res.get();

    for (const moveit_msgs::msg::AttachedCollisionObject& attached_collision_object :
         response->scene.robot_state.attached_collision_objects)
    {
      if (object_ids.empty() ||
          std::find(object_ids.begin(), object_ids.end(), attached_collision_object.object.id) != object_ids.end())
      {
        result[attached_collision_object.object.id] = attached_collision_object;
      }
    }
    return result;
  }

  bool applyPlanningScene(const moveit_msgs::msg::PlanningScene& planning_scene)
  {
    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    moveit_msgs::srv::ApplyPlanningScene::Response::SharedPtr response;
    request->scene = planning_scene;

    auto res = apply_planning_scene_service_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, res) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to call ApplyPlanningScene service");
    }
    response = res.get();
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
      {
        planning_scene.object_colors[i].id = collision_objects[i].id;
      }
      else
      {
        break;
      }
    }

    planning_scene.is_diff = true;
    planning_scene_diff_publisher_->publish(planning_scene);
  }

  void removeCollisionObjects(const std::vector<std::string>& object_ids) const
  {
    moveit_msgs::msg::PlanningScene planning_scene;
    moveit_msgs::msg::CollisionObject object;
    for (const std::string& object_id : object_ids)
    {
      object.id = object_id;
      object.operation = object.REMOVE;
      planning_scene.world.collision_objects.push_back(object);
    }
    planning_scene.is_diff = true;
    planning_scene_diff_publisher_->publish(planning_scene);
  }

private:
  void waitForService(const std::shared_ptr<rclcpp::ClientBase>& srv)
  {
    // rclcpp::Duration time_before_warning(5.0);
    // srv.waitForExistence(time_before_warning);
    std::chrono::duration<double> d(5.0);
    srv->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(d));
    if (!srv->service_is_ready())
    {
      RCLCPP_WARN_STREAM(node_->get_logger(),
                         "service '" << srv->get_service_name() << "' not advertised yet. Continue waiting...");
      srv->wait_for_service();
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr planning_scene_service_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_service_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
  moveit::core::RobotModelConstPtr robot_model_;
};

PlanningSceneInterface::PlanningSceneInterface(const std::string& ns, bool wait)
{
  impl_ = new PlanningSceneInterfaceImpl(ns, wait);
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
    {
      ps.object_colors[i].id = collision_objects[i].id;
    }
    else
    {
      break;
    }
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

void PlanningSceneInterface::addCollisionObjects(const std::vector<moveit_msgs::msg::CollisionObject>& collision_objects,
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
