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

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <memory>

static const std::string ROBOT_DESCRIPTION = "robot_description";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("visualize_robot_collision_volume");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  double radius = 0.02;
  int lifetime = 600;

  planning_scene_monitor::PlanningSceneMonitor psm(node, ROBOT_DESCRIPTION);
  if (psm.getPlanningScene())
  {
    psm.startWorldGeometryMonitor();
    psm.startSceneMonitor();
    psm.startStateMonitor();
    auto pub_markers = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    std::cout << "\nListening for planning scene...\nType the number of spheres to generate and press Enter: " << '\n';
    int num_spheres;
    std::cin >> num_spheres;

    planning_scene::PlanningScenePtr scene = psm.getPlanningScene();
    std::vector<double> aabb;
    scene->getCurrentState().computeAABB(aabb);

    // publish the bounding box
    visualization_msgs::msg::Marker mk;
    mk.header.stamp = node->now();
    mk.header.frame_id = scene->getPlanningFrame();
    mk.ns = "bounding_box";
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::CUBE;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.position.x = (aabb[0] + aabb[1]) / 2.0;
    mk.pose.position.y = (aabb[2] + aabb[3]) / 2.0;
    mk.pose.position.z = (aabb[4] + aabb[5]) / 2.0;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = aabb[1] - aabb[0];
    mk.scale.y = aabb[3] - aabb[2];
    mk.scale.z = aabb[5] - aabb[4];
    mk.color.r = 0.0f;
    mk.color.g = 0.5f;
    mk.color.b = 1.0f;
    mk.color.a = 0.3f;
    mk.lifetime = rclcpp::Duration::from_seconds(lifetime);
    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(mk);
    pub_markers->publish(arr);

    Eigen::Isometry3d t;
    t.setIdentity();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;
    std::size_t published = 0;
    random_numbers::RandomNumberGenerator rng;
    collision_detection::CollisionRequest req;

    std_msgs::msg::ColorRGBA color;
    color.r = 1.0f;
    color.g = 0.0f;
    color.b = 0.0f;
    color.a = 1.0f;

    for (int i = 0; i < num_spheres; ++i)
    {
      t.translation() = Eigen::Vector3d(rng.uniformReal(aabb[0], aabb[1]), rng.uniformReal(aabb[2], aabb[3]),
                                        rng.uniformReal(aabb[4], aabb[5]));
      scene->getWorldNonConst()->clearObjects();
      scene->getWorldNonConst()->addToObject("test", std::make_shared<shapes::Sphere>(radius), t);
      collision_detection::CollisionResult res;
      scene->checkCollision(req, res);
      if (res.collision)
      {
        points.push_back(t.translation());
        if (points.size() - published >= 100 || (points.size() > published && i + 1 >= num_spheres))
        {
          arr.markers.clear();
          for (std::size_t k = published; k < points.size(); ++k)
          {
            visualization_msgs::msg::Marker mk;
            mk.header.stamp = node->now();
            mk.header.frame_id = scene->getPlanningFrame();
            mk.ns = "colliding";
            mk.id = k;
            mk.type = visualization_msgs::msg::Marker::SPHERE;
            mk.action = visualization_msgs::msg::Marker::ADD;
            mk.pose.position.x = points[k].x();
            mk.pose.position.y = points[k].y();
            mk.pose.position.z = points[k].z();
            mk.pose.orientation.w = 1.0;
            mk.scale.x = mk.scale.y = mk.scale.z = radius;
            mk.color = color;
            mk.lifetime = rclcpp::Duration::from_seconds(lifetime);
            arr.markers.push_back(mk);
            pub_markers->publish(arr);
          }
          pub_markers->publish(arr);
          published = points.size();
        }
      }
    }
  }
  executor.spin();
  return 0;
}
