/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

/** @file
 * @author Henning Kayser
 * @brief: Helper functions for visualizing trajectory markers for STOMP planning iterations.
 */

#pragma once

#include <stomp_moveit/stomp_moveit_task.hpp>
#include <stomp_moveit/conversion_functions.hpp>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace stomp_moveit
{
namespace visualization
{

namespace
{
const auto GREEN = [](double a) {
  std_msgs::msg::ColorRGBA color;
  color.r = 0.1;
  color.g = 0.8;
  color.b = 0.1;
  color.a = a;
  return color;
};

/**
 * @brief Creates an array of markers to visualize the trajectory EE-positions
 *
 * @param robot_trajectory Trajectory to visualize
 * @param ee_parent_link End-Effector link
 * @param color Color of the markers
 * @return Marker array that can be visualized in RVIZ
 */
visualization_msgs::msg::MarkerArray
createTrajectoryMarkerArray(const robot_trajectory::RobotTrajectory& robot_trajectory,
                            const moveit::core::LinkModel* ee_parent_link,
                            const std_msgs::msg::ColorRGBA& color = GREEN(1.0))
{
  visualization_msgs::msg::MarkerArray markers_array;

  // Initialize Sphere Marker
  visualization_msgs::msg::Marker sphere_marker;
  sphere_marker.header.frame_id = robot_trajectory.getRobotModel()->getModelFrame();
  sphere_marker.ns = "Path";
  sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
  sphere_marker.action = visualization_msgs::msg::Marker::ADD;
  sphere_marker.lifetime = rclcpp::Duration(0, 0);  // Infinite lifetime
  sphere_marker.scale.x = 0.01;
  sphere_marker.scale.y = 0.01;
  sphere_marker.scale.z = 0.01;
  sphere_marker.color = color;
  sphere_marker.frame_locked = false;

  // Visualize end effector positions of Cartesian path as sphere markers
  for (std::size_t index = 0; index < robot_trajectory.getWayPointCount(); index++)
  {
    const Eigen::Isometry3d& tip_pose = robot_trajectory.getWayPoint(index).getGlobalLinkTransform(ee_parent_link);
    sphere_marker.pose = tf2::toMsg(tip_pose);
    sphere_marker.id = index;

    markers_array.markers.push_back(sphere_marker);
  }

  return markers_array;
}
}  // namespace

/**
 * @brief Get post iteration function that publishes the EE path of the generated trajectory
 *
 * @param marker_publisher Marker publisher that is used to publish the path
 * @param planning_scene Current planning scene
 * @param group JointModelGroup to identify the EE tip
 * @return If the marker_publisher is not a nullptr a post-iteration function that publishes the generated EE path is
 * returned. Otherwise, a function that does nothing.
 */
PostIterationFn
getIterationPathPublisher(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& marker_publisher,
                          const std::shared_ptr<const planning_scene::PlanningScene>& planning_scene,
                          const moveit::core::JointModelGroup* group)
{
  assert(group != nullptr);

  if (marker_publisher == nullptr)
  {
    return [](int /*iteration_number*/, double /*cost*/, const Eigen::MatrixXd& /*values*/) {
      // Do nothing
    };
  }

  auto path_publisher = [marker_publisher, group,
                         reference_state = moveit::core::RobotState(planning_scene->getCurrentState())](
                            int /*iteration_number*/, double /*cost*/, const Eigen::MatrixXd& values) {
    static thread_local robot_trajectory::RobotTrajectory trajectory(reference_state.getRobotModel(), group);
    fillRobotTrajectory(values, reference_state, trajectory);

    const moveit::core::LinkModel* ee_parent_link = group->getOnlyOneEndEffectorTip();

    if (ee_parent_link != nullptr && !trajectory.empty())
    {
      marker_publisher->publish(createTrajectoryMarkerArray(trajectory, ee_parent_link, GREEN(0.5)));
    }
  };

  return path_publisher;
}

/**
 * @brief Get Done function that publishes the EE path of the generated trajectory
 *
 * @param marker_publisher Marker publisher that is used to publish the path
 * @param planning_scene Current planning scene
 * @param group JointModelGroup to identify the EE tip
 * @return If the marker_publisher is not a nullptr a Done function that publishes the generated EE path is returned.
 * Otherwise, a function that does nothing.
 */
DoneFn
getSuccessTrajectoryPublisher(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& marker_publisher,
                              const std::shared_ptr<const planning_scene::PlanningScene>& planning_scene,
                              const moveit::core::JointModelGroup* group)
{
  assert(group != nullptr);

  if (marker_publisher == nullptr)
  {
    return [](bool /*success*/, int /*total_iterations*/, double /*final_cost*/, const Eigen::MatrixXd& /*values*/) {
      // Do nothing
    };
  }

  auto path_publisher =
      [marker_publisher, group, reference_state = moveit::core::RobotState(planning_scene->getCurrentState())](
          bool success, int /*total_iterations*/, double /*final_cost*/, const Eigen::MatrixXd& values) {
        static thread_local robot_trajectory::RobotTrajectory trajectory(reference_state.getRobotModel(), group);
        if (success)
        {
          fillRobotTrajectory(values, reference_state, trajectory);

          const moveit::core::LinkModel* ee_parent_link = group->getOnlyOneEndEffectorTip();

          if (ee_parent_link != nullptr && !trajectory.empty())
          {
            marker_publisher->publish(createTrajectoryMarkerArray(trajectory, ee_parent_link));
          }
        }
      };

  return path_publisher;
}
}  // namespace visualization
}  // namespace stomp_moveit
