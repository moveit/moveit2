/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Sebastian Jahr
   Description: Defines an interface for a local constraint solver plugin implementation for the local planner component node.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit/robot_state/robot_state.h>

#include <trajectory_msgs/msg/joint_trajectory.h>

namespace moveit_hybrid_planning
{
/**
 * Class ConstraintSolverInterface - Base class for a local constrain solver.
 */
class ConstraintSolverInterface
{
public:
  /**
   * Initialize constraint solver
   * @return True if initialization was successful
   */
  virtual bool initialize(const rclcpp::Node::SharedPtr& node) = 0;

  /**
   * Solve local planning problem for the current loop run
     @param planning_scene The planning scene to use for local planning
     @param goal The local planning goal constraint
     @param additional_constraints Additional local goal constraints
     @return Local planning solution in joint space
  */
  virtual trajectory_msgs::msg::JointTrajectory solve(moveit_msgs::msg::Constraints goal,
                                                      std::vector<moveit_msgs::msg::Constraints> additional_constraints,
                                                      planning_scene::PlanningScenePtr planning_scene) = 0;
  virtual ~ConstraintSolverInterface(){};

protected:
  /** \brief Constructor */
  ConstraintSolverInterface();
};
}  // namespace moveit_hybrid_planning
