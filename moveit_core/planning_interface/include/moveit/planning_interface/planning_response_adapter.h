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

/* Author: Sebastian Jahr */

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace planning_interface
{
MOVEIT_CLASS_FORWARD(PlanningResponseAdapter);  // Defines PlanningResponseAdapterPtr, ConstPtr, WeakPtr... etc

/** @brief Concept in MoveIt which can be used to modify the planning solution (post-processing) in a planning pipeline.
 * PlanningResponseAdapters enable using for example smoothing and trajectory generation algorithms in sequence to
 * produce a trajectory.
 */
class PlanningResponseAdapter
{
public:
  /** @brief Initialize parameters using the passed Node and parameter namespace.
   *  @param node Node instance used by the adapter
   *  @param parameter_namespace Parameter namespace for adapter
   *  @details The default implementation is empty */
  virtual void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace){};

  /** \brief Get a description of this adapter
   *  @return Returns a short string that identifies the planning response adapter
   */
  [[nodiscard]] virtual std::string getDescription() const = 0;

  /** @brief Adapt the planning response
   *  @param planning_scene Representation of the environment for the planning
   *  @param req Motion planning request with a set of constraints
   *  @param res Motion planning response containing the solution that is adapted. */
  virtual void adapt(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req,
                     planning_interface::MotionPlanResponse& res) const = 0;
};
}  // namespace planning_interface
