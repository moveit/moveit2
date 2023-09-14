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

#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

/** \brief Generic interface to adapting motion planning requests */
namespace planning_request_adapter
{
MOVEIT_CLASS_FORWARD(PlanningRequestAdapter);  // Defines PlanningRequestAdapterPtr, ConstPtr, WeakPtr... etc

/** @brief Concept in MoveIt which can be used to modify the planning problem and resulting trajectory (pre-processing
 * and/or post-processing) for a motion planner. PlanningRequestAdapter enable using multiple motion planning and
 * trajectory generation algorithms in sequence to produce robust motion plans.
 */
class PlanningRequestAdapter
{
public:
  /// \brief Functional interface to call a planning function
  using PlannerFn =
      std::function<bool(const planning_scene::PlanningSceneConstPtr&, const planning_interface::MotionPlanRequest&,
                         planning_interface::MotionPlanResponse&)>;

  /** \brief Initialize parameters using the passed Node and parameter namespace.
   *  @param node Node instance used by the adapter
   *  @param parameter_namespace Parameter namespace for adapter
      If no initialization is needed, simply implement as empty */
  virtual void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) = 0;

  /** \brief Get a description of this adapter
   *  @return Returns a short string that identifies the planning request adapter
   */
  [[nodiscard]] virtual std::string getDescription() const = 0;

  /** \brief Adapt the planning request if needed, call the planner
      function \e planner and update the planning response if
      needed. If the response is changed, the index values of the
      states added without planning are added to \e
      added_path_index
   *  @param planner Pointer to the planner used to solve the passed problem
   *  @param planning_scene Representation of the environment for the planning
   *  @param req Motion planning request with a set of constraints
   *  @param res Reference to which the generated motion plan response is written to
   *  @return True if response got generated correctly */
  [[nodiscard]] bool adaptAndPlan(const planning_interface::PlannerManagerPtr& planner,
                                  const planning_scene::PlanningSceneConstPtr& planning_scene,
                                  const planning_interface::MotionPlanRequest& req,
                                  planning_interface::MotionPlanResponse& res) const;

  /** \brief Adapt the planning request if needed, call the planner
      function \e planner and update the planning response if
      needed. If the response is changed, the index values of the
      states added without planning are added to \e
      added_path_index
   *  @param planner Pointer to the planner used to solve the passed problem
   *  @param planning_scene Representation of the environment for the planning
   *  @param req Motion planning request with a set of constraints
   *  @param res Reference to which the generated motion plan response is written to
   *  @return True if response got generated correctly */
  [[nodiscard]] virtual bool adaptAndPlan(const PlannerFn& planner,
                                          const planning_scene::PlanningSceneConstPtr& planning_scene,
                                          const planning_interface::MotionPlanRequest& req,
                                          planning_interface::MotionPlanResponse& res) const = 0;
};

/** \brief Apply a sequence of adapters to a motion plan */
class PlanningRequestAdapterChain
{
public:
  /** \brief Add new adapter to the back of the chain
   *  @param adapter Adapter to be added */
  void addAdapter(const PlanningRequestAdapterConstPtr& adapter);

  /** \brief Iterate through the chain and call all adapters and planners in the correct order
   *  @param planner Pointer to the planner used to solve the passed problem
   *  @param planning_scene Representation of the environment for the planning
   *  @param req Motion planning request with a set of constraints
   *  @param res Reference to which the generated motion plan response is written to
   *  @return True if response got generated correctly */
  [[nodiscard]] bool adaptAndPlan(const planning_interface::PlannerManagerPtr& planner,
                                  const planning_scene::PlanningSceneConstPtr& planning_scene,
                                  const planning_interface::MotionPlanRequest& req,
                                  planning_interface::MotionPlanResponse& res) const;

private:
  std::vector<PlanningRequestAdapterConstPtr> adapters_;
};
}  // namespace planning_request_adapter
