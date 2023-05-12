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

  virtual ~PlanningRequestAdapter() = default;

  /** \brief Initialize parameters using the passed Node and parameter namespace.
   *  @param node Node instance used by the adapter
   *  @param parameter_namespace Parameter namespace for adapter
      If no initialization is needed, simply implement as empty */
  virtual void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) = 0;

  /** \brief Get a description of this adapter
   *  @return Returns a short string that identifies the planning request adapter
   */
  virtual std::string getDescription() const
  {
    return "";
  }

  /** \brief Adapt the planning request if needed, call the planner
      function \e planner and update the planning response if
      needed.
   *  @param planner Pointer to the planner used to solve the passed problem
   *  @param planning_scene Representation of the environment for the planning
   *  @param req Motion planning request with a set of constraints
   *  @param res Reference to which the generated motion plan response is written to
   *  @return True if response got generated correctly */
  bool adaptAndPlan(const planning_interface::PlannerManagerPtr& planner,
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
   *  @param added_path_index Sometimes planning request adapters may add states on the solution path (e.g.,
     add the current state of the robot as prefix, when the robot started to plan only from near that state, as the
     current state itself appears to touch obstacles). This is helpful because the added states should not be considered
     invalid in all situations.
   *  @return True if response got generated correctly */
  bool adaptAndPlan(const planning_interface::PlannerManagerPtr& planner,
                    const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& added_path_index) const;

  /** \brief Adapt the planning request if needed, call the planner
      function \e planner and update the planning response if
      needed. If the response is changed, the index values of the
      states added without planning are added to \e
      added_path_index
   *  @param planner Pointer to the planner used to solve the passed problem
   *  @param planning_scene Representation of the environment for the planning
   *  @param req Motion planning request with a set of constraints
   *  @param res Reference to which the generated motion plan response is written to
   *  @param added_path_index Sometimes planning request adapters may add states on the solution path (e.g.,
        add the current state of the robot as prefix, when the robot started to plan only from near that state, as the
        current state itself appears to touch obstacles). This is helpful because the added states should not be
   considered invalid in all situations.
   *  @return True if response got generated correctly */
  virtual bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest& req,
                            planning_interface::MotionPlanResponse& res,
                            std::vector<std::size_t>& added_path_index) const = 0;

protected:
  /** \brief Helper param for getting a parameter using a namespace **/
  template <typename T>
  T getParam(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger, const std::string& parameter_namespace,
             const std::string& parameter_name, T default_value = {}) const
  {
    std::string full_name = parameter_namespace.empty() ? parameter_name : parameter_namespace + "." + parameter_name;
    T value;
    if (!node->get_parameter(full_name, value))
    {
      RCLCPP_INFO(logger, "Param '%s' was not set. Using default value: %s", full_name.c_str(),
                  std::to_string(default_value).c_str());
      return default_value;
    }
    else
    {
      RCLCPP_INFO(logger, "Param '%s' was set to %s", full_name.c_str(), std::to_string(value).c_str());
      return value;
    }
  }
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
  bool adaptAndPlan(const planning_interface::PlannerManagerPtr& planner,
                    const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req,
                    planning_interface::MotionPlanResponse& res) const;

  /** \brief Iterate through the chain and call all adapters and planners in the correct order
   *  @param planner Pointer to the planner used to solve the passed problem
   *  @param planning_scene Representation of the environment for the planning
   *  @param req Motion planning request with a set of constraints
   *  @param res Reference to which the generated motion plan response is written to
   *  @param added_path_index Sometimes planning request adapters may add states on the solution path (e.g.,
          add the current state of the robot as prefix, when the robot started to plan only from near that state, as the
          current state itself appears to touch obstacles). This is helpful because the added states should not be
          considered invalid in all situations.
   *  @return True if response got generated correctly */
  bool adaptAndPlan(const planning_interface::PlannerManagerPtr& planner,
                    const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& added_path_index) const;

private:
  std::vector<PlanningRequestAdapterConstPtr> adapters_;
};
}  // namespace planning_request_adapter
