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
   Description: Defines an interface for a planner logic plugin for the hybrid planning manager component node.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <moveit/hybrid_planning_manager/hybrid_planning_events.h>

namespace moveit_hybrid_planning
{
class HybridPlanningManager;  // Forward declaration

/**
 * Class PlannerLogicInterface - Base class for a planner logic. The logic defines how to react to different events that
 * occur during hybrid planning. Events can be triggered by callback functions of the hybrid planning manager's ROS 2
 * interfaces or timers. They are encoded either inside an enum class or as a string to easily include custom events.
 */
class PlannerLogicInterface
{
public:
  /**
   * Initialize the planner logic
   * @param hybrid_planner_handle Handle to access the hybrid planning manager's member functions and communication interfaces.
   * @return true if initialization was successful
   */
  virtual bool initialize(std::shared_ptr<moveit_hybrid_planning::HybridPlanningManager> hybrid_planner_handle) = 0;

  /**
   * React to event defined in BasicHybridPlanningEvent enum
   * @param event Basic hybrid planning event
   * @return true if reaction was successful
   */
  virtual bool react(BasicHybridPlanningEvent event) = 0;

  /**
   * React to custom event
   * @param event Encoded as string
   * @return true if reaction was successful
   */
  virtual bool react(std::string event) = 0;
  virtual ~PlannerLogicInterface(){};

protected:
  /** \brief Constructor */
  PlannerLogicInterface(){};

  // Handle to access communication interfaces and member functions of hybrid planning manager instance that uses the planner plugin
  std::shared_ptr<moveit_hybrid_planning::HybridPlanningManager> hybrid_planner_handle_ = nullptr;
};
}  // namespace moveit_hybrid_planning
