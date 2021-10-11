/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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

/* Author: Sebastian Jahr
   Description: Defines an interface for a planner logic plugin for the hybrid planning manager component node.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/hybrid_planning_manager/hybrid_planning_events.h>
#include <moveit/hybrid_planning_manager/hybrid_planning_interface.h>

namespace moveit::hybrid_planning
{
// TODO(sjahr): Move this into utility package
class MoveItErrorCode : public moveit_msgs::msg::MoveItErrorCodes
{
public:
  MoveItErrorCode()
  {
    val = 0;
  }
  MoveItErrorCode(int code)
  {
    val = code;
  }
  MoveItErrorCode(const moveit_msgs::msg::MoveItErrorCodes& code)
  {
    val = code.val;
  }
  explicit operator bool() const
  {
    return val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
  bool operator==(const int code) const
  {
    return val == code;
  }
  bool operator!=(const int code) const
  {
    return val != code;
  }
};

// Describes the outcome of a reaction to an event in the hybrid planning architecture
struct ReactionResult
{
  ReactionResult(const HybridPlanningEvent& planning_event, const std::string& error_msg, const int& error_code)
    : error_message(error_msg), error_code(error_code)
  {
    switch (planning_event)
    {
      case HybridPlanningEvent::HYBRID_PLANNING_REQUEST_RECEIVED:
        event = "Hybrid planning request received";
        break;
      case HybridPlanningEvent::GLOBAL_PLANNING_ACTION_FINISHED:
        event = "Global planning action finished";
        break;
      case HybridPlanningEvent::GLOBAL_SOLUTION_AVAILABLE:
        event = "Global solution available";
        break;
      case HybridPlanningEvent::LOCAL_PLANNING_ACTION_FINISHED:
        event = "Local planning action finished";
        break;
    }
  };
  ReactionResult(const std::string& event, const std::string& error_msg, const int& error_code)
    : event(event), error_message(error_msg), error_code(error_code){};

  // Event that triggered the reaction
  std::string event;

  // Additional error description
  std::string error_message;

  // Error code
  MoveItErrorCode error_code;
};

class HybridPlanningInterface;  // Forward declaration

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
   * @param hybrid_planning_interface Shared pointer to hybrid planning interface
   * @return true if initialization was successful
   */
  virtual bool initialize(const std::shared_ptr<HybridPlanningInterface>& hybrid_planning_interface) = 0;

  /**
   * React to event defined in HybridPlanningEvent enum
   * @param event Basic hybrid planning event
   * @return Reaction result that summarizes the outcome of the reaction
   */
  virtual ReactionResult processEvent(const HybridPlanningEvent& event) = 0;

  /**
   * React to custom event
   * @param event Encoded as string
   * @return Reaction result that summarizes the outcome of the reaction
   */
  virtual ReactionResult processEvent(const std::string& event) = 0;

protected:
  // The hybrid planning interface to access communication interfaces between the components
  std::shared_ptr<HybridPlanningInterface> hybrid_planning_interface_ = nullptr;
};
}  // namespace moveit::hybrid_planning
