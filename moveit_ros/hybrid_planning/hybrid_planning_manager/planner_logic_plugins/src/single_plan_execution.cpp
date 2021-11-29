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

#include <moveit/planner_logic_plugins/single_plan_execution.h>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("hybrid_planning_manager");
}

namespace moveit::hybrid_planning
{
bool SinglePlanExecution::initialize(const std::shared_ptr<HybridPlanningManager>& hybrid_planning_manager)
{
  hybrid_planning_manager_ = hybrid_planning_manager;
  return true;
}

ReactionResult SinglePlanExecution::react(const HybridPlanningEvent& event)
{
  switch (event)
  {
    case HybridPlanningEvent::HYBRID_PLANNING_REQUEST_RECEIVED:
      // Handle new hybrid planning request
      if (!hybrid_planning_manager_->sendGlobalPlannerAction())  // Start global planning
      {
        hybrid_planning_manager_->sendHybridPlanningResponse(false);
      }
      // Reset local planner started flag
      local_planner_started_ = false;
      return ReactionResult(event, "", moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    case HybridPlanningEvent::GLOBAL_SOLUTION_AVAILABLE:
      // Do nothing since we wait for the global planning action to finish
      return ReactionResult(event, "Do nothing", moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    case HybridPlanningEvent::GLOBAL_PLANNING_ACTION_SUCCESSFUL:
      // Activate local planner once global solution is available
      if (!local_planner_started_)
      {                                                           // ensure the local planner is not started twice
        if (!hybrid_planning_manager_->sendLocalPlannerAction())  // Start local planning
        {
          hybrid_planning_manager_->sendHybridPlanningResponse(false);
        }
        local_planner_started_ = true;
      }
      return ReactionResult(event, "", moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    case HybridPlanningEvent::GLOBAL_PLANNING_ACTION_ABORTED:
      // Abort hybrid planning if no global solution is found
      return ReactionResult(event, "Global planner failed to find a solution",
                            moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
    case HybridPlanningEvent::LOCAL_PLANNING_ACTION_SUCCESSFUL:
      // Finish hybrid planning action successfully because local planning action succeeded
      hybrid_planning_manager_->sendHybridPlanningResponse(true);
      return ReactionResult(event, "", moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    case HybridPlanningEvent::LOCAL_PLANNING_ACTION_ABORTED:
      // Local planning failed so abort hybrid planning
      return ReactionResult(event, "Local planner failed to find a solution",
                            moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
    default:
      // Unknown event, abort hybrid planning
      return ReactionResult(event, "Unknown event", moveit_msgs::msg::MoveItErrorCodes::FAILURE);
  }
}

ReactionResult SinglePlanExecution::react(const std::string& event)
{
  return ReactionResult(event, "'Single-Plan-Execution' plugin cannot handle events given as string.",
                        moveit_msgs::msg::MoveItErrorCodes::FAILURE);
};
}  // namespace moveit::hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit::hybrid_planning::SinglePlanExecution, moveit::hybrid_planning::PlannerLogicInterface)
