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
std::once_flag LOCAL_PLANNER_STARTED;

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
      if (!hybrid_planning_manager_->sendGlobalPlannerAction())  // Start global planning
      {
        hybrid_planning_manager_->sendHybridPlanningResponse(false);
      }
      break;
    case HybridPlanningEvent::GLOBAL_SOLUTION_AVAILABLE:
      std::call_once(LOCAL_PLANNER_STARTED, [this]() {            // ensure the local planner is not started twice
        if (!hybrid_planning_manager_->sendLocalPlannerAction())  // Start local planning
        {
          hybrid_planning_manager_->sendHybridPlanningResponse(false);
        }
      });
      break;
    case HybridPlanningEvent::LOCAL_PLANNING_ACTION_FINISHED:
      hybrid_planning_manager_->sendHybridPlanningResponse(true);
      break;
    default:
      break;
      // Do nothing
  }
  return ReactionResult(event, "", moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

ReactionResult SinglePlanExecution::react(const std::string& event)
{
  return ReactionResult(event, "'Single-Plan-Execution' plugin cannot handle events given as string.",
                        moveit_msgs::msg::MoveItErrorCodes::FAILURE);
};
}  // namespace moveit::hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit::hybrid_planning::SinglePlanExecution, moveit::hybrid_planning::PlannerLogicInterface)
