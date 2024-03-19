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

/* Author: Felix von Drigalski, Sebastian Jahr
 * Desc: A planning request adapter that checks if the motion plan request contains stacked path or goal constraints. If
 * that is the case, a warning is created but the planning process is not interrupted.
 */

#include <moveit/planning_interface/planning_request_adapter.h>
#include <class_loader/class_loader.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <moveit/utils/logger.hpp>

#include <default_request_adapter_parameters.hpp>

namespace default_planning_request_adapters
{

/** @brief Check if the motion plan request contains stacked path or goal constraints */
class CheckForStackedConstraints : public planning_interface::PlanningRequestAdapter
{
public:
  CheckForStackedConstraints() : logger_(moveit::getLogger("moveit.ros.check_for_stacked_constraints"))
  {
  }

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("CheckForStackedConstraints");
  }

  [[nodiscard]] moveit::core::MoveItErrorCode adapt(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
                                                    planning_interface::MotionPlanRequest& req) const override
  {
    // This should alert the user if planning failed because of contradicting constraints.
    // Could be checked more thoroughly, but it is probably not worth going to that length.
    if (req.path_constraints.position_constraints.size() > 1 || req.path_constraints.orientation_constraints.size() > 1)
    {
      RCLCPP_WARN(logger_, "More than one PATH constraint is set. If your planning group does not have multiple end "
                           "effectors/arms, this is "
                           "unusual. Are you using a move_group_interface and forgetting to call clearPoseTargets() or "
                           "equivalent?");
    }
    for (const auto& constraint : req.goal_constraints)
    {
      if (constraint.position_constraints.size() > 1 || constraint.orientation_constraints.size() > 1)
      {
        RCLCPP_WARN(logger_,
                    "More than one GOAL constraint is set. If your move_group does not have multiple end "
                    "effectors/arms, this is "
                    "unusual. Are you using a move_group_interface and forgetting to call clearPoseTargets() or "
                    "equivalent?");
        break;
      }
    }
    return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::SUCCESS, std::string(""), getDescription());
  }

private:
  std::unique_ptr<default_request_adapter_parameters::ParamListener> param_listener_;
  rclcpp::Logger logger_;
};
}  // namespace default_planning_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planning_request_adapters::CheckForStackedConstraints,
                            planning_interface::PlanningRequestAdapter)
