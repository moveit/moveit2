/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Bielefeld University
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
 *   * Neither the name of Bielefeld University nor the names of its
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

/* Author: Robert Haschke
   Desc: This adapter changes the link_name field of a constraint from an object's (sub-)frame name to the name of the
   robot link, that the object is attached to. Transforming the frame names is necessary because the frames of an
   attached object are not known to a planner.
*/

#include <moveit/planning_interface/planning_request_adapter.h>
#include <moveit/kinematic_constraints/utils.h>
#include <class_loader/class_loader.hpp>
#include <moveit/utils/logger.hpp>

namespace default_planning_request_adapters
{
/// @brief Transforms frames used in constraints to link frames in the robot model.
class ResolveConstraintFrames : public planning_interface::PlanningRequestAdapter
{
public:
  ResolveConstraintFrames() : logger_(moveit::getLogger("moveit.ros.resolve_constraint_frames"))
  {
  }

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("ResolveConstraintFrames");
  }

  [[nodiscard]] moveit::core::MoveItErrorCode adapt(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                    planning_interface::MotionPlanRequest& req) const override
  {
    RCLCPP_DEBUG(logger_, "Running '%s'", getDescription().c_str());
    // Resolve path constraint frames
    kinematic_constraints::resolveConstraintFrames(planning_scene->getCurrentState(), req.path_constraints);
    // Resolve goal constraint frames
    for (moveit_msgs::msg::Constraints& constraint : req.goal_constraints)
    {
      kinematic_constraints::resolveConstraintFrames(planning_scene->getCurrentState(), constraint);
    }
    return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::SUCCESS, std::string(""), getDescription());
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace default_planning_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planning_request_adapters::ResolveConstraintFrames,
                            planning_interface::PlanningRequestAdapter);
