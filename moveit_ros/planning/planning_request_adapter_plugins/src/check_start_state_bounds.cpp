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

/* Author: Ioan Sucan
 * Desc: The CheckStartStateBounds adapter validates if the start state is within the joint limits
 * specified in the URDF. The need for this adapter arises in situations where the joint limits for the physical robot
 * are not properly configured. The robot may then end up in a configuration where one or more of its joints is slightly
 * outside its joint limits. In this case, the motion planner is unable to plan since it will think that the starting
 * state is outside joint limits. The “CheckStartStateBounds” planning request adapter can “fix” the start state by
 * moving it to the joint limit. However, this is obviously not the right solution every time - e.g. where the joint is
 * really outside its joint limits by a large amount. A parameter for the adapter specifies how much the joint can be
 * outside its limits for it to be “fixable”.
 */

#include <moveit/planning_interface/planning_request_adapter.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/robot_state/conversions.h>
#include <class_loader/class_loader.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>
#include <moveit/utils/logger.hpp>

#include <default_request_adapter_parameters.hpp>

namespace default_planning_request_adapters
{

/** @brief The CheckStartStateBounds adapter validates if the start state is within the joint limits specified in the URDF. */
class CheckStartStateBounds : public planning_interface::PlanningRequestAdapter
{
public:
  CheckStartStateBounds() : logger_(moveit::getLogger("check_start_state_bounds"))
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    param_listener_ = std::make_unique<default_request_adapter_parameters::ParamListener>(node, parameter_namespace);
  }

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("CheckStartStateBounds");
  }

  [[nodiscard]] moveit::core::MoveItErrorCode adapt(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                    planning_interface::MotionPlanRequest& req) const override
  {
    RCLCPP_DEBUG(logger_, "Running '%s'", getDescription().c_str());

    // Get the specified start state
    moveit::core::RobotState start_state = planning_scene->getCurrentState();
    moveit::core::robotStateMsgToRobotState(planning_scene->getTransforms(), req.start_state, start_state);

    // Get joint models
    const std::vector<const moveit::core::JointModel*>& jmodels =
        planning_scene->getRobotModel()->hasJointModelGroup(req.group_name) ?
            planning_scene->getRobotModel()->getJointModelGroup(req.group_name)->getJointModels() :
            planning_scene->getRobotModel()->getJointModels();

    // Read parameters
    const auto params = param_listener_->get_params();

    bool changed_req = false;
    for (const moveit::core::JointModel* jmodel : jmodels)
    {
      // Check if we have a revolute, continuous joint. If we do, then we only need to make sure
      // it is within the model's declared bounds (usually -Pi, Pi), since the values wrap around.
      // It is possible that the encoder maintains values outside the range [-Pi, Pi], to inform
      // how many times the joint was wrapped. Because of this, we remember the offsets for continuous
      // joints, and we un-do them when the plan comes from the planner
      switch (jmodel->getType())
      {
        case moveit::core::JointModel::REVOLUTE:
        {
          if (static_cast<const moveit::core::RevoluteJointModel*>(jmodel)->isContinuous())
          {
            double initial = start_state.getJointPositions(jmodel)[0];
            start_state.enforceBounds(jmodel);
            double after = start_state.getJointPositions(jmodel)[0];
            if (fabs(initial - after) > std::numeric_limits<double>::epsilon())
            {
              changed_req = true;
            }
          }
          break;
        }
        // Normalize yaw; no offset needs to be remembered
        case moveit::core::JointModel::PLANAR:
        {
          const double* p = start_state.getJointPositions(jmodel);
          double copy[3] = { p[0], p[1], p[2] };
          if (static_cast<const moveit::core::PlanarJointModel*>(jmodel)->normalizeRotation(copy))
          {
            start_state.setJointPositions(jmodel, copy);
            changed_req = true;
          }
          break;
        }
        case moveit::core::JointModel::FLOATING:
        {
          // Normalize quaternions
          const double* p = start_state.getJointPositions(jmodel);
          double copy[7] = { p[0], p[1], p[2], p[3], p[4], p[5], p[6] };
          if (static_cast<const moveit::core::FloatingJointModel*>(jmodel)->normalizeRotation(copy))
          {
            start_state.setJointPositions(jmodel, copy);
            changed_req = true;
          }
          break;
        }
        default:
        {
          if (!start_state.satisfiesBounds(jmodel))
          {
            std::stringstream joint_values;
            std::stringstream joint_bounds_low;
            std::stringstream joint_bounds_hi;
            const double* p = start_state.getJointPositions(jmodel);
            for (std::size_t k = 0; k < jmodel->getVariableCount(); ++k)
            {
              joint_values << p[k] << ' ';
            }
            const moveit::core::JointModel::Bounds& b = jmodel->getVariableBounds();
            for (const moveit::core::VariableBounds& variable_bounds : b)
            {
              joint_bounds_low << variable_bounds.min_position_ << ' ';
              joint_bounds_hi << variable_bounds.max_position_ << ' ';
            }
            RCLCPP_ERROR(logger_,
                         "Joint '%s' from the starting state is outside bounds by: [%s] should be in "
                         "the range [%s], [%s].",
                         jmodel->getName().c_str(), joint_values.str().c_str(), joint_bounds_low.str().c_str(),
                         joint_bounds_hi.str().c_str());
          }
        }
      }
    }

    // If we made any changes, consider using them them
    if (params.fix_start_state && changed_req)
    {
      RCLCPP_WARN(logger_, "Changing start state.");
      moveit::core::robotStateToRobotStateMsg(start_state, req.start_state);
      return moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::SUCCESS, std::string(""),
                                           getDescription());
    }

    auto status = moveit::core::MoveItErrorCode();
    if (!changed_req)
    {
      status.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    }
    else
    {
      status.val = moveit_msgs::msg::MoveItErrorCodes::START_STATE_INVALID;
      status.message = std::string("Start state out of bounds.");
    }
    status.source = getDescription();
    return status;
  }

private:
  std::unique_ptr<default_request_adapter_parameters::ParamListener> param_listener_;
  rclcpp::Logger logger_;
};
}  // namespace default_planning_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planning_request_adapters::CheckStartStateBounds,
                            planning_interface::PlanningRequestAdapter)
