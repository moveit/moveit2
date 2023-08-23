/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Michael Ferguson, Ioan Sucan, E. Gil Jones */

#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>

using namespace std::placeholders;

namespace moveit_simple_controller_manager
{
bool FollowJointTrajectoryControllerHandle::sendTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  RCLCPP_DEBUG_STREAM(logger_, "new trajectory to " << name_);

  if (!controller_action_client_)
    return false;

  if (!isConnected())
  {
    RCLCPP_ERROR_STREAM(logger_, "Action client not connected to action server: " << getActionName());
    return false;
  }

  if (done_)
  {
    RCLCPP_INFO_STREAM(logger_, "sending trajectory to " << name_);
  }
  else
  {
    RCLCPP_INFO_STREAM(logger_, "sending continuation for the currently executed trajectory to " << name_);
  }

  control_msgs::action::FollowJointTrajectory::Goal goal = goal_template_;
  goal.trajectory = trajectory.joint_trajectory;
  goal.multi_dof_trajectory = trajectory.multi_dof_joint_trajectory;

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions send_goal_options;
  // Active callback
  send_goal_options.goal_response_callback =
      [this](
          const rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::GoalHandle::SharedPtr& goal_handle) {
        RCLCPP_INFO_STREAM(logger_, name_ << " started execution");
        if (!goal_handle)
        {
          RCLCPP_WARN(logger_, "Goal request rejected");
        }
        else
        {
          RCLCPP_INFO(logger_, "Goal request accepted!");
        }
      };

  done_ = false;
  last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;

  // Send goal
  auto current_goal_future = controller_action_client_->async_send_goal(goal, send_goal_options);
  current_goal_ = current_goal_future.get();
  if (!current_goal_)
  {
    RCLCPP_ERROR(logger_, "Goal was rejected by server");
    return false;
  }
  return true;
}

// TODO(JafarAbdi): Revise parameter lookup
// void FollowJointTrajectoryControllerHandle::configure(XmlRpc::XmlRpcValue& config)
//{
//  if (config.hasMember("path_tolerance"))
//    configure(config["path_tolerance"], "path_tolerance", goal_template_.path_tolerance);
//  if (config.hasMember("goal_tolerance"))
//    configure(config["goal_tolerance"], "goal_tolerance", goal_template_.goal_tolerance);
//  if (config.hasMember("goal_time_tolerance"))
//    goal_template_.goal_time_tolerance = ros::Duration(parseDouble(config["goal_time_tolerance"]));
//}

namespace
{
enum ToleranceVariables
{
  POSITION,
  VELOCITY,
  ACCELERATION
};
template <ToleranceVariables>
double& variable(control_msgs::msg::JointTolerance& msg);

template <>
inline double& variable<POSITION>(control_msgs::msg::JointTolerance& msg)
{
  return msg.position;
}
template <>
inline double& variable<VELOCITY>(control_msgs::msg::JointTolerance& msg)
{
  return msg.velocity;
}
template <>
inline double& variable<ACCELERATION>(control_msgs::msg::JointTolerance& msg)
{
  return msg.acceleration;
}

static const std::map<ToleranceVariables, std::string> VAR_NAME = { { POSITION, "position" },
                                                                    { VELOCITY, "velocity" },
                                                                    { ACCELERATION, "acceleration" } };
static const std::map<ToleranceVariables, decltype(&variable<POSITION>)> VAR_ACCESS = {
  { POSITION, &variable<POSITION> },
  { VELOCITY, &variable<VELOCITY> },
  { ACCELERATION, &variable<ACCELERATION> }
};

const char* errorCodeToMessage(int error_code)
{
  switch (error_code)
  {
    case control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL:
      return "SUCCESSFUL";
    case control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL:
      return "INVALID_GOAL";
    case control_msgs::action::FollowJointTrajectory::Result::INVALID_JOINTS:
      return "INVALID_JOINTS";
    case control_msgs::action::FollowJointTrajectory::Result::OLD_HEADER_TIMESTAMP:
      return "OLD_HEADER_TIMESTAMP";
    case control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED:
      return "PATH_TOLERANCE_VIOLATED";
    case control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED:
      return "GOAL_TOLERANCE_VIOLATED";
    default:
      return "unknown error";
  }
}
}  // namespace

// TODO(JafarAbdi): Revise parameter lookup
// void FollowJointTrajectoryControllerHandle::configure(XmlRpc::XmlRpcValue& config, const std::string& config_name,
//                                                      std::vector<control_msgs::JointTolerance>& tolerances)
//{
//  if (isStruct(config))  // config should be either a struct of position, velocity, acceleration
//  {
//    for (ToleranceVariables var : { POSITION, VELOCITY, ACCELERATION })
//    {
//      if (!config.hasMember(VAR_NAME[var]))
//        continue;
//      XmlRpc::XmlRpcValue values = config[VAR_NAME[var]];
//      if (isArray(values, joints_.size()))
//      {
//        size_t i = 0;
//        for (const auto& joint_name : joints_)
//          VAR_ACCESS[var](getTolerance(tolerances, joint_name)) = parseDouble(values[i++]);
//      }
//      else
//      {  // use common value for all joints
//        double value = parseDouble(values);
//        for (const auto& joint_name : joints_)
//          VAR_ACCESS[var](getTolerance(tolerances, joint_name)) = value;
//      }
//    }
//  }
//  else if (isArray(config))  // or an array of JointTolerance msgs
//  {
//    for (int i = 0; i < config.size(); ++i)  // NOLINT(modernize-loop-convert)
//    {
//      control_msgs::JointTolerance& tol = getTolerance(tolerances, config[i]["name"]);
//      for (ToleranceVariables var : { POSITION, VELOCITY, ACCELERATION })
//      {
//        if (!config[i].hasMember(VAR_NAME[var]))
//          continue;
//        VAR_ACCESS[var](tol) = parseDouble(config[i][VAR_NAME[var]]);
//      }
//    }
//  }
//  else
//    ROS_WARN_STREAM_NAMED(LOGNAME, "Invalid " << config_name);
//}

control_msgs::msg::JointTolerance&
FollowJointTrajectoryControllerHandle::getTolerance(std::vector<control_msgs::msg::JointTolerance>& tolerances,
                                                    const std::string& name)
{
  auto it = std::lower_bound(tolerances.begin(), tolerances.end(), name,
                             [](const control_msgs::msg::JointTolerance& lhs, const std::string& rhs) {
                               return lhs.name < rhs;
                             });
  if (it == tolerances.cend() || it->name != name)
  {  // insert new entry if not yet available
    it = tolerances.insert(it, control_msgs::msg::JointTolerance());
    it->name = name;
  }
  return *it;
}

void FollowJointTrajectoryControllerHandle::controllerDoneCallback(
    const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult& wrapped_result)
{
  // Output custom error message for FollowJointTrajectoryResult if necessary
  if (!wrapped_result.result)
  {
    RCLCPP_WARN_STREAM(logger_, "Controller '" << name_ << "' done, no result returned");
  }
  else if (wrapped_result.result->error_code == control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL)
  {
    RCLCPP_INFO_STREAM(logger_, "Controller '" << name_ << "' successfully finished");
  }
  else
  {
    RCLCPP_WARN_STREAM(logger_, "Controller '" << name_ << "' failed with error "
                                               << errorCodeToMessage(wrapped_result.result->error_code) << ": "
                                               << wrapped_result.result->error_string);
  }
  finishControllerExecution(wrapped_result.code);
}

}  // end namespace moveit_simple_controller_manager
