/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2016, Robert Haschke
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
 *   * Neither the names of the authors nor the names of its
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

/* Author: Ioan Sucan, Robert Haschke */

#include "moveit_fake_controllers.h"
#include <boost/thread.hpp>
#include <limits>

namespace moveit_fake_controller_manager
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.plugins.moveit_fake_controllers");

BaseFakeController::BaseFakeController(const std::string& name, const std::vector<std::string>& joints,
                                       const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub)
  : moveit_controller_manager::MoveItControllerHandle(name), joints_(joints), pub_(pub)
{
  std::stringstream ss;
  ss << "Fake controller '" << name << "' with joints [ ";
  std::copy(joints.begin(), joints.end(), std::ostream_iterator<std::string>(ss, " "));
  ss << "]";
  RCLCPP_INFO_STREAM(LOGGER, ss.str());
}

void BaseFakeController::getJoints(std::vector<std::string>& joints) const
{
  joints = joints_;
}

moveit_controller_manager::ExecutionStatus BaseFakeController::getLastExecutionStatus()
{
  return moveit_controller_manager::ExecutionStatus::SUCCEEDED;
}

LastPointController::LastPointController(const std::string& name, const std::vector<std::string>& joints,
                                         const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub)
  : BaseFakeController(name, joints, pub)
{
}

LastPointController::~LastPointController() = default;

bool LastPointController::sendTrajectory(const moveit_msgs::msg::RobotTrajectory& t)
{
  RCLCPP_INFO(LOGGER, "Fake execution of trajectory");
  if (t.joint_trajectory.points.empty())
    return true;

  sensor_msgs::msg::JointState js;
  const trajectory_msgs::msg::JointTrajectoryPoint& last = t.joint_trajectory.points.back();
  js.header = t.joint_trajectory.header;
  js.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  js.name = t.joint_trajectory.joint_names;
  js.position = last.positions;
  js.velocity = last.velocities;
  js.effort = last.effort;
  pub_->publish(js);

  return true;
}

bool LastPointController::cancelExecution()
{
  return true;
}

bool LastPointController::waitForExecution(const rclcpp::Duration& /*timeout*/)
{
  rclcpp::Duration dur = rclcpp::Duration::from_seconds(0.5);  // give some time to receive the published JointState
  rclcpp::sleep_for(std::chrono::nanoseconds(dur.nanoseconds()));
  return true;
}

ThreadedController::ThreadedController(const std::string& name, const std::vector<std::string>& joints,
                                       const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub)
  : BaseFakeController(name, joints, pub)
{
}

ThreadedController::~ThreadedController()
{
  ThreadedController::cancelTrajectory();
}

void ThreadedController::cancelTrajectory()
{
  cancel_ = true;
  thread_.join();
}

bool ThreadedController::sendTrajectory(const moveit_msgs::msg::RobotTrajectory& t)
{
  cancelTrajectory();  // cancel any previous fake motion
  cancel_ = false;
  status_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
  thread_ = boost::thread(boost::bind(&ThreadedController::execTrajectory, this, t));
  return true;
}

bool ThreadedController::cancelExecution()
{
  cancelTrajectory();
  RCLCPP_INFO(LOGGER, "Fake trajectory execution cancelled");
  status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
  return true;
}

bool ThreadedController::waitForExecution(const rclcpp::Duration& /*timeout*/)
{
  thread_.join();
  status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  return true;
}

moveit_controller_manager::ExecutionStatus ThreadedController::getLastExecutionStatus()
{
  return status_;
}

ViaPointController::ViaPointController(const std::string& name, const std::vector<std::string>& joints,
                                       const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub)
  : ThreadedController(name, joints, pub)
{
}

ViaPointController::~ViaPointController() = default;

void ViaPointController::execTrajectory(const moveit_msgs::msg::RobotTrajectory& t)
{
  RCLCPP_INFO(LOGGER, "Fake execution of trajectory");
  sensor_msgs::msg::JointState js;
  js.header = t.joint_trajectory.header;
  js.name = t.joint_trajectory.joint_names;

  // publish joint states for all intermediate via points of the trajectory
  // no further interpolation
  rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
  for (std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator via = t.joint_trajectory.points.begin(),
                                                                               end = t.joint_trajectory.points.end();
       !cancelled() && via != end; ++via)
  {
    js.position = via->positions;
    js.velocity = via->velocities;
    js.effort = via->effort;

    rclcpp::Duration wait_time =
        rclcpp::Duration(via->time_from_start) - (rclcpp::Clock(RCL_ROS_TIME).now() - start_time);
    if (wait_time.seconds() > std::numeric_limits<float>::epsilon())
    {
      RCLCPP_DEBUG(LOGGER, "Fake execution: waiting %0.1fs for next via point, %ld remaining", wait_time.seconds(),
                   end - via);
      rclcpp::sleep_for(std::chrono::nanoseconds(wait_time.nanoseconds()));
    }
    js.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    pub_->publish(js);
  }
  RCLCPP_DEBUG(LOGGER, "Fake execution of trajectory: done");
}

InterpolatingController::InterpolatingController(const std::string& name, const std::vector<std::string>& joints,
                                                 const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub,
                                                 double rate)
  : ThreadedController(name, joints, pub), rate_(rate)
{
}

InterpolatingController::~InterpolatingController() = default;

namespace
{
void interpolate(sensor_msgs::msg::JointState& js, const trajectory_msgs::msg::JointTrajectoryPoint& prev,
                 const trajectory_msgs::msg::JointTrajectoryPoint& next, const rclcpp::Duration& elapsed)
{
  // workaround because builtin_interfaces::msg::Duration_ doesnt have operator- function
  rclcpp::Duration next_time = next.time_from_start;
  rclcpp::Duration prev_time = prev.time_from_start;
  double duration = (next_time - prev_time).seconds();
  double alpha = 1.0;
  if (duration > std::numeric_limits<double>::epsilon())
    alpha = (elapsed - prev.time_from_start).seconds() / duration;

  js.position.resize(prev.positions.size());
  for (std::size_t i = 0, end = prev.positions.size(); i < end; ++i)
  {
    js.position[i] = prev.positions[i] + alpha * (next.positions[i] - prev.positions[i]);
  }
}
}  // namespace

void InterpolatingController::execTrajectory(const moveit_msgs::msg::RobotTrajectory& t)
{
  RCLCPP_INFO(LOGGER, "Fake execution of trajectory");
  if (t.joint_trajectory.points.empty())
    return;

  sensor_msgs::msg::JointState js;
  js.header = t.joint_trajectory.header;
  js.name = t.joint_trajectory.joint_names;

  const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& points = t.joint_trajectory.points;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator prev = points.begin(),  // previous via point
      next = points.begin() + 1,  // currently targetted via point
      end = points.end();

  rclcpp::Time start_time = rclcpp::Clock(RCL_ROS_TIME).now();
  while (!cancelled())
  {
    rclcpp::Duration elapsed = rclcpp::Clock(RCL_ROS_TIME).now() - start_time;
    // hop to next targetted via point
    while (next != end && elapsed > next->time_from_start)
    {
      ++prev;
      ++next;
    }
    if (next == end)
      break;

    // workaround because builtin_interfaces::msg::Duration_ doesnt have operator- function
    rclcpp::Duration next_time = next->time_from_start;
    rclcpp::Duration prev_time = prev->time_from_start;
    double duration = (next_time - prev_time).seconds();
    RCLCPP_DEBUG(LOGGER, "elapsed: %.3f via points %td,%td / %td  alpha: %.3f", elapsed.seconds(),
                 prev - points.begin(), next - points.begin(), end - points.begin(),
                 duration > std::numeric_limits<double>::epsilon() ?
                     (elapsed - prev->time_from_start).seconds() / duration :
                     1.0);
    interpolate(js, *prev, *next, elapsed);
    js.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    pub_->publish(js);

    rate_.sleep();
  }
  if (cancelled())
    return;

  rclcpp::Duration elapsed = rclcpp::Clock(RCL_ROS_TIME).now() - start_time;
  RCLCPP_DEBUG(LOGGER, "elapsed: %.3f via points %td,%td / %td  alpha: 1.0", elapsed.seconds(), prev - points.begin(),
               next - points.begin(), end - points.begin());

  // publish last point
  interpolate(js, *prev, *prev, prev->time_from_start);
  js.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  pub_->publish(js);

  RCLCPP_DEBUG(LOGGER, "Fake execution of trajectory: done");
}

}  // end namespace moveit_fake_controller_manager
