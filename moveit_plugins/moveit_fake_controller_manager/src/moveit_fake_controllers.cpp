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
#include <sensor_msgs/msg/joint_state.hpp>
#include <boost/thread.hpp>
#include <limits>

namespace moveit_fake_controller_manager
{
  static rclcpp::Logger LOGGER_FAKE_CONTROLLER = rclcpp::get_logger("moveit_fake_controller_manager").get_child("moveit_fake_controllers");

BaseFakeController::BaseFakeController(const std::string& name, const std::vector<std::string>& joints,
                                       const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub)
  : moveit_controller_manager::MoveItControllerHandle(name), joints_(joints), pub_(pub)
{
  std::stringstream ss;
  ss << "Fake controller '" << name << "' with joints [ ";
  std::copy(joints.begin(), joints.end(), std::ostream_iterator<std::string>(ss, " "));
  ss << "]";
  RCLCPP_INFO(LOGGER_FAKE_CONTROLLER, "%s", ss.str().c_str());
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
  RCLCPP_INFO(LOGGER_FAKE_CONTROLLER, "Fake execution of trajectory");
  if (t.joint_trajectory.points.empty())
    return true;

  sensor_msgs::msg::JointState js;
  const trajectory_msgs::msg::JointTrajectoryPoint& last = t.joint_trajectory.points.back();
  js.header = t.joint_trajectory.header;
  js.header.stamp = rclcpp::Clock().now();
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
  rclcpp::sleep_for(std::chrono::milliseconds(500)); // give some time to receive the published JointState
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
  RCLCPP_INFO(LOGGER_FAKE_CONTROLLER, "Fake trajectory execution cancelled");
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
  RCLCPP_INFO(LOGGER_FAKE_CONTROLLER, "Fake execution of trajectory");
  sensor_msgs::msg::JointState js;
  js.header = t.joint_trajectory.header;
  js.name = t.joint_trajectory.joint_names;

  // publish joint states for all intermediate via points of the trajectory
  // no further interpolation
  rclcpp::Time start_time = rclcpp::Clock().now();
  for (std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator via = t.joint_trajectory.points.begin(),
                                                                          end = t.joint_trajectory.points.end();
       !cancelled() && via != end; ++via)
  {
    js.position = via->positions;
    js.velocity = via->velocities;
    js.effort = via->effort;
    //TODO (anasarrak)
    rclcpp::Duration wait_time(via->time_from_start.sec - (rclcpp::Clock().now() - start_time).seconds());
    if (wait_time.seconds() > std::numeric_limits<float>::epsilon())
    {
      RCLCPP_DEBUG(LOGGER_FAKE_CONTROLLER, "Fake execution: waiting %0.1fs for next via point, %ld remaining", wait_time.seconds(), end - via);
      rclcpp::sleep_for(std::chrono::milliseconds((int)wait_time.seconds()));
    }
    js.header.stamp = rclcpp::Clock().now();
    pub_->publish(js);
  }
  RCLCPP_DEBUG(LOGGER_FAKE_CONTROLLER, "Fake execution of trajectory: done");
}

InterpolatingController::InterpolatingController(const std::string& name, const std::vector<std::string>& joints,
                                                 const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub,
                                                 const rclcpp::Node::SharedPtr& node_)
  : ThreadedController(name, joints, pub), rate_(10)
{
  auto fake_interpolating_controller_rate_param = std::make_shared<rclcpp::SyncParametersClient>(node_);
  double r;
  if(fake_interpolating_controller_rate_param->has_parameter("~fake_interpolating_controller_rate")){
    r = fake_interpolating_controller_rate_param->get_parameter("~fake_interpolating_controller_rate", 10);
  }
  rclcpp::WallRate rate_(r);
}

InterpolatingController::~InterpolatingController() = default;

namespace
{
void interpolate(sensor_msgs::msg::JointState& js, const trajectory_msgs::msg::JointTrajectoryPoint& prev,
                 const trajectory_msgs::msg::JointTrajectoryPoint& next, const rclcpp::Duration& elapsed)
{
  double duration = next.time_from_start.sec - prev.time_from_start.sec;
  double alpha = 1.0;
  if (duration > std::numeric_limits<double>::epsilon())
    alpha = (elapsed.seconds() - prev.time_from_start.sec) / duration;

  js.position.resize(prev.positions.size());
  for (std::size_t i = 0, end = prev.positions.size(); i < end; ++i)
  {
    js.position[i] = prev.positions[i] + alpha * (next.positions[i] - prev.positions[i]);
  }
}
}  // namespace

void InterpolatingController::execTrajectory(const moveit_msgs::msg::RobotTrajectory& t)
{
  RCLCPP_INFO(LOGGER_FAKE_CONTROLLER, "Fake execution of trajectory");
  if (t.joint_trajectory.points.empty())
    return;

  sensor_msgs::msg::JointState js;
  js.header = t.joint_trajectory.header;
  js.name = t.joint_trajectory.joint_names;

  const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& points = t.joint_trajectory.points;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator prev = points.begin(),  // previous via point
      next = points.begin() + 1,  // currently targetted via point
      end = points.end();

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (!cancelled())
  {
    rclcpp::Duration elapsed = rclcpp::Clock().now() - start_time;
    // hop to next targetted via point
    while (next != end && elapsed > next->time_from_start)
    {
      ++prev;
      ++next;
    }
    if (next == end)
      break;

    double duration = next->time_from_start.sec - prev->time_from_start.sec;
    RCLCPP_INFO(LOGGER_FAKE_CONTROLLER, "elapsed: %.3f via points %td,%td / %td  alpha: %.3f", elapsed.seconds(), prev - points.begin(),
              next - points.begin(), end - points.begin(),
              duration > std::numeric_limits<double>::epsilon() ? (elapsed.seconds() - prev->time_from_start.sec) / duration :
                                                                  1.0);
    interpolate(js, *prev, *next, elapsed);
    js.header.stamp = rclcpp::Clock().now();
    pub_->publish(js);
    rate_.sleep();
  }
  if (cancelled())
    return;

  rclcpp::Duration elapsed = rclcpp::Clock().now() - start_time;
  RCLCPP_INFO(LOGGER_FAKE_CONTROLLER, "elapsed: %.3f via points %td,%td / %td  alpha: 1.0", elapsed.seconds(), prev - points.begin(),
            next - points.begin(), end - points.begin());

  // publish last point
  interpolate(js, *prev, *prev, prev->time_from_start);
  js.header.stamp = rclcpp::Clock().now();
  pub_->publish(js);

  RCLCPP_INFO(LOGGER_FAKE_CONTROLLER, "Fake execution of trajectory: done");
}

}  // end namespace moveit_fake_controller_manager
