/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title     : input_resampler.hpp
 *      Project   : moveit_servo
 *      Created   : 11/06/2021
 *      Author    : Tyler Weaver
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <variant>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_servo/detail/input_command.hpp>
#include <rclcpp/rclcpp.hpp>

namespace moveit_servo
{
namespace detail
{
/**
 * @brief      Visitor that stores the the input command and then calls a visitor with a timer at a specified rate
 */
class InputResampler : public InputVisitor
{
  rclcpp::Node::SharedPtr node_ = nullptr;
  rclcpp::Rate rate_;
  std::shared_ptr<InputVisitor> next_ = nullptr;
  mutable std::mutex mutex_;
  InputCommand command_;
  std::thread thread_;
  std::atomic<bool> running_ = false;

  // Create a copy of the message so we don't have to lock durring the call to visit
  InputCommand getCommandCopy() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return command_;
  }

  // TODO(tylerjw): convert this back to a ROS timer after the performance of ros timers has been profiled
  void threadMain()
  {
    while (running_ and rclcpp::ok())
    {
      std::visit(*next_, getCommandCopy());
      rate_.sleep();
    }
  }

  void startThread()
  {
    running_ = true;
    thread_ = std::thread([&]() { threadMain(); });
  }

  void stopThread()
  {
    if (running_)
    {
      running_ = false;
      thread_.join();
    }
  }

public:
  InputResampler(const rclcpp::Node::SharedPtr& node, std::chrono::nanoseconds period,
                 const std::shared_ptr<InputVisitor>& next)
    : node_{ node }, rate_{ period }, next_{ next }, running_{ false }
  {
    assert(node_ != nullptr);
    assert(next_ != nullptr);
  }

  ~InputResampler() override
  {
    stopThread();
  }

  void operator()(const geometry_msgs::msg::TwistStamped& command) override
  {
    operatorImpl(command);
  }

  void operator()(const control_msgs::msg::JointJog& command) override
  {
    operatorImpl(command);
  }

private:
  template <typename T>
  void operatorImpl(const T& command)
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      command_ = command;
    }
    if (!running_)
    {
      startThread();
    }
  }
};

}  // namespace detail
}  // namespace moveit_servo
