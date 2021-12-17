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

/*      Title     : servo_pipeline.cpp
 *      Project   : moveit_servo
 *      Created   : 11/06/2021
 *      Author    : Tyler Weaver
 */

#include <chrono>
#include <functional>
#include <memory>
#include <moveit_servo/detail/input_check_valid.hpp>
#include <moveit_servo/detail/input_command.hpp>
#include <moveit_servo/detail/input_resampler.hpp>
#include <moveit_servo/detail/input_stale_command.hpp>
#include <moveit_servo/detail/input_subscriber.hpp>
#include <moveit_servo/servo_parameters.h>
#include <rclcpp/rclcpp.hpp>

#include <moveit_servo/detail/servo_pipeline.hpp>

namespace moveit_servo
{
namespace detail
{
ServoPipeline::ServoPipeline(const rclcpp::Node::SharedPtr& node,
                             const std::shared_ptr<const moveit_servo::ServoParameters>& parameters,
                             std::function<void()> halt, const std::shared_ptr<InputVisitor>& next)
{
  assert(node != nullptr);
  assert(parameters != nullptr);
  assert(next != nullptr);

  // The servo pipeline is constructed in reverse order.
  // Starting with the last object and working in reverse.
  // This is done because each step needs a pointer to the next step to be able to call it.

  // The initial value of next_step is the visitor step after the last step in the servo pipeline.
  // In the default, the happy path each step calls the next step in the chain after it does its thing.
  // Note that after we construct each step we get a pointer to it in order to construct the next step.
  std::shared_ptr<InputVisitor> next_step = next;

  // The reason these are unique_ptr inside of a vector is we hand a non-owning pointer to each one
  //  to the step that precedes it.
  // This means that the address of the pointer does not change if the ServoPipeline object is moved.

  // This is the default case where we are not in low_latency_mode and we want to resample input.
  //
  // StaleCommandHalt
  //   description: checks timeout and calls next if the message is not stale
  //   failure: on timeout calls the halt function
  // InputResampler
  //   description: call StaleCommandHalt with stored input command at a set rate
  // TimestampNow
  //   descritption: sets the timestamp in header to now(), then calls InputResampler
  //
  // next_step is set to TimestampNow
  if (!parameters->low_latency_mode)
  {
    next_step = input_visitors_.emplace_back(std::make_shared<StaleCommandHalt>(
        node, rclcpp::Duration::from_seconds(parameters->incoming_command_timeout), halt, next_step));
    next_step = input_visitors_.emplace_back(std::make_shared<InputResampler>(
        node,
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(parameters->publish_period)),
        next_step));
    next_step = input_visitors_.emplace_back(std::make_shared<TimestampNow>(node, next_step));
  }

  // InputCheckValid
  //   description: checks if the intput message is valid then calls next_step
  //   failure: logs warning and does not call next_step
  next_step =
      input_visitors_.emplace_back(std::make_shared<InputCheckValid>(node, parameters->command_in_type, next_step));

  // InputSubscriber
  //   description: calls InputCheckValid with commands from ros subscribers
  input_subscriber_ = std::make_unique<InputSubscriber>(node, parameters->cartesian_command_in_topic,
                                                        parameters->joint_command_in_topic, next_step);
}

}  // namespace detail
}  // namespace moveit_servo
