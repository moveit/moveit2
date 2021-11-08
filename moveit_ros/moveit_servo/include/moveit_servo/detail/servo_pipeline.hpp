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

/*      Title     : servo_pipeline.hpp
 *      Project   : moveit_servo
 *      Created   : 11/06/2021
 *      Author    : Tyler Weaver
 */

#pragma once

#include <functional>
#include <memory>
#include <moveit_servo/detail/input_command.hpp>
#include <moveit_servo/detail/input_subscriber.hpp>
#include <moveit_servo/servo_parameters.h>
#include <rclcpp/rclcpp.hpp>

namespace moveit_servo
{
namespace detail
{
class ServoPipeline
{
  std::vector<std::unique_ptr<InputVisitor>> input_visitors_;
  std::unique_ptr<InputSubscriber> input_subscriber_;

public:
  /**
   * @brief ServoPipeline contains all the servo steps
   * @details Currently this only contains steps for the input, hence the next pointer
   *
   * @param node ROS node
   * @param halt function for sending a message that will stop the robot
   * @param next the next visitor to execute after the pipeline, normally ServoCalcs
   */
  ServoPipeline(rclcpp::Node::SharedPtr node, std::shared_ptr<const moveit_servo::ServoParameters> parameters,
                std::function<void()> halt, InputVisitor* next);
};

}  // namespace detail
}  // namespace moveit_servo
