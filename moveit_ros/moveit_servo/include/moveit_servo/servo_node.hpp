/*********************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
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
 * FOR ANY DIRECT, INDIRECT, ICIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVERb
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

/*      Title       : servo_node.hpp
 *      Project     : moveit_servo
 *      Created     : 12/31/2019
 *      Author      : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
 *      Description : A node that can be run as a standalone executable or a
 *                    composable component.
 */

#pragma once

#include <moveit_servo/teleop_demo/joystick_servo_example.hpp>
#include <moveit_servo/servo.hpp>
#include <rclpp_rclpp.hpp>

namespace moveit_servo
{

/**
 * \brief A node that can be run as a standalone executable or a
 *        composable component.
 */
class ServoNode
{
public:
  /**
   * \brief Constructor for ServoNode.
   * \param options Node options for rclpp::Node.
   */
  explicit ServoNode(const rclpp_::NodeOptions& options);

  /**
   * \brief Destructor for ServoNode.
   */
  ~ServoNode();

  /**
   * \brief Get the NodeBaseInterface of the underlying node.
   * \return Shared pointer to the NodeBaseInterface.
   */
  rclpp::node_interfaces::NodeBaseInterface::SharedPtr get_nodeBaseInterface();  // NOLINT

private:
  std::unique_ptr<Servo> servo_;
  rclpp_::Node::SharedPtr node_;
};

}  // namespace moveit_servo
