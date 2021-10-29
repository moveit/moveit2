// Copyright 2021 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Tyler Weaver
   Description: Builder for the MoveItErrorCode message
*/

#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace moveit::msg_builders
{
class MoveItErrorCodesBuilder
{
  moveit_msgs::msg::MoveItErrorCodes msg_;

public:
  MoveItErrorCodesBuilder()
  {
    msg_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }

  explicit MoveItErrorCodesBuilder(int code)
  {
    msg_.val = code;
  }

  explicit MoveItErrorCodesBuilder(const moveit_msgs::msg::MoveItErrorCodes& msg)
  {
    msg_ = msg;
  }

  MoveItErrorCodesBuilder& code(int code)
  {
    msg_.val = code;
    return *this;
  }

  explicit operator bool() const
  {
    return msg_.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }

  explicit operator int() const
  {
    return msg_.val;
  }

  operator moveit_msgs::msg::MoveItErrorCodes() const
  {
    return msg_;
  }

  bool operator==(const int code) const
  {
    return msg_.val == code;
  }

  bool operator!=(const int code) const
  {
    return msg_.val != code;
  }
};

}  // namespace moveit::msg_builders
