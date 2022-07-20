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

/* Author: Larry Lu, Ioan Sucan */

#pragma once

#include <pybind11/pybind11.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <string>
#include <stdexcept>

namespace py = pybind11;

namespace moveit
{
namespace py_bindings_tools
{

/** \brief Convert a ROS message to a Python Bytes */
template <typename T, typename std::enable_if<rosidl_generator_traits::is_message<T>::value, int>::type = 0>
py::bytes serializeMsg(const T& msg)
{
  rclcpp::Serialization<T> serializer;
  rclcpp::SerializedMessage serialized_msg;
  serializer.serialize_message(&msg, &serialized_msg);
  return py::bytes(reinterpret_cast<const char*>(serialized_msg.get_rcl_serialized_message().buffer),
                   serialized_msg.get_rcl_serialized_message().buffer_length);
}

/** \brief Convert a Python Bytes to a ROS message */
template <typename T, typename std::enable_if<rosidl_generator_traits::is_message<T>::value, int>::type = 0>
void deserializeMsg(py::bytes data, T& msg)
{
  rcl_serialized_message_t rcl_serialized_msg = rmw_get_zero_initialized_serialized_message();
  char* serialized_buffer;
  Py_ssize_t length;
  if (PYBIND11_BYTES_AS_STRING_AND_SIZE(data.ptr(), &serialized_buffer, &length))
  {
    throw py::error_already_set();
  }
  if (length < 0)
  {
    throw py::error_already_set();
  }
  rcl_serialized_msg.buffer_capacity = length;
  rcl_serialized_msg.buffer_length = length;
  rcl_serialized_msg.buffer = reinterpret_cast<uint8_t*>(serialized_buffer);
  rmw_ret_t rmw_ret =
      rmw_deserialize(&rcl_serialized_msg, rosidl_typesupport_cpp::get_message_type_support_handle<T>(), &msg);
  if (RMW_RET_OK != rmw_ret)
  {
    throw std::runtime_error("failed to deserialize ROS message");
  }
}

}  // namespace py_bindings_tools
}  // namespace moveit
