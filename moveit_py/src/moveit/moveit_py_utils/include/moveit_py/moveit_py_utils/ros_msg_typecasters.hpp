/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Peter David Fagan, Robert Haschke
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
 *   * The name of Robert Haschke may not be use to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
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

/* Author: Peter David Fagan, Robert Haschke */

#pragma once

#include <pybind11/pybind11.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/duration.hpp>

namespace py = pybind11;

namespace moveit_py
{
namespace moveit_py_utils
{
PYBIND11_EXPORT pybind11::object createMessage(const std::string& ros_msg_name);
PYBIND11_EXPORT bool convertible(const pybind11::handle& h, const std::string& ros_msg_name);
}  // namespace moveit_py_utils
}  // namespace moveit_py

namespace pybind11
{
namespace detail
{
// Base class for type conversion (C++ <-> python) of ROS message types
template <typename T>
struct RosMsgTypeCaster
{
  // C++ -> Python
  // TODO: add error handling
  static handle cast(const T& src, return_value_policy /* policy */, handle /* parent */)
  {
    // serialize src
    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&src, &serialized_msg);
    py::bytes bytes = py::bytes(reinterpret_cast<const char*>(serialized_msg.get_rcl_serialized_message().buffer),
                                serialized_msg.get_rcl_serialized_message().buffer_length);

    // get Python object type
    const std::string ros_msg_name = rosidl_generator_traits::name<T>();

    // find delimiting '/' in ros_msg_name
    std::size_t pos1 = ros_msg_name.find('/');
    std::size_t pos2 = ros_msg_name.find('/', pos1 + 1);
    py::module m = py::module::import((ros_msg_name.substr(0, pos1) + ".msg").c_str());

    // retrieve type instance
    py::object cls = m.attr(ros_msg_name.substr(pos2 + 1).c_str());

    // deserialize into python object
    py::module rclpy = py::module::import("rclpy.serialization");
    py::object msg = rclpy.attr("deserialize_message")(bytes, cls);

    return msg.release();
  }

  // Python -> C++
  bool load(handle src, bool /*convert*/)
  {
    // check datatype of src
    if (!moveit_py::moveit_py_utils::convertible(src, rosidl_generator_traits::name<T>()))
      return false;

    // serialize src into python buffer
    py::module rclpy = py::module::import("rclpy.serialization");
    py::bytes bytes = rclpy.attr("serialize_message")(src);

    // deserialize into C++ object
    rcl_serialized_message_t rcl_serialized_msg = rmw_get_zero_initialized_serialized_message();
    char* serialized_buffer;
    Py_ssize_t length;
    if (PYBIND11_BYTES_AS_STRING_AND_SIZE(bytes.ptr(), &serialized_buffer, &length))
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
        rmw_deserialize(&rcl_serialized_msg, rosidl_typesupport_cpp::get_message_type_support_handle<T>(), &value);
    if (RMW_RET_OK != rmw_ret)
    {
      throw std::runtime_error("failed to deserialize ROS message");
    }
    return true;
  }

  PYBIND11_TYPE_CASTER(T, _<T>());
};

template <typename T>
struct type_caster<T, enable_if_t<rosidl_generator_traits::is_message<T>::value>> : RosMsgTypeCaster<T>
{
};

}  // namespace detail
}  // namespace pybind11
