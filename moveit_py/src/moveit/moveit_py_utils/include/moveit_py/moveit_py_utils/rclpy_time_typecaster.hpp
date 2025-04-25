/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Shobin Vinod
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Shobin Vinod */

#include <pybind11/pybind11.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/duration.hpp>

namespace py = pybind11;

namespace pybind11
{
namespace detail
{
template <>
struct type_caster<rclcpp::Time>
{
  PYBIND11_TYPE_CASTER(rclcpp::Time, _("rclcpp::Time"));

  // convert from rclpy::Time to rclcpp::Time
  bool load(py::handle src, bool)
  {
    if (src.is_none())
      return false;

    // check to validate if the object is a rclcpp::Time object
    if (!py::hasattr(src, "nanoseconds") || !py::hasattr(src, "clock_type"))
    {
      return false;
    }

    // Extract the value for constructing the rclcpp::Time object
    int64_t nanoseconds = src.attr("nanoseconds").cast<int64_t>();
    int clock_type = src.attr("clock_type").cast<int>();

    // Construct the rclcpp::Time object
    value = rclcpp::Time(nanoseconds, static_cast<rcl_clock_type_t>(clock_type));
    return true;
  }

  // convert from rclcpp::Time to rclpy::Time
  static py::handle cast(const rclcpp::Time& src, return_value_policy /* policy */, py::handle /* parent */)
  {
    py::module rclpy_time = py::module::import("rclpy.time");
    py::object Time = rclpy_time.attr("Time");

    int64_t nanoseconds = src.nanoseconds();
    int clock_type = static_cast<int>(src.get_clock_type());

    return Time(py::arg("nanoseconds") = nanoseconds,
                py::arg("clock_type") = clock_type)
        .release();  // release the ownership of the object
  }
};
}  // namespace detail
}  // namespace pybind11
