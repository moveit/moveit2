/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Larry Lu, Inc.
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
 *   * Neither the name of Larry Lu nor the names of its contributors
 *     may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
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

#include <rclcpp/rclcpp.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <moveit/py_bindings_tools/py_conversions.h>

namespace py = pybind11;

namespace moveit
{
namespace planning_interface
{

void init(py::list& arg)
{
  if (!rclcpp::ok())
  {
    std::vector<std::string> args = py_bindings_tools::stringFromList(arg);
    int argc = args.size();
    char** argv = new char*[args.size()];
    for (std::size_t i = 0; i < args.size(); ++i)
      argv[i] = strdup(args[i].c_str());
    rclcpp::init(argc, argv);
  }
}

void shutdown()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

}  // namespace planning_interface
}  // namespace moveit

using namespace moveit::planning_interface;

PYBIND11_MODULE(rclcpp_interface, m)
{
  m.doc() = "MOVEIT2 rclcpp interface.";
  m.def("rclcpp_init", &init);
  m.def("rclcpp_shutdown", &shutdown);
}