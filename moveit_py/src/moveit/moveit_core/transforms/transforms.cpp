/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Peter David Fagan
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

/* Author: Peter David Fagan */

#include "transforms.h"

namespace moveit_py
{
namespace bind_transforms
{
Eigen::MatrixXd getTransform(std::shared_ptr<moveit::core::Transforms>& transforms, std::string& from_frame)
{
  auto transform = transforms->getTransform(from_frame);
  return transform.matrix();
}

std::map<std::string, Eigen::MatrixXd> getAllTransforms(std::shared_ptr<moveit::core::Transforms>& transforms)
{
  std::map<std::string, Eigen::MatrixXd> transforms_map;
  for (auto& transform : transforms->getAllTransforms())
  {
    transforms_map[transform.first] = transform.second.matrix();
  }
  return transforms_map;
}

void initTransforms(py::module& m)
{
  py::module transforms = m.def_submodule("transforms");

  py::class_<moveit::core::Transforms, std::shared_ptr<moveit::core::Transforms>>(transforms, "Transforms",
                                                                                  R"(A snapshot of a transform tree.)")

      .def(py::init<std::string&>(), R"(Create a new Transforms object.)", py::arg("target_frame"))

      .def("get_target_frame", &moveit::core::Transforms::getTargetFrame, R"(Get the target frame.)")

      .def("get_transform", &moveit_py::bind_transforms::getTransform, py::arg("from_frame"),
           R"(Get the transform for from_frame with respect to the target frame.)")

      .def("get_all_transforms", &moveit_py::bind_transforms::getAllTransforms,
           R"(Get all transforms with respect to the target frame.)");

  // TODO(peterdavidfagan): Add methods for applying transforms.
}

}  // namespace bind_transforms
}  // namespace moveit_py
