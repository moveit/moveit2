/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Adam Leeper, Stuart Glaser
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <kdl/frames.hpp>

namespace moveit
{
namespace core
{
/// Converts a KDL rotation into an Eigen quaternion
void quaternionKDLToEigen(const KDL::Rotation& k, Eigen::Quaterniond& e);

/// Converts an Eigen quaternion into a KDL rotation
void quaternionEigenToKDL(const Eigen::Quaterniond& e, KDL::Rotation& k);

/// Converts a KDL frame into an Eigen Affine3d
void transformKDLToEigen(const KDL::Frame& k, Eigen::Affine3d& e);

/// Converts a KDL frame into an Eigen Isometry3d
void transformKDLToEigen(const KDL::Frame& k, Eigen::Isometry3d& e);

/// Converts an Eigen Affine3d into a KDL frame
void transformEigenToKDL(const Eigen::Affine3d& e, KDL::Frame& k);

/// Converts an Eigen Isometry3d into a KDL frame
void transformEigenToKDL(const Eigen::Isometry3d& e, KDL::Frame& k);

/// Converts a KDL twist into an Eigen matrix
void twistKDLToEigen(const KDL::Twist& k, Eigen::Matrix<double, 6, 1>& e);

/// Converts an Eigen matrix into a KDL Twist
void twistEigenToKDL(const Eigen::Matrix<double, 6, 1>& e, KDL::Twist& k);

/// Converts a KDL vector into an Eigen matrix
void vectorKDLToEigen(const KDL::Vector& k, Eigen::Matrix<double, 3, 1>& e);

/// Converts an Eigen matrix into a KDL vector
void vectorEigenToKDL(const Eigen::Matrix<double, 3, 1>& e, KDL::Vector& k);

/// Converts a KDL wrench into an Eigen matrix
void wrenchKDLToEigen(const KDL::Wrench& k, Eigen::Matrix<double, 6, 1>& e);

/// Converts an Eigen matrix into a KDL wrench
void wrenchEigenToKDL(const Eigen::Matrix<double, 6, 1>& e, KDL::Wrench& k);

}  // namespace core
}  // namespace moveit
