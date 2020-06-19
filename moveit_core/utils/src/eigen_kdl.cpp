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

#include <moveit/utils/eigen_kdl.h>

namespace moveit
{
namespace core
{
void quaternionKDLToEigen(const KDL::Rotation& k, Eigen::Quaterniond& e)
{
  // // is it faster to do this?
  k.GetQuaternion(e.x(), e.y(), e.z(), e.w());

  // or this?
  // double x, y, z, w;
  // k.GetQuaternion(x, y, z, w);
  // e = Eigen::Quaterniond(w, x, y, z);
}

void quaternionEigenToKDL(const Eigen::Quaterniond& e, KDL::Rotation& k)
{
  k = KDL::Rotation::Quaternion(e.x(), e.y(), e.z(), e.w());
}

namespace
{
template <typename T>
void transformKDLToEigenImpl(const KDL::Frame& k, T& e)
{
  // translation
  for (unsigned int i = 0; i < 3; ++i)
    e(i, 3) = k.p[i];

  // rotation matrix
  for (unsigned int i = 0; i < 9; ++i)
    e(i / 3, i % 3) = k.M.data[i];

  // "identity" row
  e(3, 0) = 0.0;
  e(3, 1) = 0.0;
  e(3, 2) = 0.0;
  e(3, 3) = 1.0;
}

template <typename T>
void transformEigenToKDLImpl(const T& e, KDL::Frame& k)
{
  for (unsigned int i = 0; i < 3; ++i)
    k.p[i] = e(i, 3);
  for (unsigned int i = 0; i < 9; ++i)
    k.M.data[i] = e(i / 3, i % 3);
}
}  // namespace

void transformKDLToEigen(const KDL::Frame& k, Eigen::Affine3d& e)
{
  transformKDLToEigenImpl(k, e);
}

void transformKDLToEigen(const KDL::Frame& k, Eigen::Isometry3d& e)
{
  transformKDLToEigenImpl(k, e);
}

void transformEigenToKDL(const Eigen::Affine3d& e, KDL::Frame& k)
{
  transformEigenToKDLImpl(e, k);
}

void transformEigenToKDL(const Eigen::Isometry3d& e, KDL::Frame& k)
{
  transformEigenToKDLImpl(e, k);
}

void twistEigenToKDL(const Eigen::Matrix<double, 6, 1>& e, KDL::Twist& k)
{
  for (int i = 0; i < 6; ++i)
    k[i] = e[i];
}

void twistKDLToEigen(const KDL::Twist& k, Eigen::Matrix<double, 6, 1>& e)
{
  for (int i = 0; i < 6; ++i)
    e[i] = k[i];
}

void vectorEigenToKDL(const Eigen::Matrix<double, 3, 1>& e, KDL::Vector& k)
{
  for (int i = 0; i < 3; ++i)
    k[i] = e[i];
}
void vectorKDLToEigen(const KDL::Vector& k, Eigen::Matrix<double, 3, 1>& e)
{
  for (int i = 0; i < 3; ++i)
    e[i] = k[i];
}

void wrenchKDLToEigen(const KDL::Wrench& k, Eigen::Matrix<double, 6, 1>& e)
{
  for (int i = 0; i < 6; ++i)
    e[i] = k[i];
}

void wrenchEigenToKDL(const Eigen::Matrix<double, 6, 1>& e, KDL::Wrench& k)
{
  for (int i = 0; i < 6; ++i)
    k[i] = e[i];
}
}  // namespace core
}  // namespace moveit
