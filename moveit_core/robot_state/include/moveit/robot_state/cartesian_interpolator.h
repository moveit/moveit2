/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  Copyright (c) 2019, PickNik Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

/* Author: Ioan Sucan, Mike Lautman */

#pragma once

#include <moveit/robot_state/robot_state.h>

namespace moveit
{
namespace core
{
/** Struct defining linear and rotational precision */
struct CartesianPrecision
{
  double translational = 0.001;  //< max deviation in translation (meters)
  double rotational = 0.01;      //< max deviation in rotation (radians)
  double max_resolution = 1e-5;  //< max resolution for waypoints (fraction of total path)
};

/** \brief Struct with options for defining joint-space jump thresholds. */
struct JumpThreshold
{
  JumpThreshold() = default;  // default is equivalent to disabled().

  /** \brief Do not define any jump threshold, i.e., disable joint-space jump detection. */
  static JumpThreshold disabled();

  /** \brief Detect joint-space jumps relative to the average joint-space displacement along the path.

     The average joint-space distance between consecutive points in the path is computed. If any individual joint-space
     motion delta is larger than the average distance by a factor of `relative_factor`, it is considered the path has a
     jump. For instance, a `relative_factor` of 10.0 will detect joint increments larger than 10x the average increment
   */
  static JumpThreshold relative(double relative_factor);

  /** \brief Detect joint-space jumps greater than the given absolute thresholds.

     `revolute` and `prismatic` are absolute joint displacement thresholds, in radians and meters respectively.
     If any two consecutive waypoints have a joint-space distance larger than these values, the path has a jump. */
  static JumpThreshold absolute(double revolute, double prismatic);

  double relative_factor = 0.0;
  double revolute = 0.0;   // Radians
  double prismatic = 0.0;  // Meters

  // Deprecated constructors. Construct using the builder methods above.
  [[deprecated("Use JumpThreshold::relative() instead.")]] JumpThreshold(double relative_factor);
  [[deprecated("Use JumpThreshold::absolute() instead.")]] JumpThreshold(double revolute, double prismatic);
};

/** \brief Struct for containing max_step for computeCartesianPath

    Setting translation to zero will disable checking for translations. The same goes for rotation.
    Initializing with only one value (translation) sets the rotation such that
    1 cm of allowed translation = 2 degrees of allowed rotation. */
struct MaxEEFStep
{
  MaxEEFStep(double translation, double rotation) : translation(translation), rotation(rotation)
  {
  }

  MaxEEFStep(double step_size) : translation(step_size), rotation(3.5 * step_size)  // 0.035 rad = 2 deg
  {
  }

  double translation;  // Meters
  double rotation;     // Radians
};

class CartesianInterpolator
{
  // TODO(mlautman): Eventually, this planner should be moved out of robot_state

public:
  struct Percentage
  {
    // value must be in [0,1]
    Percentage(double value) : value(value)
    {
      if (value < 0.0 || value > 1.0)
        throw std::runtime_error("Percentage values must be between 0 and 1, inclusive");
    }
    operator double()
    {
      return value;
    }
    double operator*()
    {
      return value;
    }
    Percentage operator*(const Percentage& p)
    {
      Percentage res(value * p.value);
      return res;
    }
    double value;
  };

  struct Distance
  {
    Distance(double meters) : meters(meters)
    {
    }
    operator double()
    {
      return meters;
    }
    double operator*()
    {
      return meters;
    }
    Distance operator*(const Percentage& p)
    {
      Distance res(meters * p.value);
      return res;
    }
    double meters;
  };

  /** \brief Compute the sequence of joint values that correspond to a straight Cartesian path for a particular link.

     The Cartesian path to be followed is specified as a \e translation vector to be followed by the robot \e link.
     This vector is assumed to be specified either in the global reference frame or in the local
     reference frame of the link.
     The resulting joint values are stored in the vector \e traj, one by one. The interpolation distance in
     Cartesian space between consecutive points on the resulting path is specified in the \e MaxEEFStep struct which
     provides two fields: translation and rotation. If a \e validCallback is specified, this is passed to the internal
     call to setFromIK(). In case of IK failure, the computation of the path stops and the value returned corresponds to
     the distance that was achieved and for which corresponding states were added to the path.

     The struct CartesianPrecision specifies the precision to which the path should follow the Cartesian straight line.
     If the deviation at the mid point of two consecutive waypoints is larger than the specified precision, another waypoint
     will be inserted at that mid point. The precision is specified separately for translation and rotation.
     The maximal resolution to consider (as fraction of the total path length) is specified by max_resolution.
  */
  static Distance computeCartesianPath(
      const RobotState* start_state, const JointModelGroup* group, std::vector<std::shared_ptr<RobotState>>& traj,
      const LinkModel* link, const Eigen::Vector3d& translation, bool global_reference_frame,
      const MaxEEFStep& max_step, const CartesianPrecision& precision,
      const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
      const kinematics::KinematicsBase::IKCostFn& cost_function = kinematics::KinematicsBase::IKCostFn());

  /** \brief Compute the sequence of joint values that correspond to a straight Cartesian path, for a particular link.

     In contrast to the previous function, the translation vector is specified as a (unit) direction vector and
     a distance. */
  static Distance computeCartesianPath(
      const RobotState* start_state, const JointModelGroup* group, std::vector<std::shared_ptr<RobotState>>& traj,
      const LinkModel* link, const Eigen::Vector3d& direction, bool global_reference_frame, double distance,
      const MaxEEFStep& max_step, const CartesianPrecision& precision,
      const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
      const kinematics::KinematicsBase::IKCostFn& cost_function = kinematics::KinematicsBase::IKCostFn())
  {
    return computeCartesianPath(start_state, group, traj, link, distance * direction, global_reference_frame, max_step,
                                precision, validCallback, options, cost_function);
  }

  /** \brief Compute the sequence of joint values that correspond to a straight Cartesian path, for a particular frame.

     In contrast to the previous function, the Cartesian path is specified as a target frame to be reached (\e target)
     for a virtual frame attached to the robot \e link with the given \e link_offset.
     The target frame is assumed to be specified either w.r.t. to the global reference frame or the virtual link frame.
     This function returns the fraction (0..1) of path that was achieved. All other comments from the previous function apply. */
  static Percentage computeCartesianPath(
      const RobotState* start_state, const JointModelGroup* group, std::vector<RobotStatePtr>& traj,
      const LinkModel* link, const Eigen::Isometry3d& target, bool global_reference_frame, const MaxEEFStep& max_step,
      const CartesianPrecision& precision, const GroupStateValidityCallbackFn& validCallback,
      const kinematics::KinematicsQueryOptions& options,
      const kinematics::KinematicsBase::IKCostFn& cost_function = kinematics::KinematicsBase::IKCostFn(),
      const Eigen::Isometry3d& link_offset = Eigen::Isometry3d::Identity());

  /** \brief Compute the sequence of joint values that perform a general Cartesian path.

     In contrast to the previous functions, the Cartesian path is specified as a set of \e waypoints to be sequentially
     reached by the virtual frame attached to the robot \e link. The waypoints are transforms given either w.r.t. the global
     reference frame or the virtual frame at the immediately preceding waypoint. The virtual frame needs
     to move in a straight line between two consecutive waypoints. All other comments apply. */
  static Percentage computeCartesianPath(
      const RobotState* start_state, const JointModelGroup* group, std::vector<std::shared_ptr<RobotState>>& traj,
      const LinkModel* link, const EigenSTL::vector_Isometry3d& waypoints, bool global_reference_frame,
      const MaxEEFStep& max_step, const CartesianPrecision& precision,
      const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
      const kinematics::KinematicsBase::IKCostFn& cost_function = kinematics::KinematicsBase::IKCostFn(),
      const Eigen::Isometry3d& link_offset = Eigen::Isometry3d::Identity());

  /** \brief Compute the sequence of joint values that correspond to a straight Cartesian path for a particular link.

     The Cartesian path to be followed is specified as a \e translation vector to be followed by the robot \e link.
     This vector is assumed to be specified either in the global reference frame or in the local
     reference frame of the link (\e global_reference_frame is false).
     The resulting joint values are stored in the vector \e path, one by one. The maximum distance in
     Cartesian space between consecutive points on the resulting path is specified in the \e MaxEEFStep struct which
     provides two fields: translation and rotation. If a \e validCallback is specified, this is passed to the internal
     call to setFromIK(). In case of IK failure, the computation of the path stops and the value returned corresponds to
     the distance that was achieved and for which corresponding states were added to the path. At the end of the
     function call, the state of the group corresponds to the last attempted Cartesian pose.

     During the computation of the path, it is usually preferred if consecutive joint values do not 'jump' by a
     large amount in joint space, even if the Cartesian distance between the corresponding points is small as expected.
     To account for this, the \e jump_threshold struct is provided, which comprises three fields:
     \e jump_threshold_factor, \e revolute_jump_threshold and \e prismatic_jump_threshold.
     If either \e revolute_jump_threshold or \e prismatic_jump_threshold  are non-zero, we test for absolute jumps.
     If \e jump_threshold_factor is non-zero, we test for relative jumps. To this end, the average joint-space distance
     between consecutive points in the trajectory is computed. If any individual joint-space motion delta is larger than
     this average distance multiplied by \e jump_threshold_factor, this step is considered a jump.

     Otherwise (if all params are zero), jump detection is disabled.
     If a jump is detected, the path is truncated up to just before the jump.

     Kinematics solvers may use cost functions to prioritize certain solutions, which may be specified with \e
     cost_function. */
  [[deprecated("Replace JumpThreshold with CartesianPrecision")]] static Distance computeCartesianPath(
      RobotState* start_state, const JointModelGroup* group, std::vector<std::shared_ptr<RobotState>>& path,
      const LinkModel* link, const Eigen::Vector3d& translation, bool global_reference_frame,
      const MaxEEFStep& max_step, const JumpThreshold& jump_threshold,
      const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
      const kinematics::KinematicsBase::IKCostFn& cost_function = kinematics::KinematicsBase::IKCostFn());

  /** \brief Compute the sequence of joint values that correspond to a straight Cartesian path, for a particular link.

     In contrast to the previous function, the translation vector is specified as a (unit) direction vector and
     a distance. */
  [[deprecated("Replace JumpThreshold with CartesianPrecision")]] static Distance computeCartesianPath(
      RobotState* start_state, const JointModelGroup* group, std::vector<std::shared_ptr<RobotState>>& path,
      const LinkModel* link, const Eigen::Vector3d& direction, bool global_reference_frame, double distance,
      const MaxEEFStep& max_step, const JumpThreshold& jump_threshold,
      const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
      const kinematics::KinematicsBase::IKCostFn& cost_function = kinematics::KinematicsBase::IKCostFn())
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    return computeCartesianPath(start_state, group, path, link, distance * direction, global_reference_frame, max_step,
                                jump_threshold, validCallback, options, cost_function);
#pragma GCC diagnostic pop
  }

  /** \brief Compute the sequence of joint values that correspond to a straight Cartesian path, for a particular frame.

     In contrast to the previous function, the Cartesian path is specified as a target frame to be reached (\e target)
     for a virtual frame attached to the robot \e link with the given \e link_offset.
     The target frame is assumed to be specified either w.r.t. to the global reference frame or the virtual link frame
     (\e global_reference_frame is false). This function returns the percentage (0..1) of the path that was achieved.
     All other comments from the previous function apply. */
  [[deprecated("Replace JumpThreshold with CartesianPrecision")]] static Percentage computeCartesianPath(
      RobotState* start_state, const JointModelGroup* group, std::vector<std::shared_ptr<RobotState>>& path,
      const LinkModel* link, const Eigen::Isometry3d& target, bool global_reference_frame, const MaxEEFStep& max_step,
      const JumpThreshold& jump_threshold,
      const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
      const kinematics::KinematicsBase::IKCostFn& cost_function = kinematics::KinematicsBase::IKCostFn(),
      const Eigen::Isometry3d& link_offset = Eigen::Isometry3d::Identity());

  /** \brief Compute the sequence of joint values that perform a general Cartesian path.

     In contrast to the previous functions, the Cartesian path is specified as a set of \e waypoints to be sequentially
     reached by the virtual frame attached to the robot \e link. The waypoints are transforms given either w.r.t. the
     global reference frame or the virtual frame at the immediately preceding waypoint. The virtual frame needs to move
     in a straight line between two consecutive waypoints. All other comments apply. */
  [[deprecated("Replace JumpThreshold with CartesianPrecision")]] static Percentage computeCartesianPath(
      RobotState* start_state, const JointModelGroup* group, std::vector<std::shared_ptr<RobotState>>& path,
      const LinkModel* link, const EigenSTL::vector_Isometry3d& waypoints, bool global_reference_frame,
      const MaxEEFStep& max_step, const JumpThreshold& jump_threshold,
      const GroupStateValidityCallbackFn& validCallback = GroupStateValidityCallbackFn(),
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
      const kinematics::KinematicsBase::IKCostFn& cost_function = kinematics::KinematicsBase::IKCostFn(),
      const Eigen::Isometry3d& link_offset = Eigen::Isometry3d::Identity());

  /** \brief Checks if a path has a joint-space jump, and truncates the path at the jump.

     Checks if a path has a jump larger than `jump_threshold` and truncates the path to the waypoint right before the
     jump. Returns the percentage of the path that doesn't have jumps.

     @param group The joint model group of the robot state.
     @param path The path that should be tested.
     @param jump_threshold The struct holding jump thresholds to determine if a joint space jump has occurred.
     @return The fraction of the path that passed.
  */
  static Percentage checkJointSpaceJump(const JointModelGroup* group, std::vector<std::shared_ptr<RobotState>>& path,
                                        const JumpThreshold& jump_threshold);
};

/** \brief Checks if a joint-space path has a jump larger than the given threshold.

   This function computes the distance between every pair of adjacent waypoints (for the given group) and checks if that
   distance is larger than the threshold defined by `jump_threshold`. If so, it is considered that the path has a
   jump, and the path index where the jump happens is returned as output.
   Otherwise the function returns a nullopt. */
std::optional<int> hasJointSpaceJump(const std::vector<moveit::core::RobotStatePtr>& waypoints,
                                     const moveit::core::JointModelGroup& group,
                                     const moveit::core::JumpThreshold& jump_threshold);

}  // end of namespace core
}  // end of namespace moveit
