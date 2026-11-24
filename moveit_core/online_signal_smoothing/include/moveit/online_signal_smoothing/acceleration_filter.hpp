/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, PickNik Inc.
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

/* Author: Paul Gesel
Description: applies smoothing by limiting the acceleration between consecutive commands.
The purpose of the plugin is to prevent the robot's acceleration limits from being violated by instantaneous changes
to the servo command topics.

 In the diagrams below, the c-v lines show the displacement that will occur given the current velocity. The t-c lines
 shows the displacement between the current position and the desired position. The dashed lines shows the maximum
 possible displacements that are within the acceleration limits. The v-t lines shows the acceleration commands that
 will be used by this acceleration-limiting plugin. The x point shows the position that will be used for each scenario.

Scenario A: The desired position is within the acceleration limits. The next commanded point will be exactly the
desired point.
     ________
    |        |
c --|-----xt |
 \__|__ v    |
    |________|

Scenario B: The line between the current position and the desired position intersects the acceleration limits, but the
reference position is not within the bounds. The next commanded point will be the point on the displacement line that
is closest to the reference.
     ________
    |        |
c --|--------x------t
 \__|__ v    |
    |________|

Scenario C: Neither the displacement line intersects the acceleration limits nor does the reference point lie within
the limits. In this case, the next commanded point will be the one that minimizes the robot's velocity while
maintaining its direction.
           ________
          |        |
c --------x--- v   |
 \        |        |
  \       |________|
   t
 */

#pragma once

#include <cstddef>

#include <moveit/online_signal_smoothing/smoothing_base_class.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/utils/logger.hpp>
#include <moveit_core/moveit_acceleration_filter_parameters.hpp>

#include <osqp.h>
#include <cstddef>  // for size_t
#include <Eigen/Sparse>

// OSQP shipped v1.0 in 2024 with a redesigned C API. Neither v0.6.x nor v1.0
// exposes a numeric version macro that's usable from a preprocessor #if
// (v0.6's OSQP_VERSION is a string literal in constants.h; v1's numeric
// macros live in a non-installed private/version.h), so we don't have a
// clean OSQP-provided switch.
//
// Preferred signal: MOVEIT_OSQP_V1 is defined at compile time from
// moveit_core/online_signal_smoothing/CMakeLists.txt based on the version
// find_package(osqp) actually resolved. That keeps compile-time API selection
// consistent with link-time library selection when a build environment has
// BOTH v0.6 (apt) and v1.0 (source-overlay) installed.
//
// Fallback for tooling that doesn't run through our CMakeLists (LSP,
// clang-tidy stand-alone, etc.): probe for a v1-only header. The osqp CMake
// target sets INTERFACE_INCLUDE_DIRECTORIES to ${prefix}/include/osqp, so
// headers are searched directly without an `osqp/` prefix.
//
// TODO(#3786): remove the v0.6.x branch once every supported distro consumes
// an osqp_vendor >= 1.0.0. Concrete trigger: Humble EOL (2027-05) or when
// Jazzy/Kilted release an osqp_vendor 1.0 of their own. See #3751 for the
// distro-compat-guard policy.
#ifndef MOVEIT_OSQP_V1
#if __has_include(<osqp_api_types.h>)
#define MOVEIT_OSQP_V1 1
#else
#define MOVEIT_OSQP_V1 0
#endif
#endif

namespace online_signal_smoothing
{
MOVEIT_STRUCT_FORWARD(OSQPDataWrapper);

// Plugin
class AccelerationLimitedPlugin : public SmoothingBaseClass
{
public:
  /**
   * Initialize the acceleration based smoothing plugin
   * @param node ROS node, used for parameter retrieval
   * @param robot_model typically used to retrieve vel/accel/jerk limits
   * @param num_joints number of actuated joints in the JointGroup Servo controls
   * @return True if initialization was successful
   */
  bool initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model,
                  size_t num_joints) override;

  /**
   * Smooth the command signals for all DOF. This function limits the change in velocity using the acceleration
   * specified in the robot model.
   * @param positions array of joint position commands
   * @param velocities array of joint velocity commands
   * @param accelerations (unused)
   * @return True if smoothing was successful
   */
  bool doSmoothing(Eigen::VectorXd& positions, Eigen::VectorXd& velocities, Eigen::VectorXd& accelerations) override;

  /**
   * Reset to a given joint state. This method must be called before doSmoothing.
   * @param positions reset the filters to the joint positions
   * @param velocities reset the filters to the joint velocities
   * @param accelerations (unused)
   * @return True if reset was successful
   */
  bool reset(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
             const Eigen::VectorXd& accelerations) override;

  /**
   * memory allocated by osqp is freed in destructor
   */
  ~AccelerationLimitedPlugin() override
  {
    if (osqp_solver_ != nullptr)
    {
      osqp_cleanup(osqp_solver_);
    }
  }

private:
  /** \brief Pointer to rclcpp node handle.  */
  rclcpp::Node::SharedPtr node_;
  /** \brief Parameters for plugin.  */
  online_signal_smoothing::Params params_;
  /** \brief The number of joints in the robot's planning group.  */
  size_t num_joints_;
  /** \brief Last velocities and positions received */
  Eigen::VectorXd last_velocities_;
  Eigen::VectorXd last_positions_;
  /** \brief Intermediate variables used in calculations */
  Eigen::VectorXd cur_acceleration_;
  Eigen::VectorXd positions_offset_;
  Eigen::VectorXd velocities_offset_;
  /** \brief Extracted joint limits from robot model */
  Eigen::VectorXd max_acceleration_limits_;
  Eigen::VectorXd min_acceleration_limits_;
  /** \brief Pointer to robot model */
  moveit::core::RobotModelConstPtr robot_model_;
  /** \brief Constraint matrix for optimization problem */
  Eigen::SparseMatrix<double> constraints_sparse_;
  /** \brief osqp types used for optimization problem */
  OSQPDataWrapperPtr osqp_data_;
#if MOVEIT_OSQP_V1
  OSQPSolver* osqp_solver_ = nullptr;
#else
  OSQPWorkspace* osqp_solver_ = nullptr;
#endif
  OSQPSettings osqp_settings_;
};
}  // namespace online_signal_smoothing
