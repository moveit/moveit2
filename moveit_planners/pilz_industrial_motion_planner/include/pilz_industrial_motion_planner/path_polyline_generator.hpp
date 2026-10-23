/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
 *  Copyright (c) 2025 Aiman Haidar
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#pragma once

#include <kdl/path.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>

#include <memory>

namespace pilz_industrial_motion_planner
{
/**
 * @brief Generator class for KDL::Path_RoundedComposite from different polyline path
 * representations
 */
class PathPolylineGenerator
{
public:
  /**
   * @brief set the path polyline from waypoints
   *
   */
  static std::unique_ptr<KDL::Path> polylineFromWaypoints(const KDL::Frame& start_pose,
                                                          const std::vector<KDL::Frame>& waypoints,
                                                          KDL::RotationalInterpolation* rot_interpo, double smoothness,
                                                          double eqradius);

  /**
   * @brief compute the maximum rounding radius from KDL::Path_RoundedComosite
   * @param waypoints_: waypoints defining the path
   * @param smoothness: smoothness level [0..1] scaling the maximum blend radius
   * @return maximum blend radius
   */
  static std::vector<KDL::Frame> filterWaypoints(const KDL::Frame& start_pose, const std::vector<KDL::Frame>& waypoints);
  static double computeBlendRadius(const std::vector<KDL::Frame>& waypoints_, double smoothness);
  static void checkConsecutiveColinearWaypoints(const KDL::Frame& p1, const KDL::Frame& p2, const KDL::Frame& p3);

private:
  PathPolylineGenerator(){};  // no instantiation of this helper class!

  static constexpr double MIN_SEGMENT_LENGTH{ 0.2e-3 };
  static constexpr double MIN_SMOOTHNESS{ 0.01 };
  static constexpr double MAX_SMOOTHNESS{ 0.99 };
  static constexpr double MIN_COLINEAR_NORM{ 1e-9 };
};

class ErrorMotionPlanningColinearConsicutiveWaypoints : public KDL::Error_MotionPlanning
{
public:
  const char* Description() const override
  {
    return "Three collinear consecutive waypoints."
           " A Polyline Path cannot be created.";
  }
  int GetType() const override
  {
    return ERROR_CODE_COLINEAR_CONSECUTIVE_WAYPOINTS;
  }  // LCOV_EXCL_LINE

private:
  static constexpr int ERROR_CODE_COLINEAR_CONSECUTIVE_WAYPOINTS{ 3104 };
};

}  // namespace pilz_industrial_motion_planner
