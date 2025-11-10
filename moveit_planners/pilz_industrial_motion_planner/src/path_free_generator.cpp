/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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

#include <pilz_industrial_motion_planner/path_free_generator.hpp>

namespace pilz_industrial_motion_planner
{
std::unique_ptr<KDL::Path> PathFreeGenerator::freeFromWaypoints(const KDL::Frame& start_pose,
                                                                const std::vector<KDL::Frame>& waypoints,
                                                                KDL::RotationalInterpolation* rot_interpo,
                                                                double smoothness, double eqradius)
{
  double blend_radius = computeBlendRadius(waypoints, smoothness);
  KDL::Path_RoundedComposite* composite_path = new KDL::Path_RoundedComposite(blend_radius, eqradius, rot_interpo);

  composite_path->Add(start_pose);
  // index for the last add point
  size_t last_added_point_indx = -1;  // -1 for the start_pose

  // to get the last added point in the rounded composite path
  auto last_point = [&]() { return last_added_point_indx != -1 ? waypoints[last_added_point_indx].p : start_pose.p; };
  // distance between start pose and first waypoint
  double dist = (start_pose.p - waypoints.front().p).Norm();

  // add points and skip the points which are too close to each other
  for (size_t i = 0; i < waypoints.size(); ++i)
  {
    dist = (last_point() - waypoints[i].p).Norm();
    if (dist > MAX_SEGMENT_LENGTH)
    {
      composite_path->Add(waypoints[i]);
      ++last_added_point_indx;
    }
  }
  composite_path->Finish();
  return std::unique_ptr<KDL::Path>(composite_path);
}
double PathFreeGenerator::computeBlendRadius(const std::vector<KDL::Frame>& waypoints_, double smoothness)
{
  double max_allowed_radius = std::numeric_limits<double>::infinity();

  auto pose_distance = [](const KDL::Frame& p1, const KDL::Frame& p2) { return (p1.p - p2.p).Norm(); };

  // to calculate the angle between two segments
  auto segment_angle = [](const KDL::Frame& p1, const KDL::Frame& p2, const KDL::Frame& p3) {
    KDL::Vector v1 = p2.p - p1.p;
    KDL::Vector v2 = p2.p - p3.p;

    double norm_product = v1.Norm() * v2.Norm();
    if (norm_product < MAX_SEGMENT_LENGTH * MAX_SEGMENT_LENGTH)
      return 0.0;  // avoid division by zero

    double cos_theta = KDL::dot(v1, v2) / norm_product;
    cos_theta = std::clamp(cos_theta, -1.0, 1.0);

    return std::acos(cos_theta);
  };

  for (size_t i = 1; i + 1 < waypoints_.size(); ++i)
  {
    double dist1 = pose_distance(waypoints_[i], waypoints_[i - 1]);
    double dist2 = pose_distance(waypoints_[i + 1], waypoints_[i]);

    if (dist1 < MAX_SEGMENT_LENGTH || dist2 < MAX_SEGMENT_LENGTH)
    {
      continue;
    }

    // The maximum feasible radius for this junction
    double local_max_radius = std::tan(segment_angle(waypoints_[i - 1], waypoints_[i], waypoints_[i + 1]) / 2.0) *
                              std::min(dist1 / 2.0, dist2 / 2.0);

    // Keep track of the tightest constraint
    // due to roundedcomposite don't support changing radius
    if (local_max_radius < max_allowed_radius)
      max_allowed_radius = local_max_radius;
  }

  // Apply the smoothness scaling factor
  double max_radius = max_allowed_radius * std::clamp(smoothness, MIN_SMOOTHNESS, MAX_SMOOTHNESS);

  return max_radius;
}

}  // namespace pilz_industrial_motion_planner
