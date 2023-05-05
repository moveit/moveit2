/*********************************************************************
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2018, Southwest Research Institute
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************/

/* Author: Levi Armstrong */

#pragma once

#include <sstream>

#include <octomap_msgs/conversions.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <rclcpp/logging.hpp>

#include <moveit/collision_detection_bullet/bullet_integration/basic_types.h>

namespace collision_detection_bullet
{
/** \brief Recursively traverses robot from root to get all active links
 *
 *   \param active_links Stores the active links
 *   \param urdf_link The current urdf link representation
 *   \param active Indicates if link is considered active */
void getActiveLinkNamesRecursive(std::vector<std::string>& active_links, const urdf::LinkConstSharedPtr& urdf_link,
                                 bool active);

shapes::ShapePtr constructShape(const urdf::Geometry* geom);

Eigen::Isometry3d urdfPose2Eigen(const urdf::Pose& pose);

}  // namespace collision_detection_bullet
