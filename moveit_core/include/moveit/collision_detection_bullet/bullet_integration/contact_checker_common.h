/*********************************************************************
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2017, Southwest Research Institute
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

#include <LinearMath/btConvexHullComputer.h>
#include <cstdio>
#include <Eigen/Geometry>
#include <fstream>

#include <moveit/collision_detection_bullet/bullet_integration/basic_types.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>

namespace collision_detection_bullet
{
/**
 * @brief Get a key for two object to search the collision matrix
 * @param obj1 First collision object name
 * @param obj2 Second collision object name
 * @return The collision pair key
 */
inline std::pair<std::string, std::string> getObjectPairKey(const std::string& obj1, const std::string& obj2)
{
  return obj1 < obj2 ? std::make_pair(obj1, obj2) : std::make_pair(obj2, obj1);
}

/**
 * @brief This will check if a link is active provided a list. If the list is empty the link is considered active.
 * @param active List of active link names
 * @param name The name of link to check if it is active.
 */
inline bool isLinkActive(const std::vector<std::string>& active, const std::string& name)
{
  return active.empty() || (std::find(active.begin(), active.end(), name) != active.end());
}

/** \brief Stores a single contact result in the requested way.
 *   \param found Indicates if a contact for this pair of objects has already been found
 *   \return Pointer to the newly inserted contact */
collision_detection::Contact* processResult(ContactTestData& cdata, collision_detection::Contact& contact,
                                            const std::pair<std::string, std::string>& key, bool found);

/**
 * @brief Create a convex hull from vertices using Bullet Convex Hull Computer
 * @param (Output) vertices A vector of vertices
 * @param (Output) faces The first values indicates the number of vertices that define the face followed by the vertice
 * index
 * @param (input) input A vector of point to create a convex hull from
 * @param (input) shrink If positive, the convex hull is shrunken by that amount (each face is moved by "shrink" length
 *                units towards the center along its normal).
 * @param (input) shrinkClamp If positive, "shrink" is clamped to not exceed "shrinkClamp * innerRadius", where
 *                "innerRadius" is the minimum distance of a face to the center of the convex hull.
 * @return The number of faces. If less than zero an error occurred when trying to create the convex hull
 */
int createConvexHull(AlignedVector<Eigen::Vector3d>& vertices, std::vector<int>& faces,
                     const AlignedVector<Eigen::Vector3d>& input, double shrink = -1, double shrinkClamp = -1);

}  // namespace collision_detection_bullet
