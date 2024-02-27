/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan, Jon Binney */

#pragma once

#include <moveit/collision_detection/occupancy_map.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <moveit_msgs/srv/load_map.hpp>
#include <moveit_msgs/srv/save_map.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace occupancy_map_monitor
{
class OccupancyMapMonitor
{
public:
  /**
   * @brief      This class describes parameters that are normally provided through ROS Parameters.
   */
  struct Parameters
  {
    double map_resolution;
    std::string map_frame;
    std::vector<std::pair<std::string, std::string>> sensor_plugins;
  };

  /**
   * @brief      This class contains the rcl interfaces for easier testing
   */
  class MiddlewareHandle
  {
  public:
    using SaveMapServiceCallback = std::function<bool(const std::shared_ptr<rmw_request_id_t> request_header,
                                                      const std::shared_ptr<moveit_msgs::srv::SaveMap::Request> request,
                                                      std::shared_ptr<moveit_msgs::srv::SaveMap::Response> response)>;
    using LoadMapServiceCallback = std::function<bool(const std::shared_ptr<rmw_request_id_t> request_header,
                                                      const std::shared_ptr<moveit_msgs::srv::LoadMap::Request> request,
                                                      std::shared_ptr<moveit_msgs::srv::LoadMap::Response> response)>;

    /**
     * @brief      Destroys the object.  Needed because this is pure virtual interface.
     */
    virtual ~MiddlewareHandle() = default;

    /**
     * @brief      Gets the parameters struct.
     *
     * @return     The parameters.
     */
    virtual Parameters getParameters() const = 0;

    /**
     * @brief      Loads an occupancy map updater based on string.
     *
     * @param[in]  sensor_plugin  The sensor plugin string.
     *
     * @return     The occupancy map updater pointer.
     */
    virtual OccupancyMapUpdaterPtr loadOccupancyMapUpdater(const std::string& sensor_plugin) = 0;

    /**
     * @brief      Initializes the occupancy map updater.  Needed because of interface to OccupancyMapUpdater
     *
     * @param[in]  occupancy_map_updater  The occupancy map updater
     */
    virtual void initializeOccupancyMapUpdater(OccupancyMapUpdaterPtr occupancy_map_updater) = 0;

    /**
     * @brief      Creates a save map service.
     *
     * @param[in]  callback  The callback
     */
    virtual void createSaveMapService(SaveMapServiceCallback callback) = 0;

    /**
     * @brief      Creates a load map service.
     *
     * @param[in]  callback  The callback
     */
    virtual void createLoadMapService(LoadMapServiceCallback callback) = 0;
  };

  /**
   * @brief      Occupancy map monitor constructor with the MiddlewareHandle
   *
   * @param[in]  middleware_handle   The rcl interface
   * @param[in]  tf_buffer           The tf buffer
   */
  OccupancyMapMonitor(std::unique_ptr<MiddlewareHandle> middleware_handle,
                      const std::shared_ptr<tf2_ros::Buffer>& tf_buffer);

  /**
   * @brief      Occupancy map monitor constructor with Node
   *
   * @param[in]  node            The node
   * @param[in]  tf_buffer       The tf buffer
   * @param[in]  map_frame       The map frame
   * @param[in]  map_resolution  The map resolution
   */
  OccupancyMapMonitor(const rclcpp::Node::SharedPtr& node, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                      const std::string& map_frame = "", double map_resolution = 0.0);

  /**
   * @brief      Occupancy map monitor constructor with Node
   *
   * @param[in]  node            The node
   * @param[in]  map_resolution  The map resolution
   */
  OccupancyMapMonitor(const rclcpp::Node::SharedPtr& node, double map_resolution = 0.0);

  /**
   * @brief      Destroys the object.
   */
  ~OccupancyMapMonitor();

  /**
   * @brief start the monitor (will begin updating the octomap
   */
  void startMonitor();

  /**
   * @brief      Stops the monitor, this also stops the updaters.
   */
  void stopMonitor();

  /** @brief Get a pointer to the underlying octree for this monitor. Lock the tree before reading or writing using this
   *  pointer. The value of this pointer stays the same throughout the existence of the monitor instance. */
  const collision_detection::OccMapTreePtr& getOcTreePtr()
  {
    return tree_;
  }

  /** @brief Get a const pointer to the underlying octree for this monitor. Lock the
   *  tree before reading this pointer */
  const collision_detection::OccMapTreeConstPtr& getOcTreePtr() const
  {
    return tree_const_;
  }

  /**
   * @brief      Gets the map frame (this is set either by the constor or a parameter).
   *
   * @return     The map frame.
   */
  const std::string& getMapFrame() const
  {
    return parameters_.map_frame;
  }

  /**
   * @brief      Sets the map frame.
   *
   * @param[in]  frame  The frame
   */
  void setMapFrame(const std::string& frame);

  /**
   * @brief      Gets the map resolution.
   *
   * @return     The map resolution.
   */
  double getMapResolution() const
  {
    return parameters_.map_resolution;
  }

  /**
   * @brief      Gets the tf client.
   *
   * @return     The tf client.
   */
  const std::shared_ptr<tf2_ros::Buffer>& getTFClient() const
  {
    return tf_buffer_;
  }

  /**
   * @brief      Adds an OccupancyMapUpdater to be monitored.
   *
   * @param[in]  updater  The OccupancyMapUpdater
   */
  void addUpdater(const OccupancyMapUpdaterPtr& updater);

  /**
   * @brief      Add this shape to the set of shapes to be filtered out from the octomap
   *
   * @param[in]  shape  The shape to be exclueded from the updaters.
   *
   * @return     The shape handle.  Can be used to forget the shape later.
   */
  ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape);

  /**
   * \brief Forget about this shape handle and the shapes it corresponds to
   *
   * @param[in]  handle  The handle to forget.
   */
  void forgetShape(ShapeHandle handle);

  /**
   * @brief      Set the callback to trigger when updates to the maintained octomap are received
   *
   * @param[in]  update_callback  The update callback function
   */
  void setUpdateCallback(const std::function<void()>& update_callback)
  {
    tree_->setUpdateCallback(update_callback);
  }

  /**
   * @brief      Sets the transform cache callback.
   *
   * @param[in]  transform_cache_callback  The transform cache callback
   */
  void setTransformCacheCallback(const TransformCacheProvider& transform_cache_callback);

  /**
   * @brief      Set the debug flag on the updaters.
   *
   * @param[in]  flag  The flag
   */
  void publishDebugInformation(bool flag);

  /**
   * @brief      Determines if active.
   *
   * @return     True if active, False otherwise.
   */
  bool isActive() const
  {
    return active_;
  }

private:
  /**
   * @brief      Save the current octree to a binary file
   *
   * @param[in]  request_header  The request header
   * @param[in]  request         The request
   * @param[in]  response        The response
   *
   * @return     True on success, False otherwise.
   */
  bool saveMapCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                       const std::shared_ptr<moveit_msgs::srv::SaveMap::Request>& request,
                       const std::shared_ptr<moveit_msgs::srv::SaveMap::Response>& response);

  /**
   * @brief      Load octree from a binary file (gets rid of current octree data)
   *
   * @param[in]  request_header  The request header
   * @param[in]  request         The request
   * @param[in]  response        The response
   *
   * @return     True on success, False otherwise.
   */
  bool loadMapCallback(const std::shared_ptr<rmw_request_id_t>& request_header,
                       const std::shared_ptr<moveit_msgs::srv::LoadMap::Request>& request,
                       const std::shared_ptr<moveit_msgs::srv::LoadMap::Response>& response);

  /**
   * @brief      Gets the shape transform cache.
   *
   * @param[in]  index         The index
   * @param[in]  target_frame  The target frame
   * @param[in]  target_time   The target time
   * @param[out] cache         The cache
   *
   * @return     True on success, False otherwise.
   */
  bool getShapeTransformCache(std::size_t index, const std::string& target_frame, const rclcpp::Time& target_time,
                              ShapeTransformCache& cache) const;

  std::unique_ptr<MiddlewareHandle> middleware_handle_; /*!< The abstract interface to ros */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;          /*!< TF buffer */
  Parameters parameters_;
  std::mutex parameters_lock_; /*!< Mutex for synchronizing access to parameters */

  collision_detection::OccMapTreePtr tree_;            /*!< Oct map tree */
  collision_detection::OccMapTreeConstPtr tree_const_; /*!< Shared pointer to a const oct map tree */

  std::vector<OccupancyMapUpdaterPtr> map_updaters_;             /*!< The Occupancy map updaters */
  std::vector<std::map<ShapeHandle, ShapeHandle>> mesh_handles_; /*!< The mesh handles */
  TransformCacheProvider transform_cache_callback_;              /*!< Callback for the transform cache */
  bool debug_info_;                                              /*!< Enable/disable debug output */

  std::size_t mesh_handle_count_; /*!< Count of mesh handles */

  bool active_; /*!< True when actively monitoring updaters */

  rclcpp::Logger logger_;
};
}  // namespace occupancy_map_monitor
