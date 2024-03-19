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

#include <moveit/collision_detection/occupancy_map.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor_middleware_handle.hpp>
#include <moveit_msgs/srv/load_map.hpp>
#include <moveit_msgs/srv/save_map.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <moveit/utils/logger.hpp>

namespace occupancy_map_monitor
{

OccupancyMapMonitor::OccupancyMapMonitor(const rclcpp::Node::SharedPtr& node, double map_resolution)
  : OccupancyMapMonitor{ std::make_unique<OccupancyMapMonitorMiddlewareHandle>(node, map_resolution, ""), nullptr }
{
}

OccupancyMapMonitor::OccupancyMapMonitor(const rclcpp::Node::SharedPtr& node,
                                         const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                         const std::string& map_frame, double map_resolution)
  : OccupancyMapMonitor{ std::make_unique<OccupancyMapMonitorMiddlewareHandle>(node, map_resolution, map_frame),
                         tf_buffer }
{
}

OccupancyMapMonitor::OccupancyMapMonitor(std::unique_ptr<MiddlewareHandle> middleware_handle,
                                         const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
  : middleware_handle_{ std::move(middleware_handle) }
  , tf_buffer_{ tf_buffer }
  , parameters_{ 0.0, "", {} }
  , debug_info_{ false }
  , mesh_handle_count_{ 0 }
  , active_{ false }
  , logger_(moveit::getLogger("moveit.ros.occupancy_map_monitor"))
{
  if (middleware_handle_ == nullptr)
  {
    throw std::invalid_argument("OccupancyMapMonitor cannot be constructed with nullptr MiddlewareHandle");
  }

  // Get the parameters
  parameters_ = middleware_handle_->getParameters();

  RCLCPP_DEBUG(logger_, "Using resolution = %lf m for building octomap", parameters_.map_resolution);

  if (tf_buffer_ != nullptr && parameters_.map_frame.empty())
  {
    RCLCPP_WARN(logger_, "No target frame specified for Octomap. No transforms will be applied to received data.");
  }
  if (tf_buffer_ == nullptr && !parameters_.map_frame.empty())
  {
    RCLCPP_WARN(logger_, "Target frame specified but no TF instance (buffer) specified."
                         "No transforms will be applied to received data.");
  }

  tree_ = std::make_shared<collision_detection::OccMapTree>(parameters_.map_resolution);
  tree_const_ = tree_;

  for (const auto& [sensor_name, sensor_type] : parameters_.sensor_plugins)
  {
    auto occupancy_map_updater = middleware_handle_->loadOccupancyMapUpdater(sensor_type);

    // Verify the updater was loaded
    if (occupancy_map_updater == nullptr)
    {
      RCLCPP_ERROR_STREAM(logger_, "Failed to load sensor: `" << sensor_name << "` of type: `" << sensor_type << '`');
      continue;
    }

    // Pass a pointer to the monitor to the updater
    occupancy_map_updater->setMonitor(this);

    // This part is done in the middleware handle because it needs the node
    middleware_handle_->initializeOccupancyMapUpdater(occupancy_map_updater);

    // Load the params in the updater
    if (!occupancy_map_updater->setParams(sensor_name))
    {
      RCLCPP_ERROR_STREAM(logger_, "Failed to configure updater of type " << occupancy_map_updater->getType());
      continue;
    }

    // Add the successfully initialized updater
    addUpdater(occupancy_map_updater);
  }

  /* advertise a service for loading octomaps from disk */
  auto save_map_service_callback = [this](const std::shared_ptr<rmw_request_id_t>& request_header,
                                          const std::shared_ptr<moveit_msgs::srv::SaveMap::Request>& request,
                                          const std::shared_ptr<moveit_msgs::srv::SaveMap::Response>& response) -> bool {
    return saveMapCallback(request_header, request, response);
  };

  auto load_map_service_callback = [this](const std::shared_ptr<rmw_request_id_t>& request_header,
                                          const std::shared_ptr<moveit_msgs::srv::LoadMap::Request>& request,
                                          const std::shared_ptr<moveit_msgs::srv::LoadMap::Response>& response) -> bool {
    return loadMapCallback(request_header, request, response);
  };

  middleware_handle_->createSaveMapService(save_map_service_callback);
  middleware_handle_->createLoadMapService(load_map_service_callback);
}

void OccupancyMapMonitor::addUpdater(const OccupancyMapUpdaterPtr& updater)
{
  if (updater)
  {
    map_updaters_.push_back(updater);
    updater->publishDebugInformation(debug_info_);
    if (map_updaters_.size() > 1)
    {
      mesh_handles_.resize(map_updaters_.size());
      // when we had one updater only, we passed directly the transform cache callback to that updater
      if (map_updaters_.size() == 2)
      {
        map_updaters_[0]->setTransformCacheCallback(
            [this](const std::string& frame, const rclcpp::Time& stamp, ShapeTransformCache& cache) {
              return getShapeTransformCache(0, frame, stamp, cache);
            });
        map_updaters_[1]->setTransformCacheCallback(
            [this](const std::string& frame, const rclcpp::Time& stamp, ShapeTransformCache& cache) {
              return getShapeTransformCache(1, frame, stamp, cache);
            });
      }
      else
      {
        map_updaters_.back()->setTransformCacheCallback(
            [this, i = map_updaters_.size() - 1](const std::string& frame, const rclcpp::Time& stamp,
                                                 ShapeTransformCache& cache) {
              return getShapeTransformCache(i, frame, stamp, cache);
            });
      }
    }
    else
      updater->setTransformCacheCallback(transform_cache_callback_);
  }
  else
    RCLCPP_ERROR(logger_, "nullptr updater was specified");
}

void OccupancyMapMonitor::publishDebugInformation(bool flag)
{
  debug_info_ = flag;
  for (OccupancyMapUpdaterPtr& map_updater : map_updaters_)
    map_updater->publishDebugInformation(debug_info_);
}

void OccupancyMapMonitor::setMapFrame(const std::string& frame)
{
  std::lock_guard<std::mutex> _(parameters_lock_);  // we lock since an updater could specify a new frame for us
  parameters_.map_frame = frame;
}

ShapeHandle OccupancyMapMonitor::excludeShape(const shapes::ShapeConstPtr& shape)
{
  // if we have just one updater, remove the additional level of indirection
  if (map_updaters_.size() == 1)
    return map_updaters_[0]->excludeShape(shape);

  ShapeHandle h = 0;
  for (std::size_t i = 0; i < map_updaters_.size(); ++i)
  {
    ShapeHandle mh = map_updaters_[i]->excludeShape(shape);
    if (mh)
    {
      if (h == 0)
        h = ++mesh_handle_count_;
      mesh_handles_[i][h] = mh;
    }
  }
  return h;
}

void OccupancyMapMonitor::forgetShape(ShapeHandle handle)
{
  // if we have just one updater, remove the additional level of indirection
  if (map_updaters_.size() == 1)
  {
    map_updaters_[0]->forgetShape(handle);
    return;
  }

  for (std::size_t i = 0; i < map_updaters_.size(); ++i)
  {
    std::map<ShapeHandle, ShapeHandle>::const_iterator it = mesh_handles_[i].find(handle);
    if (it == mesh_handles_[i].end())
      continue;
    map_updaters_[i]->forgetShape(it->second);
  }
}

void OccupancyMapMonitor::setTransformCacheCallback(const TransformCacheProvider& transform_callback)
{
  // if we have just one updater, we connect it directly to the transform provider
  if (map_updaters_.size() == 1)
  {
    map_updaters_[0]->setTransformCacheCallback(transform_callback);
  }
  else
  {
    transform_cache_callback_ = transform_callback;
  }
}

bool OccupancyMapMonitor::getShapeTransformCache(std::size_t index, const std::string& target_frame,
                                                 const rclcpp::Time& target_time, ShapeTransformCache& cache) const
{
  if (transform_cache_callback_)
  {
    ShapeTransformCache temp_cache;
    if (transform_cache_callback_(target_frame, target_time, temp_cache))
    {
      for (std::pair<const ShapeHandle, Eigen::Isometry3d>& it : temp_cache)
      {
        std::map<ShapeHandle, ShapeHandle>::const_iterator jt = mesh_handles_[index].find(it.first);
        if (jt == mesh_handles_[index].end())
        {
          rclcpp::Clock steady_clock(RCL_STEADY_TIME);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
          RCLCPP_ERROR_THROTTLE(logger_, steady_clock, 1000, "Incorrect mapping of mesh handles");
#pragma GCC diagnostic pop
          return false;
        }
        else
          cache[jt->second] = it.second;
      }
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

bool OccupancyMapMonitor::saveMapCallback(const std::shared_ptr<rmw_request_id_t>& /* unused */,
                                          const std::shared_ptr<moveit_msgs::srv::SaveMap::Request>& request,
                                          const std::shared_ptr<moveit_msgs::srv::SaveMap::Response>& response)
{
  RCLCPP_INFO(logger_, "Writing map to %s", request->filename.c_str());
  tree_->lockRead();
  try
  {
    response->success = tree_->writeBinary(request->filename);
  }
  catch (...)
  {
    response->success = false;
  }
  tree_->unlockRead();
  return true;
}

bool OccupancyMapMonitor::loadMapCallback(const std::shared_ptr<rmw_request_id_t>& /* unused */,
                                          const std::shared_ptr<moveit_msgs::srv::LoadMap::Request>& request,
                                          const std::shared_ptr<moveit_msgs::srv::LoadMap::Response>& response)
{
  RCLCPP_INFO(logger_, "Reading map from %s", request->filename.c_str());

  /* load the octree from disk */
  tree_->lockWrite();
  try
  {
    response->success = tree_->readBinary(request->filename);
  }
  catch (...)
  {
    RCLCPP_ERROR(logger_, "Failed to load map from file");
    response->success = false;
  }
  tree_->unlockWrite();

  if (response->success)
    tree_->triggerUpdateCallback();

  return true;
}

void OccupancyMapMonitor::startMonitor()
{
  active_ = true;
  /* initialize all of the occupancy map updaters */
  for (OccupancyMapUpdaterPtr& map_updater : map_updaters_)
    map_updater->start();
}

void OccupancyMapMonitor::stopMonitor()
{
  active_ = false;
  for (OccupancyMapUpdaterPtr& map_updater : map_updaters_)
    map_updater->stop();
}

OccupancyMapMonitor::~OccupancyMapMonitor()
{
  stopMonitor();
}
}  // namespace occupancy_map_monitor
