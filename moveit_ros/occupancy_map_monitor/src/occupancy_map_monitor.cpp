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

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/save_map.hpp>
#include <moveit_msgs/srv/load_map.hpp>
#include <moveit/occupancy_map_monitor/occupancy_map.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <boost/bind.hpp>

namespace occupancy_map_monitor
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.ros.occupancy_map_monitor");

OccupancyMapMonitor::OccupancyMapMonitor(const rclcpp::Node::SharedPtr& node, double map_resolution)
  : map_resolution_(map_resolution), debug_info_(false), mesh_handle_count_(0), node_(node), active_(false)
{
  initialize();
}

OccupancyMapMonitor::OccupancyMapMonitor(const rclcpp::Node::SharedPtr& node,
                                         const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                         const std::string& map_frame, double map_resolution)
  : tf_buffer_(tf_buffer)
  , map_frame_(map_frame)
  , map_resolution_(map_resolution)
  , debug_info_(false)
  , mesh_handle_count_(0)
  , node_(node)
{
  initialize();
}

void OccupancyMapMonitor::initialize()
{
  /* load params from param server */
  if (map_resolution_ <= std::numeric_limits<double>::epsilon())
    if (!node_->get_parameter("octomap_resolution", map_resolution_))
    {
      map_resolution_ = 0.1;
      RCLCPP_WARN(LOGGER, "Resolution not specified for Octomap. Assuming resolution = %g instead", map_resolution_);
    }
  RCLCPP_DEBUG(LOGGER, "Using resolution = %lf m for building octomap", map_resolution_);

  if (map_frame_.empty())
    if (!node_->get_parameter("octomap_frame", map_frame_))
      if (tf_buffer_)
        RCLCPP_WARN(LOGGER, "No target frame specified for Octomap. No transforms will be applied to received data.");

  if (!tf_buffer_ && !map_frame_.empty())
    RCLCPP_WARN(LOGGER, "Target frame specified but no TF instance (buffer) specified."
                        "No transforms will be applied to received data.");

  tree_.reset(new OccMapTree(map_resolution_));
  tree_const_ = tree_;

  std::vector<std::string> sensor_list;
  if (node_->get_parameter("sensors", sensor_list))
  {
    if (!sensor_list.empty())
    {
      for (const auto& sensor : sensor_list)
      {
        if (!node_->has_parameter(sensor + ".sensor_plugin"))
        {
          RCLCPP_ERROR(LOGGER, "No sensor plugin specified for octomap updater %s; ignoring.", sensor.c_str());
          continue;
        }

        std::string sensor_plugin;
        node_->get_parameter(sensor + ".sensor_plugin", sensor_plugin);
        if (sensor_plugin.empty() || sensor_plugin[0] == '~')
        {
          RCLCPP_INFO_STREAM(LOGGER, "Skipping octomap updater plugin '" << sensor_plugin << "'");
          continue;
        }

        if (!updater_plugin_loader_)
        {
          try
          {
            updater_plugin_loader_.reset(new pluginlib::ClassLoader<OccupancyMapUpdater>(
                "moveit_ros_occupancy_map_monitor", "occupancy_map_monitor::OccupancyMapUpdater"));
          }
          catch (pluginlib::PluginlibException& ex)
          {
            RCLCPP_FATAL_STREAM(LOGGER, "Exception while creating octomap updater plugin loader " << ex.what());
          }
        }

        OccupancyMapUpdaterPtr up;
        try
        {
          up = updater_plugin_loader_->createUniqueInstance(sensor_plugin);
          up->setMonitor(this);
        }
        catch (pluginlib::PluginlibException& ex)
        {
          RCLCPP_ERROR_STREAM(LOGGER, "Exception while loading octomap updater '" << sensor_plugin << "': " << ex.what()
                                                                                  << std::endl);
        }
        if (up)
        {
          if (!up->initialize(node_))
          {
            RCLCPP_ERROR(LOGGER, "Unable to initialize map updater of type %s (plugin %s)", up->getType().c_str(),
                         sensor_plugin.c_str());
            continue;
          }

          /* pass the params struct directly in to the updater */
          if (!up->setParams(sensor))
          {
            RCLCPP_ERROR_STREAM(LOGGER, "Failed to configure updater of type " << up->getType());
            continue;
          }

          addUpdater(up);
        }
      }
    }
    else
      RCLCPP_ERROR(LOGGER, "List of sensors is empty!");
  }
  else
    RCLCPP_ERROR(LOGGER, "Failed to find 3D sensor plugin parameters for octomap generation");

  /* advertise a service for loading octomaps from disk */
  auto save_map_service_callback = [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<moveit_msgs::srv::SaveMap::Request> request,
                                          std::shared_ptr<moveit_msgs::srv::SaveMap::Response> response) -> bool {
    return saveMapCallback(request_header, request, response);
  };

  auto load_map_service_callback = [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<moveit_msgs::srv::LoadMap::Request> request,
                                          std::shared_ptr<moveit_msgs::srv::LoadMap::Response> response) -> bool {
    return loadMapCallback(request_header, request, response);
  };

  save_map_srv_ = node_->create_service<moveit_msgs::srv::SaveMap>("save_map", save_map_service_callback);
  load_map_srv_ = node_->create_service<moveit_msgs::srv::LoadMap>("load_map", load_map_service_callback);
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
      // when we had one updater only, we passed direcly the transform cache callback to that updater
      if (map_updaters_.size() == 2)
      {
        map_updaters_[0]->setTransformCacheCallback(
            boost::bind(&OccupancyMapMonitor::getShapeTransformCache, this, 0, _1, _2, _3));
        map_updaters_[1]->setTransformCacheCallback(
            boost::bind(&OccupancyMapMonitor::getShapeTransformCache, this, 1, _1, _2, _3));
      }
      else
        map_updaters_.back()->setTransformCacheCallback(
            boost::bind(&OccupancyMapMonitor::getShapeTransformCache, this, map_updaters_.size() - 1, _1, _2, _3));
    }
    else
      updater->setTransformCacheCallback(transform_cache_callback_);
  }
  else
    RCLCPP_ERROR(LOGGER, "NULL updater was specified");
}

void OccupancyMapMonitor::publishDebugInformation(bool flag)
{
  debug_info_ = flag;
  for (OccupancyMapUpdaterPtr& map_updater : map_updaters_)
    map_updater->publishDebugInformation(debug_info_);
}

void OccupancyMapMonitor::setMapFrame(const std::string& frame)
{
  boost::mutex::scoped_lock _(parameters_lock_);  // we lock since an updater could specify a new frame for us
  map_frame_ = frame;
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
    map_updaters_[0]->setTransformCacheCallback(transform_callback);
  else
    transform_cache_callback_ = transform_callback;
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
          RCLCPP_ERROR_THROTTLE(LOGGER, steady_clock, 1000, "Incorrect mapping of mesh handles");
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

bool OccupancyMapMonitor::saveMapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<moveit_msgs::srv::SaveMap::Request> request,
                                          std::shared_ptr<moveit_msgs::srv::SaveMap::Response> response)
{
  RCLCPP_INFO(LOGGER, "Writing map to %s", request->filename.c_str());
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

bool OccupancyMapMonitor::loadMapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<moveit_msgs::srv::LoadMap::Request> request,
                                          std::shared_ptr<moveit_msgs::srv::LoadMap::Response> response)
{
  RCLCPP_INFO(LOGGER, "Reading map from %s", request->filename.c_str());

  /* load the octree from disk */
  tree_->lockWrite();
  try
  {
    response->success = tree_->readBinary(request->filename);
  }
  catch (...)
  {
    RCLCPP_ERROR(LOGGER, "Failed to load map from file");
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
