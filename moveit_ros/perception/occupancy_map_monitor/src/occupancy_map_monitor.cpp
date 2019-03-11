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


#include <moveit_msgs/srv/save_map.hpp>
#include <moveit_msgs/srv/load_map.hpp>
#include <moveit/occupancy_map_monitor/occupancy_map.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>

rclcpp::Logger logger_occupancy_map_monitor = rclcpp::get_logger("occupancy_map_monitor");

namespace occupancy_map_monitor
{
OccupancyMapMonitor::OccupancyMapMonitor(double map_resolution)
  : map_resolution_(map_resolution), debug_info_(false), mesh_handle_count_(0), active_(false)
{
  initialize();
}

OccupancyMapMonitor::OccupancyMapMonitor(const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                         const std::string& map_frame, double map_resolution)
  : tf_buffer_(tf_buffer)
  , map_frame_(map_frame)
  , map_resolution_(map_resolution)
  , debug_info_(false)
  , mesh_handle_count_(0)
{
  initialize();
}

OccupancyMapMonitor::OccupancyMapMonitor(const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, std::shared_ptr<rclcpp::Node> node,
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
  // auto node_occupancy_map = rclcpp::Node::make_shared("occupancy_map_server");
  auto ocupancy_map_monitor_parameters = std::make_shared<rclcpp::SyncParametersClient>(node_);

  for (auto & parameter : ocupancy_map_monitor_parameters->get_parameters({"octomap_resolution"})) {
      if (map_resolution_ <= std::numeric_limits<double>::epsilon()){
          map_resolution_ = 0.1;
          RCLCPP_WARN(logger_occupancy_map_monitor,"Resolution not specified for Octomap. Assuming resolution = %g instead", map_resolution_);
      }
	}

  RCLCPP_DEBUG(logger_occupancy_map_monitor,"Using resolution = %lf m for building octomap", map_resolution_);

  if (map_frame_.empty()){
    for (auto & parameter : ocupancy_map_monitor_parameters->get_parameters({"occupancy_map_server"})) {
        map_frame_ = parameter.value_to_string();
    }
    RCLCPP_WARN(logger_occupancy_map_monitor,"No target frame specified for Octomap. No transforms will be applied to received data.");
  }


  if (!tf_buffer_ && !map_frame_.empty())
    RCLCPP_WARN(logger_occupancy_map_monitor,"Target frame specified but no TF instance specified. No transforms will be applied to received data.");

  tree_.reset(new OccMapTree(map_resolution_));
  tree_const_ = tree_;

  // TODO: (@anasarrak) adapt this for ros2
  // XmlRpc::XmlRpcValue sensor_list;
  // if (nh_.getParam("sensors", sensor_list))
  // {
  //   try
  //   {
  //     if (sensor_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  //       for (int32_t i = 0; i < sensor_list.size(); ++i)
  //       {
  //         if (sensor_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
  //         {
  //           RCLCPP_ERROR(logger_occupancy_map_monitor,"Params for octomap updater %d not a struct; ignoring.", i);
  //           continue;
  //         }
  //
  //         if (!sensor_list[i].hasMember("sensor_plugin"))
  //         {
  //           RCLCPP_ERROR(logger_occupancy_map_monitor,"No sensor plugin specified for octomap updater %d; ignoring.", i);
  //           continue;
  //         }
  //
  //         std::string sensor_plugin = std::string(sensor_list[i]["sensor_plugin"]);
  //         if (sensor_plugin.empty() || sensor_plugin[0] == '~')
  //         {
  //           RCLCPP_INFO(logger_occupancy_map_monitor,"Skipping octomap updater plugin '%s'", sensor_plugin.c_str());
  //           continue;
  //         }
  //
  //         if (!updater_plugin_loader_)
  //         {
  //           try
  //           {
  //             updater_plugin_loader_.reset(new pluginlib::ClassLoader<OccupancyMapUpdater>(
  //                 "moveit_ros_perception", "occupancy_map_monitor::OccupancyMapUpdater"));
  //           }
  //           catch (pluginlib::PluginlibException& ex)
  //           {
  //             ROS_ERROR(logger_occupancy_map_monitor,"Exception while creating octomap updater plugin loader %s ", ex.what());
  //           }
  //         }
  //
  //         OccupancyMapUpdaterPtr up;
  //         try
  //         {
  //           up = updater_plugin_loader_->createUniqueInstance(sensor_plugin);
  //           up->setMonitor(this);
  //         }
  //         catch (pluginlib::PluginlibException& ex)
  //         {
  //           RCLCPP_ERROR(logger_occupancy_map_monitor,"Exception while loading octomap updater '%s': %s",sensor_plugin, ex.what());
  //         }
  //         if (up)
  //         {
  //           /* pass the params struct directly in to the updater */
  //           if (!up->setParams(sensor_list[i]))
  //           {
  //             RCLCPP_ERROR(logger_occupancy_map_monitor,"Failed to configure updater of type %s", up->getType().c_str());
  //             continue;
  //           }
  //
  //           if (!up->initialize())
  //           {
  //             RCLCPP_ERROR(logger_occupancy_map_monitor,"Unable to initialize map updater of type %s (plugin %s)", up->getType().c_str(),
  //                       sensor_plugin.c_str());
  //             continue;
  //           }
  //
  //           addUpdater(up);
  //         }
  //       }
  //     else
  //       RCLCPP_ERROR(logger_occupancy_map_monitor,"List of sensors must be an array!");
  //   }
  //   catch (XmlRpc::XmlRpcException& ex)
  //   {
  //     RCLCPP_ERROR(logger_occupancy_map_monitor,"XmlRpc Exception: %s", ex.getMessage().c_str());
  //   }
  // }

  /* advertise a service for loading octomaps from disk */

  std::function<bool( std::shared_ptr<rmw_request_id_t>,
                       const std::shared_ptr<moveit_msgs::srv::SaveMap::Request>,
                       std::shared_ptr<moveit_msgs::srv::SaveMap::Response>)> cb_savemap_function = std::bind(
         &OccupancyMapMonitor::saveMapCallback, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);

  std::function<bool( std::shared_ptr<rmw_request_id_t>,
                      const std::shared_ptr<moveit_msgs::srv::LoadMap::Request>,
                      std::shared_ptr<moveit_msgs::srv::LoadMap::Response>)> cb_loadmap_function = std::bind(
        &OccupancyMapMonitor::loadMapCallback, this, std::placeholders::_1,  std::placeholders::_2,  std::placeholders::_3);

  save_map_srv_ = node_->create_service<moveit_msgs::srv::SaveMap>("save_map", cb_savemap_function);
  load_map_srv_ = node_->create_service<moveit_msgs::srv::LoadMap>("load_map", cb_loadmap_function);

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
      if (map_updaters_.size() ==
          2)  // when we had one updater only, we passed direcly the transform cache callback to that updater
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
    RCLCPP_ERROR(logger_occupancy_map_monitor,"NULL updater was specified");
}

void OccupancyMapMonitor::publishDebugInformation(bool flag)
{
  debug_info_ = flag;
  for (std::size_t i = 0; i < map_updaters_.size(); ++i)
    map_updaters_[i]->publishDebugInformation(debug_info_);
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
    ShapeTransformCache tempCache;
    if (transform_cache_callback_(target_frame, target_time, tempCache))
    {
      for (ShapeTransformCache::iterator it = tempCache.begin(); it != tempCache.end(); ++it)
      {
        std::map<ShapeHandle, ShapeHandle>::const_iterator jt = mesh_handles_[index].find(it->first);
        if (jt == mesh_handles_[index].end())
        {
          RCUTILS_LOG_ERROR_THROTTLE(RCUTILS_STEADY_TIME,1, "Incorrect mapping of mesh handles");
          return false;
        }
        else
          cache[jt->second] = it->second;
      }
      return true;
    }
    else
      return false;
  }
  else
    return false;
}

bool OccupancyMapMonitor::saveMapCallback(std::shared_ptr<rmw_request_id_t> request_header,
                     const std::shared_ptr<moveit_msgs::srv::SaveMap::Request> req,
                     std::shared_ptr<moveit_msgs::srv::SaveMap::Response> res)
{
  (void)request_header; // avoid warning

  RCLCPP_INFO(logger_occupancy_map_monitor,"Writing map to %s", req->filename.c_str());
  tree_->lockRead();
  try
  {
    res->success = tree_->writeBinary(req->filename);
  }
  catch (...)
  {
    res->success = false;
  }
  tree_->unlockRead();
  return true;
}

bool OccupancyMapMonitor::loadMapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                        const std::shared_ptr<moveit_msgs::srv::LoadMap::Request> req,
                        std::shared_ptr<moveit_msgs::srv::LoadMap::Response> res)
{
  (void)request_header; // avoid warning

  RCLCPP_INFO(logger_occupancy_map_monitor,"Reading map from %s", req->filename.c_str());

  /* load the octree from disk */
  tree_->lockWrite();
  try
  {
    res->success = tree_->readBinary(req->filename);
  }
  catch (...)
  {
    RCLCPP_ERROR(logger_occupancy_map_monitor,"Failed to load map from file");
    res->success = false;
  }
  tree_->unlockWrite();

  return true;
}

void OccupancyMapMonitor::startMonitor()
{
  active_ = true;
  /* initialize all of the occupancy map updaters */
  for (std::size_t i = 0; i < map_updaters_.size(); ++i)
    map_updaters_[i]->start();
}

void OccupancyMapMonitor::stopMonitor()
{
  active_ = false;
  for (std::size_t i = 0; i < map_updaters_.size(); ++i)
    map_updaters_[i]->stop();
}

OccupancyMapMonitor::~OccupancyMapMonitor()
{
  stopMonitor();
}
}  // namespace occupancy_map_monitor
