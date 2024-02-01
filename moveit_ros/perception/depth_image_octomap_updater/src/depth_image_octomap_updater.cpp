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

/* Author: Ioan Sucan, Suat Gedikli */

#include <moveit/depth_image_octomap_updater/depth_image_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <geometric_shapes/shape_operations.h>
#include <sensor_msgs/image_encodings.hpp>
#include <stdint.h>
#include <moveit/utils/logger.hpp>

#include <memory>

namespace occupancy_map_monitor
{

DepthImageOctomapUpdater::DepthImageOctomapUpdater()
  : OccupancyMapUpdater("DepthImageUpdater")
  , image_topic_("depth")
  , queue_size_(5)
  , near_clipping_plane_distance_(0.3)
  , far_clipping_plane_distance_(5.0)
  , shadow_threshold_(0.04)
  , padding_scale_(0.0)
  , padding_offset_(0.02)
  , max_update_rate_(0)
  , skip_vertical_pixels_(4)
  , skip_horizontal_pixels_(6)
  , image_callback_count_(0)
  , average_callback_dt_(0.0)
  , good_tf_(5)
  ,  // start optimistically, so we do not output warnings right from the beginning
  failed_tf_(0)
  , K0_(0.0)
  , K2_(0.0)
  , K4_(0.0)
  , K5_(0.0)
  , logger_(moveit::getLogger("depth_image_octomap_updater"))
{
}

DepthImageOctomapUpdater::~DepthImageOctomapUpdater()
{
  sub_depth_image_.shutdown();
}

bool DepthImageOctomapUpdater::setParams(const std::string& name_space)
{
  try
  {
    node_->get_parameter(name_space + ".image_topic", image_topic_) &&
        node_->get_parameter(name_space + ".queue_size", queue_size_) &&
        node_->get_parameter(name_space + ".near_clipping_plane_distance", near_clipping_plane_distance_) &&
        node_->get_parameter(name_space + ".far_clipping_plane_distance", far_clipping_plane_distance_) &&
        node_->get_parameter(name_space + ".shadow_threshold", shadow_threshold_) &&
        node_->get_parameter(name_space + ".padding_scale", padding_scale_) &&
        node_->get_parameter(name_space + ".padding_offset", padding_offset_) &&
        node_->get_parameter(name_space + ".max_update_rate", max_update_rate_) &&
        node_->get_parameter(name_space + ".skip_vertical_pixels", skip_vertical_pixels_) &&
        node_->get_parameter(name_space + ".skip_horizontal_pixels", skip_horizontal_pixels_) &&
        node_->get_parameter(name_space + ".filtered_cloud_topic", filtered_cloud_topic_) &&
        node_->get_parameter(name_space + ".ns", ns_);
    return true;
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
  {
    RCLCPP_ERROR_STREAM(logger_, e.what() << '\n');
    return false;
  }
}

bool DepthImageOctomapUpdater::initialize(const rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  input_depth_transport_ = std::make_unique<image_transport::ImageTransport>(node_);
  model_depth_transport_ = std::make_unique<image_transport::ImageTransport>(node_);
  filtered_depth_transport_ = std::make_unique<image_transport::ImageTransport>(node_);
  filtered_label_transport_ = std::make_unique<image_transport::ImageTransport>(node_);

  tf_buffer_ = monitor_->getTFClient();
  free_space_updater_ = std::make_unique<LazyFreeSpaceUpdater>(tree_);

  // create our mesh filter
  mesh_filter_ = std::make_unique<mesh_filter::MeshFilter<mesh_filter::StereoCameraModel>>(
      mesh_filter::MeshFilterBase::TransformCallback(), mesh_filter::StereoCameraModel::REGISTERED_PSDK_PARAMS);
  mesh_filter_->parameters().setDepthRange(near_clipping_plane_distance_, far_clipping_plane_distance_);
  mesh_filter_->setShadowThreshold(shadow_threshold_);
  mesh_filter_->setPaddingOffset(padding_offset_);
  mesh_filter_->setPaddingScale(padding_scale_);
  mesh_filter_->setTransformCallback(
      [this](mesh_filter::MeshHandle mesh, Eigen::Isometry3d& tf) { return getShapeTransform(mesh, tf); });

  return true;
}

void DepthImageOctomapUpdater::start()
{
  pub_model_depth_image_ = model_depth_transport_->advertiseCamera("model_depth", 1);

  std::string prefix = "";
  if (!ns_.empty())
    prefix = ns_ + "/";

  pub_model_depth_image_ = model_depth_transport_->advertiseCamera(prefix + "model_depth", 1);
  if (!filtered_cloud_topic_.empty())
  {
    pub_filtered_depth_image_ = filtered_depth_transport_->advertiseCamera(prefix + filtered_cloud_topic_, 1);
  }
  else
  {
    pub_filtered_depth_image_ = filtered_depth_transport_->advertiseCamera(prefix + "filtered_depth", 1);
  }

  pub_filtered_label_image_ = filtered_label_transport_->advertiseCamera(prefix + "filtered_label", 1);

  sub_depth_image_ = image_transport::create_camera_subscription(
      node_.get(), image_topic_,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
             const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
        return depthImageCallback(depth_msg, info_msg);
      },
      "raw", rmw_qos_profile_sensor_data);
}

void DepthImageOctomapUpdater::stop()
{
  sub_depth_image_.shutdown();
}

mesh_filter::MeshHandle DepthImageOctomapUpdater::excludeShape(const shapes::ShapeConstPtr& shape)
{
  mesh_filter::MeshHandle h = 0;
  if (mesh_filter_)
  {
    if (shape->type == shapes::MESH)
    {
      h = mesh_filter_->addMesh(static_cast<const shapes::Mesh&>(*shape));
    }
    else
    {
      std::unique_ptr<shapes::Mesh> m(shapes::createMeshFromShape(shape.get()));
      if (m)
        h = mesh_filter_->addMesh(*m);
    }
  }
  else
    RCLCPP_ERROR(logger_, "Mesh filter not yet initialized!");
  return h;
}

void DepthImageOctomapUpdater::forgetShape(mesh_filter::MeshHandle handle)
{
  if (mesh_filter_)
    mesh_filter_->removeMesh(handle);
}

bool DepthImageOctomapUpdater::getShapeTransform(mesh_filter::MeshHandle h, Eigen::Isometry3d& transform) const
{
  ShapeTransformCache::const_iterator it = transform_cache_.find(h);
  if (it == transform_cache_.end())
  {
    RCLCPP_ERROR(logger_, "Internal error. Mesh filter handle %u not found", h);
    return false;
  }
  transform = it->second;
  return true;
}

namespace
{
const bool HOST_IS_BIG_ENDIAN = []() {
  union
  {
    uint32_t i;
    char c[sizeof(uint32_t)];
  } bint = { 0x01020304 };
  return bint.c[0] == 1;
}();
}  // namespace

void DepthImageOctomapUpdater::depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                                                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
{
  RCLCPP_DEBUG(logger_, "Received a new depth image message (frame = '%s', encoding='%s')",
               depth_msg->header.frame_id.c_str(), depth_msg->encoding.c_str());
  rclcpp::Time start = node_->now();

  if (max_update_rate_ > 0)
  {
    // ensure we are not updating the octomap representation too often
    if (node_->now() - last_update_time_ <= rclcpp::Duration::from_seconds(1.0 / max_update_rate_))
      return;
    last_update_time_ = node_->now();
  }

  // measure the frequency at which we receive updates
  if (image_callback_count_ < 1000)
  {
    if (image_callback_count_ > 0)
    {
      const double dt_start = (start - last_depth_callback_start_).seconds();
      if (image_callback_count_ < 2)
      {
        average_callback_dt_ = dt_start;
      }
      else
      {
        average_callback_dt_ = ((image_callback_count_ - 1) * average_callback_dt_ + dt_start) /
                               static_cast<double>(image_callback_count_);
      }
    }
  }
  else
  {
    // every 1000 updates we reset the counter almost to the beginning (use 2 so we don't have so much of a ripple in
    // the measured average)
    image_callback_count_ = 2;
  }
  last_depth_callback_start_ = start;
  ++image_callback_count_;

  if (monitor_->getMapFrame().empty())
    monitor_->setMapFrame(depth_msg->header.frame_id);

  /* get transform for cloud into map frame */
  tf2::Stamped<tf2::Transform> map_h_sensor;
  if (monitor_->getMapFrame() == depth_msg->header.frame_id)
  {
    map_h_sensor.setIdentity();
  }
  else
  {
    if (tf_buffer_)
    {
      // wait at most 50ms
      static const double TEST_DT = 0.005;
      const int nt =
          static_cast<int>((0.5 + average_callback_dt_ / TEST_DT) * std::max(1, (static_cast<int>(queue_size_) / 2)));
      bool found = false;
      std::string err;
      for (int t = 0; t < nt; ++t)
      {
        try
        {
          tf2::fromMsg(tf_buffer_->lookupTransform(monitor_->getMapFrame(), depth_msg->header.frame_id,
                                                   depth_msg->header.stamp),
                       map_h_sensor);
          found = true;
          break;
        }
        catch (tf2::TransformException& ex)
        {
          std::chrono::duration<double, std::nano> tmp_duration(TEST_DT);
          static const rclcpp::Duration D(tmp_duration);
          err = ex.what();
          std::this_thread::sleep_for(D.to_chrono<std::chrono::seconds>());
        }
      }
      static const unsigned int MAX_TF_COUNTER = 1000;  // so we avoid int overflow
      if (found)
      {
        good_tf_++;
        if (good_tf_ > MAX_TF_COUNTER)
        {
          const unsigned int div = MAX_TF_COUNTER / 10;
          good_tf_ /= div;
          failed_tf_ /= div;
        }
      }
      else
      {
        failed_tf_++;
        if (failed_tf_ > good_tf_)
        {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
          RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000,
                               "More than half of the image messages discarded due to TF being unavailable (%u%%). "
                               "Transform error of sensor data: %s; quitting callback.",
                               (100 * failed_tf_) / (good_tf_ + failed_tf_), err.c_str());
#pragma GCC diagnostic pop
        }
        else
        {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
          RCLCPP_DEBUG_THROTTLE(logger_, *node_->get_clock(), 1000,
                                "Transform error of sensor data: %s; quitting callback", err.c_str());
#pragma GCC diagnostic pop
        }
        if (failed_tf_ > MAX_TF_COUNTER)
        {
          const unsigned int div = MAX_TF_COUNTER / 10;
          good_tf_ /= div;
          failed_tf_ /= div;
        }
        return;
      }
    }
    else
      return;
  }

  if (!updateTransformCache(depth_msg->header.frame_id, depth_msg->header.stamp))
    return;

  if (depth_msg->is_bigendian && !HOST_IS_BIG_ENDIAN)
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
    RCLCPP_ERROR_THROTTLE(logger_, *node_->get_clock(), 1000, "endian problem: received image data does not match host");
#pragma GCC diagnostic pop
  }

  const int w = depth_msg->width;
  const int h = depth_msg->height;

  // call the mesh filter
  mesh_filter::StereoCameraModel::Parameters& params = mesh_filter_->parameters();
  params.setCameraParameters(info_msg->k[0], info_msg->k[4], info_msg->k[2], info_msg->k[5]);
  params.setImageSize(w, h);

  const bool is_u_short = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1;
  if (is_u_short)
  {
    mesh_filter_->filter(&depth_msg->data[0], GL_UNSIGNED_SHORT);
  }
  else
  {
    if (depth_msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1)
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
      RCLCPP_ERROR_THROTTLE(logger_, *node_->get_clock(), 1000, "Unexpected encoding type: '%s'. Ignoring input.",
                            depth_msg->encoding.c_str());
#pragma GCC diagnostic pop
      return;
    }
    mesh_filter_->filter(&depth_msg->data[0], GL_FLOAT);
  }

  // the mesh filter runs in background; compute extra things in the meantime

  // Use correct principal point from calibration
  const double px = info_msg->k[2];
  const double py = info_msg->k[5];

  // if the camera parameters have changed at all, recompute the cache we had
  if (w >= static_cast<int>(x_cache_.size()) || h >= static_cast<int>(y_cache_.size()) || K2_ != px || K5_ != py ||
      K0_ != info_msg->k[0] || K4_ != info_msg->k[4])
  {
    K2_ = px;
    K5_ = py;
    K0_ = info_msg->k[0];
    K4_ = info_msg->k[4];

    inv_fx_ = 1.0 / K0_;
    inv_fy_ = 1.0 / K4_;

    // if there are any NaNs, discard data
    if (isnan(px) || isnan(py) || isnan(inv_fx_) || isnan(inv_fy_))
      return;

    // Pre-compute some constants
    if (static_cast<int>(x_cache_.size()) < w)
      x_cache_.resize(w);
    if (static_cast<int>(y_cache_.size()) < h)
      y_cache_.resize(h);

    for (int x = 0; x < w; ++x)
      x_cache_[x] = (x - px) * inv_fx_;

    for (int y = 0; y < h; ++y)
      y_cache_[y] = (y - py) * inv_fy_;
  }

  const octomap::point3d sensor_origin(map_h_sensor.getOrigin().getX(), map_h_sensor.getOrigin().getY(),
                                       map_h_sensor.getOrigin().getZ());

  octomap::KeySet* occupied_cells_ptr = new octomap::KeySet();
  octomap::KeySet* model_cells_ptr = new octomap::KeySet();
  octomap::KeySet& occupied_cells = *occupied_cells_ptr;
  octomap::KeySet& model_cells = *model_cells_ptr;

  // allocate memory if needed
  std::size_t img_size = h * w;
  if (filtered_labels_.size() < img_size)
    filtered_labels_.resize(img_size);

  // get the labels of the filtered data
  const unsigned int* labels_row = &filtered_labels_[0];
  mesh_filter_->getFilteredLabels(&filtered_labels_[0]);

  // publish debug information if needed
  if (debug_info_)
  {
    sensor_msgs::msg::Image debug_msg;
    debug_msg.header = depth_msg->header;
    debug_msg.height = h;
    debug_msg.width = w;
    debug_msg.is_bigendian = HOST_IS_BIG_ENDIAN;
    debug_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    debug_msg.step = w * sizeof(float);
    debug_msg.data.resize(img_size * sizeof(float));
    mesh_filter_->getModelDepth(reinterpret_cast<double*>(&debug_msg.data[0]));
    pub_model_depth_image_.publish(debug_msg, *info_msg);

    sensor_msgs::msg::Image filtered_depth_msg;
    filtered_depth_msg.header = depth_msg->header;
    filtered_depth_msg.height = h;
    filtered_depth_msg.width = w;
    filtered_depth_msg.is_bigendian = HOST_IS_BIG_ENDIAN;
    filtered_depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    filtered_depth_msg.step = w * sizeof(float);
    filtered_depth_msg.data.resize(img_size * sizeof(float));
    mesh_filter_->getFilteredDepth(reinterpret_cast<double*>(&filtered_depth_msg.data[0]));
    pub_filtered_depth_image_.publish(filtered_depth_msg, *info_msg);

    sensor_msgs::msg::Image label_msg;
    label_msg.header = depth_msg->header;
    label_msg.height = h;
    label_msg.width = w;
    label_msg.is_bigendian = HOST_IS_BIG_ENDIAN;
    label_msg.encoding = sensor_msgs::image_encodings::RGBA8;
    label_msg.step = w * sizeof(unsigned int);
    label_msg.data.resize(img_size * sizeof(unsigned int));
    mesh_filter_->getFilteredLabels(reinterpret_cast<unsigned int*>(&label_msg.data[0]));

    pub_filtered_label_image_.publish(label_msg, *info_msg);
  }

  if (!filtered_cloud_topic_.empty())
  {
    sensor_msgs::msg::Image filtered_msg;
    filtered_msg.header = depth_msg->header;
    filtered_msg.height = h;
    filtered_msg.width = w;
    filtered_msg.is_bigendian = HOST_IS_BIG_ENDIAN;
    filtered_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    filtered_msg.step = w * sizeof(unsigned short);
    filtered_msg.data.resize(img_size * sizeof(unsigned short));

    // reuse float buffer across callbacks
    static std::vector<float> filtered_data;
    if (filtered_data.size() < img_size)
      filtered_data.resize(img_size);

    mesh_filter_->getFilteredDepth(reinterpret_cast<double*>(&filtered_data[0]));
    unsigned short* msg_data = reinterpret_cast<unsigned short*>(&filtered_msg.data[0]);
    for (std::size_t i = 0; i < img_size; ++i)
    {
      // rescale depth to millimeter to work with `unsigned short`
      msg_data[i] = static_cast<unsigned short>(filtered_data[i] * 1000 + 0.5);
    }
    pub_filtered_depth_image_.publish(filtered_msg, *info_msg);
  }

  // figure out occupied cells and model cells
  tree_->lockRead();

  try
  {
    const int h_bound = h - skip_vertical_pixels_;
    const int w_bound = w - skip_horizontal_pixels_;

    if (is_u_short)
    {
      const uint16_t* input_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);

      for (int y = skip_vertical_pixels_; y < h_bound; ++y, labels_row += w, input_row += w)
      {
        for (int x = skip_horizontal_pixels_; x < w_bound; ++x)
        {
          // not filtered
          if (labels_row[x] == mesh_filter::MeshFilterBase::BACKGROUND)
          {
            float zz = static_cast<float>(input_row[x]) * 1e-3;  // scale from mm to m
            float yy = y_cache_[y] * zz;
            float xx = x_cache_[x] * zz;
            /* transform to map frame */
            tf2::Vector3 point_tf = map_h_sensor * tf2::Vector3(xx, yy, zz);
            occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
          // on far plane or a model point -> remove
          else if (labels_row[x] >= mesh_filter::MeshFilterBase::FAR_CLIP)
          {
            float zz = input_row[x] * 1e-3;
            float yy = y_cache_[y] * zz;
            float xx = x_cache_[x] * zz;
            /* transform to map frame */
            tf2::Vector3 point_tf = map_h_sensor * tf2::Vector3(xx, yy, zz);
            // add to the list of model cells
            model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
        }
      }
    }
    else
    {
      const float* input_row = reinterpret_cast<const float*>(&depth_msg->data[0]);

      for (int y = skip_vertical_pixels_; y < h_bound; ++y, labels_row += w, input_row += w)
      {
        for (int x = skip_horizontal_pixels_; x < w_bound; ++x)
        {
          if (labels_row[x] == mesh_filter::MeshFilterBase::BACKGROUND)
          {
            float zz = input_row[x];
            float yy = y_cache_[y] * zz;
            float xx = x_cache_[x] * zz;
            /* transform to map frame */
            tf2::Vector3 point_tf = map_h_sensor * tf2::Vector3(xx, yy, zz);
            occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
          else if (labels_row[x] >= mesh_filter::MeshFilterBase::FAR_CLIP)
          {
            float zz = input_row[x];
            float yy = y_cache_[y] * zz;
            float xx = x_cache_[x] * zz;
            /* transform to map frame */
            tf2::Vector3 point_tf = map_h_sensor * tf2::Vector3(xx, yy, zz);
            // add to the list of model cells
            model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
          }
        }
      }
    }
  }
  catch (...)
  {
    tree_->unlockRead();
    RCLCPP_ERROR(logger_, "Internal error while parsing depth data");
    return;
  }
  tree_->unlockRead();

  /* cells that overlap with the model are not occupied */
  for (const octomap::OcTreeKey& model_cell : model_cells)
    occupied_cells.erase(model_cell);

  // mark occupied cells
  tree_->lockWrite();
  try
  {
    /* now mark all occupied cells */
    for (const octomap::OcTreeKey& occupied_cell : occupied_cells)
      tree_->updateNode(occupied_cell, true);
  }
  catch (...)
  {
    RCLCPP_ERROR(logger_, "Internal error while updating octree");
  }
  tree_->unlockWrite();
  tree_->triggerUpdateCallback();

  // at this point we still have not freed the space
  free_space_updater_->pushLazyUpdate(occupied_cells_ptr, model_cells_ptr, sensor_origin);

  RCLCPP_DEBUG(logger_, "Processed depth image in %lf ms", (node_->now() - start).seconds() * 1000.0);
}
}  // namespace occupancy_map_monitor
