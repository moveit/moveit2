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

/* Author: Ioan Sucan */

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/utils/message_checks.h>
#include <moveit/exceptions/exceptions.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/algorithm/string/join.hpp>
#include <memory>

#include <std_msgs/msg/string.hpp>

#include <chrono>
using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.planning_scene_monitor.planning_scene_monitor");

namespace planning_scene_monitor
{
const std::string PlanningSceneMonitor::DEFAULT_JOINT_STATES_TOPIC = "joint_states";
const std::string PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC = "attached_collision_object";
const std::string PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC = "collision_object";
const std::string PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC = "planning_scene_world";
const std::string PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC = "planning_scene";
const std::string PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE = "get_planning_scene";
const std::string PlanningSceneMonitor::MONITORED_PLANNING_SCENE_TOPIC = "monitored_planning_scene";

PlanningSceneMonitor::PlanningSceneMonitor(const rclcpp::Node::SharedPtr& node, const std::string& robot_description,
                                           const std::string& name)
  : PlanningSceneMonitor(node, planning_scene::PlanningScenePtr(), robot_description, name)
{
}

PlanningSceneMonitor::PlanningSceneMonitor(const rclcpp::Node::SharedPtr& node,
                                           const planning_scene::PlanningScenePtr& scene,
                                           const std::string& robot_description, const std::string& name)
  : PlanningSceneMonitor(node, scene, std::make_shared<robot_model_loader::RobotModelLoader>(node, robot_description),
                         name)
{
}

PlanningSceneMonitor::PlanningSceneMonitor(const rclcpp::Node::SharedPtr& node,
                                           const robot_model_loader::RobotModelLoaderPtr& rm_loader,
                                           const std::string& name)
  : PlanningSceneMonitor(node, planning_scene::PlanningScenePtr(), rm_loader, name)
{
}

PlanningSceneMonitor::PlanningSceneMonitor(const rclcpp::Node::SharedPtr& node,
                                           const planning_scene::PlanningScenePtr& scene,
                                           const robot_model_loader::RobotModelLoaderPtr& rm_loader,
                                           const std::string& name)
  : monitor_name_(name)
  , node_(node)
  , private_executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  , tf_buffer_(std::make_shared<tf2_ros::Buffer>(node->get_clock()))
  , dt_state_update_(0.0)
  , shape_transform_cache_lookup_wait_time_(0, 0)
  , rm_loader_(rm_loader)
{
  std::vector<std::string> new_args = rclcpp::NodeOptions().arguments();
  new_args.push_back("--ros-args");
  new_args.push_back("-r");
  // Add random ID to prevent warnings about multiple publishers within the same node
  new_args.push_back(std::string("__node:=") + node_->get_name() + "_private_" +
                     std::to_string(reinterpret_cast<std::size_t>(this)));
  new_args.push_back("--");
  pnode_ = std::make_shared<rclcpp::Node>("_", node_->get_namespace(), rclcpp::NodeOptions().arguments(new_args));
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_buffer_->setUsingDedicatedThread(true);
  // use same callback queue as root_nh_
  // root_nh_.setCallbackQueue(&queue_);
  // nh_.setCallbackQueue(&queue_);
  // spinner_.reset(new ros::AsyncSpinner(1, &queue_));
  // spinner_->start();
  initialize(scene);

  use_sim_time_ = node->get_parameter("use_sim_time").as_bool();
}

PlanningSceneMonitor::~PlanningSceneMonitor()
{
  if (scene_)
  {
    scene_->setCollisionObjectUpdateCallback(collision_detection::World::ObserverCallbackFn());
    scene_->setAttachedBodyUpdateCallback(moveit::core::AttachedBodyCallback());
  }
  stopPublishingPlanningScene();
  stopStateMonitor();
  stopWorldGeometryMonitor();
  stopSceneMonitor();

  private_executor_->cancel();
  if (private_executor_thread_.joinable())
    private_executor_thread_.join();
  private_executor_.reset();

  current_state_monitor_.reset();
  scene_const_.reset();
  scene_.reset();
  parent_scene_.reset();
  robot_model_.reset();
  rm_loader_.reset();
}

void PlanningSceneMonitor::initialize(const planning_scene::PlanningScenePtr& scene)
{
  if (monitor_name_.empty())
    monitor_name_ = "planning_scene_monitor";
  robot_description_ = rm_loader_->getRobotDescription();
  if (rm_loader_->getModel())
  {
    robot_model_ = rm_loader_->getModel();
    scene_ = scene;
    if (!scene_)
    {
      try
      {
        scene_ = std::make_shared<planning_scene::PlanningScene>(rm_loader_->getModel());
        configureCollisionMatrix(scene_);
        configureDefaultPadding();

        scene_->getCollisionEnvNonConst()->setPadding(default_robot_padd_);
        scene_->getCollisionEnvNonConst()->setScale(default_robot_scale_);
        for (const std::pair<const std::string, double>& it : default_robot_link_padd_)
        {
          scene_->getCollisionEnvNonConst()->setLinkPadding(it.first, it.second);
        }
        for (const std::pair<const std::string, double>& it : default_robot_link_scale_)
        {
          scene_->getCollisionEnvNonConst()->setLinkScale(it.first, it.second);
        }
      }
      catch (moveit::ConstructException& e)
      {
        RCLCPP_ERROR(LOGGER, "Configuration of planning scene failed");
        scene_.reset();
      }
    }
    // scene_const_ is set regardless if scene_ is null or not
    scene_const_ = scene_;
    if (scene_)
    {
      // The scene_ is loaded on the collision loader only if it was correctly instantiated
      collision_loader_.setupScene(node_, scene_);
      scene_->setAttachedBodyUpdateCallback([this](moveit::core::AttachedBody* body, bool attached) {
        currentStateAttachedBodyUpdateCallback(body, attached);
      });
      scene_->setCollisionObjectUpdateCallback(
          [this](const collision_detection::World::ObjectConstPtr& object, collision_detection::World::Action action) {
            currentWorldObjectUpdateCallback(object, action);
          });
    }
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Robot model not loaded");
  }

  publish_planning_scene_frequency_ = 2.0;
  new_scene_update_ = UPDATE_NONE;

  last_update_time_ = last_robot_motion_time_ = rclcpp::Clock().now();
  last_robot_state_update_wall_time_ = std::chrono::system_clock::now();
  dt_state_update_ = std::chrono::duration<double>(0.03);

  double temp_wait_time = 0.05;

  if (!robot_description_.empty())
  {
    node_->get_parameter_or(robot_description_ + "_planning.shape_transform_cache_lookup_wait_time", temp_wait_time,
                            temp_wait_time);
  }

  shape_transform_cache_lookup_wait_time_ = rclcpp::Duration::from_seconds(temp_wait_time);

  state_update_pending_ = false;
  // Period for 0.1 sec
  using std::chrono::nanoseconds;
  state_update_timer_ = pnode_->create_wall_timer(dt_state_update_, [this]() { return stateUpdateTimerCallback(); });
  private_executor_->add_node(pnode_);

  // start executor on a different thread now
  private_executor_thread_ = std::thread([this]() { private_executor_->spin(); });

  auto declare_parameter = [this](const std::string& param_name, auto default_val,
                                  const std::string& description) -> auto
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.set__description(description);
    return pnode_->declare_parameter(param_name, default_val, desc);
  };

  try
  {
    // Set up publishing parameters
    bool publish_planning_scene =
        declare_parameter("publish_planning_scene", false, "Set to True to publish Planning Scenes");
    bool publish_geometry_updates = declare_parameter("publish_geometry_updates", false,
                                                      "Set to True to publish geometry updates of the planning scene");
    bool publish_state_updates =
        declare_parameter("publish_state_updates", false, "Set to True to publish state updates of the planning scene");
    bool publish_transform_updates = declare_parameter(
        "publish_transforms_updates", false, "Set to True to publish transform updates of the planning scene");
    double publish_planning_scene_hz = declare_parameter(
        "publish_planning_scene_hz", 4.0, "Set the maximum frequency at which planning scene updates are published");
    updatePublishSettings(publish_geometry_updates, publish_state_updates, publish_transform_updates,
                          publish_planning_scene, publish_planning_scene_hz);
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Invalid parameter type in PlanningSceneMonitor: " << e.what());
    RCLCPP_ERROR(LOGGER, "Dynamic publishing parameters won't be available");
    return;
  }

  auto psm_parameter_set_callback = [this](const std::vector<rclcpp::Parameter>& parameters) -> auto
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    bool publish_planning_scene = false, publish_geometry_updates = false, publish_state_updates = false,
         publish_transform_updates = false;
    double publish_planning_scene_hz = 4.0;
    bool declared_params_valid = pnode_->get_parameter("publish_planning_scene", publish_planning_scene) &&
                                 pnode_->get_parameter("publish_geometry_updates", publish_geometry_updates) &&
                                 pnode_->get_parameter("publish_state_updates", publish_state_updates) &&
                                 pnode_->get_parameter("publish_transforms_updates", publish_transform_updates) &&
                                 pnode_->get_parameter("publish_planning_scene_hz", publish_planning_scene_hz);

    if (!declared_params_valid)
    {
      RCLCPP_ERROR(LOGGER, "Initially declared parameters are invalid - failed to process update callback");
      result.successful = false;
      return result;
    }

    for (const auto& parameter : parameters)
    {
      const auto& name = parameter.get_name();
      const auto& type = parameter.get_type();

      // Only allow already declared parameters with same value type
      if (!pnode_->has_parameter(name) || pnode_->get_parameter(name).get_type() != type)
      {
        RCLCPP_ERROR(LOGGER, "Invalid parameter in PlanningSceneMonitor parameter callback");
        result.successful = false;
        return result;
      }

      // Update parameter values
      if (name == "planning_scene_monitor.publish_planning_scene")
      {
        publish_planning_scene = parameter.as_bool();
      }
      else if (name == "planning_scene_monitor.publish_geometry_updates")
      {
        publish_geometry_updates = parameter.as_bool();
      }
      else if (name == "planning_scene_monitor.publish_state_updates")
      {
        publish_state_updates = parameter.as_bool();
      }
      else if (name == "planning_scene_monitor.publish_transforms_updates")
      {
        publish_transform_updates = parameter.as_bool();
      }
      else if (name == "planning_scene_monitor.publish_planning_scene_hz")
      {
        publish_planning_scene_hz = parameter.as_double();
      }
    }

    if (result.successful)
    {
      updatePublishSettings(publish_geometry_updates, publish_state_updates, publish_transform_updates,
                            publish_planning_scene, publish_planning_scene_hz);
    }
    return result;
  };

  callback_handler_ = pnode_->add_on_set_parameters_callback(psm_parameter_set_callback);
}

void PlanningSceneMonitor::monitorDiffs(bool flag)
{
  if (scene_)
  {
    if (flag)
    {
      std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
      if (scene_)
      {
        scene_->setAttachedBodyUpdateCallback(moveit::core::AttachedBodyCallback());
        scene_->setCollisionObjectUpdateCallback(collision_detection::World::ObserverCallbackFn());
        scene_->decoupleParent();
        parent_scene_ = scene_;
        scene_ = parent_scene_->diff();
        scene_const_ = scene_;
        scene_->setAttachedBodyUpdateCallback([this](moveit::core::AttachedBody* body, bool attached) {
          currentStateAttachedBodyUpdateCallback(body, attached);
        });
        scene_->setCollisionObjectUpdateCallback(
            [this](const collision_detection::World::ObjectConstPtr& object,
                   collision_detection::World::Action action) { currentWorldObjectUpdateCallback(object, action); });
      }
    }
    else
    {
      if (publish_planning_scene_)
      {
        RCLCPP_WARN(LOGGER, "Diff monitoring was stopped while publishing planning scene diffs. "
                            "Stopping planning scene diff publisher");
        stopPublishingPlanningScene();
      }
      {
        std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
        if (scene_)
        {
          scene_->decoupleParent();
          parent_scene_.reset();
          // remove the '+' added by .diff() at the end of the scene name
          if (!scene_->getName().empty())
          {
            if (scene_->getName()[scene_->getName().length() - 1] == '+')
              scene_->setName(scene_->getName().substr(0, scene_->getName().length() - 1));
          }
        }
      }
    }
  }
}

void PlanningSceneMonitor::stopPublishingPlanningScene()
{
  if (publish_planning_scene_)
  {
    std::unique_ptr<std::thread> copy;
    copy.swap(publish_planning_scene_);
    new_scene_update_condition_.notify_all();
    copy->join();
    monitorDiffs(false);
    planning_scene_publisher_.reset();
    RCLCPP_INFO(LOGGER, "Stopped publishing maintained planning scene.");
  }
}

void PlanningSceneMonitor::startPublishingPlanningScene(SceneUpdateType update_type,
                                                        const std::string& planning_scene_topic)
{
  publish_update_types_ = update_type;
  if (!publish_planning_scene_ && scene_)
  {
    planning_scene_publisher_ = pnode_->create_publisher<moveit_msgs::msg::PlanningScene>(planning_scene_topic, 100);
    RCLCPP_INFO(LOGGER, "Publishing maintained planning scene on '%s'", planning_scene_topic.c_str());
    monitorDiffs(true);
    publish_planning_scene_ = std::make_unique<std::thread>([this] { scenePublishingThread(); });
  }
}

void PlanningSceneMonitor::scenePublishingThread()
{
  RCLCPP_DEBUG(LOGGER, "Started scene publishing thread ...");

  // publish the full planning scene once
  {
    moveit_msgs::msg::PlanningScene msg;
    {
      collision_detection::OccMapTree::ReadLock lock;
      if (octomap_monitor_)
        lock = octomap_monitor_->getOcTreePtr()->reading();
      scene_->getPlanningSceneMsg(msg);
    }
    planning_scene_publisher_->publish(msg);
    RCLCPP_DEBUG(LOGGER, "Published the full planning scene: '%s'", msg.name.c_str());
  }

  do
  {
    moveit_msgs::msg::PlanningScene msg;
    bool publish_msg = false;
    bool is_full = false;
    rclcpp::Rate rate(publish_planning_scene_frequency_);
    {
      std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
      while (new_scene_update_ == UPDATE_NONE && publish_planning_scene_)
        new_scene_update_condition_.wait(ulock);
      if (new_scene_update_ != UPDATE_NONE)
      {
        if ((publish_update_types_ & new_scene_update_) || new_scene_update_ == UPDATE_SCENE)
        {
          if (new_scene_update_ == UPDATE_SCENE)
          {
            is_full = true;
          }
          else
          {
            collision_detection::OccMapTree::ReadLock lock;
            if (octomap_monitor_)
              lock = octomap_monitor_->getOcTreePtr()->reading();
            scene_->getPlanningSceneDiffMsg(msg);
            if (new_scene_update_ == UPDATE_STATE)
            {
              msg.robot_state.attached_collision_objects.clear();
              msg.robot_state.is_diff = true;
            }
          }
          std::scoped_lock prevent_shape_cache_updates(shape_handles_lock_);  // we don't want the
                                                                              // transform cache to
                                                                              // update while we are
                                                                              // potentially changing
                                                                              // attached bodies
          scene_->setAttachedBodyUpdateCallback(moveit::core::AttachedBodyCallback());
          scene_->setCollisionObjectUpdateCallback(collision_detection::World::ObserverCallbackFn());
          scene_->pushDiffs(parent_scene_);
          scene_->clearDiffs();
          scene_->setAttachedBodyUpdateCallback([this](moveit::core::AttachedBody* body, bool attached) {
            currentStateAttachedBodyUpdateCallback(body, attached);
          });
          scene_->setCollisionObjectUpdateCallback(
              [this](const collision_detection::World::ObjectConstPtr& object,
                     collision_detection::World::Action action) { currentWorldObjectUpdateCallback(object, action); });
          if (octomap_monitor_)
          {
            excludeAttachedBodiesFromOctree();  // in case updates have happened to the attached bodies, put them in
            excludeWorldObjectsFromOctree();    // in case updates have happened to the attached bodies, put them in
          }
          if (is_full)
          {
            collision_detection::OccMapTree::ReadLock lock;
            if (octomap_monitor_)
              lock = octomap_monitor_->getOcTreePtr()->reading();
            scene_->getPlanningSceneMsg(msg);
          }
          // also publish timestamp of this robot_state
          msg.robot_state.joint_state.header.stamp = last_robot_motion_time_;
          publish_msg = true;
        }
        new_scene_update_ = UPDATE_NONE;
      }
    }
    if (publish_msg)
    {
      planning_scene_publisher_->publish(msg);
      if (is_full)
        RCLCPP_DEBUG(LOGGER, "Published full planning scene: '%s'", msg.name.c_str());
      rate.sleep();
    }
  } while (publish_planning_scene_);
}

void PlanningSceneMonitor::getMonitoredTopics(std::vector<std::string>& topics) const
{
  // TODO(anasarrak): Do we need this for ROS2?
  topics.clear();
  if (current_state_monitor_)
  {
    const std::string& t = current_state_monitor_->getMonitoredTopic();
    if (!t.empty())
      topics.push_back(t);
  }
  if (planning_scene_subscriber_)
    topics.push_back(planning_scene_subscriber_->get_topic_name());
  if (collision_object_subscriber_)
  {
    // TODO (anasarrak) This has been changed to subscriber on Moveit, look at
    // https://github.com/ros-planning/moveit/pull/1406/files/cb9488312c00e9c8949d89b363766f092330954d#diff-fb6e26ecc9a73d59dbdae3f3e08145e6
    topics.push_back(collision_object_subscriber_->get_topic_name());
  }
  if (planning_scene_world_subscriber_)
    topics.push_back(planning_scene_world_subscriber_->get_topic_name());
}

namespace
{
bool sceneIsParentOf(const planning_scene::PlanningSceneConstPtr& scene,
                     const planning_scene::PlanningScene* possible_parent)
{
  if (scene && scene.get() == possible_parent)
    return true;
  if (scene)
    return sceneIsParentOf(scene->getParent(), possible_parent);
  return false;
}
}  // namespace

bool PlanningSceneMonitor::updatesScene(const planning_scene::PlanningScenePtr& scene) const
{
  return sceneIsParentOf(scene_const_, scene.get());
}

bool PlanningSceneMonitor::updatesScene(const planning_scene::PlanningSceneConstPtr& scene) const
{
  return sceneIsParentOf(scene_const_, scene.get());
}

void PlanningSceneMonitor::triggerSceneUpdateEvent(SceneUpdateType update_type)
{
  // do not modify update functions while we are calling them
  std::scoped_lock lock(update_lock_);

  for (std::function<void(SceneUpdateType)>& update_callback : update_callbacks_)
    update_callback(update_type);
  new_scene_update_ = static_cast<SceneUpdateType>(static_cast<int>(new_scene_update_) | static_cast<int>(update_type));
  new_scene_update_condition_.notify_all();
}

bool PlanningSceneMonitor::requestPlanningSceneState(const std::string& service_name)
{
  if (get_scene_service_ && get_scene_service_->get_service_name() == service_name)
  {
    RCLCPP_FATAL_STREAM(LOGGER, "requestPlanningSceneState() to self-provided service '" << service_name << '\'');
    throw std::runtime_error("requestPlanningSceneState() to self-provided service: " + service_name);
  }
  // use global namespace for service
  auto client = pnode_->create_client<moveit_msgs::srv::GetPlanningScene>(service_name);
  auto srv = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  srv->components.components = srv->components.SCENE_SETTINGS | srv->components.ROBOT_STATE |
                               srv->components.ROBOT_STATE_ATTACHED_OBJECTS | srv->components.WORLD_OBJECT_NAMES |
                               srv->components.WORLD_OBJECT_GEOMETRY | srv->components.OCTOMAP |
                               srv->components.TRANSFORMS | srv->components.ALLOWED_COLLISION_MATRIX |
                               srv->components.LINK_PADDING_AND_SCALING | srv->components.OBJECT_COLORS;

  // Make sure client is connected to server
  RCLCPP_DEBUG(LOGGER, "Waiting for GetPlanningScene service `%s` to exist.", service_name.c_str());
  if (client->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_DEBUG(LOGGER, "Sending service request to `%s`.", service_name.c_str());
    auto service_result = client->async_send_request(srv);
    const auto service_result_status = service_result.wait_for(std::chrono::seconds(5));
    if (service_result_status == std::future_status::ready)  // Success
    {
      RCLCPP_DEBUG(LOGGER, "Service request succeeded, applying new planning scene");
      newPlanningSceneMessage(service_result.get()->scene);
      return true;
    }
    if (service_result_status == std::future_status::timeout)  // Timeout
    {
      RCLCPP_INFO(LOGGER, "GetPlanningScene service call to %s timed out. at %s:%d", service_name.c_str(), __FILE__,
                  __LINE__);
      return false;
    }
  }

  // If we are here, service is not available or call failed
  RCLCPP_INFO(LOGGER,
              "Failed to call service %s, have you launched move_group or called psm.providePlanningSceneService()?",
              service_name.c_str());

  return false;
}

void PlanningSceneMonitor::providePlanningSceneService(const std::string& service_name)
{
  // Load the service
  get_scene_service_ = pnode_->create_service<moveit_msgs::srv::GetPlanningScene>(
      service_name, [this](const moveit_msgs::srv::GetPlanningScene::Request::SharedPtr& req,
                           const moveit_msgs::srv::GetPlanningScene::Response::SharedPtr& res) {
        return getPlanningSceneServiceCallback(req, res);
      });
}

void PlanningSceneMonitor::getPlanningSceneServiceCallback(
    const moveit_msgs::srv::GetPlanningScene::Request::SharedPtr& req,
    const moveit_msgs::srv::GetPlanningScene::Response::SharedPtr& res)
{
  if (req->components.components & moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS)
    updateFrameTransforms();

  moveit_msgs::msg::PlanningSceneComponents all_components;
  all_components.components = UINT_MAX;  // Return all scene components if nothing is specified.

  std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
  scene_->getPlanningSceneMsg(res->scene, req->components.components ? req->components : all_components);
}

void PlanningSceneMonitor::updatePublishSettings(bool publish_geom_updates, bool publish_state_updates,
                                                 bool publish_transform_updates, bool publish_planning_scene,
                                                 double publish_planning_scene_hz)
{
  PlanningSceneMonitor::SceneUpdateType event = PlanningSceneMonitor::UPDATE_NONE;
  if (publish_geom_updates)
  {
    event = static_cast<PlanningSceneMonitor::SceneUpdateType>(static_cast<int>(event) |
                                                               static_cast<int>(PlanningSceneMonitor::UPDATE_GEOMETRY));
  }
  if (publish_state_updates)
  {
    event = static_cast<PlanningSceneMonitor::SceneUpdateType>(static_cast<int>(event) |
                                                               static_cast<int>(PlanningSceneMonitor::UPDATE_STATE));
  }
  if (publish_transform_updates)
  {
    event = static_cast<PlanningSceneMonitor::SceneUpdateType>(
        static_cast<int>(event) | static_cast<int>(PlanningSceneMonitor::UPDATE_TRANSFORMS));
  }
  if (publish_planning_scene)
  {
    setPlanningScenePublishingFrequency(publish_planning_scene_hz);
    startPublishingPlanningScene(event);
  }
  else
    stopPublishingPlanningScene();
}

void PlanningSceneMonitor::newPlanningSceneCallback(const moveit_msgs::msg::PlanningScene::ConstSharedPtr& scene)
{
  newPlanningSceneMessage(*scene);
}

void PlanningSceneMonitor::clearOctomap()
{
  bool removed = false;
  {
    std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
    removed = scene_->getWorldNonConst()->removeObject(scene_->OCTOMAP_NS);

    if (octomap_monitor_)
    {
      octomap_monitor_->getOcTreePtr()->lockWrite();
      octomap_monitor_->getOcTreePtr()->clear();
      octomap_monitor_->getOcTreePtr()->unlockWrite();
    }
    else
    {
      RCLCPP_WARN(LOGGER, "Unable to clear octomap since no octomap monitor has been initialized");
    }  // Lift the scoped lock before calling triggerSceneUpdateEvent to avoid deadlock
  }

  if (removed)
    triggerSceneUpdateEvent(UPDATE_GEOMETRY);
}

bool PlanningSceneMonitor::newPlanningSceneMessage(const moveit_msgs::msg::PlanningScene& scene)
{
  if (!scene_)
    return false;

  bool result;

  SceneUpdateType upd = UPDATE_SCENE;
  std::string old_scene_name;
  {
    std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
    // we don't want the transform cache to update while we are potentially changing attached bodies
    std::scoped_lock prevent_shape_cache_updates(shape_handles_lock_);

    last_update_time_ = rclcpp::Clock().now();
    last_robot_motion_time_ = scene.robot_state.joint_state.header.stamp;
    RCLCPP_DEBUG(LOGGER, "scene update %f robot stamp: %f", fmod(last_update_time_.seconds(), 10.),
                 fmod(last_robot_motion_time_.seconds(), 10.));
    old_scene_name = scene_->getName();
    result = scene_->usePlanningSceneMsg(scene);
    if (octomap_monitor_)
    {
      if (!scene.is_diff && scene.world.octomap.octomap.data.empty())
      {
        octomap_monitor_->getOcTreePtr()->lockWrite();
        octomap_monitor_->getOcTreePtr()->clear();
        octomap_monitor_->getOcTreePtr()->unlockWrite();
      }
    }
    robot_model_ = scene_->getRobotModel();

    // if we just reset the scene completely but we were maintaining diffs, we need to fix that
    if (!scene.is_diff && parent_scene_)
    {
      // the scene is now decoupled from the parent, since we just reset it
      scene_->setAttachedBodyUpdateCallback(moveit::core::AttachedBodyCallback());
      scene_->setCollisionObjectUpdateCallback(collision_detection::World::ObserverCallbackFn());
      parent_scene_ = scene_;
      scene_ = parent_scene_->diff();
      scene_const_ = scene_;
      scene_->setAttachedBodyUpdateCallback([this](moveit::core::AttachedBody* body, bool attached) {
        currentStateAttachedBodyUpdateCallback(body, attached);
      });
      scene_->setCollisionObjectUpdateCallback(
          [this](const collision_detection::World::ObjectConstPtr& object, collision_detection::World::Action action) {
            currentWorldObjectUpdateCallback(object, action);
          });
    }
    if (octomap_monitor_)
    {
      excludeAttachedBodiesFromOctree();  // in case updates have happened to the attached bodies, put them in
      excludeWorldObjectsFromOctree();    // in case updates have happened to the attached bodies, put them in
    }
  }

  // if we have a diff, try to more accurately determine the update type
  if (scene.is_diff)
  {
    bool no_other_scene_upd = (scene.name.empty() || scene.name == old_scene_name) &&
                              scene.allowed_collision_matrix.entry_names.empty() && scene.link_padding.empty() &&
                              scene.link_scale.empty();
    if (no_other_scene_upd)
    {
      upd = UPDATE_NONE;
      if (!moveit::core::isEmpty(scene.world))
        upd = static_cast<SceneUpdateType>(static_cast<int>(upd) | static_cast<int>(UPDATE_GEOMETRY));

      if (!scene.fixed_frame_transforms.empty())
        upd = static_cast<SceneUpdateType>(static_cast<int>(upd) | static_cast<int>(UPDATE_TRANSFORMS));

      if (!moveit::core::isEmpty(scene.robot_state))
      {
        upd = static_cast<SceneUpdateType>(static_cast<int>(upd) | static_cast<int>(UPDATE_STATE));
        if (!scene.robot_state.attached_collision_objects.empty() || !scene.robot_state.is_diff)
          upd = static_cast<SceneUpdateType>(static_cast<int>(upd) | static_cast<int>(UPDATE_GEOMETRY));
      }
    }
  }
  triggerSceneUpdateEvent(upd);
  return result;
}

void PlanningSceneMonitor::newPlanningSceneWorldCallback(
    const moveit_msgs::msg::PlanningSceneWorld::ConstSharedPtr& world)
{
  if (scene_)
  {
    updateFrameTransforms();
    {
      std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
      last_update_time_ = rclcpp::Clock().now();
      scene_->getWorldNonConst()->clearObjects();
      scene_->processPlanningSceneWorldMsg(*world);
      if (octomap_monitor_)
      {
        if (world->octomap.octomap.data.empty())
        {
          octomap_monitor_->getOcTreePtr()->lockWrite();
          octomap_monitor_->getOcTreePtr()->clear();
          octomap_monitor_->getOcTreePtr()->unlockWrite();
        }
      }
    }
    triggerSceneUpdateEvent(UPDATE_SCENE);
  }
}

void PlanningSceneMonitor::collisionObjectCallback(const moveit_msgs::msg::CollisionObject::ConstSharedPtr& obj)
{
  if (!scene_)
    return;

  updateFrameTransforms();
  {
    std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
    last_update_time_ = rclcpp::Clock().now();
    if (!scene_->processCollisionObjectMsg(*obj))
      return;
  }
  triggerSceneUpdateEvent(UPDATE_GEOMETRY);
}

void PlanningSceneMonitor::attachObjectCallback(const moveit_msgs::msg::AttachedCollisionObject::ConstSharedPtr& obj)
{
  if (scene_)
  {
    updateFrameTransforms();
    {
      std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
      last_update_time_ = rclcpp::Clock().now();
      scene_->processAttachedCollisionObjectMsg(*obj);
    }
    triggerSceneUpdateEvent(UPDATE_GEOMETRY);
  }
}

void PlanningSceneMonitor::excludeRobotLinksFromOctree()
{
  if (!octomap_monitor_)
    return;

  std::scoped_lock _(shape_handles_lock_);

  includeRobotLinksInOctree();
  const std::vector<const moveit::core::LinkModel*>& links = getRobotModel()->getLinkModelsWithCollisionGeometry();
  auto start = std::chrono::system_clock::now();
  bool warned = false;
  for (const moveit::core::LinkModel* link : links)
  {
    std::vector<shapes::ShapeConstPtr> shapes = link->getShapes();  // copy shared ptrs on purpuse
    for (std::size_t j = 0; j < shapes.size(); ++j)
    {
      // merge mesh vertices up to 0.1 mm apart
      if (shapes[j]->type == shapes::MESH)
      {
        shapes::Mesh* m = static_cast<shapes::Mesh*>(shapes[j]->clone());
        m->mergeVertices(1e-4);
        shapes[j].reset(m);
      }

      occupancy_map_monitor::ShapeHandle h = octomap_monitor_->excludeShape(shapes[j]);
      if (h)
        link_shape_handles_[link].push_back(std::make_pair(h, j));
    }

    if (!warned && ((std::chrono::system_clock::now() - start) > std::chrono::seconds(30)))
    {
      RCLCPP_WARN(LOGGER, "It is likely there are too many vertices in collision geometry");
      warned = true;
    }
  }
}

void PlanningSceneMonitor::includeRobotLinksInOctree()
{
  if (!octomap_monitor_)
    return;

  std::scoped_lock _(shape_handles_lock_);

  for (std::pair<const moveit::core::LinkModel* const,
                 std::vector<std::pair<occupancy_map_monitor::ShapeHandle, std::size_t>>>& link_shape_handle :
       link_shape_handles_)
  {
    for (std::pair<occupancy_map_monitor::ShapeHandle, std::size_t>& it : link_shape_handle.second)
      octomap_monitor_->forgetShape(it.first);
  }
  link_shape_handles_.clear();
}

void PlanningSceneMonitor::includeAttachedBodiesInOctree()
{
  if (!octomap_monitor_)
    return;

  std::scoped_lock _(shape_handles_lock_);

  // clear information about any attached body, without referring to the AttachedBody* ptr (could be invalid)
  for (std::pair<const moveit::core::AttachedBody* const,
                 std::vector<std::pair<occupancy_map_monitor::ShapeHandle, std::size_t>>>& attached_body_shape_handle :
       attached_body_shape_handles_)
  {
    for (std::pair<occupancy_map_monitor::ShapeHandle, std::size_t>& it : attached_body_shape_handle.second)
      octomap_monitor_->forgetShape(it.first);
  }
  attached_body_shape_handles_.clear();
}

void PlanningSceneMonitor::excludeAttachedBodiesFromOctree()
{
  std::scoped_lock _(shape_handles_lock_);

  includeAttachedBodiesInOctree();
  // add attached objects again
  std::vector<const moveit::core::AttachedBody*> ab;
  scene_->getCurrentState().getAttachedBodies(ab);
  for (const moveit::core::AttachedBody* body : ab)
    excludeAttachedBodyFromOctree(body);
}

void PlanningSceneMonitor::includeWorldObjectsInOctree()
{
  if (!octomap_monitor_)
    return;

  std::scoped_lock _(shape_handles_lock_);

  // clear information about any attached object
  for (std::pair<const std::string, std::vector<std::pair<occupancy_map_monitor::ShapeHandle, const Eigen::Isometry3d*>>>&
           collision_body_shape_handle : collision_body_shape_handles_)
  {
    for (std::pair<occupancy_map_monitor::ShapeHandle, const Eigen::Isometry3d*>& it :
         collision_body_shape_handle.second)
      octomap_monitor_->forgetShape(it.first);
  }
  collision_body_shape_handles_.clear();
}

void PlanningSceneMonitor::excludeWorldObjectsFromOctree()
{
  std::scoped_lock _(shape_handles_lock_);

  includeWorldObjectsInOctree();
  for (const std::pair<const std::string, collision_detection::World::ObjectPtr>& it : *scene_->getWorld())
    excludeWorldObjectFromOctree(it.second);
}

void PlanningSceneMonitor::excludeAttachedBodyFromOctree(const moveit::core::AttachedBody* attached_body)
{
  if (!octomap_monitor_)
    return;

  std::scoped_lock _(shape_handles_lock_);
  bool found = false;
  for (std::size_t i = 0; i < attached_body->getShapes().size(); ++i)
  {
    if (attached_body->getShapes()[i]->type == shapes::PLANE || attached_body->getShapes()[i]->type == shapes::OCTREE)
      continue;
    occupancy_map_monitor::ShapeHandle h = octomap_monitor_->excludeShape(attached_body->getShapes()[i]);
    if (h)
    {
      found = true;
      attached_body_shape_handles_[attached_body].push_back(std::make_pair(h, i));
    }
  }
  if (found)
    RCLCPP_DEBUG(LOGGER, "Excluding attached body '%s' from monitored octomap", attached_body->getName().c_str());
}

void PlanningSceneMonitor::includeAttachedBodyInOctree(const moveit::core::AttachedBody* attached_body)
{
  if (!octomap_monitor_)
    return;

  std::scoped_lock _(shape_handles_lock_);

  AttachedBodyShapeHandles::iterator it = attached_body_shape_handles_.find(attached_body);
  if (it != attached_body_shape_handles_.end())
  {
    for (std::pair<occupancy_map_monitor::ShapeHandle, std::size_t>& shape_handle : it->second)
      octomap_monitor_->forgetShape(shape_handle.first);
    RCLCPP_DEBUG(LOGGER, "Including attached body '%s' in monitored octomap", attached_body->getName().c_str());
    attached_body_shape_handles_.erase(it);
  }
}

void PlanningSceneMonitor::excludeWorldObjectFromOctree(const collision_detection::World::ObjectConstPtr& obj)
{
  if (!octomap_monitor_)
    return;

  std::scoped_lock _(shape_handles_lock_);

  bool found = false;
  for (std::size_t i = 0; i < obj->shapes_.size(); ++i)
  {
    if (obj->shapes_[i]->type == shapes::PLANE || obj->shapes_[i]->type == shapes::OCTREE)
      continue;
    occupancy_map_monitor::ShapeHandle h = octomap_monitor_->excludeShape(obj->shapes_[i]);
    if (h)
    {
      collision_body_shape_handles_[obj->id_].push_back(std::make_pair(h, &obj->global_shape_poses_[i]));
      found = true;
    }
  }
  if (found)
    RCLCPP_DEBUG(LOGGER, "Excluding collision object '%s' from monitored octomap", obj->id_.c_str());
}

void PlanningSceneMonitor::includeWorldObjectInOctree(const collision_detection::World::ObjectConstPtr& obj)
{
  if (!octomap_monitor_)
    return;

  std::scoped_lock _(shape_handles_lock_);

  CollisionBodyShapeHandles::iterator it = collision_body_shape_handles_.find(obj->id_);
  if (it != collision_body_shape_handles_.end())
  {
    for (std::pair<occupancy_map_monitor::ShapeHandle, const Eigen::Isometry3d*>& shape_handle : it->second)
      octomap_monitor_->forgetShape(shape_handle.first);
    RCLCPP_DEBUG(LOGGER, "Including collision object '%s' in monitored octomap", obj->id_.c_str());
    collision_body_shape_handles_.erase(it);
  }
}

void PlanningSceneMonitor::currentStateAttachedBodyUpdateCallback(moveit::core::AttachedBody* attached_body,
                                                                  bool just_attached)
{
  if (!octomap_monitor_)
    return;

  if (just_attached)
  {
    excludeAttachedBodyFromOctree(attached_body);
  }
  else
  {
    includeAttachedBodyInOctree(attached_body);
  }
}

void PlanningSceneMonitor::currentWorldObjectUpdateCallback(const collision_detection::World::ObjectConstPtr& obj,
                                                            collision_detection::World::Action action)
{
  if (!octomap_monitor_)
    return;
  if (obj->id_ == planning_scene::PlanningScene::OCTOMAP_NS)
    return;

  if (action & collision_detection::World::CREATE)
  {
    excludeWorldObjectFromOctree(obj);
  }
  else if (action & collision_detection::World::DESTROY)
  {
    includeWorldObjectInOctree(obj);
  }
  else
  {
    excludeWorldObjectFromOctree(obj);
    includeWorldObjectInOctree(obj);
  }
}

bool PlanningSceneMonitor::waitForCurrentRobotState(const rclcpp::Time& t, double wait_time)
{
  if (t.nanoseconds() == 0)
    return false;
  RCLCPP_DEBUG(LOGGER, "sync robot state to: %.3fs", fmod(t.seconds(), 10.));

  if (current_state_monitor_)
  {
    // Wait for next robot update in state monitor.
    bool success = current_state_monitor_->waitForCurrentState(t, wait_time);

    /* As robot updates are passed to the planning scene only in throttled fashion, there might
       be still an update pending. If so, explicitly update the planning scene here.
       If waitForCurrentState failed, we didn't get any new state updates within wait_time. */
    if (success)
    {
      std::unique_lock<std::mutex> lock(state_pending_mutex_);
      if (state_update_pending_)  // enforce state update
      {
        state_update_pending_ = false;
        last_robot_state_update_wall_time_ = std::chrono::system_clock::now();
        lock.unlock();
        updateSceneWithCurrentState();
      }
      return true;
    }

    RCLCPP_WARN(LOGGER, "Failed to fetch current robot state.");
    return false;
  }

  // Sometimes there is no state monitor. In this case state updates are received as part of scene updates only.
  // However, scene updates are only published if the robot actually moves. Hence we need a timeout!
  // As publishing planning scene updates is throttled (2Hz by default), a 1s timeout is a suitable default.
  auto start = node_->get_clock()->now();
  auto timeout = rclcpp::Duration::from_seconds(wait_time);
  std::shared_lock<std::shared_mutex> lock(scene_update_mutex_);
  rclcpp::Time prev_robot_motion_time = last_robot_motion_time_;
  while (last_robot_motion_time_ < t &&  // Wait until the state update actually reaches the scene.
         timeout > rclcpp::Duration(0, 0))
  {
    RCLCPP_DEBUG(LOGGER, "last robot motion: %f ago", (t - last_robot_motion_time_).seconds());
    new_scene_update_condition_.wait_for(lock, std::chrono::nanoseconds(timeout.nanoseconds()));
    timeout = timeout - (node_->get_clock()->now() - start);  // compute remaining wait_time
  }
  bool success = last_robot_motion_time_ >= t;
  // suppress warning if we received an update at all
  if (!success && prev_robot_motion_time != last_robot_motion_time_)
    RCLCPP_WARN(LOGGER, "Maybe failed to update robot state, time diff: %.3fs", (t - last_robot_motion_time_).seconds());

  RCLCPP_DEBUG(LOGGER, "sync done: robot motion: %f scene update: %f", (t - last_robot_motion_time_).seconds(),
               (t - last_update_time_).seconds());
  return success;
}

void PlanningSceneMonitor::lockSceneRead()
{
  scene_update_mutex_.lock_shared();
  if (octomap_monitor_)
    octomap_monitor_->getOcTreePtr()->lockRead();
}

void PlanningSceneMonitor::unlockSceneRead()
{
  if (octomap_monitor_)
    octomap_monitor_->getOcTreePtr()->unlockRead();
  scene_update_mutex_.unlock_shared();
}

void PlanningSceneMonitor::lockSceneWrite()
{
  scene_update_mutex_.lock();
  if (octomap_monitor_)
    octomap_monitor_->getOcTreePtr()->lockWrite();
}

void PlanningSceneMonitor::unlockSceneWrite()
{
  if (octomap_monitor_)
    octomap_monitor_->getOcTreePtr()->unlockWrite();
  scene_update_mutex_.unlock();
}

void PlanningSceneMonitor::startSceneMonitor(const std::string& scene_topic)
{
  stopSceneMonitor();

  RCLCPP_INFO(LOGGER, "Starting planning scene monitor");
  // listen for planning scene updates; these messages include transforms, so no need for filters
  if (!scene_topic.empty())
  {
    planning_scene_subscriber_ = pnode_->create_subscription<moveit_msgs::msg::PlanningScene>(
        scene_topic, 100, [this](const moveit_msgs::msg::PlanningScene::ConstSharedPtr& scene) {
          return newPlanningSceneCallback(scene);
        });
    RCLCPP_INFO(LOGGER, "Listening to '%s'", planning_scene_subscriber_->get_topic_name());
  }
}

void PlanningSceneMonitor::stopSceneMonitor()
{
  if (planning_scene_subscriber_)
  {
    RCLCPP_INFO(LOGGER, "Stopping planning scene monitor");
    planning_scene_subscriber_.reset();
  }
}

bool PlanningSceneMonitor::getShapeTransformCache(const std::string& target_frame, const rclcpp::Time& target_time,
                                                  occupancy_map_monitor::ShapeTransformCache& cache) const
{
  try
  {
    std::scoped_lock _(shape_handles_lock_);

    for (const std::pair<const moveit::core::LinkModel* const,
                         std::vector<std::pair<occupancy_map_monitor::ShapeHandle, std::size_t>>>& link_shape_handle :
         link_shape_handles_)
    {
      if (tf_buffer_->canTransform(target_frame, link_shape_handle.first->getName(), target_time,
                                   shape_transform_cache_lookup_wait_time_))
      {
        Eigen::Isometry3d ttr = tf2::transformToEigen(
            tf_buffer_->lookupTransform(target_frame, link_shape_handle.first->getName(), target_time));
        for (std::size_t j = 0; j < link_shape_handle.second.size(); ++j)
        {
          cache[link_shape_handle.second[j].first] =
              ttr * link_shape_handle.first->getCollisionOriginTransforms()[link_shape_handle.second[j].second];
        }
      }
    }
    for (const std::pair<const moveit::core::AttachedBody* const,
                         std::vector<std::pair<occupancy_map_monitor::ShapeHandle, std::size_t>>>&
             attached_body_shape_handle : attached_body_shape_handles_)
    {
      if (tf_buffer_->canTransform(target_frame, attached_body_shape_handle.first->getAttachedLinkName(), target_time,
                                   shape_transform_cache_lookup_wait_time_))
      {
        Eigen::Isometry3d transform = tf2::transformToEigen(tf_buffer_->lookupTransform(
            target_frame, attached_body_shape_handle.first->getAttachedLinkName(), target_time));
        for (std::size_t k = 0; k < attached_body_shape_handle.second.size(); ++k)
        {
          cache[attached_body_shape_handle.second[k].first] =
              transform *
              attached_body_shape_handle.first->getShapePosesInLinkFrame()[attached_body_shape_handle.second[k].second];
        }
      }
    }
    {
      if (tf_buffer_->canTransform(target_frame, scene_->getPlanningFrame(), target_time,
                                   shape_transform_cache_lookup_wait_time_))
      {
        Eigen::Isometry3d transform =
            tf2::transformToEigen(tf_buffer_->lookupTransform(target_frame, scene_->getPlanningFrame(), target_time));
        for (const std::pair<const std::string,
                             std::vector<std::pair<occupancy_map_monitor::ShapeHandle, const Eigen::Isometry3d*>>>&
                 collision_body_shape_handle : collision_body_shape_handles_)
        {
          for (const std::pair<occupancy_map_monitor::ShapeHandle, const Eigen::Isometry3d*>& it :
               collision_body_shape_handle.second)
            cache[it.first] = transform * (*it.second);
        }
      }
    }
  }
  catch (tf2::TransformException& ex)
  {
    rclcpp::Clock steady_clock = rclcpp::Clock();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
    RCLCPP_ERROR_THROTTLE(LOGGER, steady_clock, 1000, "Transform error: %s", ex.what());
#pragma GCC diagnostic pop
    return false;
  }
  return true;
}

void PlanningSceneMonitor::startWorldGeometryMonitor(const std::string& collision_objects_topic,
                                                     const std::string& planning_scene_world_topic,
                                                     const bool load_octomap_monitor)
{
  stopWorldGeometryMonitor();
  RCLCPP_INFO(LOGGER, "Starting world geometry update monitor for collision objects, attached objects, octomap "
                      "updates.");

  // Listen to the /collision_objects topic to detect requests to add/remove/update collision objects to/from the world
  if (!collision_objects_topic.empty())
  {
    collision_object_subscriber_ = pnode_->create_subscription<moveit_msgs::msg::CollisionObject>(
        collision_objects_topic, 1024,
        [this](const moveit_msgs::msg::CollisionObject::ConstSharedPtr& obj) { return collisionObjectCallback(obj); });
    RCLCPP_INFO(LOGGER, "Listening to '%s'", collision_objects_topic.c_str());
  }

  if (!planning_scene_world_topic.empty())
  {
    planning_scene_world_subscriber_ = pnode_->create_subscription<moveit_msgs::msg::PlanningSceneWorld>(
        planning_scene_world_topic, 1, [this](const moveit_msgs::msg::PlanningSceneWorld::ConstSharedPtr& world) {
          return newPlanningSceneWorldCallback(world);
        });
    RCLCPP_INFO(LOGGER, "Listening to '%s' for planning scene world geometry", planning_scene_world_topic.c_str());
  }

  // Ocotomap monitor is optional
  if (load_octomap_monitor)
  {
    if (!octomap_monitor_)
    {
      octomap_monitor_ =
          std::make_unique<occupancy_map_monitor::OccupancyMapMonitor>(node_, tf_buffer_, scene_->getPlanningFrame());
      excludeRobotLinksFromOctree();
      excludeAttachedBodiesFromOctree();
      excludeWorldObjectsFromOctree();

      octomap_monitor_->setTransformCacheCallback([this](const std::string& frame, const rclcpp::Time& stamp,
                                                         occupancy_map_monitor::ShapeTransformCache& cache) {
        return getShapeTransformCache(frame, stamp, cache);
      });
      octomap_monitor_->setUpdateCallback([this] { octomapUpdateCallback(); });
    }
    octomap_monitor_->startMonitor();
  }
}

void PlanningSceneMonitor::stopWorldGeometryMonitor()
{
  if (collision_object_subscriber_)
  {
    RCLCPP_INFO(LOGGER, "Stopping world geometry monitor");
    collision_object_subscriber_.reset();
  }
  else if (planning_scene_world_subscriber_)
  {
    RCLCPP_INFO(LOGGER, "Stopping world geometry monitor");
    planning_scene_world_subscriber_.reset();
  }
  if (octomap_monitor_)
    octomap_monitor_->stopMonitor();
}

void PlanningSceneMonitor::startStateMonitor(const std::string& joint_states_topic,
                                             const std::string& attached_objects_topic)
{
  stopStateMonitor();
  if (scene_)
  {
    if (!current_state_monitor_)
    {
      current_state_monitor_ =
          std::make_shared<CurrentStateMonitor>(pnode_, getRobotModel(), tf_buffer_, use_sim_time_);
    }
    current_state_monitor_->addUpdateCallback(
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state) { return onStateUpdate(joint_state); });
    current_state_monitor_->startStateMonitor(joint_states_topic);

    {
      std::unique_lock<std::mutex> lock(state_pending_mutex_);
      if (dt_state_update_.count() > 0)
      {
        // ROS original: state_update_timer_.start();
        // TODO: re-enable WallTimer start()
        state_update_timer_ =
            pnode_->create_wall_timer(dt_state_update_, [this]() { return stateUpdateTimerCallback(); });
      }
    }

    if (!attached_objects_topic.empty())
    {
      // using regular message filter as there's no header
      attached_collision_object_subscriber_ = pnode_->create_subscription<moveit_msgs::msg::AttachedCollisionObject>(
          attached_objects_topic, 1024, [this](const moveit_msgs::msg::AttachedCollisionObject::ConstSharedPtr& obj) {
            return attachObjectCallback(obj);
          });
      RCLCPP_INFO(LOGGER, "Listening to '%s' for attached collision objects",
                  attached_collision_object_subscriber_->get_topic_name());
    }
  }
  else
    RCLCPP_ERROR(LOGGER, "Cannot monitor robot state because planning scene is not configured");
}

void PlanningSceneMonitor::stopStateMonitor()
{
  if (current_state_monitor_)
    current_state_monitor_->stopStateMonitor();
  if (attached_collision_object_subscriber_)
    attached_collision_object_subscriber_.reset();

  // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
  if (state_update_timer_)
    state_update_timer_->cancel();
  {
    std::unique_lock<std::mutex> lock(state_pending_mutex_);
    state_update_pending_ = false;
  }
}

void PlanningSceneMonitor::onStateUpdate(const sensor_msgs::msg::JointState::ConstSharedPtr& /*joint_state */)
{
  const std::chrono::system_clock::time_point& n = std::chrono::system_clock::now();
  std::chrono::duration<double> dt = n - last_robot_state_update_wall_time_;

  bool update = false;
  {
    std::unique_lock<std::mutex> lock(state_pending_mutex_);

    if (dt.count() < dt_state_update_.count())
    {
      state_update_pending_ = true;
    }
    else
    {
      state_update_pending_ = false;
      last_robot_state_update_wall_time_ = n;
      update = true;
    }
  }
  // run the state update with state_pending_mutex_ unlocked
  if (update)
    updateSceneWithCurrentState();
}

void PlanningSceneMonitor::stateUpdateTimerCallback()
{
  if (state_update_pending_)
  {
    bool update = false;

    std::chrono::system_clock::time_point n = std::chrono::system_clock::now();
    std::chrono::duration<double> dt = n - last_robot_state_update_wall_time_;

    {
      // lock for access to dt_state_update_ and state_update_pending_
      std::unique_lock<std::mutex> lock(state_pending_mutex_);
      if (state_update_pending_ && dt.count() >= dt_state_update_.count())
      {
        state_update_pending_ = false;
        last_robot_state_update_wall_time_ = std::chrono::system_clock::now();
        auto sec = std::chrono::duration<double>(last_robot_state_update_wall_time_.time_since_epoch()).count();
        update = true;
        RCLCPP_DEBUG(LOGGER, "performPendingStateUpdate: %f", fmod(sec, 10));
      }
    }

    // run the state update with state_pending_mutex_ unlocked
    if (update)
    {
      updateSceneWithCurrentState();
      RCLCPP_DEBUG(LOGGER, "performPendingStateUpdate done");
    }
  }
}

void PlanningSceneMonitor::octomapUpdateCallback()
{
  if (!octomap_monitor_)
    return;

  updateFrameTransforms();
  {
    std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
    last_update_time_ = rclcpp::Clock().now();
    octomap_monitor_->getOcTreePtr()->lockRead();
    try
    {
      scene_->processOctomapPtr(octomap_monitor_->getOcTreePtr(), Eigen::Isometry3d::Identity());
      octomap_monitor_->getOcTreePtr()->unlockRead();
    }
    catch (...)
    {
      octomap_monitor_->getOcTreePtr()->unlockRead();  // unlock and rethrow
      throw;
    }
  }
  triggerSceneUpdateEvent(UPDATE_GEOMETRY);
}

void PlanningSceneMonitor::setStateUpdateFrequency(double hz)
{
  bool update = false;
  if (hz > std::numeric_limits<double>::epsilon())
  {
    std::unique_lock<std::mutex> lock(state_pending_mutex_);
    dt_state_update_ = std::chrono::duration<double>(1.0 / hz);
    // ROS original: state_update_timer_.start();
    // TODO: re-enable WallTimer start()
    state_update_timer_ = pnode_->create_wall_timer(dt_state_update_, [this]() { return stateUpdateTimerCallback(); });
  }
  else
  {
    // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
    // ROS original: state_update_timer_.stop();
    // TODO: re-enable WallTimer stop()
    if (state_update_timer_)
      state_update_timer_->cancel();
    std::unique_lock<std::mutex> lock(state_pending_mutex_);
    dt_state_update_ = std::chrono::duration<double>(0.0);
    if (state_update_pending_)
      update = true;
  }
  RCLCPP_INFO(LOGGER, "Updating internal planning scene state at most every %lf seconds", dt_state_update_.count());

  if (update)
    updateSceneWithCurrentState();
}

void PlanningSceneMonitor::updateSceneWithCurrentState()
{
  rclcpp::Time time = node_->now();
  rclcpp::Clock steady_clock = rclcpp::Clock(RCL_STEADY_TIME);
  if (current_state_monitor_)
  {
    std::vector<std::string> missing;
    if (!current_state_monitor_->haveCompleteState(missing) &&
        (time - current_state_monitor_->getMonitorStartTime()).seconds() > 1.0)
    {
      std::string missing_str = boost::algorithm::join(missing, ", ");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
      RCLCPP_WARN_THROTTLE(LOGGER, steady_clock, 1000, "The complete state of the robot is not yet known.  Missing %s",
                           missing_str.c_str());
#pragma GCC diagnostic pop
    }

    {
      std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
      last_update_time_ = last_robot_motion_time_ = current_state_monitor_->getCurrentStateTime();
      RCLCPP_DEBUG(LOGGER, "robot state update %f", fmod(last_robot_motion_time_.seconds(), 10.));
      current_state_monitor_->setToCurrentState(scene_->getCurrentStateNonConst());
      scene_->getCurrentStateNonConst().update();  // compute all transforms
    }
    triggerSceneUpdateEvent(UPDATE_STATE);
  }
  else
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
    RCLCPP_ERROR_THROTTLE(LOGGER, steady_clock, 1000,
                          "State monitor is not active. Unable to set the planning scene state");
#pragma GCC diagnostic pop
  }
}

void PlanningSceneMonitor::addUpdateCallback(const std::function<void(SceneUpdateType)>& fn)
{
  std::scoped_lock lock(update_lock_);
  if (fn)
    update_callbacks_.push_back(fn);
}

void PlanningSceneMonitor::clearUpdateCallbacks()
{
  std::scoped_lock lock(update_lock_);
  update_callbacks_.clear();
}

void PlanningSceneMonitor::setPlanningScenePublishingFrequency(double hz)
{
  publish_planning_scene_frequency_ = hz;
  RCLCPP_DEBUG(LOGGER, "Maximum frequency for publishing a planning scene is now %lf Hz",
               publish_planning_scene_frequency_);
}

void PlanningSceneMonitor::getUpdatedFrameTransforms(std::vector<geometry_msgs::msg::TransformStamped>& transforms)
{
  const std::string& target = getRobotModel()->getModelFrame();

  std::vector<std::string> all_frame_names;
  tf_buffer_->_getFrameStrings(all_frame_names);
  for (const std::string& all_frame_name : all_frame_names)
  {
    if (all_frame_name == target || getRobotModel()->hasLinkModel(all_frame_name))
      continue;

    geometry_msgs::msg::TransformStamped f;
    try
    {
      f = tf_buffer_->lookupTransform(target, all_frame_name, tf2::TimePointZero);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(LOGGER, "Unable to transform object from frame '%s' to planning frame'%s' (%s)",
                  all_frame_name.c_str(), target.c_str(), ex.what());
      continue;
    }
    f.header.frame_id = all_frame_name;
    f.child_frame_id = target;
    transforms.push_back(f);
  }
}

void PlanningSceneMonitor::updateFrameTransforms()
{
  if (scene_)
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    getUpdatedFrameTransforms(transforms);
    {
      std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
      scene_->getTransformsNonConst().setTransforms(transforms);
      last_update_time_ = rclcpp::Clock().now();
    }
    triggerSceneUpdateEvent(UPDATE_TRANSFORMS);
  }
}

void PlanningSceneMonitor::publishDebugInformation(bool flag)
{
  if (octomap_monitor_)
    octomap_monitor_->publishDebugInformation(flag);
}

void PlanningSceneMonitor::configureCollisionMatrix(const planning_scene::PlanningScenePtr& scene)
{
  if (!scene || robot_description_.empty())
    return;
  // TODO: Uncomment when XmlRpc is refactored
  // collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();

  // read overriding values from the param server

  // first we do default collision operations
  if (!node_->has_parameter(robot_description_ + "_planning.default_collision_operations"))
  {
    RCLCPP_DEBUG(LOGGER, "No additional default collision operations specified");
  }
  else
  {
    RCLCPP_DEBUG(LOGGER, "Reading additional default collision operations");

    // TODO: codebase wide refactoring for XmlRpc
    // XmlRpc::XmlRpcValue coll_ops;
    // node_->get_parameter(robot_description_ + "_planning/default_collision_operations", coll_ops);

    // if (coll_ops.getType() != XmlRpc::XmlRpcValue::TypeArray)
    // {
    //   RCLCPP_WARN(LOGGER, "default_collision_operations is not an array");
    //   return;
    // }

    // if (coll_ops.size() == 0)
    // {
    //   RCLCPP_WARN(LOGGER, "No collision operations in default collision operations");
    //   return;
    // }

    // for (int i = 0; i < coll_ops.size(); ++i)  // NOLINT(modernize-loop-convert)
    // {
    //   if (!coll_ops[i].hasMember("object1") || !coll_ops[i].hasMember("object2") ||
    //   !coll_ops[i].hasMember("operation"))
    //   {
    //     RCLCPP_WARN(LOGGER, "All collision operations must have two objects and an operation");
    //     continue;
    //   }
    //   acm.setEntry(std::string(coll_ops[i]["object1"]), std::string(coll_ops[i]["object2"]),
    //                std::string(coll_ops[i]["operation"]) == "disable");
    // }
  }
}

void PlanningSceneMonitor::configureDefaultPadding()
{
  if (robot_description_.empty())
  {
    default_robot_padd_ = 0.0;
    default_robot_scale_ = 1.0;
    default_object_padd_ = 0.0;
    default_attached_padd_ = 0.0;
    return;
  }

  // Ensure no leading slash creates a bad param server address
  static const std::string ROBOT_DESCRIPTION =
      (robot_description_[0] == '.') ? robot_description_.substr(1) : robot_description_;

  node_->get_parameter_or(ROBOT_DESCRIPTION + "_planning.default_robot_padding", default_robot_padd_, 0.0);
  node_->get_parameter_or(ROBOT_DESCRIPTION + "_planning.default_robot_scale", default_robot_scale_, 1.0);
  node_->get_parameter_or(ROBOT_DESCRIPTION + "_planning.default_object_padding", default_object_padd_, 0.0);
  node_->get_parameter_or(ROBOT_DESCRIPTION + "_planning.default_attached_padding", default_attached_padd_, 0.0);
  default_robot_link_padd_ = std::map<std::string, double>();
  default_robot_link_scale_ = std::map<std::string, double>();
  // TODO: enable parameter type support to std::map
  // node_->get_parameter_or(robot_description + "_planning/default_robot_link_padding", default_robot_link_padd_,
  //           std::map<std::string, double>());
  // node_->get_parameter_or(robot_description + "_planning/default_robot_link_scale", default_robot_link_scale_,
  //           std::map<std::string, double>());

  RCLCPP_DEBUG_STREAM(LOGGER, "Loaded " << default_robot_link_padd_.size() << " default link paddings");
  RCLCPP_DEBUG_STREAM(LOGGER, "Loaded " << default_robot_link_scale_.size() << " default link scales");
}
}  // namespace planning_scene_monitor
