/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Dave Coleman */

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <moveit/rviz_plugin_render_tools/trajectory_visualization.hpp>

#include <moveit/rviz_plugin_render_tools/planning_link_updater.hpp>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.hpp>
#include <rviz_default_plugins/robot/robot.hpp>
#include <moveit/utils/logger.hpp>
#include <moveit/trajectory_processing/trajectory_tools.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>
#include <rviz_common/window_manager_interface.hpp>

#include <rclcpp/qos.hpp>

#include <string>

using namespace std::placeholders;

namespace moveit_rviz_plugin
{

TrajectoryVisualization::TrajectoryVisualization(rviz_common::properties::Property* widget,
                                                 rviz_common::Display* display)
  : animating_path_(false)
  , drop_displaying_trajectory_(false)
  , current_state_(-1)
  , display_(display)
  , widget_(widget)
  , trajectory_slider_panel_(nullptr)
  , trajectory_slider_dock_panel_(nullptr)
  , logger_(moveit::getLogger("moveit.ros.trajectory_visualization"))
{
  trajectory_topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Trajectory Topic", "/display_planned_path",
      rosidl_generator_traits::data_type<moveit_msgs::msg::DisplayTrajectory>(),
      "The topic on which the moveit_msgs::msg::DisplayTrajectory messages are received", widget,
      SLOT(changedTrajectoryTopic()), this);

  display_path_visual_enabled_property_ =
      new rviz_common::properties::BoolProperty("Show Robot Visual", true,
                                                "Indicates whether the geometry of the robot as defined for "
                                                "visualisation purposes should be displayed",
                                                widget, SLOT(changedDisplayPathVisualEnabled()), this);

  display_path_collision_enabled_property_ =
      new rviz_common::properties::BoolProperty("Show Robot Collision", false,
                                                "Indicates whether the geometry of the robot as defined "
                                                "for collision detection purposes should be displayed",
                                                widget, SLOT(changedDisplayPathCollisionEnabled()), this);

  robot_path_alpha_property_ = new rviz_common::properties::FloatProperty(
      "Robot Alpha", 0.5f, "Specifies the alpha for the robot links", widget, SLOT(changedRobotPathAlpha()), this);
  robot_path_alpha_property_->setMin(0.0);
  robot_path_alpha_property_->setMax(1.0);

  state_display_time_property_ =
      new rviz_common::properties::EditableEnumProperty("State Display Time", "3x",
                                                        "Replay speed of trajectory. Either as factor of its time"
                                                        "parameterization or as constant time (s) per waypoint.",
                                                        widget, SLOT(changedStateDisplayTime()), this);
  state_display_time_property_->addOptionStd("3x");
  state_display_time_property_->addOptionStd("1x");
  state_display_time_property_->addOptionStd("0.5x");
  state_display_time_property_->addOptionStd("0.05s");
  state_display_time_property_->addOptionStd("0.1s");
  state_display_time_property_->addOptionStd("0.5s");

  use_sim_time_property_ = new rviz_common::properties::BoolProperty(
      "Use Sim Time", false, "Indicates whether simulation time or wall-time is used for state display timing.", widget,
      nullptr, this);

  loop_display_property_ = new rviz_common::properties::BoolProperty("Loop Animation", false,
                                                                     "Indicates whether the last received path "
                                                                     "is to be animated in a loop",
                                                                     widget, SLOT(changedLoopDisplay()), this);

  trail_display_property_ = new rviz_common::properties::BoolProperty("Show Trail", false, "Show a path trail", widget,
                                                                      SLOT(changedShowTrail()), this);

  trail_step_size_property_ = new rviz_common::properties::IntProperty("Trail Step Size", 1,
                                                                       "Specifies the step size of the samples "
                                                                       "shown in the trajectory trail.",
                                                                       widget, SLOT(changedTrailStepSize()), this);
  trail_step_size_property_->setMin(1);

  interrupt_display_property_ = new rviz_common::properties::BoolProperty(
      "Interrupt Display", false,
      "Immediately show newly planned trajectory, interrupting the currently displayed one.", widget);

  robot_color_property_ = new rviz_common::properties::ColorProperty(
      "Robot Color", QColor(150, 50, 150), "The color of the animated robot", widget, SLOT(changedRobotColor()), this);

  enable_robot_color_property_ = new rviz_common::properties::BoolProperty(
      "Color Enabled", false, "Specifies whether robot coloring is enabled", widget, SLOT(enabledRobotColor()), this);
}

TrajectoryVisualization::~TrajectoryVisualization()
{
  clearTrajectoryTrail();
  trajectory_message_to_display_.reset();
  displaying_trajectory_message_.reset();

  display_path_robot_.reset();
  if (trajectory_slider_dock_panel_)
    delete trajectory_slider_dock_panel_;
}

void TrajectoryVisualization::onInitialize(const rclcpp::Node::SharedPtr& node, Ogre::SceneNode* scene_node,
                                           rviz_common::DisplayContext* context)
{
  // Save pointers for later use
  scene_node_ = scene_node;
  context_ = context;
  node_ = node;

  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction)
  {
    RVIZ_COMMON_LOG_WARNING("Unable to lock weak_ptr from DisplayContext in TrajectoryVisualization constructor");
    return;
  }
  trajectory_topic_property_->initialize(ros_node_abstraction);

  // Load trajectory robot
  display_path_robot_ = std::make_shared<RobotStateVisualization>(scene_node_, context_, "Planned Path", widget_);
  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_robot_->setVisible(false);

  rviz_common::WindowManagerInterface* window_context = context_->getWindowManager();
  if (window_context)
  {
    trajectory_slider_panel_ = new TrajectoryPanel(window_context->getParentWindow());
    trajectory_slider_dock_panel_ =
        window_context->addPane(display_->getName() + " - Trajectory Slider", trajectory_slider_panel_);
    trajectory_slider_dock_panel_->setIcon(display_->getIcon());
    connect(trajectory_slider_dock_panel_, SIGNAL(visibilityChanged(bool)), this,
            SLOT(trajectorySliderPanelVisibilityChange(bool)));
    trajectory_slider_panel_->onInitialize();
  }

  trajectory_topic_sub_ = node_->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      trajectory_topic_property_->getStdString(), rclcpp::ServicesQoS(),
      [this](const moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr& msg) { return incomingDisplayTrajectory(msg); });
}

void TrajectoryVisualization::setName(const QString& name)
{
  if (trajectory_slider_dock_panel_)
    trajectory_slider_dock_panel_->setWindowTitle(name + " - Slider");
}

void TrajectoryVisualization::onRobotModelLoaded(const moveit::core::RobotModelConstPtr& robot_model)
{
  robot_model_ = robot_model;

  // Error check
  if (!robot_model_)
  {
    RCLCPP_ERROR_STREAM(logger_, "No robot model found");
    return;
  }

  // Load robot state
  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state_->setToDefaultValues();

  // Load rviz robot
  display_path_robot_->load(*robot_model_->getURDF());
  enabledRobotColor();  // force-refresh to account for saved display configuration
  // perform post-poned subscription to trajectory topic
  // Check if topic name is empty
  if (trajectory_topic_sub_->get_topic_name())
    changedTrajectoryTopic();
}

void TrajectoryVisualization::reset()
{
  clearTrajectoryTrail();
  trajectory_message_to_display_.reset();
  displaying_trajectory_message_.reset();
  animating_path_ = false;

  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_robot_->setVisible(false);
}

void TrajectoryVisualization::clearTrajectoryTrail()
{
  trajectory_trail_.clear();
}

void TrajectoryVisualization::changedLoopDisplay()
{
  display_path_robot_->setVisible(display_->isEnabled() && displaying_trajectory_message_ && animating_path_);
  if (loop_display_property_->getBool() && trajectory_slider_panel_)
    trajectory_slider_panel_->pauseButton(false);
}

void TrajectoryVisualization::changedShowTrail()
{
  clearTrajectoryTrail();

  if (!trail_display_property_->getBool())
    return;
  robot_trajectory::RobotTrajectoryPtr t = trajectory_message_to_display_;
  if (!t)
    t = displaying_trajectory_message_;
  if (!t)
    return;

  int stepsize = trail_step_size_property_->getInt();
  // always include last trajectory point
  trajectory_trail_.resize(
      static_cast<int>(std::ceil((t->getWayPointCount() + stepsize - 1) / static_cast<float>(stepsize))));
  for (std::size_t i = 0; i < trajectory_trail_.size(); ++i)
  {
    int waypoint_i = std::min(i * stepsize, t->getWayPointCount() - 1);  // limit to last trajectory point
    auto r =
        std::make_unique<RobotStateVisualization>(scene_node_, context_, "Trail Robot " + std::to_string(i), nullptr);
    r->load(*robot_model_->getURDF());
    r->setVisualVisible(display_path_visual_enabled_property_->getBool());
    r->setCollisionVisible(display_path_collision_enabled_property_->getBool());
    r->setAlpha(robot_path_alpha_property_->getFloat());
    r->update(t->getWayPointPtr(waypoint_i), default_attached_object_color_);
    if (enable_robot_color_property_->getBool())
      setRobotColor(&(r->getRobot()), robot_color_property_->getColor());
    r->setVisible(display_->isEnabled() && (!animating_path_ || waypoint_i <= current_state_));
    r->updateAttachedObjectColors(default_attached_object_color_);
    trajectory_trail_[i] = std::move(r);
  }
}

void TrajectoryVisualization::changedTrailStepSize()
{
  if (trail_display_property_->getBool())
    changedShowTrail();
}

void TrajectoryVisualization::changedRobotPathAlpha()
{
  display_path_robot_->setAlpha(robot_path_alpha_property_->getFloat());
  for (const RobotStateVisualizationUniquePtr& r : trajectory_trail_)
    r->setAlpha(robot_path_alpha_property_->getFloat());
}

void TrajectoryVisualization::changedTrajectoryTopic()
{
  // post-pone subscription if robot_state_ is not yet defined, i.e. onRobotModelLoaded() not yet called
  if (!trajectory_topic_property_->getStdString().empty() && robot_state_)
  {
    trajectory_topic_sub_ = node_->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
        trajectory_topic_property_->getStdString(), rclcpp::ServicesQoS(),
        [this](const moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr& msg) {
          return incomingDisplayTrajectory(msg);
        });
  }
}

void TrajectoryVisualization::changedDisplayPathVisualEnabled()
{
  if (display_->isEnabled())
  {
    display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
    display_path_robot_->setVisible(display_->isEnabled() && displaying_trajectory_message_ && animating_path_);
    for (const RobotStateVisualizationUniquePtr& r : trajectory_trail_)
      r->setVisualVisible(display_path_visual_enabled_property_->getBool());
  }
}

void TrajectoryVisualization::changedStateDisplayTime()
{
}

void TrajectoryVisualization::changedDisplayPathCollisionEnabled()
{
  if (display_->isEnabled())
  {
    display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
    display_path_robot_->setVisible(display_->isEnabled() && displaying_trajectory_message_ && animating_path_);
    for (const RobotStateVisualizationUniquePtr& r : trajectory_trail_)
      r->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  }
}

void TrajectoryVisualization::onEnable()
{
  changedRobotPathAlpha();  // set alpha property

  display_path_robot_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_robot_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_robot_->setVisible(displaying_trajectory_message_ && animating_path_);
  for (const RobotStateVisualizationUniquePtr& r : trajectory_trail_)
  {
    r->setVisualVisible(display_path_visual_enabled_property_->getBool());
    r->setCollisionVisible(display_path_collision_enabled_property_->getBool());
    r->setVisible(true);
  }

  changedTrajectoryTopic();  // load topic at startup if default used
}

void TrajectoryVisualization::onDisable()
{
  display_path_robot_->setVisible(false);
  for (const RobotStateVisualizationUniquePtr& r : trajectory_trail_)
    r->setVisible(false);
  displaying_trajectory_message_.reset();
  animating_path_ = false;
  if (trajectory_slider_panel_)
    trajectory_slider_panel_->onDisable();
}

void TrajectoryVisualization::interruptCurrentDisplay()
{
  // update() starts a new trajectory as soon as it is available
  // interrupting may cause the newly received trajectory to interrupt
  // hence, only interrupt when current_state_ already advanced past first
  if (current_state_ > 0)
    animating_path_ = false;
}

double TrajectoryVisualization::getStateDisplayTime()
{
  constexpr char default_time_string[] = "3x";
  constexpr double default_time_value = -3.0;

  std::string tm = state_display_time_property_->getStdString();
  boost::trim(tm);

  double type;

  if (tm.back() == 'x')
  {
    type = -1.0f;
  }
  else if (tm.back() == 's')
  {
    type = 1.0f;
  }
  else
  {
    state_display_time_property_->setStdString(default_time_string);
    return default_time_value;
  }

  tm.resize(tm.size() - 1);
  boost::trim_right(tm);

  double value;
  try
  {
    value = std::stof(tm);
  }
  catch (const std::invalid_argument& ex)
  {
    state_display_time_property_->setStdString(default_time_string);
    return default_time_value;
  }

  if (value <= 0)
  {
    state_display_time_property_->setStdString(default_time_string);
    return default_time_value;
  }

  return type * value;
}

void TrajectoryVisualization::dropTrajectory()
{
  drop_displaying_trajectory_ = true;
}

void TrajectoryVisualization::update(double wall_dt, double sim_dt)
{
  if (drop_displaying_trajectory_)
  {
    animating_path_ = false;
    displaying_trajectory_message_.reset();
    trajectory_slider_panel_->update(0);
    drop_displaying_trajectory_ = false;
  }
  if (!animating_path_)
  {  // finished last animation?
    std::scoped_lock lock(update_trajectory_message_);

    // new trajectory available to display?
    if (trajectory_message_to_display_ && !trajectory_message_to_display_->empty())
    {
      animating_path_ = true;
      displaying_trajectory_message_ = trajectory_message_to_display_;
      changedShowTrail();
      if (trajectory_slider_panel_)
        trajectory_slider_panel_->update(trajectory_message_to_display_->getWayPointCount());
    }
    else if (displaying_trajectory_message_)
    {
      if (loop_display_property_->getBool())
      {  // do loop? -> start over too
        animating_path_ = true;
      }
      else if (trajectory_slider_panel_ && trajectory_slider_panel_->isVisible())
      {
        if (static_cast<unsigned int>(trajectory_slider_panel_->getSliderPosition()) >=
            displaying_trajectory_message_->getWayPointCount() - 1)
        {
          return;  // nothing more to do
        }
        else
        {
          animating_path_ = true;
        }
      }
    }
    trajectory_message_to_display_.reset();

    if (animating_path_)
    {
      // restart animation
      current_state_ = -1;
    }
  }

  if (animating_path_)
  {
    int previous_state = current_state_;
    int waypoint_count = displaying_trajectory_message_->getWayPointCount();
    if (use_sim_time_property_->getBool())
    {
      current_state_time_ += sim_dt;
    }
    else
    {
      current_state_time_ += wall_dt;
    }
    double tm = getStateDisplayTime();

    if (trajectory_slider_panel_ && trajectory_slider_panel_->isVisible() && trajectory_slider_panel_->isPaused())
    {
      current_state_ = trajectory_slider_panel_->getSliderPosition();
      current_state_time_ = displaying_trajectory_message_->getWayPointDurationFromPrevious(current_state_);
    }
    else if (current_state_ < 0)
    {  // special case indicating restart of animation
      current_state_ = 0;
      current_state_time_ = 0.0;
    }
    else if (tm < 0.0)
    {
      // using realtime factors: skip to next waypoint based on elapsed display time
      const double rt_factor = -tm;  // negative tm is the intended realtime factor
      while (current_state_ < waypoint_count &&
             (tm = displaying_trajectory_message_->getWayPointDurationFromPrevious(current_state_ + 1) / rt_factor) <
                 current_state_time_)
      {
        current_state_time_ -= tm;
        // if we are stuck in the while loop we should move the robot along the path to keep up
        if (tm < current_state_time_)
          display_path_robot_->update(displaying_trajectory_message_->getWayPointPtr(current_state_));
        ++current_state_;
      }
    }
    else if (current_state_time_ > tm)
    {  // fixed display time per state
      current_state_time_ = 0.0;
      ++current_state_;
    }

    if (current_state_ == previous_state)
      return;

    if (current_state_ < waypoint_count)
    {
      if (trajectory_slider_panel_)
        trajectory_slider_panel_->setSliderPosition(current_state_);
      display_path_robot_->update(displaying_trajectory_message_->getWayPointPtr(current_state_));
      for (std::size_t i = 0; i < trajectory_trail_.size(); ++i)
      {
        trajectory_trail_[i]->setVisible(
            std::min(waypoint_count - 1, static_cast<int>(i) * trail_step_size_property_->getInt()) <= current_state_);
      }
    }
    else
    {
      animating_path_ = false;  // animation finished
      if (trajectory_slider_panel_)
      {  // make sure we move the slider to the end
         // so the user can re-play
        trajectory_slider_panel_->setSliderPosition(waypoint_count);
      }
      display_path_robot_->update(displaying_trajectory_message_->getWayPointPtr(waypoint_count - 1));
      display_path_robot_->setVisible(loop_display_property_->getBool());
      if (!loop_display_property_->getBool() && trajectory_slider_panel_)
        trajectory_slider_panel_->pauseButton(true);
    }
  }
  // set visibility
  display_path_robot_->setVisible(display_->isEnabled() && displaying_trajectory_message_ &&
                                  (animating_path_ || trail_display_property_->getBool() ||
                                   (trajectory_slider_panel_ && trajectory_slider_panel_->isVisible())));
  display_path_robot_->updateAttachedObjectColors(default_attached_object_color_);
}

void TrajectoryVisualization::incomingDisplayTrajectory(const moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr& msg)
{
  // Error check
  if (!robot_state_ || !robot_model_)
  {
    RCLCPP_ERROR_STREAM(logger_, "No robot state or robot model loaded");
    return;
  }

  if (!msg->model_id.empty() && msg->model_id != robot_model_->getName())
  {
    RCLCPP_WARN(logger_, "Received a trajectory to display for model '%s' but model '%s' was expected",
                msg->model_id.c_str(), robot_model_->getName().c_str());
  }

  trajectory_message_to_display_.reset();

  auto t = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, "");
  try
  {
    for (std::size_t i = 0; i < msg->trajectory.size(); ++i)
    {
      if (t->empty())
      {
        t->setRobotTrajectoryMsg(*robot_state_, msg->trajectory_start, msg->trajectory[i]);
      }
      else
      {
        robot_trajectory::RobotTrajectory tmp(robot_model_, "");
        tmp.setRobotTrajectoryMsg(t->getLastWayPoint(), msg->trajectory[i]);
        t->append(tmp, 0.0);
      }
    }
    display_->setStatus(rviz_common::properties::StatusProperty::Ok, "Trajectory", "");
  }
  catch (const moveit::Exception& e)
  {
    display_->setStatus(rviz_common::properties::StatusProperty::Error, "Trajectory", e.what());
    return;
  }

  if (!t->empty())
  {
    std::scoped_lock lock(update_trajectory_message_);
    trajectory_message_to_display_.swap(t);
    if (interrupt_display_property_->getBool())
      interruptCurrentDisplay();
  }
}

void TrajectoryVisualization::changedRobotColor()
{
  if (enable_robot_color_property_->getBool())
    setRobotColor(&(display_path_robot_->getRobot()), robot_color_property_->getColor());
}

void TrajectoryVisualization::enabledRobotColor()
{
  if (enable_robot_color_property_->getBool())
  {
    setRobotColor(&(display_path_robot_->getRobot()), robot_color_property_->getColor());
  }
  else
  {
    unsetRobotColor(&(display_path_robot_->getRobot()));
  }
}

void TrajectoryVisualization::unsetRobotColor(rviz_default_plugins::robot::Robot* robot)
{
  for (auto& link : robot->getLinks())
    link.second->unsetColor();
}

void TrajectoryVisualization::setDefaultAttachedObjectColor(const QColor& color)
{
  default_attached_object_color_.r = color.redF();
  default_attached_object_color_.g = color.greenF();
  default_attached_object_color_.b = color.blueF();
  default_attached_object_color_.a = color.alphaF();

  if (display_path_robot_)
  {
    display_path_robot_->setDefaultAttachedObjectColor(default_attached_object_color_);
    display_path_robot_->updateAttachedObjectColors(default_attached_object_color_);
  }
  for (const RobotStateVisualizationUniquePtr& r : trajectory_trail_)
    r->updateAttachedObjectColors(default_attached_object_color_);
}

void TrajectoryVisualization::setRobotColor(rviz_default_plugins::robot::Robot* robot, const QColor& color)
{
  for (auto& link : robot->getLinks())
    robot->getLink(link.first)->setColor(color.redF(), color.greenF(), color.blueF());
}

void TrajectoryVisualization::trajectorySliderPanelVisibilityChange(bool enable)
{
  if (!trajectory_slider_panel_)
    return;

  if (enable)
  {
    trajectory_slider_panel_->onEnable();
  }
  else
  {
    trajectory_slider_panel_->onDisable();
  }
}

void TrajectoryVisualization::clearRobotModel()
{
  robot_model_.reset();
  robot_state_.reset();
}

}  // namespace moveit_rviz_plugin
