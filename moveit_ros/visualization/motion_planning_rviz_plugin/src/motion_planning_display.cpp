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

/* Author: Ioan Sucan, Dave Coleman, Adam Leeper, Sachin Chitta */

#include <moveit/motion_planning_rviz_plugin/motion_planning_display.hpp>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame_joints_widget.hpp>
#include <moveit/robot_interaction/kinematic_options_map.hpp>
#include <moveit/rviz_plugin_render_tools/planning_link_updater.hpp>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.hpp>

#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>
#include <moveit/motion_planning_rviz_plugin/interactive_marker_display.hpp>

#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/display.hpp>
#include <rviz_rendering/objects/movable_text.hpp>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <rviz_rendering/objects/shape.hpp>

#include <moveit/robot_state/conversions.hpp>
#include <moveit/trajectory_processing/trajectory_tools.hpp>

#include <QShortcut>

#include "ui_motion_planning_rviz_plugin_frame.h"
#include <moveit/utils/rclcpp_utils.hpp>
#include <moveit/utils/logger.hpp>

#include <rclcpp/qos.hpp>

namespace moveit_rviz_plugin
{

// ******************************************************************************************
// Base class constructor
// ******************************************************************************************
MotionPlanningDisplay::MotionPlanningDisplay()
  : PlanningSceneDisplay()
  , text_to_display_(nullptr)
  , frame_(nullptr)
  , frame_dock_(nullptr)
  , menu_handler_start_(std::make_shared<interactive_markers::MenuHandler>())
  , menu_handler_goal_(std::make_shared<interactive_markers::MenuHandler>())
  , int_marker_display_(nullptr)
{
  // Category Groups
  plan_category_ = new rviz_common::properties::Property("Planning Request", QVariant(), "", this);
  metrics_category_ = new rviz_common::properties::Property("Planning Metrics", QVariant(), "", this);
  path_category_ = new rviz_common::properties::Property("Planned Path", QVariant(), "", this);

  // Metrics category -----------------------------------------------------------------------------------------
  compute_weight_limit_property_ =
      new rviz_common::properties::BoolProperty("Show Weight Limit", false,
                                                "Shows the weight limit at a particular pose for an end-effector",
                                                metrics_category_, SLOT(changedShowWeightLimit()), this);

  show_manipulability_index_property_ =
      new rviz_common::properties::BoolProperty("Show Manipulability Index", false,
                                                "Shows the manipulability index for an end-effector", metrics_category_,
                                                SLOT(changedShowManipulabilityIndex()), this);

  show_manipulability_property_ =
      new rviz_common::properties::BoolProperty("Show Manipulability", false,
                                                "Shows the manipulability for an end-effector", metrics_category_,
                                                SLOT(changedShowManipulability()), this);

  show_joint_torques_property_ =
      new rviz_common::properties::BoolProperty("Show Joint Torques", false,
                                                "Shows the joint torques for a given configuration and payload",
                                                metrics_category_, SLOT(changedShowJointTorques()), this);

  metrics_set_payload_property_ =
      new rviz_common::properties::FloatProperty("Payload", 1.0f, "Specify the payload at the end effector (kg)",
                                                 metrics_category_, SLOT(changedMetricsSetPayload()), this);
  metrics_set_payload_property_->setMin(0.0);

  metrics_text_height_property_ = new rviz_common::properties::FloatProperty(
      "TextHeight", 0.08f, "Text height", metrics_category_, SLOT(changedMetricsTextHeight()), this);
  metrics_text_height_property_->setMin(0.001);

  // Planning request category -----------------------------------------------------------------------------------------

  planning_group_property_ = new rviz_common::properties::EditableEnumProperty(
      "Planning Group", "", "The name of the group of links to plan for (from the ones defined in the SRDF)",
      plan_category_, SLOT(changedPlanningGroup()), this);
  show_workspace_property_ = new rviz_common::properties::BoolProperty("Show Workspace", false,
                                                                       "Shows the axis-aligned bounding box for "
                                                                       "the workspace allowed for planning",
                                                                       plan_category_, SLOT(changedWorkspace()), this);
  query_start_state_property_ =
      new rviz_common::properties::BoolProperty("Query Start State", false,
                                                "Set a custom start state for the motion planning query",
                                                plan_category_, SLOT(changedQueryStartState()), this);
  query_goal_state_property_ =
      new rviz_common::properties::BoolProperty("Query Goal State", true,
                                                "Shows the goal state for the motion planning query", plan_category_,
                                                SLOT(changedQueryGoalState()), this);
  query_marker_scale_property_ = new rviz_common::properties::FloatProperty(
      "Interactive Marker Size", 0.0f,
      "Specifies scale of the interactive marker overlaid on the robot. 0 is auto scale.", plan_category_,
      SLOT(changedQueryMarkerScale()), this);
  query_marker_scale_property_->setMin(0.0f);

  query_start_color_property_ =
      new rviz_common::properties::ColorProperty("Start State Color", QColor(0, 255, 0),
                                                 "The highlight color for the start state", plan_category_,
                                                 SLOT(changedQueryStartColor()), this);
  query_start_alpha_property_ =
      new rviz_common::properties::FloatProperty("Start State Alpha", 1.0f, "Specifies the alpha for the robot links",
                                                 plan_category_, SLOT(changedQueryStartAlpha()), this);
  query_start_alpha_property_->setMin(0.0);
  query_start_alpha_property_->setMax(1.0);

  query_goal_color_property_ =
      new rviz_common::properties::ColorProperty("Goal State Color", QColor(250, 128, 0),
                                                 "The highlight color for the goal state", plan_category_,
                                                 SLOT(changedQueryGoalColor()), this);

  query_goal_alpha_property_ =
      new rviz_common::properties::FloatProperty("Goal State Alpha", 1.0f, "Specifies the alpha for the robot links",
                                                 plan_category_, SLOT(changedQueryGoalAlpha()), this);
  query_goal_alpha_property_->setMin(0.0);
  query_goal_alpha_property_->setMax(1.0);

  query_colliding_link_color_property_ =
      new rviz_common::properties::ColorProperty("Colliding Link Color", QColor(255, 0, 0),
                                                 "The highlight color for colliding links", plan_category_,
                                                 SLOT(changedQueryCollidingLinkColor()), this);

  query_outside_joint_limits_link_color_property_ = new rviz_common::properties::ColorProperty(
      "Joint Violation Color", QColor(255, 0, 255),
      "The highlight color for child links of joints that are outside bounds", plan_category_,
      SLOT(changedQueryJointViolationColor()), this);

  // Trajectory playback / planned path category ---------------------------------------------
  trajectory_visual_ = std::make_shared<TrajectoryVisualization>(path_category_, this);

  // Start background jobs
  background_process_.setJobUpdateEvent([this](moveit::tools::BackgroundProcessing::JobEvent event,
                                               const std::string& name) { backgroundJobUpdate(event, name); });
}

// ******************************************************************************************
// Deconstructor
// ******************************************************************************************
MotionPlanningDisplay::~MotionPlanningDisplay()
{
  background_process_.clearJobUpdateEvent();
  clearJobs();

  query_robot_start_.reset();
  query_robot_goal_.reset();

  delete text_to_display_;
  delete int_marker_display_;
  delete frame_dock_;
}

void MotionPlanningDisplay::onInitialize()
{
  PlanningSceneDisplay::onInitialize();

  // Planned Path Display
  trajectory_visual_->onInitialize(node_, planning_scene_node_, context_);
  QColor qcolor = attached_body_color_property_->getColor();
  trajectory_visual_->setDefaultAttachedObjectColor(qcolor);

  query_robot_start_ =
      std::make_shared<RobotStateVisualization>(planning_scene_node_, context_, "Planning Request Start", nullptr);
  query_robot_start_->setCollisionVisible(false);
  query_robot_start_->setVisualVisible(true);
  query_robot_start_->setVisible(query_start_state_property_->getBool());
  std_msgs::msg::ColorRGBA color;
  qcolor = query_start_color_property_->getColor();
  color.r = qcolor.redF();
  color.g = qcolor.greenF();
  color.b = qcolor.blueF();
  color.a = 1.0f;
  query_robot_start_->setDefaultAttachedObjectColor(color);

  query_robot_goal_ =
      std::make_shared<RobotStateVisualization>(planning_scene_node_, context_, "Planning Request Goal", nullptr);
  query_robot_goal_->setCollisionVisible(false);
  query_robot_goal_->setVisualVisible(true);
  query_robot_goal_->setVisible(query_goal_state_property_->getBool());
  qcolor = query_goal_color_property_->getColor();
  color.r = qcolor.redF();
  color.g = qcolor.greenF();
  color.b = qcolor.blueF();
  query_robot_goal_->setDefaultAttachedObjectColor(color);

  rviz_common::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new MotionPlanningFrame(this, context_, window_context ? window_context->getParentWindow() : nullptr);

  connect(frame_, SIGNAL(configChanged()), getModel(), SIGNAL(configChanged()));
  resetStatusTextColor();
  addStatusText("Initialized.");

  // immediately switch to next trajectory display after planning
  connect(frame_, SIGNAL(planningFinished()), trajectory_visual_.get(), SLOT(interruptCurrentDisplay()));

  if (window_context)
  {
    frame_dock_ = window_context->addPane(getName(), frame_);
    connect(frame_dock_, SIGNAL(visibilityChanged(bool)), this, SLOT(motionPanelVisibilityChange(bool)));
    frame_dock_->setIcon(getIcon());
  }

  int_marker_display_ = new rviz_default_plugins::displays::InteractiveMarkerDisplay();
  int_marker_display_->initialize(context_);

  text_display_scene_node_ = planning_scene_node_->createChildSceneNode();
  text_to_display_ = new rviz_rendering::MovableText("EMPTY");
  text_to_display_->setTextAlignment(rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);
  text_to_display_->setCharacterHeight(metrics_text_height_property_->getFloat());
  text_to_display_->showOnTop();
  text_to_display_->setVisible(false);
  text_display_for_start_ = false;
  text_display_scene_node_->attachObject(text_to_display_);

  if (context_ && context_->getWindowManager() && context_->getWindowManager()->getParentWindow())
  {
    QShortcut* im_reset_shortcut =
        new QShortcut(QKeySequence("Ctrl+I"), context_->getWindowManager()->getParentWindow());
    connect(im_reset_shortcut, SIGNAL(activated()), this, SLOT(resetInteractiveMarkers()));
  }
}

void MotionPlanningDisplay::motionPanelVisibilityChange(bool enable)
{
  if (enable)
    setEnabled(true);
}

void MotionPlanningDisplay::toggleSelectPlanningGroupSubscription(bool enable)
{
  if (enable)
  {
    planning_group_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/rviz/moveit/select_planning_group", rclcpp::ServicesQoS(),
        [this](const std_msgs::msg::String::ConstSharedPtr& msg) { return selectPlanningGroupCallback(msg); });
  }
  else
  {
    planning_group_sub_.reset();
  }
}

void MotionPlanningDisplay::selectPlanningGroupCallback(const std_msgs::msg::String::ConstSharedPtr& msg)
{
  // synchronize ROS callback with main loop
  addMainLoopJob([this, group = msg->data] { changePlanningGroup(group); });
}

void MotionPlanningDisplay::reset()
{
  text_to_display_->setVisible(false);

  query_robot_start_->clear();
  query_robot_goal_->clear();

  PlanningSceneDisplay::reset();

  // Planned Path Display
  trajectory_visual_->reset();

  bool enabled = isEnabled();
  frame_->disable();
  if (enabled)
  {
    frame_->enable();
    query_robot_start_->setVisible(query_start_state_property_->getBool());
    query_robot_goal_->setVisible(query_goal_state_property_->getBool());
  }
}

void MotionPlanningDisplay::backgroundJobUpdate(moveit::tools::BackgroundProcessing::JobEvent /*unused*/,
                                                const std::string& /*unused*/)
{
  addMainLoopJob([this] { updateBackgroundJobProgressBar(); });
}

void MotionPlanningDisplay::updateBackgroundJobProgressBar()
{
  if (!frame_)
    return;
  QProgressBar* p = frame_->ui_->background_job_progress;
  int n = background_process_.getJobCount();

  if (n == 0)
  {
    p->hide();
    p->setMaximum(0);
    p->setValue(0);
  }
  else
  {
    if (p->maximum() < n)  // increase max
    {
      p->setMaximum(n);
      if (n > 1)  // only show bar if there will be a progress to show
        p->show();
    }
    else  // progress
      p->setValue(p->maximum() - n);
    p->update();
  }
}

void MotionPlanningDisplay::changedShowWeightLimit()
{
  if (text_display_for_start_)
  {
    if (query_start_state_property_->getBool())
      displayMetrics(true);
  }
  else
  {
    if (query_goal_state_property_->getBool())
      displayMetrics(false);
  }
}

void MotionPlanningDisplay::changedShowManipulabilityIndex()
{
  if (text_display_for_start_)
  {
    if (query_start_state_property_->getBool())
      displayMetrics(true);
  }
  else
  {
    if (query_goal_state_property_->getBool())
      displayMetrics(false);
  }
}

void MotionPlanningDisplay::changedShowManipulability()
{
  if (text_display_for_start_)
  {
    if (query_start_state_property_->getBool())
      displayMetrics(true);
  }
  else
  {
    if (query_goal_state_property_->getBool())
      displayMetrics(false);
  }
}

void MotionPlanningDisplay::changedShowJointTorques()
{
  if (text_display_for_start_)
  {
    if (query_start_state_property_->getBool())
      displayMetrics(true);
  }
  else
  {
    if (query_goal_state_property_->getBool())
      displayMetrics(false);
  }
}

void MotionPlanningDisplay::changedMetricsSetPayload()
{
  if (text_display_for_start_)
  {
    if (query_start_state_property_->getBool())
      displayMetrics(true);
  }
  else
  {
    if (query_goal_state_property_->getBool())
      displayMetrics(false);
  }
}

void MotionPlanningDisplay::changedMetricsTextHeight()
{
  text_to_display_->setCharacterHeight(metrics_text_height_property_->getFloat());
}

void MotionPlanningDisplay::displayTable(const std::map<std::string, double>& values, const Ogre::ColourValue& color,
                                         const Ogre::Vector3& pos, const Ogre::Quaternion& orient)
{
  if (values.empty())
  {
    text_to_display_->setVisible(false);
    return;
  }

  // the line we want to render
  std::stringstream ss;
  ss.setf(std::ios_base::fixed);
  ss.precision(2);

  for (const auto& [label, value] : values)
    ss << label << ':' << value << '\n';

  text_to_display_->setCaption(ss.str());
  text_to_display_->setColor(color);
  text_display_scene_node_->setPosition(pos);
  text_display_scene_node_->setOrientation(orient);

  // make sure the node is visible
  text_to_display_->setVisible(true);
}

void MotionPlanningDisplay::renderWorkspaceBox()
{
  if (!frame_ || !show_workspace_property_->getBool())
  {
    if (workspace_box_)
      workspace_box_.reset();
    return;
  }

  if (!workspace_box_)
  {
    workspace_box_ = std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Cube, context_->getSceneManager(),
                                                             planning_scene_node_);
    workspace_box_->setColor(0.0f, 0.0f, 0.6f, 0.3f);
  }

  Ogre::Vector3 center(frame_->ui_->wcenter_x->value(), frame_->ui_->wcenter_y->value(),
                       frame_->ui_->wcenter_z->value());
  Ogre::Vector3 extents(frame_->ui_->wsize_x->value(), frame_->ui_->wsize_y->value(), frame_->ui_->wsize_z->value());
  workspace_box_->setScale(extents);
  workspace_box_->setPosition(center);
}

void MotionPlanningDisplay::computeMetrics(bool start, const std::string& group, double payload)
{
  if (!robot_interaction_)
    return;
  const std::vector<robot_interaction::EndEffectorInteraction>& eef = robot_interaction_->getActiveEndEffectors();
  if (eef.empty())
    return;
  std::scoped_lock slock(update_metrics_lock_);

  moveit::core::RobotStateConstPtr state = start ? getQueryStartState() : getQueryGoalState();
  for (const robot_interaction::EndEffectorInteraction& ee : eef)
  {
    if (ee.parent_group == group)
      computeMetricsInternal(computed_metrics_[std::make_pair(start, group)], ee, *state, payload);
  }
}

void MotionPlanningDisplay::computeMetricsInternal(std::map<std::string, double>& metrics,
                                                   const robot_interaction::EndEffectorInteraction& ee,
                                                   const moveit::core::RobotState& state, double payload)
{
  metrics.clear();
  dynamics_solver::DynamicsSolverPtr ds;
  std::map<std::string, dynamics_solver::DynamicsSolverPtr>::const_iterator it = dynamics_solver_.find(ee.parent_group);
  if (it != dynamics_solver_.end())
    ds = it->second;

  // Max payload
  if (ds)
  {
    double max_payload;
    unsigned int saturated_joint;
    std::vector<double> joint_values;
    state.copyJointGroupPositions(ee.parent_group, joint_values);
    if (ds->getMaxPayload(joint_values, max_payload, saturated_joint))
    {
      metrics["max_payload"] = max_payload;
      metrics["saturated_joint"] = saturated_joint;
    }
    std::vector<double> joint_torques;
    joint_torques.resize(joint_values.size());
    if (ds->getPayloadTorques(joint_values, payload, joint_torques))
    {
      for (std::size_t i = 0; i < joint_torques.size(); ++i)
      {
        std::stringstream stream;
        stream << "torque[" << i << ']';
        metrics[stream.str()] = joint_torques[i];
      }
    }
  }

  if (kinematics_metrics_)
  {
    if (position_only_ik_.find(ee.parent_group) == position_only_ik_.end())
      node_->get_parameter_or(ee.parent_group + ".position_only_ik", position_only_ik_[ee.parent_group], false);

    double manipulability_index, manipulability;
    bool position_ik = position_only_ik_[ee.parent_group];
    if (kinematics_metrics_->getManipulabilityIndex(state, ee.parent_group, manipulability_index, position_ik))
      metrics["manipulability_index"] = manipulability_index;
    if (kinematics_metrics_->getManipulability(state, ee.parent_group, manipulability))
      metrics["manipulability"] = manipulability;
  }
}

namespace
{
inline void copyItemIfExists(const std::map<std::string, double>& source, std::map<std::string, double>& dest,
                             const std::string& key)
{
  std::map<std::string, double>::const_iterator it = source.find(key);
  if (it != source.end())
    dest[key] = it->second;
}
}  // namespace

void MotionPlanningDisplay::displayMetrics(bool start)
{
  if (!robot_interaction_ || !planning_scene_monitor_)
    return;

  static const Ogre::Quaternion ORIENTATION(1.0, 0.0, 0.0, 0.0);
  const std::vector<robot_interaction::EndEffectorInteraction>& eef = robot_interaction_->getActiveEndEffectors();
  if (eef.empty())
    return;

  moveit::core::RobotStateConstPtr state = start ? getQueryStartState() : getQueryGoalState();

  for (const robot_interaction::EndEffectorInteraction& ee : eef)
  {
    Ogre::Vector3 position(0.0, 0.0, 0.0);
    std::map<std::string, double> text_table;
    const std::map<std::string, double>& metrics_table = computed_metrics_[std::make_pair(start, ee.parent_group)];
    if (compute_weight_limit_property_->getBool())
    {
      copyItemIfExists(metrics_table, text_table, "max_payload");
      copyItemIfExists(metrics_table, text_table, "saturated_joint");
    }
    if (show_manipulability_index_property_->getBool())
      copyItemIfExists(metrics_table, text_table, "manipulability_index");
    if (show_manipulability_property_->getBool())
      copyItemIfExists(metrics_table, text_table, "manipulability");
    if (show_joint_torques_property_->getBool())
    {
      std::size_t nj = getRobotModel()->getJointModelGroup(ee.parent_group)->getJointModelNames().size();
      for (size_t j = 0; j < nj; ++j)
      {
        std::stringstream stream;
        stream << "torque[" << j << ']';
        copyItemIfExists(metrics_table, text_table, stream.str());
      }
    }

    const moveit::core::LinkModel* lm = nullptr;
    const moveit::core::JointModelGroup* jmg = getRobotModel()->getJointModelGroup(ee.parent_group);
    if (jmg)
    {
      if (!jmg->getLinkModelNames().empty())
        lm = state->getLinkModel(jmg->getLinkModelNames().back());
    }
    if (lm)
    {
      const Eigen::Vector3d& t = state->getGlobalLinkTransform(lm).translation();
      position[0] = t.x();
      position[1] = t.y();
      position[2] = t.z() + 0.2;  // \todo this should be a param
    }
    if (start)
    {
      displayTable(text_table, query_start_color_property_->getOgreColor(), position, ORIENTATION);
    }
    else
    {
      displayTable(text_table, query_goal_color_property_->getOgreColor(), position, ORIENTATION);
    }
    text_display_for_start_ = start;
  }
}

void MotionPlanningDisplay::drawQueryStartState()
{
  if (!planning_scene_monitor_)
    return;

  if (query_start_state_property_->getBool())
  {
    if (isEnabled())
    {
      moveit::core::RobotStateConstPtr state = getQueryStartState();

      // update link poses
      query_robot_start_->update(state);
      query_robot_start_->setVisible(true);

      // update link colors
      std::vector<std::string> collision_links;
      getPlanningSceneRO()->getCollidingLinks(collision_links, *state);
      status_links_start_.clear();
      for (const std::string& collision_link : collision_links)
        status_links_start_[collision_link] = COLLISION_LINK;
      if (!collision_links.empty())
      {
        collision_detection::CollisionResult::ContactMap pairs;
        getPlanningSceneRO()->getCollidingPairs(pairs, *state);
        setStatusTextColor(query_start_color_property_->getColor());
        addStatusText("Start state colliding links:");
        for (collision_detection::CollisionResult::ContactMap::const_iterator it = pairs.begin(); it != pairs.end();
             ++it)
          addStatusText(it->first.first + " - " + it->first.second);
        addStatusText(".");
      }
      if (!getCurrentPlanningGroup().empty())
      {
        const moveit::core::JointModelGroup* jmg = state->getJointModelGroup(getCurrentPlanningGroup());
        if (jmg)
        {
          std::vector<std::string> outside_bounds;
          const std::vector<const moveit::core::JointModel*>& jmodels = jmg->getActiveJointModels();
          for (const moveit::core::JointModel* jmodel : jmodels)
          {
            if (!state->satisfiesBounds(jmodel, jmodel->getMaximumExtent() * 1e-2))
            {
              outside_bounds.push_back(jmodel->getChildLinkModel()->getName());
              status_links_start_[outside_bounds.back()] = OUTSIDE_BOUNDS_LINK;
            }
          }
          if (!outside_bounds.empty())
          {
            setStatusTextColor(query_start_color_property_->getColor());
            addStatusText("Links descending from joints that are outside bounds in start state:");
            addStatusText(outside_bounds);
          }
        }
      }
      updateLinkColors();
      // update metrics text
      displayMetrics(true);
    }
  }
  else
    query_robot_start_->setVisible(false);
  context_->queueRender();
}

void MotionPlanningDisplay::resetStatusTextColor()
{
  setStatusTextColor(Qt::darkGray);
}

void MotionPlanningDisplay::setStatusTextColor(const QColor& color)
{
  if (frame_)
    frame_->ui_->status_text->setTextColor(color);
}

void MotionPlanningDisplay::addStatusText(const std::string& text)
{
  if (frame_)
    frame_->ui_->status_text->append(QString::fromStdString(text));
}

void MotionPlanningDisplay::addStatusText(const std::vector<std::string>& text)
{
  for (const std::string& it : text)
    addStatusText(it);
}

void MotionPlanningDisplay::recomputeQueryStartStateMetrics()
{
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
    computeMetrics(true, group, metrics_set_payload_property_->getFloat());
}

void MotionPlanningDisplay::recomputeQueryGoalStateMetrics()
{
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
    computeMetrics(false, group, metrics_set_payload_property_->getFloat());
}

void MotionPlanningDisplay::changedQueryStartState()
{
  if (!planning_scene_monitor_)
    return;
  setStatusTextColor(query_start_color_property_->getColor());
  addStatusText("Changed start state");
  drawQueryStartState();
  addBackgroundJob([this] { publishInteractiveMarkers(true); }, "publishInteractiveMarkers");
}

void MotionPlanningDisplay::changedQueryGoalState()
{
  if (!planning_scene_monitor_)
    return;
  setStatusTextColor(query_goal_color_property_->getColor());
  addStatusText("Changed goal state");
  drawQueryGoalState();
  addBackgroundJob([this] { publishInteractiveMarkers(true); }, "publishInteractiveMarkers");
}

void MotionPlanningDisplay::drawQueryGoalState()
{
  if (!planning_scene_monitor_)
    return;
  if (query_goal_state_property_->getBool())
  {
    if (isEnabled())
    {
      moveit::core::RobotStateConstPtr state = getQueryGoalState();

      // update link poses
      query_robot_goal_->update(state);
      query_robot_goal_->setVisible(true);

      // update link colors
      std::vector<std::string> collision_links;
      getPlanningSceneRO()->getCollidingLinks(collision_links, *state);
      status_links_goal_.clear();
      for (const std::string& collision_link : collision_links)
        status_links_goal_[collision_link] = COLLISION_LINK;
      if (!collision_links.empty())
      {
        collision_detection::CollisionResult::ContactMap pairs;
        getPlanningSceneRO()->getCollidingPairs(pairs, *state);
        setStatusTextColor(query_goal_color_property_->getColor());
        addStatusText("Goal state colliding links:");
        for (collision_detection::CollisionResult::ContactMap::const_iterator it = pairs.begin(); it != pairs.end();
             ++it)
          addStatusText(it->first.first + " - " + it->first.second);
        addStatusText(".");
      }

      if (!getCurrentPlanningGroup().empty())
      {
        const moveit::core::JointModelGroup* jmg = state->getJointModelGroup(getCurrentPlanningGroup());
        if (jmg)
        {
          const std::vector<const moveit::core::JointModel*>& jmodels = jmg->getActiveJointModels();
          std::vector<std::string> outside_bounds;
          for (const moveit::core::JointModel* jmodel : jmodels)
          {
            if (!state->satisfiesBounds(jmodel, jmodel->getMaximumExtent() * 1e-2))
            {
              outside_bounds.push_back(jmodel->getChildLinkModel()->getName());
              status_links_goal_[outside_bounds.back()] = OUTSIDE_BOUNDS_LINK;
            }
          }

          if (!outside_bounds.empty())
          {
            setStatusTextColor(query_goal_color_property_->getColor());
            addStatusText("Links descending from joints that are outside bounds in goal state:");
            addStatusText(outside_bounds);
          }
        }
      }
      updateLinkColors();

      // update metrics text
      displayMetrics(false);
    }
  }
  else
    query_robot_goal_->setVisible(false);
  context_->queueRender();
}

void MotionPlanningDisplay::resetInteractiveMarkers()
{
  query_start_state_->clearError();
  query_goal_state_->clearError();
  addBackgroundJob([this] { publishInteractiveMarkers(false); }, "publishInteractiveMarkers");
}

void MotionPlanningDisplay::publishInteractiveMarkers(bool pose_update)
{
  if (robot_interaction_)
  {
    if (pose_update &&
        robot_interaction_->showingMarkers(query_start_state_) == query_start_state_property_->getBool() &&
        robot_interaction_->showingMarkers(query_goal_state_) == query_goal_state_property_->getBool())
    {
      if (query_start_state_property_->getBool())
        robot_interaction_->updateInteractiveMarkers(query_start_state_);
      if (query_goal_state_property_->getBool())
        robot_interaction_->updateInteractiveMarkers(query_goal_state_);
    }
    else
    {
      robot_interaction_->clearInteractiveMarkers();
      if (query_start_state_property_->getBool())
        robot_interaction_->addInteractiveMarkers(query_start_state_, query_marker_scale_property_->getFloat());
      if (query_goal_state_property_->getBool())
        robot_interaction_->addInteractiveMarkers(query_goal_state_, query_marker_scale_property_->getFloat());
      robot_interaction_->publishInteractiveMarkers();
    }
    if (frame_)
    {
      frame_->updateExternalCommunication();
    }
  }
}

void MotionPlanningDisplay::changedQueryStartColor()
{
  std_msgs::msg::ColorRGBA color;
  QColor qcolor = query_start_color_property_->getColor();
  color.r = qcolor.redF();
  color.g = qcolor.greenF();
  color.b = qcolor.blueF();
  color.a = 1.0f;
  query_robot_start_->setDefaultAttachedObjectColor(color);
  changedQueryStartState();
}

void MotionPlanningDisplay::changedQueryStartAlpha()
{
  query_robot_start_->setAlpha(query_start_alpha_property_->getFloat());
  changedQueryStartState();
}

void MotionPlanningDisplay::changedQueryMarkerScale()
{
  if (!planning_scene_monitor_)
    return;

  if (isEnabled())
  {
    // Clear the interactive markers and re-add them:
    publishInteractiveMarkers(false);
  }
}

void MotionPlanningDisplay::changedQueryGoalColor()
{
  std_msgs::msg::ColorRGBA color;
  QColor qcolor = query_goal_color_property_->getColor();
  color.r = qcolor.redF();
  color.g = qcolor.greenF();
  color.b = qcolor.blueF();
  color.a = 1.0f;
  query_robot_goal_->setDefaultAttachedObjectColor(color);
  changedQueryGoalState();
}

void MotionPlanningDisplay::changedQueryGoalAlpha()
{
  query_robot_goal_->setAlpha(query_goal_alpha_property_->getFloat());
  changedQueryGoalState();
}

void MotionPlanningDisplay::changedQueryCollidingLinkColor()
{
  changedQueryStartState();
  changedQueryGoalState();
}

void MotionPlanningDisplay::changedQueryJointViolationColor()
{
  changedQueryStartState();
  changedQueryGoalState();
}

void MotionPlanningDisplay::changedAttachedBodyColor()
{
  PlanningSceneDisplay::changedAttachedBodyColor();
  // forward color to TrajectoryVisualization
  const QColor& color = attached_body_color_property_->getColor();
  trajectory_visual_->setDefaultAttachedObjectColor(color);
}

void MotionPlanningDisplay::scheduleDrawQueryStartState(robot_interaction::InteractionHandler* /*unused*/,
                                                        bool error_state_changed)
{
  if (!planning_scene_monitor_)
    return;
  addBackgroundJob([this, pose_update = !error_state_changed] { publishInteractiveMarkers(pose_update); },
                   "publishInteractiveMarkers");
  updateQueryStartState();
}

void MotionPlanningDisplay::scheduleDrawQueryGoalState(robot_interaction::InteractionHandler* /*unused*/,
                                                       bool error_state_changed)
{
  if (!planning_scene_monitor_)
    return;
  addBackgroundJob([this, pose_update = !error_state_changed] { publishInteractiveMarkers(pose_update); },
                   "publishInteractiveMarkers");
  updateQueryGoalState();
}

void MotionPlanningDisplay::updateQueryStartState()
{
  queryStartStateChanged();
  recomputeQueryStartStateMetrics();
  addMainLoopJob([this] { changedQueryStartState(); });
  context_->queueRender();
}

void MotionPlanningDisplay::updateQueryGoalState()
{
  queryGoalStateChanged();
  recomputeQueryGoalStateMetrics();
  addMainLoopJob([this] { changedQueryGoalState(); });
  context_->queueRender();
}

void MotionPlanningDisplay::rememberPreviousStartState()
{
  *previous_state_ = *query_start_state_->getState();
}

void MotionPlanningDisplay::setQueryStartState(const moveit::core::RobotState& start)
{
  query_start_state_->setState(start);
  updateQueryStartState();
}

void MotionPlanningDisplay::setQueryGoalState(const moveit::core::RobotState& goal)
{
  query_goal_state_->setState(goal);
  updateQueryGoalState();
}

void MotionPlanningDisplay::useApproximateIK(bool flag)
{
  if (robot_interaction_)
  {
    robot_interaction::KinematicOptions o;
    o.options_.return_approximate_solution = flag;
    robot_interaction_->getKinematicOptionsMap()->setOptions(
        robot_interaction::KinematicOptionsMap::DEFAULT, o,
        robot_interaction::KinematicOptions::RETURN_APPROXIMATE_SOLUTION);
  }
}

bool MotionPlanningDisplay::isIKSolutionCollisionFree(moveit::core::RobotState* state,
                                                      const moveit::core::JointModelGroup* group,
                                                      const double* ik_solution) const
{
  if (frame_->ui_->collision_aware_ik->isChecked() && planning_scene_monitor_)
  {
    state->setJointGroupPositions(group, ik_solution);
    state->update();
    bool res = !getPlanningSceneRO()->isStateColliding(*state, group->getName());
    return res;
  }
  else
    return true;
}

void MotionPlanningDisplay::updateLinkColors()
{
  unsetAllColors(&query_robot_start_->getRobot());
  unsetAllColors(&query_robot_goal_->getRobot());
  std::string group = planning_group_property_->getStdString();
  if (!group.empty())
  {
    setGroupColor(&query_robot_start_->getRobot(), group, query_start_color_property_->getColor());
    setGroupColor(&query_robot_goal_->getRobot(), group, query_goal_color_property_->getColor());

    for (std::map<std::string, LinkDisplayStatus>::const_iterator it = status_links_start_.begin();
         it != status_links_start_.end(); ++it)
    {
      if (it->second == COLLISION_LINK)
      {
        setLinkColor(&query_robot_start_->getRobot(), it->first, query_colliding_link_color_property_->getColor());
      }
      else
      {
        setLinkColor(&query_robot_start_->getRobot(), it->first,
                     query_outside_joint_limits_link_color_property_->getColor());
      }
    }

    for (std::map<std::string, LinkDisplayStatus>::const_iterator it = status_links_goal_.begin();
         it != status_links_goal_.end(); ++it)
    {
      if (it->second == COLLISION_LINK)
      {
        setLinkColor(&query_robot_goal_->getRobot(), it->first, query_colliding_link_color_property_->getColor());
      }
      else
      {
        setLinkColor(&query_robot_goal_->getRobot(), it->first,
                     query_outside_joint_limits_link_color_property_->getColor());
      }
    }
  }
}

void MotionPlanningDisplay::changePlanningGroup(const std::string& group)
{
  if (!getRobotModel() || !robot_interaction_)
    return;

  if (getRobotModel()->hasJointModelGroup(group))
  {
    planning_group_property_->setStdString(group);
  }
  else
  {
    RCLCPP_ERROR(moveit::getLogger("moveit.ros.motion_planning_display"), "Group [%s] not found in the robot model.",
                 group.c_str());
  }
}

void MotionPlanningDisplay::changedPlanningGroup()
{
  if (!getRobotModel() || !robot_interaction_)
    return;

  if (!planning_group_property_->getStdString().empty() &&
      !getRobotModel()->hasJointModelGroup(planning_group_property_->getStdString()))
  {
    planning_group_property_->setStdString("");
    return;
  }
  modified_groups_.insert(planning_group_property_->getStdString());

  robot_interaction_->decideActiveComponents(planning_group_property_->getStdString());

  updateQueryStartState();
  updateQueryGoalState();
  updateLinkColors();

  if (frame_)
    frame_->changePlanningGroup();
  addBackgroundJob([this] { publishInteractiveMarkers(false); }, "publishInteractiveMarkers");
}

void MotionPlanningDisplay::changedWorkspace()
{
  renderWorkspaceBox();
}

std::string MotionPlanningDisplay::getCurrentPlanningGroup() const
{
  return planning_group_property_->getStdString();
}

void MotionPlanningDisplay::setQueryStateHelper(bool use_start_state, const std::string& state_name)
{
  moveit::core::RobotState state = use_start_state ? *getQueryStartState() : *getQueryGoalState();

  std::string v = "<" + state_name + ">";

  if (v == "<random>")
  {
    if (const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(getCurrentPlanningGroup()))
      state.setToRandomPositions(jmg);
  }
  else if (v == "<current>")
  {
    const planning_scene_monitor::LockedPlanningSceneRO& ps = getPlanningSceneRO();
    if (ps)
      state = ps->getCurrentState();
  }
  else if (v == "<same as goal>")
  {
    state = *getQueryGoalState();
  }
  else if (v == "<same as start>")
  {
    state = *getQueryStartState();
  }
  else
  {
    // maybe it is a named state
    if (const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(getCurrentPlanningGroup()))
      state.setToDefaultValues(jmg, state_name);
  }

  use_start_state ? setQueryStartState(state) : setQueryGoalState(state);
}

void MotionPlanningDisplay::populateMenuHandler(std::shared_ptr<interactive_markers::MenuHandler>& mh)
{
  typedef interactive_markers::MenuHandler immh;
  std::vector<std::string> state_names;
  state_names.push_back("random");
  state_names.push_back("current");
  state_names.push_back("same as start");
  state_names.push_back("same as goal");

  // hacky way to distinguish between the start and goal handlers...
  bool is_start = (mh.get() == menu_handler_start_.get());

  // Commands for changing the state
  immh::EntryHandle menu_states =
      mh->insert(is_start ? "Set start state to" : "Set goal state to", immh::FeedbackCallback());
  for (const std::string& state_name : state_names)
  {
    // Don't add "same as start" to the start state handler, and vice versa.
    if ((state_name == "same as start" && is_start) || (state_name == "same as goal" && !is_start))
      continue;
    mh->insert(menu_states, state_name,
               [this, is_start, state_name](auto&&) { setQueryStateHelper(is_start, state_name); });
  }

  //  // Group commands, which end up being the same for both interaction handlers
  //  const std::vector<std::string>& group_names = getRobotModel()->getJointModelGroupNames();
  //  immh::EntryHandle menu_groups = mh->insert("Planning Group", immh::FeedbackCallback());
  //  for (int i = 0; i < group_names.size(); ++i)
  //    mh->insert(menu_groups, group_names[i],
  //               [this, &name = group_names[i]] { changePlanningGroup(name); });
}

void MotionPlanningDisplay::clearRobotModel()
{
  // Invalidate all references to the RobotModel ...
  if (frame_)
    frame_->clearRobotModel();
  if (trajectory_visual_)
    trajectory_visual_->clearRobotModel();
  previous_state_.reset();
  query_start_state_.reset();
  query_goal_state_.reset();
  kinematics_metrics_.reset();
  robot_interaction_.reset();
  dynamics_solver_.clear();
  // ... before calling the parent's method, which finally destroys the creating RobotModelLoader.
  PlanningSceneDisplay::clearRobotModel();
}

void MotionPlanningDisplay::onRobotModelLoaded()
{
  PlanningSceneDisplay::onRobotModelLoaded();
  trajectory_visual_->onRobotModelLoaded(getRobotModel());

  robot_interaction_ = std::make_shared<robot_interaction::RobotInteraction>(
      getRobotModel(), node_, rclcpp::names::append(getMoveGroupNS(), "rviz_moveit_motion_planning_display"));
  robot_interaction::KinematicOptions o;
  o.state_validity_callback_ = [this](moveit::core::RobotState* robot_state,
                                      const moveit::core::JointModelGroup* joint_group,
                                      const double* joint_group_variable_values) {
    return isIKSolutionCollisionFree(robot_state, joint_group, joint_group_variable_values);
  };
  robot_interaction_->getKinematicOptionsMap()->setOptions(
      robot_interaction::KinematicOptionsMap::ALL, o, robot_interaction::KinematicOptions::STATE_VALIDITY_CALLBACK);

  int_marker_display_->subProp("Interactive Markers Namespace")
      ->setValue(QString::fromStdString(robot_interaction_->getServerTopic()));
  query_robot_start_->load(*getRobotModel()->getURDF());
  query_robot_goal_->load(*getRobotModel()->getURDF());

  // initialize previous state, start state, and goal state to current state
  previous_state_ = std::make_shared<moveit::core::RobotState>(getPlanningSceneRO()->getCurrentState());
  query_start_state_ = std::make_shared<robot_interaction::InteractionHandler>(
      robot_interaction_, "start", *previous_state_, planning_scene_monitor_->getTFClient());
  query_goal_state_ = std::make_shared<robot_interaction::InteractionHandler>(
      robot_interaction_, "goal", *previous_state_, planning_scene_monitor_->getTFClient());
  query_start_state_->setUpdateCallback(
      [this](robot_interaction::InteractionHandler* handler, bool error_state_changed) {
        scheduleDrawQueryStartState(handler, error_state_changed);
      });
  query_goal_state_->setUpdateCallback([this](robot_interaction::InteractionHandler* handler, bool error_state_changed) {
    scheduleDrawQueryGoalState(handler, error_state_changed);
  });

  // Interactive marker menus
  populateMenuHandler(menu_handler_start_);
  populateMenuHandler(menu_handler_goal_);
  query_start_state_->setMenuHandler(menu_handler_start_);
  query_goal_state_->setMenuHandler(menu_handler_goal_);

  if (!planning_group_property_->getStdString().empty())
  {
    if (!getRobotModel()->hasJointModelGroup(planning_group_property_->getStdString()))
      planning_group_property_->setStdString("");
  }

  const std::vector<std::string>& groups = getRobotModel()->getJointModelGroupNames();
  planning_group_property_->clearOptions();
  for (const std::string& group : groups)
    planning_group_property_->addOptionStd(group);
  planning_group_property_->sortOptions();
  if (!groups.empty() && planning_group_property_->getStdString().empty())
    planning_group_property_->setStdString(groups[0]);

  modified_groups_.clear();
  kinematics_metrics_ = std::make_shared<kinematics_metrics::KinematicsMetrics>(getRobotModel());

  geometry_msgs::msg::Vector3 gravity_vector;
  gravity_vector.x = 0.0;
  gravity_vector.y = 0.0;
  gravity_vector.z = 9.81;

  dynamics_solver_.clear();
  for (const std::string& group : groups)
  {
    if (getRobotModel()->getJointModelGroup(group)->isChain())
    {
      dynamics_solver_[group] =
          std::make_shared<dynamics_solver::DynamicsSolver>(getRobotModel(), group, gravity_vector);
    }
  }

  if (frame_)
    frame_->fillPlanningGroupOptions();
  changedPlanningGroup();
}
void MotionPlanningDisplay::onNewPlanningSceneState()
{
  frame_->onNewPlanningSceneState();
}

void MotionPlanningDisplay::updateStateExceptModified(moveit::core::RobotState& dest,
                                                      const moveit::core::RobotState& src)
{
  moveit::core::RobotState src_copy = src;
  for (const std::string& modified_group : modified_groups_)
  {
    const moveit::core::JointModelGroup* jmg = dest.getJointModelGroup(modified_group);
    if (jmg)
    {
      std::vector<double> values_to_keep;
      dest.copyJointGroupPositions(jmg, values_to_keep);
      src_copy.setJointGroupPositions(jmg, values_to_keep);
    }
  }

  // overwrite the destination state
  dest = src_copy;
}

void MotionPlanningDisplay::updateQueryStates(const moveit::core::RobotState& current_state)
{
  std::string group = planning_group_property_->getStdString();

  if (query_start_state_ && query_start_state_property_->getBool() && !group.empty())
  {
    moveit::core::RobotState start = *getQueryStartState();
    updateStateExceptModified(start, current_state);
    setQueryStartState(start);
  }

  if (query_goal_state_ && query_goal_state_property_->getBool() && !group.empty())
  {
    moveit::core::RobotState goal = *getQueryGoalState();
    updateStateExceptModified(goal, current_state);
    setQueryGoalState(goal);
  }
}

void MotionPlanningDisplay::onSceneMonitorReceivedUpdate(
    planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  PlanningSceneDisplay::onSceneMonitorReceivedUpdate(update_type);
  updateQueryStates(getPlanningSceneRO()->getCurrentState());
  if (frame_)
    frame_->sceneUpdate(update_type);
}

// ******************************************************************************************
// Enable
// ******************************************************************************************
void MotionPlanningDisplay::onEnable()
{
  PlanningSceneDisplay::onEnable();

  // Planned Path Display
  trajectory_visual_->onEnable();

  text_to_display_->setVisible(false);

  query_robot_start_->setVisible(query_start_state_property_->getBool());
  query_robot_goal_->setVisible(query_goal_state_property_->getBool());

  int_marker_display_->setEnabled(true);
  int_marker_display_->setFixedFrame(fixed_frame_);

  frame_->enable();
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void MotionPlanningDisplay::onDisable()
{
  if (robot_interaction_)
    robot_interaction_->clear();
  int_marker_display_->setEnabled(false);

  query_robot_start_->setVisible(false);
  query_robot_goal_->setVisible(false);
  text_to_display_->setVisible(false);

  PlanningSceneDisplay::onDisable();

  // Planned Path Display
  trajectory_visual_->onDisable();

  frame_->disable();
}

// ******************************************************************************************
// Update
// ******************************************************************************************
void MotionPlanningDisplay::update(float wall_dt, float ros_dt)
{
  if (int_marker_display_)
    int_marker_display_->update(wall_dt, ros_dt);
  if (frame_)
    frame_->updateSceneMarkers(wall_dt, ros_dt);

  PlanningSceneDisplay::update(wall_dt, ros_dt);
}

void MotionPlanningDisplay::updateInternal(double wall_dt, double ros_dt)
{
  PlanningSceneDisplay::updateInternal(wall_dt, ros_dt);

  // Planned Path Display
  trajectory_visual_->update(wall_dt, ros_dt);

  renderWorkspaceBox();
}

void MotionPlanningDisplay::load(const rviz_common::Config& config)
{
  PlanningSceneDisplay::load(config);
  if (frame_)
  {
    float d;
    if (config.mapGetFloat("MoveIt_Planning_Time", &d))
      frame_->ui_->planning_time->setValue(d);
    int attempts;
    if (config.mapGetInt("MoveIt_Planning_Attempts", &attempts))
      frame_->ui_->planning_attempts->setValue(attempts);
    if (config.mapGetFloat("Velocity_Scaling_Factor", &d))
      frame_->ui_->velocity_scaling_factor->setValue(d);
    if (config.mapGetFloat("Acceleration_Scaling_Factor", &d))
      frame_->ui_->acceleration_scaling_factor->setValue(d);

    bool b;
    if (config.mapGetBool("MoveIt_Allow_Replanning", &b))
      frame_->ui_->allow_replanning->setChecked(b);
    if (config.mapGetBool("MoveIt_Allow_Sensor_Positioning", &b))
      frame_->ui_->allow_looking->setChecked(b);
    if (config.mapGetBool("MoveIt_Allow_External_Program", &b))
      frame_->ui_->allow_external_program->setChecked(b);
    if (config.mapGetBool("MoveIt_Use_Cartesian_Path", &b))
      frame_->ui_->use_cartesian_path->setChecked(b);
    if (config.mapGetBool("MoveIt_Use_Constraint_Aware_IK", &b))
      frame_->ui_->collision_aware_ik->setChecked(b);
    if (config.mapGetBool("MoveIt_Allow_Approximate_IK", &b))
      frame_->ui_->approximate_ik->setChecked(b);

    rviz_common::Config workspace = config.mapGetChild("MoveIt_Workspace");
    rviz_common::Config ws_center = workspace.mapGetChild("Center");
    float val;
    if (ws_center.mapGetFloat("X", &val))
      frame_->ui_->wcenter_x->setValue(val);
    if (ws_center.mapGetFloat("Y", &val))
      frame_->ui_->wcenter_y->setValue(val);
    if (ws_center.mapGetFloat("Z", &val))
      frame_->ui_->wcenter_z->setValue(val);

    rviz_common::Config ws_size = workspace.mapGetChild("Size");
    if (ws_size.isValid())
    {
      if (ws_size.mapGetFloat("X", &val))
        frame_->ui_->wsize_x->setValue(val);
      if (ws_size.mapGetFloat("Y", &val))
        frame_->ui_->wsize_y->setValue(val);
      if (ws_size.mapGetFloat("Z", &val))
        frame_->ui_->wsize_z->setValue(val);
    }
    else
    {
      double val;
      if (node_->get_parameter("default_workspace_bounds", val))
      {
        frame_->ui_->wsize_x->setValue(val);
        frame_->ui_->wsize_y->setValue(val);
        frame_->ui_->wsize_z->setValue(val);
      }
    }
  }
}

void MotionPlanningDisplay::save(rviz_common::Config config) const
{
  PlanningSceneDisplay::save(config);
  if (frame_)
  {
    // "Options" Section
    config.mapSetValue("MoveIt_Planning_Time", frame_->ui_->planning_time->value());
    config.mapSetValue("MoveIt_Planning_Attempts", frame_->ui_->planning_attempts->value());
    config.mapSetValue("Velocity_Scaling_Factor", frame_->ui_->velocity_scaling_factor->value());
    config.mapSetValue("Acceleration_Scaling_Factor", frame_->ui_->acceleration_scaling_factor->value());

    config.mapSetValue("MoveIt_Allow_Replanning", frame_->ui_->allow_replanning->isChecked());
    config.mapSetValue("MoveIt_Allow_Sensor_Positioning", frame_->ui_->allow_looking->isChecked());
    config.mapSetValue("MoveIt_Allow_External_Program", frame_->ui_->allow_external_program->isChecked());
    config.mapSetValue("MoveIt_Use_Cartesian_Path", frame_->ui_->use_cartesian_path->isChecked());
    config.mapSetValue("MoveIt_Use_Constraint_Aware_IK", frame_->ui_->collision_aware_ik->isChecked());
    config.mapSetValue("MoveIt_Allow_Approximate_IK", frame_->ui_->approximate_ik->isChecked());

    rviz_common::Config workspace = config.mapMakeChild("MoveIt_Workspace");
    rviz_common::Config ws_center = workspace.mapMakeChild("Center");
    ws_center.mapSetValue("X", frame_->ui_->wcenter_x->value());
    ws_center.mapSetValue("Y", frame_->ui_->wcenter_y->value());
    ws_center.mapSetValue("Z", frame_->ui_->wcenter_z->value());
    rviz_common::Config ws_size = workspace.mapMakeChild("Size");
    ws_size.mapSetValue("X", frame_->ui_->wsize_x->value());
    ws_size.mapSetValue("Y", frame_->ui_->wsize_y->value());
    ws_size.mapSetValue("Z", frame_->ui_->wsize_z->value());
  }
}

void MotionPlanningDisplay::fixedFrameChanged()
{
  PlanningSceneDisplay::fixedFrameChanged();
  if (int_marker_display_)
    int_marker_display_->setFixedFrame(fixed_frame_);
  // When the fixed frame changes we need to tell RViz to update the rendered interactive marker display
  if (frame_ && frame_->scene_marker_)
  {
    frame_->scene_marker_->requestPoseUpdate(frame_->scene_marker_->getPosition(),
                                             frame_->scene_marker_->getOrientation());
  }
  changedPlanningGroup();
}

// Pick and place
void MotionPlanningDisplay::clearPlaceLocationsDisplay()
{
  for (std::shared_ptr<rviz_rendering::Shape>& place_location_shape : place_locations_display_)
    place_location_shape.reset();
  place_locations_display_.clear();
}

void MotionPlanningDisplay::visualizePlaceLocations(const std::vector<geometry_msgs::msg::PoseStamped>& place_poses)
{
  clearPlaceLocationsDisplay();
  place_locations_display_.resize(place_poses.size());
  for (std::size_t i = 0; i < place_poses.size(); ++i)
  {
    place_locations_display_[i] =
        std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, context_->getSceneManager());
    place_locations_display_[i]->setColor(1.0f, 0.0f, 0.0f, 0.3f);
    Ogre::Vector3 center(place_poses[i].pose.position.x, place_poses[i].pose.position.y, place_poses[i].pose.position.z);
    Ogre::Vector3 extents(0.02, 0.02, 0.02);
    place_locations_display_[i]->setScale(extents);
    place_locations_display_[i]->setPosition(center);
  }
}

}  // namespace moveit_rviz_plugin
