/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Larry Lu, Inc.
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
 *   * Neither the name of Larry Lu nor the names of its contributors
 *     may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
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

/* Author: Larry Lu */

#include <moveit/robot_interface/robot_interface.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/robot_state/conversions.h>
#include <rcl_yaml_param_parser/parser.h>

namespace moveit
{
namespace planning_interface
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_interface");

class RobotInterface::RobotInterfaceImpl
{
public:
  RobotInterfaceImpl(const rclcpp::Node::SharedPtr& node, const std::string& robot_description = "robot_description")
    : node_(node)
  {
    robot_model_ = planning_interface::getSharedRobotModel(node_, robot_description);
    if (!robot_model_)
      throw std::runtime_error("RobotInterface: invalid robot model");
    current_state_monitor_ = planning_interface::getSharedStateMonitor(node_, robot_model_, getSharedTF());
  }

  const char* getRobotName() const
  {
    return robot_model_->getName().c_str();
  }

  const std::vector<std::string> getActiveJointNames() const
  {
    return robot_model_->getActiveJointModelNames();
  }

  const std::vector<std::string> getGroupActiveJointNames(const std::string& group) const
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
      return jmg->getActiveJointModelNames();
    else
      return {};
  }

  const std::vector<std::string> getJointNames() const
  {
    return robot_model_->getJointModelNames();
  }

  const std::vector<std::string> getGroupJointNames(const std::string& group) const
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
      return jmg->getJointModelNames();
    else
      return {};
  }

  const std::vector<std::string> getGroupJointTips(const std::string& group) const
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
    {
      std::vector<std::string> tips;
      jmg->getEndEffectorTips(tips);
      return tips;
    }
    else
      return {};
  }

  const std::vector<std::string> getLinkNames() const
  {
    return robot_model_->getLinkModelNames();
  }

  const std::vector<std::string> getGroupLinkNames(const std::string& group) const
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
      return jmg->getLinkModelNames();
    else
      return {};
  }

  const std::vector<std::string> getGroupNames() const
  {
    return robot_model_->getJointModelGroupNames();
  }

  const std::vector<std::vector<double> > getJointLimits(const std::string& name) const
  {
    std::vector<std::vector<double> > result;
    const moveit::core::JointModel* jm = robot_model_->getJointModel(name);
    if (jm)
    {
      const std::vector<moveit_msgs::msg::JointLimits>& lim = jm->getVariableBoundsMsg();
      for (const moveit_msgs::msg::JointLimits& joint_limit : lim)
      {
        std::vector<double> l;
        l.push_back(joint_limit.min_position);
        l.push_back(joint_limit.max_position);
        result.push_back(l);
      }
    }
    return result;
  }

  const char* getPlanningFrame() const
  {
    return robot_model_->getModelFrame().c_str();
  }

  const std::vector<std::string> getDefaultStateNames(const std::string& group)
  {
    std::vector<std::string> l;
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
    {
      for (auto& known_state : jmg->getDefaultStateNames())
      {
        l.push_back(known_state);
      }
    }
    return l;
  }

  const std::map<std::string, double> getJointValues(const std::string& group, const std::string& named_state)
  {
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (!jmg)
      return std::map<std::string, double>();
    std::map<std::string, double> values;
    jmg->getVariableDefaultPositions(named_state, values);
    return values;
  }

  const char* getRobotRootLink() const
  {
    return robot_model_->getRootLinkName().c_str();
  }

  bool hasGroup(const std::string& group) const
  {
    return robot_model_->hasJointModelGroup(group);
  }

  const std::pair<std::string, std::string> getEndEffectorParentGroup(const std::string& group)
  {
    // name of the group that is parent to this end-effector group;
    // Second: the link this in the parent group that this group attaches to
    std::pair<std::string, std::string> parent_group;
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (jmg)
      parent_group = jmg->getEndEffectorParentGroup();
    return parent_group;
  }

  bool ensureCurrentState(double wait = 200.0)
  {
    if (!current_state_monitor_)
    {
      RCLCPP_ERROR(LOGGER, "Unable to get current robot state");
      return false;
    }

    // if needed, start the monitor and wait up to 1 second for a full robot state
    if (!current_state_monitor_->isActive())
    {
      current_state_monitor_->startStateMonitor();
      if (!current_state_monitor_->waitForCompleteState(wait))
        RCLCPP_WARN(LOGGER, "Joint values for monitored state are requested but the full state is not known");
    }
    return true;
  }

  const std::vector<double> getLinkPose(const std::string& name)
  {
    std::vector<double> l;
    if (!ensureCurrentState())
      return l;
    moveit::core::RobotStatePtr state = current_state_monitor_->getCurrentState();
    const moveit::core::LinkModel* lm = state->getLinkModel(name);
    if (lm)
    {
      // getGlobalLinkTransform() returns a valid isometry by contract
      const Eigen::Isometry3d& t = state->getGlobalLinkTransform(lm);
      std::vector<double> v(7);
      v[0] = t.translation().x();
      v[1] = t.translation().y();
      v[2] = t.translation().z();
      Eigen::Quaterniond q(t.linear());
      v[3] = q.x();
      v[4] = q.y();
      v[5] = q.z();
      v[6] = q.w();
      l = v;
    }
    return l;
  }

  std::vector<double> getCurrentJointValues(const std::string& name)
  {
    std::vector<double> l;
    if (!ensureCurrentState())
      return l;
    moveit::core::RobotStatePtr state = current_state_monitor_->getCurrentState();
    const moveit::core::JointModel* jm = state->getJointModel(name);
    if (jm)
    {
      const double* pos = state->getJointPositions(jm);
      const unsigned int sz = jm->getVariableCount();
      for (unsigned int i = 0; i < sz; ++i)
        l.push_back(pos[i]);
    }

    return l;
  }

  const std::map<std::string, double> getCurrentVariableValues()
  {
    std::map<std::string, double> d;

    if (!ensureCurrentState())
      return d;

    const std::map<std::string, double>& vars = current_state_monitor_->getCurrentStateValues();
    for (const std::pair<const std::string, double>& var : vars)
      d[var.first] = var.second;

    return d;
  }

  moveit_msgs::msg::RobotState getCurrentState()
  {
    moveit_msgs::msg::RobotState msg;
    if (ensureCurrentState())
    {
      moveit::core::RobotStatePtr s = current_state_monitor_->getCurrentState();
      moveit::core::robotStateToRobotStateMsg(*s, msg);
    }
    return msg;
  }

  visualization_msgs::msg::MarkerArray getRobotMarkers()
  {
    visualization_msgs::msg::MarkerArray msg;
    if (ensureCurrentState())
    {
      moveit::core::RobotStatePtr s = current_state_monitor_->getCurrentState();
      s->getRobotMarkers(msg, s->getRobotModel()->getLinkModelNames());
    }
    return msg;
  }

  visualization_msgs::msg::MarkerArray getRobotMarkers(const std::vector<std::string>& link_names)
  {
    visualization_msgs::msg::MarkerArray msg;
    if (ensureCurrentState())
    {
      moveit::core::RobotStatePtr s = current_state_monitor_->getCurrentState();
      s->getRobotMarkers(msg, link_names);
    }
    return msg;
  }

  visualization_msgs::msg::MarkerArray getRobotMarkers(const std::map<std::string, double>& values,
                                                       const std::vector<std::string>& link_names)
  {
    moveit::core::RobotStatePtr state;
    if (ensureCurrentState())
    {
      state = current_state_monitor_->getCurrentState();
    }
    else
    {
      state.reset(new moveit::core::RobotState(robot_model_));
    }

    std::vector<std::string> k;
    std::vector<double> v;
    for (auto& it : values)
    {
      k.push_back(it.first);
      v.push_back(it.second);
    }

    int l = k.size();

    sensor_msgs::msg::JointState joint_state;
    joint_state.name.resize(l);
    joint_state.position.resize(l);
    for (int i = 0; i < l; ++i)
    {
      joint_state.name[i] = k[i];
      joint_state.position[i] = v[i];
    }
    state->setVariableValues(joint_state);
    visualization_msgs::msg::MarkerArray msg;
    state->getRobotMarkers(msg, link_names);

    return msg;
  }

  visualization_msgs::msg::MarkerArray getRobotMarkers(const std::map<std::string, double>& values)
  {
    std::vector<std::string> links = robot_model_->getLinkModelNames();
    return getRobotMarkers(values, links);
  }

  visualization_msgs::msg::MarkerArray getRobotMarkers(const moveit_msgs::msg::RobotState state_msg)
  {
    moveit::core::RobotState state(robot_model_);
    moveit::core::robotStateMsgToRobotState(state_msg, state);

    visualization_msgs::msg::MarkerArray msg;
    state.getRobotMarkers(msg, state.getRobotModel()->getLinkModelNames());

    return msg;
  }

  visualization_msgs::msg::MarkerArray getRobotMarkersGroup(const std::string& group)
  {
    visualization_msgs::msg::MarkerArray msg;
    if (ensureCurrentState())
    {
      moveit::core::RobotStatePtr s = current_state_monitor_->getCurrentState();
      const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
      if (jmg)
      {
        s->getRobotMarkers(msg, jmg->getLinkModelNames());
      }
    }
    return msg;
  }

  visualization_msgs::msg::MarkerArray getRobotMarkersGroup(const std::string& group,
                                                            const std::map<std::string, double>& values)
  {
    visualization_msgs::msg::MarkerArray msg;
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group);
    if (!jmg)
      return msg;
    std::vector<std::string> links = jmg->getLinkModelNames();
    return getRobotMarkers(values, links);
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
};

RobotInterface::RobotInterface()
{
}

RobotInterface::RobotInterface(const rclcpp::Node::SharedPtr& node, const std::string& robot_description)
{
  if (!rclcpp::ok())
    throw std::runtime_error("ROS does not seem to be running");
  impl_ = new RobotInterfaceImpl(node, robot_description);
}

RobotInterface::~RobotInterface()
{
  delete impl_;
}

const char* RobotInterface::getRobotName() const
{
  return impl_->getRobotName();
}

const std::vector<std::string> RobotInterface::getActiveJointNames() const
{
  return impl_->getActiveJointNames();
}

const std::vector<std::string> RobotInterface::getGroupActiveJointNames(const std::string& group) const
{
  return impl_->getGroupActiveJointNames(group);
}

const std::vector<std::string> RobotInterface::getJointNames() const
{
  return impl_->getJointNames();
}

const std::vector<std::string> RobotInterface::getGroupJointNames(const std::string& group) const
{
  return impl_->getGroupJointNames(group);
}

const std::vector<std::string> RobotInterface::getGroupJointTips(const std::string& group) const
{
  return impl_->getGroupJointTips(group);
}

const std::vector<std::string> RobotInterface::getLinkNames() const
{
  return impl_->getLinkNames();
}

const std::vector<std::string> RobotInterface::getGroupLinkNames(const std::string& group) const
{
  return impl_->getGroupLinkNames(group);
}

const std::vector<std::string> RobotInterface::getGroupNames() const
{
  return impl_->getGroupNames();
}

const std::vector<std::vector<double> > RobotInterface::getJointLimits(const std::string& name) const
{
  return impl_->getJointLimits(name);
}

const char* RobotInterface::getPlanningFrame() const
{
  return impl_->getPlanningFrame();
}

const std::vector<std::string> RobotInterface::getDefaultStateNames(const std::string& group)
{
  return impl_->getDefaultStateNames(group);
}

const std::map<std::string, double> RobotInterface::getJointValues(const std::string& group,
                                                                   const std::string& named_state)
{
  return impl_->getJointValues(group, named_state);
}

const char* RobotInterface::getRobotRootLink() const
{
  return impl_->getRobotRootLink();
}

bool RobotInterface::hasGroup(const std::string& group) const
{
  return impl_->hasGroup(group);
}

const std::pair<std::string, std::string> RobotInterface::getEndEffectorParentGroup(const std::string& group)
{
  return impl_->getEndEffectorParentGroup(group);
}

const std::vector<double> RobotInterface::getLinkPose(const std::string& name)
{
  return impl_->getLinkPose(name);
}

std::vector<double> RobotInterface::getCurrentJointValues(const std::string& name)
{
  return impl_->getCurrentJointValues(name);
}

const std::map<std::string, double> RobotInterface::getCurrentVariableValues()
{
  return impl_->getCurrentVariableValues();
}

moveit_msgs::msg::RobotState RobotInterface::getCurrentState()
{
  return impl_->getCurrentState();
}

visualization_msgs::msg::MarkerArray RobotInterface::getRobotMarkers()
{
  return impl_->getRobotMarkers();
}

visualization_msgs::msg::MarkerArray RobotInterface::getRobotMarkers(const std::vector<std::string>& link_names)
{
  return impl_->getRobotMarkers(link_names);
}

visualization_msgs::msg::MarkerArray RobotInterface::getRobotMarkers(const std::map<std::string, double>& values,
                                                                     const std::vector<std::string>& link_names)
{
  return impl_->getRobotMarkers(values, link_names);
}

visualization_msgs::msg::MarkerArray RobotInterface::getRobotMarkers(const std::map<std::string, double>& values)
{
  return impl_->getRobotMarkers(values);
}

visualization_msgs::msg::MarkerArray RobotInterface::getRobotMarkers(const moveit_msgs::msg::RobotState state_msg)
{
  return impl_->getRobotMarkers(state_msg);
}

visualization_msgs::msg::MarkerArray RobotInterface::getRobotMarkersGroup(const std::string& group)
{
  return impl_->getRobotMarkersGroup(group);
}

visualization_msgs::msg::MarkerArray RobotInterface::getRobotMarkersGroup(const std::string& group,
                                                                          const std::map<std::string, double>& values)
{
  return impl_->getRobotMarkersGroup(group, values);
}

}  // namespace planning_interface
}  // namespace moveit
