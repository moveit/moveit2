/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Robert Haschke */

#include <moveit/motion_planning_rviz_plugin/motion_planning_param_widget.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace mpi = moveit::planning_interface;

namespace moveit_rviz_plugin
{

MotionPlanningParamWidget::MotionPlanningParamWidget(QWidget* parent)
  : rviz_common::properties::PropertyTreeWidget(parent)
{
  property_tree_model_ = nullptr;
}

MotionPlanningParamWidget::~MotionPlanningParamWidget()
{
  delete property_tree_model_;
}

void MotionPlanningParamWidget::setMoveGroup(const mpi::MoveGroupInterfacePtr& mg)
{
  move_group_ = mg;
  if (mg)
    setPlannerId(planner_id_);
}

void MotionPlanningParamWidget::setGroupName(const std::string& group_name)
{
  group_name_ = group_name;
  setModel(nullptr);
  if (property_tree_model_)
    delete property_tree_model_;
  property_tree_model_ = nullptr;
}

bool tryLexicalConvert(const QString& value, long& lvalue)
{
  bool ok;
  lvalue = value.toLong(&ok);
  return ok;
}

bool tryLexicalConvert(const QString& value, double& dvalue)
{
  bool ok;
  dvalue = value.toDouble(&ok);
  return ok;
}

rviz_common::properties::Property* MotionPlanningParamWidget::createPropertyTree()
{
  if (planner_id_.empty())
    return nullptr;
  const std::map<std::string, std::string>& params = move_group_->getPlannerParams(planner_id_, group_name_);

  auto root = new rviz_common::properties::Property(QString::fromStdString(planner_id_ + " parameters"));
  for (const std::pair<const std::string, std::string>& param : params)
  {
    const QString key = QString::fromStdString(param.first);
    const QString value = QString::fromStdString(param.second);
    long value_long;
    double value_double;

    if (tryLexicalConvert(value, value_long))
    {
      new rviz_common::properties::IntProperty(key, value_long, QString(), root, SLOT(changedValue()), this);
    }
    else if (tryLexicalConvert(value, value_double))
    {
      new rviz_common::properties::FloatProperty(key, value_double, QString(), root, SLOT(changedValue()), this);
    }
    else
      new rviz_common::properties::StringProperty(key, value, QString(), root, SLOT(changedValue()), this);
  }
  return root;
}

void MotionPlanningParamWidget::changedValue()
{
  if (!move_group_)
    return;
  rviz_common::properties::Property* source = qobject_cast<rviz_common::properties::Property*>(QObject::sender());
  std::map<std::string, std::string> params;
  params[source->getName().toStdString()] = source->getValue().toString().toStdString();
  move_group_->setPlannerParams(planner_id_, group_name_, params);
}

void MotionPlanningParamWidget::setPlannerId(const std::string& planner_id)
{
  planner_id_ = planner_id;
  if (!move_group_)
    return;

  rviz_common::properties::PropertyTreeModel* old_model = property_tree_model_;
  rviz_common::properties::Property* root = createPropertyTree();
  property_tree_model_ = root ? new rviz_common::properties::PropertyTreeModel(root) : nullptr;
  setModel(property_tree_model_);
  if (old_model)
    delete old_model;
}
}  // namespace moveit_rviz_plugin
