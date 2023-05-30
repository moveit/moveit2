/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, CITEC, Bielefeld University
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

#include <moveit_setup_srdf_plugins/collision_matrix_model.hpp>
#include <boost/assign.hpp>
#include <QVector>
#include <QBrush>
#include <QColor>
#include <QPalette>
#include <QApplication>
#include <QItemSelection>
#include <unordered_map>

namespace moveit_setup
{
namespace srdf_setup
{
/// Mapping of reasons for disabling a link pair to a background color
static const std::unordered_map<DisabledReason, QVariant> LONG_REASONS_TO_BRUSH =
    boost::assign::map_list_of  // clang-format off
    ( NEVER, QBrush(QColor("lightgreen")) )
    ( DEFAULT, QBrush(QColor("lightpink")) )
    ( ADJACENT, QBrush(QColor("powderblue")) )
    ( ALWAYS, QBrush(QColor("tomato")) )
    ( USER, QBrush(QColor("yellow")) )
    ( NOT_DISABLED, QBrush());  // clang-format on

CollisionMatrixModel::CollisionMatrixModel(DefaultCollisions& default_collisions, const std::vector<std::string>& names, QObject* parent)
  : QAbstractTableModel(parent), default_collisions_(default_collisions), std_names_(names)
{
  int idx = 0;
  for (std::vector<std::string>::const_iterator it = names.begin(), end = names.end(); it != end; ++it, ++idx)
  {
    visual_to_index_ << idx;
    q_names_ << QString::fromStdString(*it);
  }
}

int CollisionMatrixModel::rowCount(const QModelIndex& /*parent*/) const
{
  return visual_to_index_.size();
}

int CollisionMatrixModel::columnCount(const QModelIndex& /*parent*/) const
{
  return visual_to_index_.size();
}

QVariant CollisionMatrixModel::data(const QModelIndex& index, int role) const
{
  static QBrush default_collision_brush(QColor("lightpink").darker(110));

  if (index.isValid() && index.row() == index.column())
  {
    switch (role)
    {
      case Qt::BackgroundRole:
        return QApplication::palette().window();
      default:
        return QVariant();
    }
  }

  int r = visual_to_index_[index.row()], c = visual_to_index_[index.column()];
  const std::string reason = default_collisions_.getCollisionDisablingReason(std_names_[r], std_names_[c]);

  switch (role)
  {
    case Qt::CheckStateRole:
      return (reason.empty() || reason == DefaultCollisions::COLLISION_DISABLING_REASON_ENABLED) ? Qt::Unchecked : Qt::Checked;
    case Qt::ToolTipRole:
      return !reason.empty() ? QString::fromStdString(reason) : QString();
    case Qt::BackgroundRole:
      if (reason.empty() || reason == DefaultCollisions::COLLISION_DISABLING_REASON_ENABLED)
        return QVariant();
      else if (reason == DefaultCollisions::COLLISION_DISABLING_REASON_DISABLED)
        return default_collision_brush;
      else
        return LONG_REASONS_TO_BRUSH.at(disabledReasonFromString(reason));
  }
  return QVariant();
}

bool CollisionMatrixModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
  if (role != Qt::CheckStateRole)
    return false;

  bool new_value = (value.toInt() == Qt::Checked);

  bool changed = default_collisions_.setDefault(std_names_[visual_to_index_[index.row()]], std_names_[visual_to_index_[index.column()]], new_value);
  if (changed)
  {
    QModelIndex mirror = this->index(index.column(), index.row());
    Q_EMIT dataChanged(index, index);
    Q_EMIT dataChanged(mirror, mirror);
  }
  return changed;
}

void CollisionMatrixModel::setEnabled(const QItemSelection& selection, bool value)
{
  // perform changes without signalling
  QItemSelection changes;
  blockSignals(true);
  for (const auto& range : selection)
  {
    setEnabled(range.indexes(), value);

    const QModelIndex& top_left = range.topLeft();
    const QModelIndex& bottom_right = range.bottomRight();
    changes.select(top_left, bottom_right);
    changes.select(createIndex(top_left.column(), top_left.row()),
                   createIndex(bottom_right.column(), bottom_right.row()));
  }
  blockSignals(false);

  // emit changes
  for (const auto& range : changes)
    Q_EMIT dataChanged(range.topLeft(), range.bottomRight());
}

void CollisionMatrixModel::setEnabled(const QModelIndexList& indexes, bool value)
{
  for (const auto idx : indexes)
    setData(idx, value ? Qt::Checked : Qt::Unchecked, Qt::CheckStateRole);
}

void CollisionMatrixModel::setFilterRegExp(const QString& filter)
{
  beginResetModel();
  QRegExp regexp(filter);
  visual_to_index_.clear();
  for (int idx = 0, end = q_names_.size(); idx != end; ++idx)
  {
    if (q_names_[idx].contains(regexp))
      visual_to_index_ << idx;
  }
  endResetModel();
}

QVariant CollisionMatrixModel::headerData(int section, Qt::Orientation /*orientation*/, int role) const
{
  if (role == Qt::DisplayRole)
    return q_names_[visual_to_index_[section]];
  return QVariant();
}

Qt::ItemFlags CollisionMatrixModel::flags(const QModelIndex& index) const
{
  if (!index.isValid())
    return Qt::NoItemFlags;

  Qt::ItemFlags f = QAbstractTableModel::flags(index);
  if (index.row() != index.column())
    f |= Qt::ItemIsUserCheckable;
  return f;
}
}  // namespace srdf_setup
}  // namespace moveit_setup
