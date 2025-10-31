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
#include <QRegularExpression>
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
/// Mapping of reasons for disabling a link pair to strings
static const std::unordered_map<DisabledReason, const char*> LONG_REASONS_TO_STRING =
    boost::assign::map_list_of  // clang-format off
    ( NEVER, "Never in Collision" )
    ( DEFAULT, "Collision by Default" )
    ( ADJACENT, "Adjacent Links" )
    ( ALWAYS, "Always in Collision" )
    ( USER, "User Disabled" )
    ( NOT_DISABLED, "");  // clang-format on

/// Mapping of reasons to a background color
static const std::unordered_map<DisabledReason, QVariant> LONG_REASONS_TO_BRUSH =
    boost::assign::map_list_of  // clang-format off
    ( NEVER, QBrush(QColor("lightgreen")) )
    ( DEFAULT, QBrush(QColor("lightpink")) )
    ( ADJACENT, QBrush(QColor("powderblue")) )
    ( ALWAYS, QBrush(QColor("tomato")) )
    ( USER, QBrush(QColor("yellow")) )
    ( NOT_DISABLED, QBrush());  // clang-format on

CollisionMatrixModel::CollisionMatrixModel(LinkPairMap& pairs, const std::vector<std::string>& names, QObject* parent)
  : QAbstractTableModel(parent), pairs_(pairs), std_names_(names)
{
  int idx = 0;
  for (std::vector<std::string>::const_iterator it = names.begin(), end = names.end(); it != end; ++it, ++idx)
  {
    visual_to_index_ << idx;
    q_names_ << QString::fromStdString(*it);
  }
}

// return item in pairs map given a normalized index, use item(normalized(index))
LinkPairMap::iterator CollisionMatrixModel::item(const QModelIndex& index)
{
  int r = visual_to_index_[index.row()], c = visual_to_index_[index.column()];
  if (r == c)
    return pairs_.end();

  // setLinkPair() actually inserts the pair (A,B) where A < B
  if (std_names_[r] >= std_names_[c])
    std::swap(r, c);

  return pairs_.find(std::make_pair(std_names_[r], std_names_[c]));
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
  if (index.isValid() && index.row() == index.column() && role == Qt::BackgroundRole)
    return QApplication::palette().window();

  LinkPairMap::const_iterator item = this->item(index);
  if (item == pairs_.end())
    return QVariant();

  switch (role)
  {
    case Qt::CheckStateRole:
      return item->second.disable_check ? Qt::Checked : Qt::Unchecked;
    case Qt::ToolTipRole:
      return LONG_REASONS_TO_STRING.at(item->second.reason);
    case Qt::BackgroundRole:
      return LONG_REASONS_TO_BRUSH.at(item->second.reason);
  }
  return QVariant();
}

DisabledReason CollisionMatrixModel::reason(const QModelIndex& index) const
{
  LinkPairMap::const_iterator item = this->item(index);
  if (item == pairs_.end())
    return NOT_DISABLED;
  return item->second.reason;
}

bool CollisionMatrixModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
  if (role == Qt::CheckStateRole)
  {
    LinkPairMap::iterator item = this->item(index);
    if (item == pairs_.end())
      return false;

    bool new_value = (value.toInt() == Qt::Checked);
    if (item->second.disable_check == new_value)
      return true;

    item->second.disable_check = new_value;

    // Handle USER Reasons: 1) pair is disabled by user
    if (item->second.disable_check && item->second.reason == NOT_DISABLED)
    {
      item->second.reason = USER;

      // Handle USER Reasons: 2) pair was disabled by user and now is enabled (not checked)
    }
    else if (!item->second.disable_check && item->second.reason == USER)
    {
      item->second.reason = NOT_DISABLED;
    }

    QModelIndex mirror = this->index(index.column(), index.row());
    Q_EMIT dataChanged(index, index);
    Q_EMIT dataChanged(mirror, mirror);
    return true;
  }
  return false;  // reject all other changes
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
  QRegularExpression regexp;
  regexp.setPattern(filter);
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
