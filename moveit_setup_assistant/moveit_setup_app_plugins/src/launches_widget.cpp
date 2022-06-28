/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Metro Robots
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
 *   * Neither the name of Metro Robots nor the names of its
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

/* Author: David V. Lu!! */
#include <moveit_setup_app_plugins/launches_widget.hpp>
#include <moveit_setup_framework/qt/helper_widgets.hpp>

#include <QSplitter>

namespace moveit_setup
{
namespace app
{
void LaunchesWidget::onInit()
{
  // Basic widget container
  QVBoxLayout* layout = new QVBoxLayout();

  // Top Header Area ------------------------------------------------

  auto header = new HeaderWidget("Configure Desired Launch Files",
                                 "Figure out which launch files you want to be generated.", this);
  layout->addWidget(header);

  QSplitter* splitter = new QSplitter(Qt::Horizontal, this);
  splitter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  list_widget_ = new QListWidget(this);
  list_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  list_widget_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  connect(list_widget_, SIGNAL(currentItemChanged(QListWidgetItem*, QListWidgetItem*)), this,
          SLOT(onNewSelectedItem(QListWidgetItem*, QListWidgetItem*)));

  splitter->addWidget(list_widget_);

  for (const LaunchBundle& lb : setup_step_.getAvailableLaunchBundles())
  {
    QListWidgetItem* item = new QListWidgetItem(QString(lb.getTitle().c_str()), list_widget_);
    item->setData(Qt::UserRole, QVariant(lb.getID()));

    list_widget_->addItem(item);
  }

  text_widget_ = new QLabel(this);
  text_widget_->setFrameShape(QFrame::StyledPanel);
  text_widget_->setFrameShadow(QFrame::Raised);
  text_widget_->setLineWidth(1);
  text_widget_->setMidLineWidth(0);
  text_widget_->setWordWrap(true);
  text_widget_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  text_widget_->setMinimumWidth(100);
  text_widget_->setAlignment(Qt::AlignTop);
  splitter->addWidget(text_widget_);
  layout->addWidget(splitter);

  // Finish Layout --------------------------------------------------
  this->setLayout(layout);
}

unsigned int getID(QListWidgetItem* item)
{
  QVariant data = item->data(Qt::UserRole);
  return data.toUInt();
}

void LaunchesWidget::focusGiven()
{
  disconnect(list_widget_, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(changeCheckedState(QListWidgetItem*)));
  for (int i = 0; i < list_widget_->count(); ++i)
  {
    QListWidgetItem* item = list_widget_->item(i);

    unsigned int id = getID(item);

    item->setCheckState(setup_step_.getState(id) ? Qt::Checked : Qt::Unchecked);
  }
  connect(list_widget_, SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(changeCheckedState(QListWidgetItem*)));
}

void LaunchesWidget::onNewSelectedItem(QListWidgetItem* current, QListWidgetItem* /*previous*/)
{
  unsigned int id = getID(current);
  text_widget_->setText(setup_step_.getDescription(id).c_str());
}

void LaunchesWidget::changeCheckedState(QListWidgetItem* item)
{
  unsigned int id = getID(item);
  bool state = (item->checkState() == Qt::Checked);
  setup_step_.setState(id, state);
}

}  // namespace app
}  // namespace moveit_setup

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(moveit_setup::app::LaunchesWidget, moveit_setup::SetupStepWidget)
