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

/* Author: Dave Coleman */

#pragma once

#include <moveit_setup_framework/qt/setup_step_widget.hpp>
#include <moveit_setup_srdf_plugins/default_collisions.hpp>

#include <QThread>
#include <QAbstractItemModel>
#include <QAction>
#include <QButtonGroup>
#include <QCheckBox>
#include <QGroupBox>
#include <QHeaderView>
#include <QItemSelection>
#include <QItemSelectionModel>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QTableView>
#include <QVBoxLayout>

namespace moveit_setup
{
namespace srdf_setup
{
class MonitorThread;

/**
 * \brief User interface for editing the default collision matrix list in an SRDF
 */
class DefaultCollisionsWidget : public SetupStepWidget
{
  Q_OBJECT

public:
  enum ViewMode
  {
    MATRIX_MODE = 0,
    LINEAR_MODE = 1
  };

  // ******************************************************************************************
  // Public Functions
  // ******************************************************************************************
  void onInit() override;

  ~DefaultCollisionsWidget() override;

  SetupStep& getSetupStep() override
  {
    return setup_step_;
  }

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************

  /**
   * \brief start generating collision matrix in a worker thread
   */
  void startGeneratingCollisionTable();
  /**
   * \brief finish generating collision matrix after worker thread has finished
   */
  void finishGeneratingCollisionTable();

  /**
   * \brief GUI func for showing sampling density amount
   * \param value Sampling density
   */
  void changeDensityLabel(int value);

  /**
   * \brief Update view and data model for the link_pairs data structure
   */
  void loadCollisionTable();

  /**
   * \brief Change filter settings to show/hide enabled collisions
   */
  void checkedFilterChanged();

  /**
   * \brief Collision model changed
   */
  void collisionsChanged(const QModelIndex& index);

  /**
   * \brief Revert current changes to collision matrix
   */
  void revertChanges();

  /**
   * \brief Called when current row has changed
   */
  void previewSelectedMatrix(const QModelIndex& index);
  void previewSelectedLinear(const QModelIndex& index);

  /**
   * \brief Called when setup assistant navigation switches to this screen
   */
  void focusGiven() override;

  /**
   * \brief Called when setup assistant navigation switches away from this screen
   */
  bool focusLost() override;

  void showHeaderContextMenu(const QPoint& p);
  void hideSections();
  void hideOtherSections();
  void showSections();

private:
  // ******************************************************************************************
  // Qt Components
  // ******************************************************************************************
  QLabel* page_title_;
  QTableView* collision_table_;
  QAbstractItemModel* model_;
  QItemSelectionModel* selection_model_;
  QVBoxLayout* layout_;
  QLabel* density_value_label_;
  QSlider* density_slider_;
  QPushButton* btn_generate_;
  QGroupBox* controls_box_;
  QProgressBar* progress_bar_;
  QLabel* progress_label_;
  QLineEdit* link_name_filter_;
  QCheckBox* collision_checkbox_;
  QLabel* fraction_label_;
  QSpinBox* fraction_spinbox_;
  QPushButton* btn_revert_;
  QButtonGroup* view_mode_buttons_;

  QList<QAction*> header_actions_;    // context actions for header sections
  Qt::Orientations clicked_headers_;  // remember which header section activated context actions
  int clicked_section_;               // remember which header section activated context actions

  // ******************************************************************************************
  // Variables
  // ******************************************************************************************
  MonitorThread* worker_;

  DefaultCollisions setup_step_;

  // ******************************************************************************************
  // Private Functions
  // ******************************************************************************************

  /**
   * \brief Helper function to disable parts of GUI during computation
   * \param disable A command
   */
  void disableControls(bool disable);

  /**
   * \brief Allow toggling of all checkboxes in selection by filtering <space> keypresses
   */
  bool eventFilter(QObject* object, QEvent* event) override;

  /**
   * \brief Show header's sections in logicalIndexes and everything in between
   */
  void showSections(QHeaderView* header, const QList<int>& logicalIndexes);
  /**
   * \brief Toggle enabled status of selection
   */
  void toggleSelection(QItemSelection selection);
};

/**
 * \brief QThread to monitor progress of the setup step
 */
class MonitorThread : public QThread
{
  Q_OBJECT

public:
  MonitorThread(DefaultCollisions& setup_step, QProgressBar* progress_bar = nullptr);
  void run() override;
  void cancel()
  {
    canceled_ = true;
  }
  bool canceled() const
  {
    return canceled_;
  }

Q_SIGNALS:
  void progress(int /*_t1*/);

private:
  DefaultCollisions& setup_step_;
  bool canceled_;
};
}  // namespace srdf_setup
}  // namespace moveit_setup
