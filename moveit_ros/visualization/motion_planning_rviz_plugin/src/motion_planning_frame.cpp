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

/* Author: Ioan Sucan */

#include <functional>

#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame_joints_widget.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/move_group/capability_names.h>

#include <geometric_shapes/shape_operations.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <tf2_ros/buffer.h>

#include <std_srvs/srv/empty.hpp>

#include <QMessageBox>
#include <QInputDialog>
#include <QFileDialog>
#include <QComboBox>
#include <QShortcut>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros_visualization.motion_planning_frame");

MotionPlanningFrame::MotionPlanningFrame(MotionPlanningDisplay* pdisplay, rviz_common::DisplayContext* context,
                                         QWidget* parent)
  : QWidget(parent), planning_display_(pdisplay), context_(context), ui_(new Ui::MotionPlanningUI()), first_time_(true)
{
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction)
  {
    RCLCPP_INFO(LOGGER, "Unable to lock weak_ptr from DisplayContext in MotionPlanningFrame constructor");
    return;
  }
  node_ = ros_node_abstraction->get_raw_node();

  // Prepare database parameters
  if (!node_->has_parameter("warehouse_host"))
    node_->declare_parameter<std::string>("warehouse_host", "127.0.0.1");
  if (!node_->has_parameter("warehouse_plugin"))
    node_->declare_parameter<std::string>("warehouse_plugin", "warehouse_ros_mongo::MongoDatabaseConnection");
  if (!node_->has_parameter("warehouse_port"))
    node_->declare_parameter<int>("warehouse_port", 33829);

  // set up the GUI
  ui_->setupUi(this);
  ui_->shapes_combo_box->addItem("Box", shapes::BOX);
  ui_->shapes_combo_box->addItem("Sphere", shapes::SPHERE);
  ui_->shapes_combo_box->addItem("Cylinder", shapes::CYLINDER);
  ui_->shapes_combo_box->addItem("Cone", shapes::CONE);
  ui_->shapes_combo_box->addItem("Mesh from file", shapes::MESH);
  ui_->shapes_combo_box->addItem("Mesh from URL", shapes::MESH);
  setLocalSceneEdited(false);

  // add more tabs
  joints_tab_ = new MotionPlanningFrameJointsWidget(planning_display_, ui_->tabWidget);
  ui_->tabWidget->insertTab(2, joints_tab_, "Joints");
  connect(planning_display_, SIGNAL(queryStartStateChanged()), joints_tab_, SLOT(queryStartStateChanged()));
  connect(planning_display_, SIGNAL(queryGoalStateChanged()), joints_tab_, SLOT(queryGoalStateChanged()));

  // connect buttons to actions; each action usually registers the function pointer for the actual computation,
  // to keep the GUI more responsive (using the background job processing)
  connect(ui_->plan_button, SIGNAL(clicked()), this, SLOT(planButtonClicked()));
  connect(ui_->execute_button, SIGNAL(clicked()), this, SLOT(executeButtonClicked()));
  connect(ui_->plan_and_execute_button, SIGNAL(clicked()), this, SLOT(planAndExecuteButtonClicked()));
  connect(ui_->stop_button, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
  connect(ui_->start_state_combo_box, SIGNAL(activated(QString)), this, SLOT(startStateTextChanged(QString)));
  connect(ui_->goal_state_combo_box, SIGNAL(activated(QString)), this, SLOT(goalStateTextChanged(QString)));
  connect(ui_->planning_group_combo_box, SIGNAL(currentIndexChanged(QString)), this,
          SLOT(planningGroupTextChanged(QString)));
  connect(ui_->database_connect_button, SIGNAL(clicked()), this, SLOT(databaseConnectButtonClicked()));
  connect(ui_->save_scene_button, SIGNAL(clicked()), this, SLOT(saveSceneButtonClicked()));
  connect(ui_->save_query_button, SIGNAL(clicked()), this, SLOT(saveQueryButtonClicked()));
  connect(ui_->delete_scene_button, SIGNAL(clicked()), this, SLOT(deleteSceneButtonClicked()));
  connect(ui_->delete_query_button, SIGNAL(clicked()), this, SLOT(deleteQueryButtonClicked()));
  connect(ui_->planning_scene_tree, SIGNAL(itemSelectionChanged()), this, SLOT(planningSceneItemClicked()));
  connect(ui_->load_scene_button, SIGNAL(clicked()), this, SLOT(loadSceneButtonClicked()));
  connect(ui_->load_query_button, SIGNAL(clicked()), this, SLOT(loadQueryButtonClicked()));
  connect(ui_->allow_looking, SIGNAL(toggled(bool)), this, SLOT(allowLookingToggled(bool)));
  connect(ui_->allow_replanning, SIGNAL(toggled(bool)), this, SLOT(allowReplanningToggled(bool)));
  connect(ui_->allow_external_program, SIGNAL(toggled(bool)), this, SLOT(allowExternalProgramCommunication(bool)));
  connect(ui_->planning_pipeline_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(planningPipelineIndexChanged(int)));
  connect(ui_->planning_algorithm_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(planningAlgorithmIndexChanged(int)));
  connect(ui_->clear_scene_button, SIGNAL(clicked()), this, SLOT(clearScene()));
  connect(ui_->scene_scale, SIGNAL(valueChanged(int)), this, SLOT(sceneScaleChanged(int)));
  connect(ui_->scene_scale, SIGNAL(sliderPressed()), this, SLOT(sceneScaleStartChange()));
  connect(ui_->scene_scale, SIGNAL(sliderReleased()), this, SLOT(sceneScaleEndChange()));
  connect(ui_->remove_object_button, SIGNAL(clicked()), this, SLOT(removeSceneObject()));
  connect(ui_->object_x, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_y, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_z, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_rx, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_ry, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->object_rz, SIGNAL(valueChanged(double)), this, SLOT(objectPoseValueChanged(double)));
  connect(ui_->publish_current_scene_button, SIGNAL(clicked()), this, SLOT(publishScene()));
  connect(ui_->collision_objects_list, SIGNAL(itemSelectionChanged()), this, SLOT(selectedCollisionObjectChanged()));
  connect(ui_->collision_objects_list, SIGNAL(itemChanged(QListWidgetItem*)), this,
          SLOT(collisionObjectChanged(QListWidgetItem*)));
  connect(ui_->path_constraints_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(pathConstraintsIndexChanged(int)));
  connect(ui_->clear_octomap_button, SIGNAL(clicked()), this, SLOT(onClearOctomapClicked()));
  connect(ui_->planning_scene_tree, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this,
          SLOT(warehouseItemNameChanged(QTreeWidgetItem*, int)));
  connect(ui_->reset_db_button, SIGNAL(clicked()), this, SLOT(resetDbButtonClicked()));

  connect(ui_->add_object_button, &QPushButton::clicked, this, &MotionPlanningFrame::addSceneObject);
  connect(ui_->shapes_combo_box, &QComboBox::currentTextChanged, this, &MotionPlanningFrame::shapesComboBoxChanged);
  connect(ui_->export_scene_geometry_text_button, SIGNAL(clicked()), this, SLOT(exportGeometryAsTextButtonClicked()));
  connect(ui_->import_scene_geometry_text_button, SIGNAL(clicked()), this, SLOT(importGeometryFromTextButtonClicked()));
  connect(ui_->load_state_button, SIGNAL(clicked()), this, SLOT(loadStateButtonClicked()));
  connect(ui_->save_start_state_button, SIGNAL(clicked()), this, SLOT(saveStartStateButtonClicked()));
  connect(ui_->save_goal_state_button, SIGNAL(clicked()), this, SLOT(saveGoalStateButtonClicked()));
  connect(ui_->set_as_start_state_button, SIGNAL(clicked()), this, SLOT(setAsStartStateButtonClicked()));
  connect(ui_->set_as_goal_state_button, SIGNAL(clicked()), this, SLOT(setAsGoalStateButtonClicked()));
  connect(ui_->remove_state_button, SIGNAL(clicked()), this, SLOT(removeStateButtonClicked()));
  connect(ui_->clear_states_button, SIGNAL(clicked()), this, SLOT(clearStatesButtonClicked()));
  connect(ui_->approximate_ik, SIGNAL(stateChanged(int)), this, SLOT(approximateIKChanged(int)));

  connect(ui_->detect_objects_button, SIGNAL(clicked()), this, SLOT(detectObjectsButtonClicked()));
  connect(ui_->pick_button, SIGNAL(clicked()), this, SLOT(pickObjectButtonClicked()));
  connect(ui_->place_button, SIGNAL(clicked()), this, SLOT(placeObjectButtonClicked()));
  connect(ui_->detected_objects_list, SIGNAL(itemSelectionChanged()), this, SLOT(selectedDetectedObjectChanged()));
  connect(ui_->detected_objects_list, SIGNAL(itemChanged(QListWidgetItem*)), this,
          SLOT(detectedObjectChanged(QListWidgetItem*)));
  connect(ui_->support_surfaces_list, SIGNAL(itemSelectionChanged()), this, SLOT(selectedSupportSurfaceChanged()));

  connect(ui_->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabChanged(int)));

  /* Notice changes to be safed in config file */
  connect(ui_->database_host, SIGNAL(textChanged(QString)), this, SIGNAL(configChanged()));
  connect(ui_->database_port, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));

  connect(ui_->planning_time, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
  connect(ui_->planning_attempts, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
  connect(ui_->velocity_scaling_factor, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
  connect(ui_->acceleration_scaling_factor, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

  connect(ui_->allow_replanning, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
  connect(ui_->allow_looking, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
  connect(ui_->allow_external_program, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
  connect(ui_->use_cartesian_path, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
  connect(ui_->collision_aware_ik, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
  connect(ui_->approximate_ik, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));

  connect(ui_->wcenter_x, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
  connect(ui_->wcenter_y, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
  connect(ui_->wcenter_z, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
  connect(ui_->wsize_x, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
  connect(ui_->wsize_y, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
  connect(ui_->wsize_z, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

  QShortcut* copy_object_shortcut = new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_C), ui_->collision_objects_list);
  connect(copy_object_shortcut, SIGNAL(activated()), this, SLOT(copySelectedCollisionObject()));

  ui_->reset_db_button->hide();
  ui_->background_job_progress->hide();
  ui_->background_job_progress->setMaximum(0);

  ui_->tabWidget->setCurrentIndex(1);

  known_collision_objects_version_ = 0;

  initFromMoveGroupNS();

  object_recognition_client_ = rclcpp_action::create_client<object_recognition_msgs::action::ObjectRecognition>(
      node_, OBJECT_RECOGNITION_ACTION);

  if (object_recognition_client_)
  {
    if (!object_recognition_client_->wait_for_action_server(std::chrono::seconds(3)))
    {
      RCLCPP_ERROR(LOGGER, "Action server: %s not available", OBJECT_RECOGNITION_ACTION.c_str());
      object_recognition_client_.reset();
    }
  }
  // TODO (ddengster): Enable when moveit_ros_perception is ported

  // try
  // {
  // const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
  //    if (ps)
  //    {
  //      semantic_world_.reset(new moveit::semantic_world::SemanticWorld(ps));
  //    }
  //    else
  //      semantic_world_.reset();
  //    if (semantic_world_)
  //    {
  //      semantic_world_->addTableCallback([this] { updateTables(); });
  //    }
  //  }
  //  catch (std::exception& ex)
  //  {
  //    RCLCPP_ERROR(LOGGER, "Failed to get semantic world: %s", ex.what());
  //  }
}

MotionPlanningFrame::~MotionPlanningFrame()
{
  delete ui_;
}

void MotionPlanningFrame::approximateIKChanged(int state)
{
  planning_display_->useApproximateIK(state == Qt::Checked);
}

void MotionPlanningFrame::setItemSelectionInList(const std::string& item_name, bool selection, QListWidget* list)
{
  QList<QListWidgetItem*> found_items = list->findItems(QString(item_name.c_str()), Qt::MatchExactly);
  for (QListWidgetItem* found_item : found_items)
    found_item->setSelected(selection);
}

void MotionPlanningFrame::allowExternalProgramCommunication(bool enable)
{
  // This is needed to prevent UI event (resuming the options) triggered
  // before getRobotInteraction() is loaded and ready
  if (first_time_)
    return;

  planning_display_->getRobotInteraction()->toggleMoveInteractiveMarkerTopic(enable);
  planning_display_->toggleSelectPlanningGroupSubscription(enable);
  if (enable)
  {
    using std::placeholders::_1;
    plan_subscriber_ = node_->create_subscription<std_msgs::msg::Empty>(
        "/rviz/moveit/plan", 1,
        [this](const std_msgs::msg::Empty::ConstSharedPtr msg) { return remotePlanCallback(msg); });
    execute_subscriber_ = node_->create_subscription<std_msgs::msg::Empty>(
        "/rviz/moveit/execute", 1,
        [this](const std_msgs::msg::Empty::ConstSharedPtr msg) { return remoteExecuteCallback(msg); });
    stop_subscriber_ = node_->create_subscription<std_msgs::msg::Empty>(
        "/rviz/moveit/stop", 1,
        [this](const std_msgs::msg::Empty::ConstSharedPtr msg) { return remoteStopCallback(msg); });
    update_start_state_subscriber_ = node_->create_subscription<std_msgs::msg::Empty>(
        "/rviz/moveit/update_start_state", 1,
        [this](const std_msgs::msg::Empty::ConstSharedPtr msg) { return remoteUpdateStartStateCallback(msg); });
    update_goal_state_subscriber_ = node_->create_subscription<std_msgs::msg::Empty>(
        "/rviz/moveit/update_goal_state", 1,
        [this](const std_msgs::msg::Empty::ConstSharedPtr msg) { return remoteUpdateGoalStateCallback(msg); });
    update_custom_start_state_subscriber_ = node_->create_subscription<moveit_msgs::msg::RobotState>(
        "/rviz/moveit/update_custom_start_state", 1, [this](const moveit_msgs::msg::RobotState::ConstSharedPtr msg) {
          return remoteUpdateCustomStartStateCallback(msg);
        });
    update_custom_goal_state_subscriber_ = node_->create_subscription<moveit_msgs::msg::RobotState>(
        "/rviz/moveit/update_custom_goal_state", 1, [this](const moveit_msgs::msg::RobotState::ConstSharedPtr msg) {
          return remoteUpdateCustomGoalStateCallback(msg);
        });
  }
  else
  {  // disable
    plan_subscriber_.reset();
    execute_subscriber_.reset();
    stop_subscriber_.reset();
    update_start_state_subscriber_.reset();
    update_goal_state_subscriber_.reset();
    update_custom_start_state_subscriber_.reset();
    update_custom_goal_state_subscriber_.reset();
  }
}

void MotionPlanningFrame::fillPlanningGroupOptions()
{
  const QSignalBlocker planning_group_blocker(ui_->planning_group_combo_box);
  ui_->planning_group_combo_box->clear();

  const moveit::core::RobotModelConstPtr& kmodel = planning_display_->getRobotModel();
  for (const std::string& group_name : kmodel->getJointModelGroupNames())
    ui_->planning_group_combo_box->addItem(QString::fromStdString(group_name));
}

void MotionPlanningFrame::fillStateSelectionOptions()
{
  const QSignalBlocker start_state_blocker(ui_->start_state_combo_box);
  const QSignalBlocker goal_state_blocker(ui_->goal_state_combo_box);
  ui_->start_state_combo_box->clear();
  ui_->goal_state_combo_box->clear();

  if (!planning_display_->getPlanningSceneMonitor())
    return;

  const moveit::core::RobotModelConstPtr& robot_model = planning_display_->getRobotModel();
  std::string group = planning_display_->getCurrentPlanningGroup();
  if (group.empty())
    return;
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group);
  if (jmg)
  {
    ui_->start_state_combo_box->addItem(QString("<random valid>"));
    ui_->start_state_combo_box->addItem(QString("<random>"));
    ui_->start_state_combo_box->addItem(QString("<current>"));
    ui_->start_state_combo_box->addItem(QString("<same as goal>"));
    ui_->start_state_combo_box->addItem(QString("<previous>"));

    ui_->goal_state_combo_box->addItem(QString("<random valid>"));
    ui_->goal_state_combo_box->addItem(QString("<random>"));
    ui_->goal_state_combo_box->addItem(QString("<current>"));
    ui_->goal_state_combo_box->addItem(QString("<same as start>"));
    ui_->goal_state_combo_box->addItem(QString("<previous>"));

    const std::vector<std::string>& known_states = jmg->getDefaultStateNames();
    if (!known_states.empty())
    {
      ui_->start_state_combo_box->insertSeparator(ui_->start_state_combo_box->count());
      ui_->goal_state_combo_box->insertSeparator(ui_->goal_state_combo_box->count());
      for (const std::string& known_state : known_states)
      {
        ui_->start_state_combo_box->addItem(QString::fromStdString(known_state));
        ui_->goal_state_combo_box->addItem(QString::fromStdString(known_state));
      }
    }

    ui_->start_state_combo_box->setCurrentIndex(2);  // default to 'current'
    ui_->goal_state_combo_box->setCurrentIndex(2);   // default to 'current'
  }
}

void MotionPlanningFrame::changePlanningGroupHelper()
{
  if (!planning_display_->getPlanningSceneMonitor())
    return;

  planning_display_->addMainLoopJob([this] { fillStateSelectionOptions(); });
  planning_display_->addMainLoopJob([this]() { populateConstraintsList(std::vector<std::string>()); });

  const moveit::core::RobotModelConstPtr& robot_model = planning_display_->getRobotModel();
  std::string group = planning_display_->getCurrentPlanningGroup();
  planning_display_->addMainLoopJob([&view = *ui_->planner_param_treeview, group] { view.setGroupName(group); });
  planning_display_->addMainLoopJob(
      [=]() { ui_->planning_group_combo_box->setCurrentText(QString::fromStdString(group)); });

  if (!group.empty() && robot_model)
  {
    RCLCPP_INFO(LOGGER, "group %s", group.c_str());
    if (move_group_ && move_group_->getName() == group)
      return;
    RCLCPP_INFO(LOGGER, "Constructing new MoveGroup connection for group '%s' in namespace '%s'", group.c_str(),
                planning_display_->getMoveGroupNS().c_str());
    moveit::planning_interface::MoveGroupInterface::Options opt(
        group, moveit::planning_interface::MoveGroupInterface::ROBOT_DESCRIPTION, planning_display_->getMoveGroupNS());
    opt.robot_model_ = robot_model;
    opt.robot_description_.clear();
    try
    {
#ifdef RVIZ_TF1
      std::shared_ptr<tf2_ros::Buffer> tf_buffer = moveit::planning_interface::getSharedTF();
#else
      //@note: tf2 no longer accessible?
      // /std::shared_ptr<tf2_ros::Buffer> tf_buffer = context_->getFrameManager()->getTF2BufferPtr();
      std::shared_ptr<tf2_ros::Buffer> tf_buffer = moveit::planning_interface::getSharedTF();
#endif
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          node_, opt, tf_buffer, rclcpp::Duration::from_seconds(30));
      if (planning_scene_storage_)
        move_group_->setConstraintsDatabase(ui_->database_host->text().toStdString(), ui_->database_port->value());
    }
    catch (std::exception& ex)
    {
      RCLCPP_ERROR(LOGGER, "%s", ex.what());
    }
    planning_display_->addMainLoopJob([&view = *ui_->planner_param_treeview, this] { view.setMoveGroup(move_group_); });
    if (move_group_)
    {
      move_group_->allowLooking(ui_->allow_looking->isChecked());
      move_group_->allowReplanning(ui_->allow_replanning->isChecked());
      bool has_unique_endeffector = !move_group_->getEndEffectorLink().empty();
      planning_display_->addMainLoopJob([=]() { ui_->use_cartesian_path->setEnabled(has_unique_endeffector); });
      std::vector<moveit_msgs::msg::PlannerInterfaceDescription> desc;
      if (move_group_->getInterfaceDescriptions(desc))
        planning_display_->addMainLoopJob([this, desc] { populatePlannersList(desc); });
      planning_display_->addBackgroundJob([this]() { populateConstraintsList(); }, "populateConstraintsList");

      if (first_time_)
      {
        first_time_ = false;
        const planning_scene_monitor::LockedPlanningSceneRO& ps = planning_display_->getPlanningSceneRO();
        if (ps)
        {
          planning_display_->setQueryStartState(ps->getCurrentState());
          planning_display_->setQueryGoalState(ps->getCurrentState());
        }
        // This ensures saved UI settings applied after planning_display_ is ready
        planning_display_->useApproximateIK(ui_->approximate_ik->isChecked());
        if (ui_->allow_external_program->isChecked())
          planning_display_->addMainLoopJob([this] { allowExternalProgramCommunication(true); });
      }
    }
  }
}

void MotionPlanningFrame::clearRobotModel()
{
  ui_->planner_param_treeview->setMoveGroup(moveit::planning_interface::MoveGroupInterfacePtr());
  joints_tab_->clearRobotModel();
  move_group_.reset();
}

void MotionPlanningFrame::changePlanningGroup()
{
  planning_display_->addBackgroundJob([this] { changePlanningGroupHelper(); }, "Frame::changePlanningGroup");
  joints_tab_->changePlanningGroup(planning_display_->getCurrentPlanningGroup(),
                                   planning_display_->getQueryStartStateHandler(),
                                   planning_display_->getQueryGoalStateHandler());
}

void MotionPlanningFrame::sceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  if (update_type & planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY)
    planning_display_->addMainLoopJob([this] { populateCollisionObjectsList(); });
}

void MotionPlanningFrame::addSceneObject()
{
  static const double MIN_VAL = 1e-6;

  if (!planning_display_->getPlanningSceneMonitor())
  {
    return;
  }

  // get size values
  double x_length = ui_->shape_size_x_spin_box->isEnabled() ? ui_->shape_size_x_spin_box->value() : MIN_VAL;
  double y_length = ui_->shape_size_y_spin_box->isEnabled() ? ui_->shape_size_y_spin_box->value() : MIN_VAL;
  double z_length = ui_->shape_size_z_spin_box->isEnabled() ? ui_->shape_size_z_spin_box->value() : MIN_VAL;
  if (x_length < MIN_VAL || y_length < MIN_VAL || z_length < MIN_VAL)
  {
    QMessageBox::warning(this, QString("Dimension is too small"), QString("Size values need to be >= %1").arg(MIN_VAL));
    return;
  }

  // by default, name object by shape type
  std::string selected_shape = ui_->shapes_combo_box->currentText().toStdString();
  shapes::ShapeConstPtr shape;
  switch (ui_->shapes_combo_box->currentData().toInt())  // fetch shape ID from current combobox item
  {
    case shapes::BOX:
      shape = std::make_shared<shapes::Box>(x_length, y_length, z_length);
      break;
    case shapes::SPHERE:
      shape = std::make_shared<shapes::Sphere>(0.5 * x_length);
      break;
    case shapes::CONE:
      shape = std::make_shared<shapes::Cone>(0.5 * x_length, z_length);
      break;
    case shapes::CYLINDER:
      shape = std::make_shared<shapes::Cylinder>(0.5 * x_length, z_length);
      break;
    case shapes::MESH:
    {
      QUrl url;
      if (ui_->shapes_combo_box->currentText().contains("file"))  // open from file
        url = QFileDialog::getOpenFileUrl(this, tr("Import Object Mesh"), QString(),
                                          "CAD files (*.stl *.obj *.dae);;All files (*.*)");
      else  // open from URL
        url = QInputDialog::getText(this, tr("Import Object Mesh"), tr("URL for file to import from:"),
                                    QLineEdit::Normal, QString("http://"));
      if (!url.isEmpty())
        shape = loadMeshResource(url.toString().toStdString());
      if (!shape)
        return;
      // name mesh objects by their file name
      selected_shape = url.fileName().toStdString();
      break;
    }
    default:
      QMessageBox::warning(this, QString("Unsupported shape"),
                           QString("The '%1' is not supported.").arg(ui_->shapes_combo_box->currentText()));
  }

  // find available (initial) name of object
  int idx = 0;
  std::string shape_name = selected_shape + "_" + std::to_string(idx);
  while (planning_display_->getPlanningSceneRO()->getWorld()->hasObject(shape_name))
  {
    idx++;
    shape_name = selected_shape + "_" + std::to_string(idx);
  }

  // Actually add object to the plugin's PlanningScene
  {
    planning_scene_monitor::LockedPlanningSceneRW ps = planning_display_->getPlanningSceneRW();
    ps->getWorldNonConst()->addToObject(shape_name, shape, Eigen::Isometry3d::Identity());
  }
  setLocalSceneEdited();

  planning_display_->addMainLoopJob([this] { populateCollisionObjectsList(); });

  // Automatically select the inserted object so that its IM is displayed
  planning_display_->addMainLoopJob([this, shape_name, list_widget = ui_->collision_objects_list] {
    setItemSelectionInList(shape_name, true, list_widget);
  });

  planning_display_->queueRenderSceneGeometry();
}

shapes::ShapePtr MotionPlanningFrame::loadMeshResource(const std::string& url)
{
  shapes::Mesh* mesh = shapes::createMeshFromResource(url);
  if (mesh)
  {
    // If the object is very large, ask the user if the scale should be reduced.
    bool object_is_very_large = false;
    for (unsigned int i = 0; i < mesh->vertex_count; ++i)
    {
      if ((abs(mesh->vertices[i * 3 + 0]) > LARGE_MESH_THRESHOLD) ||
          (abs(mesh->vertices[i * 3 + 1]) > LARGE_MESH_THRESHOLD) ||
          (abs(mesh->vertices[i * 3 + 2]) > LARGE_MESH_THRESHOLD))
      {
        object_is_very_large = true;
        break;
      }
    }
    if (object_is_very_large)
    {
      QMessageBox msg_box;
      msg_box.setText(
          "The object is very large (greater than 10 m). The file may be in millimeters instead of meters.");
      msg_box.setInformativeText("Attempt to fix the size by shrinking the object?");
      msg_box.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
      msg_box.setDefaultButton(QMessageBox::Yes);
      if (msg_box.exec() == QMessageBox::Yes)
      {
        for (unsigned int i = 0; i < mesh->vertex_count; ++i)
        {
          unsigned int i3 = i * 3;
          mesh->vertices[i3] *= 0.001;
          mesh->vertices[i3 + 1] *= 0.001;
          mesh->vertices[i3 + 2] *= 0.001;
        }
      }
    }

    return shapes::ShapePtr(mesh);
  }
  else
  {
    QMessageBox::warning(this, QString("Import error"), QString("Unable to import object"));
    return shapes::ShapePtr();
  }
}

void MotionPlanningFrame::enable()
{
  ui_->planning_algorithm_combo_box->clear();
  ui_->library_label->setText("NO PLANNING LIBRARY LOADED");
  ui_->library_label->setStyleSheet("QLabel { color : red; font: bold }");
  ui_->object_status->setText("");

  const std::string new_ns = planning_display_->getMoveGroupNS();
  if (node_->get_namespace() != new_ns)
  {
    RCLCPP_INFO(LOGGER, "MoveGroup namespace changed: %s -> %s. Reloading params.", node_->get_namespace(),
                new_ns.c_str());
    initFromMoveGroupNS();
  }

  // activate the frame
  if (parentWidget())
    parentWidget()->show();
}

// (re)initialize after MotionPlanningDisplay::changedMoveGroupNS()
// Should be called from constructor and enable() only
void MotionPlanningFrame::initFromMoveGroupNS()
{
  // Create namespace-dependent services, topics, and subscribers
  clear_octomap_service_client_ = node_->create_client<std_srvs::srv::Empty>(move_group::CLEAR_OCTOMAP_SERVICE_NAME);

  object_recognition_subscriber_ = node_->create_subscription<object_recognition_msgs::msg::RecognizedObjectArray>(
      "recognized_object_array", 1,
      [this](const object_recognition_msgs::msg::RecognizedObjectArray::ConstSharedPtr msg) {
        return listenDetectedObjects(msg);
      });

  planning_scene_publisher_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
  planning_scene_world_publisher_ =
      node_->create_publisher<moveit_msgs::msg::PlanningSceneWorld>("planning_scene_world", 1);

  // Declare parameter for default planning pipeline
  if (!node_->has_parameter(planning_display_->getMoveGroupNS() + "default_planning_pipeline"))
    node_->declare_parameter<std::string>(planning_display_->getMoveGroupNS() + "default_planning_pipeline", "");

  // Query default planning pipeline id
  node_->get_parameter(planning_display_->getMoveGroupNS() + "default_planning_pipeline", default_planning_pipeline_);

  // Set initial velocity and acceleration scaling factors from ROS parameters
  double factor;
  node_->get_parameter_or("robot_description_planning.default_velocity_scaling_factor", factor, 0.1);
  ui_->velocity_scaling_factor->setValue(factor);
  node_->get_parameter_or("robot_description_planning.default_acceleration_scaling_factor", factor, 0.1);
  ui_->acceleration_scaling_factor->setValue(factor);

  // Fetch parameters from private move_group sub space
  std::string host_param;
  if (node_->get_parameter("warehouse_host", host_param))
  {
    ui_->database_host->setText(QString::fromStdString(host_param));
  }

  int port;
  if (node_->get_parameter("warehouse_port", port))
  {
    ui_->database_port->setValue(port);
  }
}

void MotionPlanningFrame::disable()
{
  move_group_.reset();
  scene_marker_.reset();
  if (parentWidget())
    parentWidget()->hide();
}

void MotionPlanningFrame::tabChanged(int index)
{
  if (scene_marker_ && ui_->tabWidget->tabText(index).toStdString() != TAB_OBJECTS)
    scene_marker_.reset();
  else if (ui_->tabWidget->tabText(index).toStdString() == TAB_OBJECTS)
    selectedCollisionObjectChanged();
}

void MotionPlanningFrame::updateSceneMarkers(float /*wall_dt*/, float /*ros_dt*/)
{
  if (scene_marker_)
    scene_marker_->update();
}

void MotionPlanningFrame::updateExternalCommunication()
{
  if (ui_->allow_external_program->isChecked())
  {
    planning_display_->getRobotInteraction()->toggleMoveInteractiveMarkerTopic(true);
  }
}

}  // namespace moveit_rviz_plugin
