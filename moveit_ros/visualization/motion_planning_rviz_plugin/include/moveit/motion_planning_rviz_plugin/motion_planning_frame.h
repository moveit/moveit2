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

#pragma once

#include <QWidget>
#include <QTreeWidgetItem>
#include <QListWidgetItem>

#ifndef Q_MOC_RUN
#include <moveit/macros/class_forward.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit/robot_interaction/interaction_handler.h>
// TODO (ddengster): Enable when moveit_ros_perception is ported
// #include <moveit/semantic_world/semantic_world.h>

#include <interactive_markers/interactive_marker_server.hpp>
#include <rviz_default_plugins/displays/interactive_markers/interactive_marker.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <object_recognition_msgs/action/object_recognition.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>
#endif

#include <map>
#include <string>
#include <memory>

namespace rviz_common
{
class DisplayContext;
}

namespace Ui
{
class MotionPlanningUI;
}

namespace moveit_warehouse
{
MOVEIT_CLASS_FORWARD(PlanningSceneStorage);  // Defines PlanningSceneStoragePtr, ConstPtr, WeakPtr... etc
MOVEIT_CLASS_FORWARD(ConstraintsStorage);    // Defines ConstraintsStoragePtr, ConstPtr, WeakPtr... etc
MOVEIT_CLASS_FORWARD(RobotStateStorage);     // Defines RobotStateStoragePtr, ConstPtr, WeakPtr... etc
}  // namespace moveit_warehouse

namespace moveit_rviz_plugin
{
class MotionPlanningDisplay;
class MotionPlanningFrameJointsWidget;

const std::string OBJECT_RECOGNITION_ACTION = "/recognize_objects";

static const std::string TAB_CONTEXT = "Context";
static const std::string TAB_PLANNING = "Planning";
static const std::string TAB_MANIPULATION = "Manipulation";
static const std::string TAB_OBJECTS = "Scene Objects";
static const std::string TAB_SCENES = "Stored Scenes";
static const std::string TAB_STATES = "Stored States";
static const std::string TAB_STATUS = "Status";

static const double LARGE_MESH_THRESHOLD = 10.0;

class MotionPlanningFrame : public QWidget
{
  friend class MotionPlanningDisplay;
  Q_OBJECT

public:
  MotionPlanningFrame(const MotionPlanningFrame&) = delete;
  MotionPlanningFrame(MotionPlanningDisplay* pdisplay, rviz_common::DisplayContext* context, QWidget* parent = nullptr);
  ~MotionPlanningFrame() override;

  void clearRobotModel();
  void changePlanningGroup();
  void enable();
  void disable();
  void sceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type);

protected:
  static const int ITEM_TYPE_SCENE = 1;
  static const int ITEM_TYPE_QUERY = 2;

  void initFromMoveGroupNS();
  void constructPlanningRequest(moveit_msgs::msg::MotionPlanRequest& mreq);

  void updateSceneMarkers(float wall_dt, float ros_dt);

  void updateExternalCommunication();

  MotionPlanningDisplay* planning_display_;
  rviz_common::DisplayContext* context_;
  Ui::MotionPlanningUI* ui_;
  MotionPlanningFrameJointsWidget* joints_tab_;

  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  // TODO (ddengster): Enable when moveit_ros_perception is ported
  //  moveit::semantic_world::SemanticWorldPtr semantic_world_;

  moveit::planning_interface::MoveGroupInterface::PlanPtr current_plan_;
  moveit_warehouse::PlanningSceneStoragePtr planning_scene_storage_;
  moveit_warehouse::ConstraintsStoragePtr constraints_storage_;
  moveit_warehouse::RobotStateStoragePtr robot_state_storage_;

  std::shared_ptr<rviz_default_plugins::displays::InteractiveMarker> scene_marker_;

  typedef std::map<std::string, moveit_msgs::msg::RobotState> RobotStateMap;
  typedef std::pair<std::string, moveit_msgs::msg::RobotState> RobotStatePair;
  RobotStateMap robot_states_;
  std::string default_planning_pipeline_;
  std::vector<moveit_msgs::msg::PlannerInterfaceDescription> planner_descriptions_;

Q_SIGNALS:
  void planningFinished();
  void configChanged();

private Q_SLOTS:

  // Context tab
  void databaseConnectButtonClicked();
  void planningPipelineIndexChanged(int index);
  void planningAlgorithmIndexChanged(int index);
  void resetDbButtonClicked();
  void approximateIKChanged(int state);

  // Planning tab
  bool computeCartesianPlan();
  bool computeJointSpacePlan();
  void planButtonClicked();
  void executeButtonClicked();
  void planAndExecuteButtonClicked();
  void stopButtonClicked();
  void allowReplanningToggled(bool checked);
  void allowLookingToggled(bool checked);
  void allowExternalProgramCommunication(bool enable);
  void pathConstraintsIndexChanged(int index);
  void onNewPlanningSceneState();
  void startStateTextChanged(const QString& start_state);
  void goalStateTextChanged(const QString& goal_state);
  void planningGroupTextChanged(const QString& planning_group);
  void onClearOctomapClicked();

  // Scene Objects tab
  void clearScene();
  void publishScene();
  void publishSceneIfNeeded();
  void setLocalSceneEdited(bool dirty = true);
  bool isLocalSceneDirty() const;
  void sceneScaleChanged(int value);
  void sceneScaleStartChange();
  void sceneScaleEndChange();
  void shapesComboBoxChanged(const QString& text);
  void addSceneObject();
  void removeSceneObject();
  void selectedCollisionObjectChanged();
  void objectPoseValueChanged(double value);
  void collisionObjectChanged(QListWidgetItem* item);
  void imProcessFeedback(visualization_msgs::msg::InteractiveMarkerFeedback& feedback);
  void copySelectedCollisionObject();
  void exportGeometryAsTextButtonClicked();
  void importGeometryFromTextButtonClicked();

  // Stored scenes tab
  void saveSceneButtonClicked();
  void planningSceneItemClicked();
  void saveQueryButtonClicked();
  void deleteSceneButtonClicked();
  void deleteQueryButtonClicked();
  void loadSceneButtonClicked();
  void loadQueryButtonClicked();
  void warehouseItemNameChanged(QTreeWidgetItem* item, int column);

  // States tab
  void loadStateButtonClicked();
  void saveStartStateButtonClicked();
  void saveGoalStateButtonClicked();
  void removeStateButtonClicked();
  void clearStatesButtonClicked();
  void setAsStartStateButtonClicked();
  void setAsGoalStateButtonClicked();

  // Pick and place
  void detectObjectsButtonClicked();
  void pickObjectButtonClicked();
  void placeObjectButtonClicked();
  void selectedDetectedObjectChanged();
  void detectedObjectChanged(QListWidgetItem* item);
  void selectedSupportSurfaceChanged();

  // General
  void tabChanged(int index);

private:
  // Context tab
  void computeDatabaseConnectButtonClicked();
  void computeDatabaseConnectButtonClickedHelper(int mode);
  void computeResetDbButtonClicked(const std::string& db);
  void populatePlannersList(const std::vector<moveit_msgs::msg::PlannerInterfaceDescription>& desc);
  void populatePlannerDescription(const moveit_msgs::msg::PlannerInterfaceDescription& desc);

  // Planning tab
  void computePlanButtonClicked();
  void computeExecuteButtonClicked();
  void computePlanAndExecuteButtonClicked();
  void computePlanAndExecuteButtonClickedDisplayHelper();
  void computeStopButtonClicked();
  void onFinishedExecution(bool success);
  void populateConstraintsList();
  void populateConstraintsList(const std::vector<std::string>& constr);
  void configureForPlanning();
  void configureWorkspace();
  void updateQueryStateHelper(moveit::core::RobotState& state, const std::string& v);
  void fillStateSelectionOptions();
  void fillPlanningGroupOptions();
  void startStateTextChangedExec(const std::string& start_state);
  void goalStateTextChangedExec(const std::string& goal_state);

  // Scene objects tab
  void updateCollisionObjectPose(bool update_marker_position);
  void createSceneInteractiveMarker();
  void renameCollisionObject(QListWidgetItem* item);
  void attachDetachCollisionObject(QListWidgetItem* item);
  void populateCollisionObjectsList();
  void computeImportGeometryFromText(const std::string& path);
  void computeExportGeometryAsText(const std::string& path);
  visualization_msgs::msg::InteractiveMarker
  createObjectMarkerMsg(const collision_detection::CollisionEnv::ObjectConstPtr& obj);

  // Stored scenes tab
  void computeSaveSceneButtonClicked();
  void computeSaveQueryButtonClicked(const std::string& scene, const std::string& query_name);
  void computeLoadSceneButtonClicked();
  void computeLoadQueryButtonClicked();
  void populatePlanningSceneTreeView();
  void computeDeleteSceneButtonClicked();
  void computeDeleteQueryButtonClicked();
  void computeDeleteQueryButtonClickedHelper(QTreeWidgetItem* s);
  void checkPlanningSceneTreeEnabledButtons();

  // States tab
  void saveRobotStateButtonClicked(const moveit::core::RobotState& state);
  void populateRobotStatesList();

  // Pick and place
  void processDetectedObjects();
  void updateDetectedObjectsList(const std::vector<std::string>& object_ids);
  void publishTables();
  void updateSupportSurfacesList();

  std::map<std::string, std::string> pick_object_name_;
  std::string place_object_name_;
  std::vector<geometry_msgs::msg::PoseStamped> place_poses_;
  // void pickObject();
  // void placeObject();
  void triggerObjectDetection();
  void updateTables();
  std::string support_surface_name_;
  // For coloring
  std::string selected_object_name_;
  std::string selected_support_surface_name_;

  rclcpp_action::Client<object_recognition_msgs::action::ObjectRecognition>::SharedPtr object_recognition_client_;
  void listenDetectedObjects(const object_recognition_msgs::msg::RecognizedObjectArray::ConstSharedPtr msg);
  rclcpp::Subscription<object_recognition_msgs::msg::RecognizedObjectArray>::SharedPtr object_recognition_subscriber_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr plan_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr execute_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr update_start_state_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr update_goal_state_subscriber_;
  rclcpp::Subscription<moveit_msgs::msg::RobotState>::SharedPtr update_custom_start_state_subscriber_;
  rclcpp::Subscription<moveit_msgs::msg::RobotState>::SharedPtr update_custom_goal_state_subscriber_;

  // General
  void changePlanningGroupHelper();
  shapes::ShapePtr loadMeshResource(const std::string& url);
  void loadStoredStates(const std::string& pattern);

  void remotePlanCallback(const std_msgs::msg::Empty::ConstSharedPtr msg);
  void remoteExecuteCallback(const std_msgs::msg::Empty::ConstSharedPtr msg);
  void remoteStopCallback(const std_msgs::msg::Empty::ConstSharedPtr msg);
  void remoteUpdateStartStateCallback(const std_msgs::msg::Empty::ConstSharedPtr msg);
  void remoteUpdateGoalStateCallback(const std_msgs::msg::Empty::ConstSharedPtr msg);
  void remoteUpdateCustomStartStateCallback(const moveit_msgs::msg::RobotState::ConstSharedPtr msg);
  void remoteUpdateCustomGoalStateCallback(const moveit_msgs::msg::RobotState::ConstSharedPtr msg);

  /* Selects or unselects a item in a list by the item name */
  void setItemSelectionInList(const std::string& item_name, bool selection, QListWidget* list);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr planning_scene_world_publisher_;

  collision_detection::CollisionEnv::ObjectConstPtr scaled_object_;
  moveit::core::FixedTransformsMap scaled_object_subframes_;
  EigenSTL::vector_Isometry3d scaled_object_shape_poses_;

  std::vector<std::pair<std::string, bool> > known_collision_objects_;
  long unsigned int known_collision_objects_version_;
  bool first_time_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_service_client_;
};
}  // namespace moveit_rviz_plugin
