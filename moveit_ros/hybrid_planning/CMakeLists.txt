cmake_minimum_required(VERSION 3.10.2)
project(moveit_hybrid_planning)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_toolbox REQUIRED)

set(LIBRARIES
  moveit_hybrid_planning_manager
  moveit_local_planner_component
  moveit_global_planner_component
  planner_logic_interface
  single_plan_execution_plugin
  trajectory_operator_interface
  local_constraint_solver_interface
  next_waypoint_sampler_plugin
  single_plan_execution_plugin
  decelerate_before_collision_plugin
  replan_invalidated_trajectory_plugin
  forward_trajectory_plugin
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  rclcpp_action
  moveit_core
  moveit_ros_planning
  moveit_msgs
  std_msgs
  std_srvs
  tf2_ros
  trajectory_msgs
  control_toolbox
)

set(THIS_PACKAGE_INCLUDE_DIRS
  global_planner/include
  hybrid_planning_manager/hybrid_planning_manager_component/include
  hybrid_planning_manager/planner_logic_plugins/include
  local_planner/local_planner_component/include
  local_planner/trajectory_operator_plugins/include
  local_planner/local_constraint_solver_plugins/include
)
include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_subdirectory(hybrid_planning_manager)
add_subdirectory(global_planner)
add_subdirectory(local_planner)
add_subdirectory(test)

rclcpp_components_register_nodes(moveit_hybrid_planning_manager "moveit_hybrid_planning::HybridPlanningManager")
rclcpp_components_register_nodes(moveit_global_planner_component "moveit_hybrid_planning::GlobalPlannerComponent")
rclcpp_components_register_nodes(moveit_local_planner_component "moveit_hybrid_planning::LocalPlannerComponent")

install(TARGETS ${LIBRARIES} hybrid_planning_test_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY ${THIS_PACKAGE_INCLUDE_DIRS} DESTINATION include)

install(DIRECTORY test/launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(moveit_hybrid_planning single_plan_execution_plugin.xml)
pluginlib_export_plugin_description_file(moveit_hybrid_planning next_waypoint_sampler_plugin.xml)
pluginlib_export_plugin_description_file(moveit_hybrid_planning decelerate_before_collision_plugin.xml)
pluginlib_export_plugin_description_file(moveit_hybrid_planning replan_invalidated_trajectory_plugin.xml)
pluginlib_export_plugin_description_file(moveit_hybrid_planning forward_trajectory_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(${LIBRARIES})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
