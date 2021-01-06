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
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(LIBRARIES
  moveit_hybrid_planning_manager
  moveit_local_planner_component
  moveit_global_planner_component
  planner_logic_interface
  single_plan_execution_plugin
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  rclcpp_action
  moveit_core
  moveit_ros_planning
  moveit_msgs
  std_srvs
  tf2_ros
  trajectory_msgs
)

set(THIS_PACKAGE_INCLUDE_DIRS
  global_planner/include
  local_planner/include
  hybrid_planning_manager/include
  planner_logic_plugins/include
)
include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

add_subdirectory(hybrid_planning_manager)
add_subdirectory(global_planner)
add_subdirectory(local_planner)
add_subdirectory(planner_logic_plugins)
add_subdirectory(test)

rclcpp_components_register_nodes(moveit_hybrid_planning_manager "moveit_hybrid_planning::HybridPlanningManager")
rclcpp_components_register_nodes(moveit_global_planner_component "moveit_hybrid_planning::GlobalPlannerComponent")
rclcpp_components_register_nodes(moveit_local_planner_component "moveit_hybrid_planning::LocalPlannerComponent")

install(TARGETS ${LIBRARIES} hybrid_planning_test_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY ${THIS_PACKAGE_INCLUDE_DIRS} DESTINATION include)

install(DIRECTORY test/launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(moveit_hybrid_planning single_plan_execution_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(${LIBRARIES})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
