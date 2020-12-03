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

set(LIBRARIES
  moveit_hybrid_planning_manager
  moveit_local_planner_component
  moveit_global_planner_component
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
)

include_directories(include)

add_library(moveit_global_planner_component SHARED src/global_planner_component.cpp)
set_target_properties(moveit_global_planner_component PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_global_planner_component ${THIS_PACKAGE_INCLUDE_DEPENDS})

rclcpp_components_register_nodes(moveit_global_planner_component "moveit::hybrid_planning::GlobalPlannerComponent")

add_library(moveit_hybrid_planning_manager SHARED src/hybrid_planning_manager.cpp)
set_target_properties(moveit_hybrid_planning_manager PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_hybrid_planning_manager ${THIS_PACKAGE_INCLUDE_DEPENDS})

rclcpp_components_register_nodes(moveit_hybrid_planning_manager "moveit::hybrid_planning::HybridPlanningManager")

add_library(moveit_local_planner_component SHARED src/local_planner_component.cpp)
set_target_properties(moveit_local_planner_component PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_local_planner_component ${THIS_PACKAGE_INCLUDE_DEPENDS})

rclcpp_components_register_nodes(moveit_local_planner_component "moveit::hybrid_planning::LocalPlannerComponent")

add_executable(dummy_action_client test/dummy_action_client.cpp)
ament_target_dependencies(dummy_action_client ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(dummy_action_client ${LIBRARIES})

install(TARGETS ${LIBRARIES} dummy_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY include DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_LIBRARIES(${LIBRARIES})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()