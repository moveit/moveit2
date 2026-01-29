add_library(moveit_trajectory_execution_manager SHARED
            src/trajectory_execution_manager.cpp)
include(GenerateExportHeader)
generate_export_header(moveit_trajectory_execution_manager)
target_include_directories(
  moveit_trajectory_execution_manager
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_trajectory_execution_manager
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(moveit_trajectory_execution_manager
                      moveit_planning_scene_monitor)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_trajectory_execution_manager_export.h
  DESTINATION include/moveit_ros_planning)
