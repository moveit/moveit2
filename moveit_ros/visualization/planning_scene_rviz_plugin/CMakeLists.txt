add_library(
  moveit_planning_scene_rviz_plugin_core SHARED
  src/planning_scene_display.cpp src/background_processing.cpp
  include/moveit/planning_scene_rviz_plugin/planning_scene_display.h)
include(GenerateExportHeader)
generate_export_header(moveit_planning_scene_rviz_plugin_core)
target_include_directories(
  moveit_planning_scene_rviz_plugin_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_planning_scene_rviz_plugin_core
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_planning_scene_rviz_plugin_core
                      moveit_rviz_plugin_render_tools)
ament_target_dependencies(
  moveit_planning_scene_rviz_plugin_core
  rclcpp
  rviz2
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_msgs
  pluginlib
  rviz_ogre_vendor)
target_include_directories(moveit_planning_scene_rviz_plugin_core
                           PRIVATE "${OGRE_PREFIX_DIR}/include")

add_library(moveit_planning_scene_rviz_plugin SHARED src/plugin_init.cpp)
set_target_properties(moveit_planning_scene_rviz_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_planning_scene_rviz_plugin
                      moveit_planning_scene_rviz_plugin_core Qt5::Widgets)
ament_target_dependencies(moveit_planning_scene_rviz_plugin pluginlib
                          rviz_ogre_vendor)
target_include_directories(moveit_planning_scene_rviz_plugin
                           PRIVATE "${OGRE_PREFIX_DIR}/include")

install(DIRECTORY include/ DESTINATION include/moveit_ros_visualization)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/moveit_planning_scene_rviz_plugin_core_export.h
  DESTINATION include/moveit_ros_visualization)
