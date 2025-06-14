add_library(
  moveit_planning_scene_rviz_plugin_core SHARED
  src/planning_scene_display.cpp src/background_processing.cpp
  include/moveit/planning_scene_rviz_plugin/planning_scene_display.hpp)
include(GenerateExportHeader)
generate_export_header(moveit_planning_scene_rviz_plugin_core)
target_include_directories(
  moveit_planning_scene_rviz_plugin_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_planning_scene_rviz_plugin_core
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_planning_scene_rviz_plugin_core
  PRIVATE
    moveit_ros_planning_interface::moveit_common_planning_interface_objects
  PUBLIC moveit_rviz_plugin_render_tools
         ${moveit_msgs_TARGETS}
         moveit_ros_planning::kinematics_parameters
         moveit_ros_planning::moveit_collision_plugin_loader
         moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
         pluginlib::pluginlib
         rclcpp::rclcpp
         rviz_ogre_vendor::OgreMain)
target_include_directories(moveit_planning_scene_rviz_plugin_core
                           PRIVATE "${OGRE_PREFIX_DIR}/include")

add_library(moveit_planning_scene_rviz_plugin SHARED src/plugin_init.cpp)
set_target_properties(moveit_planning_scene_rviz_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_planning_scene_rviz_plugin
  PUBLIC moveit_planning_scene_rviz_plugin_core Qt5::Widgets
         pluginlib::pluginlib rviz_ogre_vendor::OgreMain)
target_include_directories(moveit_planning_scene_rviz_plugin
                           PRIVATE "${OGRE_PREFIX_DIR}/include")

install(DIRECTORY include/ DESTINATION include/moveit_ros_visualization)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/moveit_planning_scene_rviz_plugin_core_export.h
  DESTINATION include/moveit_ros_visualization)
