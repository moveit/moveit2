# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set(HEADERS include/moveit/trajectory_rviz_plugin/trajectory_display.hpp)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Trajectory Display
add_library(moveit_trajectory_rviz_plugin_core SHARED src/trajectory_display.cpp
                                                      ${HEADERS})
set_target_properties(moveit_trajectory_rviz_plugin_core
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_trajectory_rviz_plugin_core
  PUBLIC moveit_rviz_plugin_render_tools
         moveit_planning_scene_rviz_plugin_core
         rviz_ogre_vendor::OgreMain
         ${moveit_msgs_TARGETS}
         pluginlib::pluginlib
         rclcpp::rclcpp
         rviz_ogre_vendor::OgreMain)

target_include_directories(moveit_trajectory_rviz_plugin_core
                           PRIVATE "${OGRE_PREFIX_DIR}/include")

# Plugin Initializer
add_library(moveit_trajectory_rviz_plugin SHARED src/plugin_init.cpp)
set_target_properties(moveit_trajectory_rviz_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_trajectory_rviz_plugin
  PUBLIC moveit_trajectory_rviz_plugin_core pluginlib::pluginlib rclcpp::rclcpp
         rviz_ogre_vendor::OgreMain)
target_include_directories(moveit_trajectory_rviz_plugin
                           PRIVATE "${OGRE_PREFIX_DIR}/include")

install(DIRECTORY include/ DESTINATION include/moveit_ros_visualization)
