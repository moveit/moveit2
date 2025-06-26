add_library(
  moveit_robot_state_rviz_plugin_core SHARED
  src/robot_state_display.cpp
  include/moveit/robot_state_rviz_plugin/robot_state_display.hpp)
set_target_properties(moveit_robot_state_rviz_plugin_core
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_robot_state_rviz_plugin_core
  PUBLIC moveit_rviz_plugin_render_tools
         ${moveit_msgs_TARGETS}
         moveit_ros_planning::kinematics_parameters
         moveit_ros_planning::moveit_collision_plugin_loader
         moveit_ros_planning::moveit_rdf_loader
         pluginlib::pluginlib
         rclcpp::rclcpp
         rviz_ogre_vendor::OgreMain)
target_include_directories(moveit_robot_state_rviz_plugin_core
                           PRIVATE "${OGRE_PREFIX_DIR}/include")

add_library(moveit_robot_state_rviz_plugin SHARED src/plugin_init.cpp)
set_target_properties(moveit_robot_state_rviz_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_robot_state_rviz_plugin
  PUBLIC moveit_robot_state_rviz_plugin_core pluginlib::pluginlib
         rclcpp::rclcpp rviz_ogre_vendor::OgreMain)
target_include_directories(moveit_robot_state_rviz_plugin
                           PRIVATE "${OGRE_PREFIX_DIR}/include")

install(DIRECTORY include/ DESTINATION include/moveit_ros_visualization)
