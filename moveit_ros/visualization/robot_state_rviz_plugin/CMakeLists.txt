add_library(moveit_robot_state_rviz_plugin_core SHARED
  src/robot_state_display.cpp
  include/moveit/robot_state_rviz_plugin/robot_state_display.h)
set_target_properties(moveit_robot_state_rviz_plugin_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_robot_state_rviz_plugin_core
    moveit_rviz_plugin_render_tools
)
ament_target_dependencies(moveit_robot_state_rviz_plugin_core
    rclcpp
    rviz2
    moveit_ros_planning
    moveit_msgs
    pluginlib
    Boost
    rviz_ogre_vendor
)
target_include_directories(moveit_robot_state_rviz_plugin_core PRIVATE "${OGRE_PREFIX_DIR}/include")

add_library(moveit_robot_state_rviz_plugin SHARED src/plugin_init.cpp)
set_target_properties(moveit_robot_state_rviz_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_robot_state_rviz_plugin moveit_robot_state_rviz_plugin_core)
ament_target_dependencies(moveit_robot_state_rviz_plugin
    rclcpp
    pluginlib
    Boost
    rviz_ogre_vendor
)
target_include_directories(moveit_robot_state_rviz_plugin PRIVATE "${OGRE_PREFIX_DIR}/include")

install(DIRECTORY include/ DESTINATION include/moveit_ros_visualization)
