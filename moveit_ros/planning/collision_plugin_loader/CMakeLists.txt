add_library(moveit_collision_plugin_loader SHARED
            src/collision_plugin_loader.cpp)
set_target_properties(moveit_collision_plugin_loader
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_collision_plugin_loader PUBLIC moveit_core::moveit_collision_detection
                                        pluginlib::pluginlib rclcpp::rclcpp)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
