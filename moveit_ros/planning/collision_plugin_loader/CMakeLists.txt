add_library(moveit_collision_plugin_loader SHARED
            src/collision_plugin_loader.cpp)
set_target_properties(moveit_collision_plugin_loader
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_collision_plugin_loader moveit_core rclcpp
                          pluginlib)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
