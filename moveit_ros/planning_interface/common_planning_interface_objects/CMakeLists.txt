add_library(moveit_common_planning_interface_objects SHARED
            src/common_objects.cpp)
set_target_properties(moveit_common_planning_interface_objects
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_common_planning_interface_objects rclcpp
                          moveit_ros_planning tf2_ros)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning_interface)
