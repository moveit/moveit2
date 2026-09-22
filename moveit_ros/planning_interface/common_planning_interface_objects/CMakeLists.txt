add_library(moveit_common_planning_interface_objects SHARED
            src/common_objects.cpp)
set_target_properties(moveit_common_planning_interface_objects
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_common_planning_interface_objects rclcpp::rclcpp
                      moveit_ros_planning::moveit_ros_planning tf2_ros::tf2_ros)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning_interface)
