add_library(moveit_pointcloud_octomap_updater_core SHARED
            src/pointcloud_octomap_updater.cpp)
set_target_properties(moveit_pointcloud_octomap_updater_core
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_pointcloud_octomap_updater_core
  PUBLIC moveit_point_containment_filter
         ${sensor_msgs_TARGETS}
         message_filters::message_filters
         moveit_core::moveit_collision_detection
         moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
         rclcpp::rclcpp
         sensor_msgs::sensor_msgs_library
         tf2::tf2
         tf2_geometry_msgs::tf2_geometry_msgs
         tf2_ros::tf2_ros)
set_target_properties(
  moveit_pointcloud_octomap_updater_core
  PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(
  moveit_pointcloud_octomap_updater_core
  PROPERTIES LINK_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_library(moveit_pointcloud_octomap_updater SHARED src/plugin_init.cpp)
set_target_properties(moveit_pointcloud_octomap_updater
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_pointcloud_octomap_updater
  PUBLIC moveit_pointcloud_octomap_updater_core
         ${sensor_msgs_TARGETS}
         message_filters::message_filters
         moveit_core::moveit_collision_detection
         moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
         pluginlib::pluginlib
         rclcpp::rclcpp
         sensor_msgs::sensor_msgs_library
         tf2::tf2
         tf2_geometry_msgs::tf2_geometry_msgs
         tf2_ros::tf2_ros)

install(DIRECTORY include/ DESTINATION include/moveit_ros_perception)
