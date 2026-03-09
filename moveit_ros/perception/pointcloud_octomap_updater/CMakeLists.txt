add_library(moveit_pointcloud_octomap_updater_core SHARED
            src/pointcloud_octomap_updater.cpp)
set_target_properties(moveit_pointcloud_octomap_updater_core
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_pointcloud_octomap_updater_core
  moveit_point_containment_filter
  rclcpp::rclcpp
  moveit_core::moveit_core
  tf2_ros::tf2_ros
  message_filters::message_filters
  ${sensor_msgs_TARGETS}
  moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
  ${tf2_geometry_msgs_TARGETS}
  tf2::tf2)
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
  moveit_pointcloud_octomap_updater_core
  rclcpp::rclcpp
  moveit_core::moveit_core
  tf2_ros::tf2_ros
  message_filters::message_filters
  ${sensor_msgs_TARGETS}
  moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
  ${tf2_geometry_msgs_TARGETS}
  tf2::tf2
  pluginlib::pluginlib)

install(DIRECTORY include/ DESTINATION include/moveit_ros_perception)
