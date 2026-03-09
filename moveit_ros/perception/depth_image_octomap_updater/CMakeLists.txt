add_library(moveit_depth_image_octomap_updater_core SHARED
            src/depth_image_octomap_updater.cpp)
set_target_properties(moveit_depth_image_octomap_updater_core
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_depth_image_octomap_updater_core
  moveit_lazy_free_space_updater
  moveit_mesh_filter
  rclcpp::rclcpp
  moveit_core::moveit_core
  image_transport::image_transport
  ${sensor_msgs_TARGETS}
  tf2::tf2
  ${tf2_geometry_msgs_TARGETS}
  geometric_shapes::geometric_shapes
  moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor)

add_library(moveit_depth_image_octomap_updater SHARED src/updater_plugin.cpp)
set_target_properties(moveit_depth_image_octomap_updater
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_depth_image_octomap_updater
  moveit_depth_image_octomap_updater_core
  rclcpp::rclcpp
  moveit_core::moveit_core
  image_transport::image_transport
  ${sensor_msgs_TARGETS}
  tf2::tf2
  ${tf2_geometry_msgs_TARGETS}
  geometric_shapes::geometric_shapes
  moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
  pluginlib::pluginlib)

install(DIRECTORY include/ DESTINATION include/moveit_ros_perception)
