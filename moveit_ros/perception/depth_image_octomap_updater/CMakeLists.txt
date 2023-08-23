add_library(moveit_depth_image_octomap_updater_core SHARED src/depth_image_octomap_updater.cpp)
set_target_properties(moveit_depth_image_octomap_updater_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_depth_image_octomap_updater_core
  rclcpp
  moveit_core
  image_transport
  sensor_msgs
  tf2
  tf2_geometry_msgs
  geometric_shapes
  moveit_ros_occupancy_map_monitor
)
target_link_libraries(moveit_depth_image_octomap_updater_core moveit_lazy_free_space_updater moveit_mesh_filter)

add_library(moveit_depth_image_octomap_updater SHARED src/updater_plugin.cpp)
set_target_properties(moveit_depth_image_octomap_updater PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_depth_image_octomap_updater
  rclcpp
  moveit_core
  image_transport
  sensor_msgs
  tf2
  tf2_geometry_msgs
  geometric_shapes
  moveit_ros_occupancy_map_monitor
  pluginlib
)
target_link_libraries(moveit_depth_image_octomap_updater moveit_depth_image_octomap_updater_core)

install(DIRECTORY include/ DESTINATION include/moveit_ros_perception)
