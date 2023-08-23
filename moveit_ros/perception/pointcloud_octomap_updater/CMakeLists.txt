add_library(moveit_pointcloud_octomap_updater_core SHARED src/pointcloud_octomap_updater.cpp)
set_target_properties(moveit_pointcloud_octomap_updater_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_pointcloud_octomap_updater_core
  rclcpp
  moveit_core
  tf2_ros
  message_filters
  sensor_msgs
  moveit_ros_occupancy_map_monitor
  tf2_geometry_msgs
  tf2
)
target_link_libraries(moveit_pointcloud_octomap_updater_core moveit_point_containment_filter)
set_target_properties(moveit_pointcloud_octomap_updater_core PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(moveit_pointcloud_octomap_updater_core PROPERTIES LINK_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_library(moveit_pointcloud_octomap_updater SHARED src/plugin_init.cpp)
set_target_properties(moveit_pointcloud_octomap_updater PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_pointcloud_octomap_updater
  rclcpp
  moveit_core
  tf2_ros
  message_filters
  sensor_msgs
  moveit_ros_occupancy_map_monitor
  tf2_geometry_msgs
  tf2
  pluginlib
)
target_link_libraries(moveit_pointcloud_octomap_updater moveit_pointcloud_octomap_updater_core)

install(DIRECTORY include/ DESTINATION include/moveit_ros_perception)
