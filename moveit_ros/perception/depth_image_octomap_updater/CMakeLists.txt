set(MOVEIT_LIB_NAME moveit_depth_image_octomap_updater)

add_library(${MOVEIT_LIB_NAME}_core SHARED src/depth_image_octomap_updater.cpp)
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}_core
  rclcpp
  moveit_core
  image_transport
  sensor_msgs
  tf2
  tf2_geometry_msgs
  geometric_shapes
  moveit_ros_occupancy_map_monitor
)
target_link_libraries(${MOVEIT_LIB_NAME}_core moveit_lazy_free_space_updater moveit_mesh_filter)

add_library(${MOVEIT_LIB_NAME} SHARED src/updater_plugin.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
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
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core)

install(DIRECTORY include/ DESTINATION include)
