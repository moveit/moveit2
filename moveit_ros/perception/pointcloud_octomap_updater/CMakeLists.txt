set(MOVEIT_LIB_NAME moveit_pointcloud_octomap_updater)

add_library(${MOVEIT_LIB_NAME}_core SHARED src/pointcloud_octomap_updater.cpp)
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}_core
  rclcpp
  moveit_core
  tf2_ros
  message_filters
  sensor_msgs
  moveit_ros_occupancy_map_monitor
  tf2_geometry_msgs
  tf2
)
target_link_libraries(${MOVEIT_LIB_NAME}_core moveit_point_containment_filter)
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES LINK_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_library(${MOVEIT_LIB_NAME} SHARED src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
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
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core)

install(DIRECTORY include/ DESTINATION include)
