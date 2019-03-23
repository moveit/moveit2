set(MOVEIT_LIB_NAME moveit_occupancy_map_monitor)

add_library(${MOVEIT_LIB_NAME}
  src/occupancy_map_monitor.cpp
  src/occupancy_map_updater.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
    ${rclcpp_LIBRARIES}
    ${Boost_LIBRARIES}
    ${moveit_msgs_LIBRARIES}
    pluginlib)

# add_executable(moveit_occupancy_map_server src/occupancy_map_server.cpp)
# ament_target_dependencies(moveit_occupancy_map_server
#     ${MOVEIT_LIB_NAME}
#     ${rclcpp_LIBRARIES}
#     pluginlib
#     tf2_ros
#     octomap_msgs
#     octomap
#     Boost
#     ${Boost_LIBRARIES}
#     ${tf2_ros_LIBRARIES}
#     ${OCTOMAP_LIBRARIES}
#     ${octomap_msgs_LIBRARIES}
#     ${pluginlib_LIBRARIES}
#   )

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
