set(MOVEIT_LIB_NAME moveit_collision_detection_fcl)

add_library(${MOVEIT_LIB_NAME}
  src/collision_common.cpp
  src/collision_robot_fcl.cpp
  src/collision_world_fcl.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_collision_detection
  rclcpp
  rmw_implementation
  urdf
  urdfdom
  urdfdom_headers
  # LIBFCL
  Boost
  visualization_msgs)

add_library(collision_detector_fcl_plugin src/collision_detector_fcl_plugin_loader.cpp)
set_target_properties(collision_detector_fcl_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(collision_detector_fcl_plugin
  rclcpp
  urdf
  visualization_msgs
  pluginlib
  rmw_implementation
  ${MOVEIT_LIB_NAME})

install(TARGETS ${MOVEIT_LIB_NAME} collision_detector_fcl_plugin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

install(FILES ../collision_detector_fcl_description.xml DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(moveit_resources REQUIRED)
#   include_directories(${moveit_resources_INCLUDE_DIRS})
#   ament_add_gtest(test_fcl_collision_detection test/test_fcl_collision_detection.cpp)
#   ament_target_dependencies(test_fcl_collision_detection
#     urdf
#     visualization_msgs
#     moveit_test_utils
#     ${MOVEIT_LIB_NAME}
#     Boost)
# endif()
