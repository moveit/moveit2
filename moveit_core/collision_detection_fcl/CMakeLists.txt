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

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

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
