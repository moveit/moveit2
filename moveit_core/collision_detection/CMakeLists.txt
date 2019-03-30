set(MOVEIT_LIB_NAME moveit_collision_detection)

add_library(${MOVEIT_LIB_NAME}
  src/allvalid/collision_robot_allvalid.cpp
  src/allvalid/collision_world_allvalid.cpp
  src/collision_matrix.cpp
  src/collision_octomap_filter.cpp
  src/collision_robot.cpp
  src/collision_tools.cpp
  src/collision_world.cpp
  src/world.cpp
  src/world_diff.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_robot_state
  rclcpp
  rmw_implementation
  urdf
  urdfdom
  urdfdom_headers
  visualization_msgs
  tf2_eigen
  Boost)

# # unit tests
# if(BUILD_TESTING)
#   ament_add_gtest(test_world test/test_world.cpp)
#   ament_target_dependencies(test_world
#     ${MOVEIT_LIB_NAME}
#     rclcpp
#     rmw_implementation
#     urdfdom
#     urdfdom_headers
#     Boost)
#
#   ament_add_gtest(test_world_diff test/test_world_diff.cpp)
#   ament_target_dependencies(test_world_diff
#     ${MOVEIT_LIB_NAME}
#     rclcpp
#     rmw_implementation
#     urdfdom
#     urdfdom_headers
#     Boost)
#
#   ament_add_gtest(test_all_valid test/test_all_valid.cpp)
#   ament_target_dependencies(test_all_valid
#     ${MOVEIT_LIB_NAME}
#     rclcpp
#     rmw_implementation
#     urdf
#     urdfdom
#     urdfdom_headers
#     visualization_msgs
#     Boost)
# endif()

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)
