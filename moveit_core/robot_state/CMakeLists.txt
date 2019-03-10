set(MOVEIT_LIB_NAME moveit_robot_state)

add_library(${MOVEIT_LIB_NAME}
  src/attached_body.cpp
  src/conversions.cpp
  src/robot_state.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_model moveit_kinematics_base moveit_transforms ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_robot_model
  moveit_kinematics_base
  moveit_transforms
  urdf
  tf2_geometry_msgs
)

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# # Unit tests
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(moveit_resources REQUIRED)
#
#   include_directories(${moveit_resources_INCLUDE_DIRS})
#
#   ament_add_gtest(test_robot_state test/robot_state_test.cpp)
#   ament_target_dependencies(test_robot_state
#     tf2_geometry_msgs
#     ${MOVEIT_LIB_NAME}
#     urdf
#     visualization_msgs
#   )
#
#   # As an executable, this benchmark is not run as a test by default
#   add_executable(robot_state_benchmark test/robot_state_benchmark.cpp)
#   ament_target_dependencies(robot_state_benchmark
#     moveit_test_utils
#     tf2_geometry_msgs
#     ${MOVEIT_LIB_NAME}
#     urdf
#     visualization_msgs
#   )
#
#   ament_add_gtest(test_robot_state_complex test/test_kinematic_complex.cpp)
#   ament_target_dependencies(test_robot_state_complex
#     moveit_test_utils
#     ${MOVEIT_LIB_NAME}
#     urdf
#     visualization_msgs
#   )
#
#   ament_add_gtest(test_aabb test/test_aabb.cpp)
#   ament_target_dependencies(test_aabb
#     moveit_test_utils
#     ${MOVEIT_LIB_NAME}
#     urdf
#     visualization_msgs
#     tf2_geometry_msgs
#   )
# endif()
