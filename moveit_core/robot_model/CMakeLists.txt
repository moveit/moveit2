cmake_minimum_required(VERSION 3.5)
set(MOVEIT_LIB_NAME moveit_robot_model)

add_library(${MOVEIT_LIB_NAME}
  src/aabb.cpp
  src/fixed_joint_model.cpp
  src/floating_joint_model.cpp
  src/joint_model.cpp
  src/joint_model_group.cpp
  src/link_model.cpp
  src/planar_joint_model.cpp
  src/prismatic_joint_model.cpp
  src/revolute_joint_model.cpp
  src/robot_model.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  urdf
  urdfdom_headers
  visualization_msgs
  moveit_profiler
  moveit_exceptions
  moveit_kinematics_base)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(moveit_resources REQUIRED)
#   find_package(moveit_core REQUIRED)
#   find_package(geometric_shapes REQUIRED)
#   include_directories(${moveit_resources_INCLUDE_DIRS})
#
#   ament_add_gtest(test_robot_model test/test.cpp)
#   ament_target_dependencies(test_robot_model
#     ${MOVEIT_LIB_NAME}
#     ${Boost_LIBRARIES}
#     ${rclcpp_LIBRARIES}
#     ${rmw_implementation_LIBRARIES}
#     ${urdfdom_LIBRARIES}
#     ${urdfdom_headers_LIBRARIES}
#     moveit_test_utils
#     random_numbers
#     srdf
#     urdf
#     geometric_shapes)
# endif()

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
