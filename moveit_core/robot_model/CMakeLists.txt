cmake_minimum_required(VERSION 3.22)

add_library(moveit_robot_model SHARED
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
set_target_properties(moveit_robot_model PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_robot_model
  angles
  moveit_msgs
  Eigen3
  geometric_shapes
  urdf
  urdfdom_headers
  srdfdom
  visualization_msgs
)
target_link_libraries(moveit_robot_model
  moveit_exceptions
  moveit_kinematics_base
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_robot_model test/test.cpp)
  ament_target_dependencies(test_robot_model
    rclcpp
  )
  target_link_libraries(test_robot_model moveit_test_utils moveit_robot_model)
endif()

install(DIRECTORY include/ DESTINATION include)
