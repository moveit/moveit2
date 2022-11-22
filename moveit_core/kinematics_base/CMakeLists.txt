cmake_minimum_required(VERSION 3.22)

add_library(moveit_kinematics_base SHARED src/kinematics_base.cpp)
include(GenerateExportHeader)
generate_export_header(moveit_kinematics_base)
target_include_directories(moveit_kinematics_base PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)  # for this library

# This line ensures that messages are built before the library is built
ament_target_dependencies(
  moveit_kinematics_base
  rclcpp
  urdf
  urdfdom_headers
  srdfdom moveit_msgs
  geometric_shapes
  geometry_msgs
)

install(DIRECTORY include/ DESTINATION include)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_kinematics_base_export.h DESTINATION include)
