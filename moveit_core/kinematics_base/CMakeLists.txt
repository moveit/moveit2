add_library(moveit_kinematics_base SHARED src/kinematics_base.cpp)
target_include_directories(
  moveit_kinematics_base
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_core>)
target_link_libraries(moveit_kinematics_base moveit_macros moveit_robot_model
                      moveit_utils)
include(GenerateExportHeader)
generate_export_header(moveit_kinematics_base)
target_include_directories(
  moveit_kinematics_base PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)# for this library

# This line ensures that messages are built before the library is built
ament_target_dependencies(
  moveit_kinematics_base
  rclcpp
  urdf
  urdfdom_headers
  srdfdom
  moveit_msgs
  geometric_shapes
  geometry_msgs)

install(DIRECTORY include/ DESTINATION include/moveit_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_kinematics_base_export.h
        DESTINATION include/moveit_core)
