add_library(moveit_kinematics_base SHARED src/kinematics_base.cpp)
target_include_directories(
  moveit_kinematics_base
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_core>)
target_link_libraries(
  moveit_kinematics_base
  PUBLIC moveit_macros
         moveit_robot_model
         moveit_utils
         ${geometry_msgs_TARGETS}
         ${moveit_msgs_TARGETS}
         geometric_shapes::geometric_shapes
         rclcpp::rclcpp
         srdfdom::srdfdom
         urdf::urdf)
include(GenerateExportHeader)
generate_export_header(moveit_kinematics_base)
target_include_directories(
  moveit_kinematics_base PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)# for this library

# This line ensures that messages are built before the library is built

install(DIRECTORY include/ DESTINATION include/moveit_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_kinematics_base_export.h
        DESTINATION include/moveit_core)
