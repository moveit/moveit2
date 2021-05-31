cmake_minimum_required(VERSION 3.5)
set(MOVEIT_LIB_NAME moveit_kinematics_base)

add_library(${MOVEIT_LIB_NAME} SHARED src/kinematics_base.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_compile_definitions(${MOVEIT_LIB_NAME} PRIVATE "MOVEIT_KINEMATICS_BASE_BUILDING_DLL")

# This line is needed to ensure that messages are done being built before this is built
ament_target_dependencies(
  ${MOVEIT_LIB_NAME}
  rclcpp
  urdf
  urdfdom_headers
  srdfdom moveit_msgs
  geometric_shapes
  geometry_msgs
  Boost
)

install(DIRECTORY include/ DESTINATION include)
