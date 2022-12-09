target_sources(moveit_core PRIVATE src/kinematics_base.cpp)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# This line ensures that messages are built before the library is built
ament_target_dependencies(moveit_core
  rclcpp
  urdf
  urdfdom_headers
  srdfdom moveit_msgs
  geometric_shapes
  geometry_msgs
)

install(DIRECTORY include/ DESTINATION include/moveit_core)
