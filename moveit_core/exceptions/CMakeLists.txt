target_sources(moveit_core PRIVATE src/exceptions.cpp)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(moveit_core
  Boost
  rclcpp
  urdfdom
  urdfdom_headers
)

install(DIRECTORY include/ DESTINATION include/moveit_core)
