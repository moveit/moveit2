add_library(moveit_exceptions SHARED src/exceptions.cpp)
target_include_directories(moveit_exceptions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
set_target_properties(moveit_exceptions PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_exceptions
  Boost
  rclcpp
  urdfdom
  urdfdom_headers
)

install(DIRECTORY include/ DESTINATION include/moveit_core)
