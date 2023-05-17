add_library(moveit_planning_request_adapter SHARED src/planning_request_adapter.cpp)
target_include_directories(moveit_planning_request_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
set_target_properties(moveit_planning_request_adapter PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_planning_request_adapter
  rclcpp
  rmw_implementation
  urdf
  urdfdom
  urdfdom_headers
  visualization_msgs
)
target_link_libraries(moveit_planning_request_adapter
  moveit_planning_scene
  moveit_planning_interface
)

install(DIRECTORY include/ DESTINATION include/moveit_core)
