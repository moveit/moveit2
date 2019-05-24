set(MOVEIT_LIB_NAME moveit_planning_request_adapter)

add_library(${MOVEIT_LIB_NAME} src/planning_request_adapter.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  "moveit_planning_scene"
  "rclcpp"
  "rmw_implementation"
  "urdf"
  "urdfdom"
  "urdfdom_headers"
  "visualization_msgs"
)

install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)
