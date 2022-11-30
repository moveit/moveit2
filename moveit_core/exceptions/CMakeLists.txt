add_library(moveit_exceptions SHARED src/exceptions.cpp)
set_target_properties(moveit_exceptions PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_exceptions
  Boost
  rclcpp
  urdfdom
  urdfdom_headers
)

install(DIRECTORY include/ DESTINATION include)
