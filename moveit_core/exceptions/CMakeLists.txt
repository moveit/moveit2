set(MOVEIT_LIB_NAME moveit_exceptions)

add_library(${MOVEIT_LIB_NAME} SHARED src/exceptions.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  Boost
  rclcpp
  urdfdom
  urdfdom_headers
)

install(DIRECTORY include/ DESTINATION include)
