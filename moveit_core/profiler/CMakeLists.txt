set(MOVEIT_LIB_NAME moveit_profiler)

add_library(${MOVEIT_LIB_NAME} SHARED src/profiler.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  rmw_implementation
  urdfdom
  urdfdom_headers
  Boost
  random_numbers
)
target_link_libraries(${MOVEIT_LIB_NAME} Boost::thread)

install(DIRECTORY include/ DESTINATION include)
