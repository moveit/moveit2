target_sources(moveit_core PRIVATE src/robot_trajectory.cpp)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(moveit_core
  rclcpp
  urdfdom
  urdfdom_headers
)


install(DIRECTORY include/ DESTINATION include/moveit_core)

if(BUILD_TESTING)
  ament_add_gtest(test_robot_trajectory test/test_robot_trajectory.cpp)
  target_link_libraries(test_robot_trajectory moveit_core)
endif()
