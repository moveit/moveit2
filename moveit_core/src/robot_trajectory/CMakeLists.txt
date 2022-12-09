target_sources(moveit_core PRIVATE robot_trajectory.cpp)

if(BUILD_TESTING)
  ament_add_gtest(test_robot_trajectory test/test_robot_trajectory.cpp)
  target_link_libraries(test_robot_trajectory moveit::core)
endif()
