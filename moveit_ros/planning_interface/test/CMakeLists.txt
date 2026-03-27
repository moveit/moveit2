if(BUILD_TESTING)
  find_package(moveit_resources REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # TODO(henningkayser): migrate to launch_test find_package(rostest REQUIRED)
  find_package(eigen_conversions REQUIRED)

  # add_executable(test_cleanup test_cleanup.cpp)
  # target_link_libraries(test_cleanup moveit_move_group_interface)

  ament_add_gtest(moveit_cpp_test moveit_cpp_test.test moveit_cpp_test.cpp)
  # ament_target_dependencies(moveit_cpp_test rclcpp)
  target_link_libraries(moveit_cpp_test moveit_cpp)

endif()
