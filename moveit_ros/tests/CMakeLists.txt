cmake_minimum_required(VERSION 3.22)
project(moveit_ros_tests LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ros_testing REQUIRED)

  include_directories(include)
  ament_add_gtest_executable(move_group_api_test src/move_group_api_test.cpp)
  ament_target_dependencies(move_group_api_test rclcpp)
  add_ros_test(launch/move_group_api.test.py TIMEOUT 30 ARGS
               "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()
