cmake_minimum_required(VERSION 3.22)
project(moveit_ros_tests LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(ament_cmake_python REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(moveit_ros_planning REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ros_testing REQUIRED)

  include_directories(include)
  ament_add_gtest_executable(move_group_api_test src/move_group_api_test.cpp)
  ament_target_dependencies(move_group_api_test rclcpp)
  add_ros_test(launch/move_group_api.py TIMEOUT 30)

  ament_add_gtest_executable(move_group_default_planning_scene_test
                             src/move_group_default_planning_scene_test.cpp)
  ament_target_dependencies(move_group_default_planning_scene_test rclcpp
                            moveit_ros_planning)
  # run default_planning_scene test isolated test to avoid interference
  ament_add_ros_isolated_pytest_test(
    move_group_default_planning_scene
    launch/move_group_default_planning_scene.py TIMEOUT 30)

  # install Python helper script and test
  ament_python_install_package(moveit_ros_tests)
  install(DIRECTORY data DESTINATION share/${PROJECT_NAME})
  install(TARGETS move_group_api_test move_group_default_planning_scene_test
          RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
