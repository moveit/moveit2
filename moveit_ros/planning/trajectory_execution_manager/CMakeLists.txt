add_library(moveit_trajectory_execution_manager SHARED
            src/trajectory_execution_manager.cpp)
include(GenerateExportHeader)
generate_export_header(moveit_trajectory_execution_manager)
target_include_directories(
  moveit_trajectory_execution_manager
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_trajectory_execution_manager
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_trajectory_execution_manager
  moveit_planning_scene_monitor
  moveit_core::moveit_core
  moveit_ros_occupancy_map_monitor::moveit_ros_occupancy_map_monitor
  rclcpp::rclcpp
  pluginlib::pluginlib
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  tf2_eigen::tf2_eigen)

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_trajectory_execution_manager_export.h
  DESTINATION include/moveit_ros_planning)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  add_library(test_trajectory_controller_manager_plugin SHARED
              test/test_trajectory_controller_manager_plugin.cpp)
  target_link_libraries(
    test_trajectory_controller_manager_plugin moveit_core::moveit_core
    rclcpp::rclcpp pluginlib::pluginlib ${moveit_msgs_TARGETS})
  install(TARGETS test_trajectory_controller_manager_plugin
          LIBRARY DESTINATION lib)

  ament_add_gtest(test_trajectory_distribution
                  test/test_trajectory_distribution.cpp)
  target_link_libraries(
    test_trajectory_distribution moveit_trajectory_execution_manager
    moveit_core::moveit_core rclcpp::rclcpp pluginlib::pluginlib
    ${moveit_msgs_TARGETS})
endif()
