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

  # Fake controller manager plugin used by the tests below. It is installed
  # so that pluginlib can discover it when the tests run.
  add_library(test_controller_manager_plugin SHARED test/test_moveit_controller_manager_plugin.cpp)
  target_link_libraries(test_controller_manager_plugin moveit_core::moveit_core rclcpp::rclcpp
                        ${moveit_msgs_TARGETS})
  install(TARGETS test_controller_manager_plugin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin)
  ament_add_gtest(test_execution_thread_safety test/test_execution_thread_safety.cpp)
  target_link_libraries(test_execution_thread_safety
                        moveit_trajectory_execution_manager
                        moveit_planning_scene_monitor
                        moveit_core::moveit_core
                        rclcpp::rclcpp
                        tf2_ros::tf2_ros
                        urdf::urdf
                        srdfdom::srdfdom
                        ${moveit_msgs_TARGETS})
endif()
