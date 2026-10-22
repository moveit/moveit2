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

  set(test_plugin_package moveit_ros_planning_trajectory_test_plugins)
  set(test_plugin_prefix "${CMAKE_CURRENT_BINARY_DIR}/${test_plugin_package}")
  set(test_plugin_share "${test_plugin_prefix}/share/${test_plugin_package}")
  file(MAKE_DIRECTORY
       "${test_plugin_prefix}/share/ament_index/resource_index/packages"
       "${test_plugin_prefix}/share/ament_index/resource_index/moveit_core__pluginlib__plugin"
       "${test_plugin_share}")
  file(WRITE
       "${test_plugin_prefix}/share/ament_index/resource_index/packages/${test_plugin_package}"
       "")
  file(WRITE "${test_plugin_share}/package.xml"
       "<?xml version=\"1.0\"?>\n"
       "<package format=\"3\">\n"
       "  <name>${test_plugin_package}</name>\n"
       "  <version>0.0.0</version>\n"
       "  <description>Isolated controller manager plugin for tests</description>\n"
       "  <maintainer email=\"moveit-users@googlegroups.com\">MoveIt</maintainer>\n"
       "  <license>BSD-3-Clause</license>\n"
       "</package>\n")
  file(WRITE
       "${test_plugin_prefix}/share/ament_index/resource_index/moveit_core__pluginlib__plugin/${test_plugin_package}"
       "share/${test_plugin_package}/test_trajectory_controller_manager_plugin.xml\n")
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/../test_trajectory_controller_manager_plugin.xml"
    "${test_plugin_share}/test_trajectory_controller_manager_plugin.xml"
    COPYONLY)

  add_library(test_trajectory_controller_manager_plugin SHARED
              test/test_trajectory_controller_manager_plugin.cpp)
  set_target_properties(
    test_trajectory_controller_manager_plugin
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${test_plugin_prefix}/lib")
  target_link_libraries(
    test_trajectory_controller_manager_plugin moveit_core::moveit_core
    rclcpp::rclcpp pluginlib::pluginlib ${moveit_msgs_TARGETS})

  ament_add_gtest(
    test_trajectory_distribution test/test_trajectory_distribution.cpp
    APPEND_ENV "AMENT_PREFIX_PATH=${test_plugin_prefix}")
  target_link_libraries(
    test_trajectory_distribution moveit_trajectory_execution_manager
    moveit_core::moveit_core rclcpp::rclcpp pluginlib::pluginlib
    ${moveit_msgs_TARGETS})
endif()
