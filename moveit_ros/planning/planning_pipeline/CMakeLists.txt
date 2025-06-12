generate_parameter_library(planning_pipeline_parameters
                           res/planning_pipeline_parameters.yaml)

add_library(moveit_planning_pipeline SHARED src/planning_pipeline.cpp)
target_link_libraries(
  moveit_planning_pipeline
  PUBLIC planning_pipeline_parameters
         ${moveit_msgs_TARGETS}
         moveit_core::moveit_collision_detection
         moveit_core::moveit_planning_interface
         moveit_core::moveit_planning_scene
         pluginlib::pluginlib
         rclcpp::rclcpp)
include(GenerateExportHeader)
generate_export_header(moveit_planning_pipeline)
target_include_directories(
  moveit_planning_pipeline
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_planning_pipeline
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  add_library(moveit_pipeline_test_plugins SHARED
              test/planning_pipeline_test_plugins.cpp)
  target_include_directories(
    moveit_pipeline_test_plugins
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
  target_link_libraries(
    moveit_pipeline_test_plugins PUBLIC moveit_core::moveit_collision_detection
                                        pluginlib::pluginlib rclcpp::rclcpp)
  set_target_properties(moveit_pipeline_test_plugins
                        PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

  ament_add_gtest(
    moveit_planning_pipeline_test test/planning_pipeline_tests.cpp
    APPEND_LIBRARY_DIRS "${APPEND_LIBRARY_DIRS}")
  target_include_directories(
    moveit_planning_pipeline_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
  target_link_libraries(
    moveit_planning_pipeline_test
    class_loader::class_loader
    moveit_planning_pipeline
    ${moveit_msgs_TARGETS}
    moveit_core::moveit_collision_detection
    moveit_core::moveit_utils
    moveit_core::moveit_test_utils
    moveit_core::moveit_planning_interface
    moveit_core::moveit_planning_scene
    pluginlib::pluginlib
    rclcpp::rclcpp)

  install(
    TARGETS moveit_pipeline_test_plugins
    EXPORT moveit_pipeline_test_pluginsTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
endif()

install(DIRECTORY include/ DESTINATION include/moveit_ros_planning)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_planning_pipeline_export.h
        DESTINATION include/moveit_ros_planning)
