generate_parameter_library(planning_pipeline_parameters
                           res/planning_pipeline_parameters.yaml)

add_library(moveit_planning_pipeline SHARED src/planning_pipeline.cpp)
target_link_libraries(moveit_planning_pipeline planning_pipeline_parameters)
include(GenerateExportHeader)
generate_export_header(moveit_planning_pipeline)
target_include_directories(
  moveit_planning_pipeline
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_planning_pipeline
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(moveit_planning_pipeline moveit_core moveit_msgs
                          rclcpp pluginlib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  add_library(moveit_pipeline_test_plugins SHARED
              test/planning_pipeline_test_plugins.cpp)
  target_include_directories(
    moveit_pipeline_test_plugins
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
  ament_target_dependencies(moveit_pipeline_test_plugins moveit_core rclcpp
                            pluginlib)
  set_target_properties(moveit_pipeline_test_plugins
                        PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

  ament_add_gtest(
    moveit_planning_pipeline_test test/planning_pipeline_tests.cpp
    APPEND_LIBRARY_DIRS "${APPEND_LIBRARY_DIRS}")
  target_include_directories(
    moveit_planning_pipeline_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
  ament_target_dependencies(moveit_planning_pipeline_test moveit_core
                            moveit_msgs rclcpp pluginlib)
  target_link_libraries(moveit_planning_pipeline_test moveit_planning_pipeline)

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
