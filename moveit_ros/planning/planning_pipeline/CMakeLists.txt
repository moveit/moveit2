add_library(moveit_planning_pipeline SHARED
  src/planning_pipeline.cpp
  src/planning_pipeline_middleware_handle.cpp
)
include(GenerateExportHeader)
generate_export_header(moveit_planning_pipeline)
target_include_directories(moveit_planning_pipeline PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

set_target_properties(moveit_planning_pipeline PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(moveit_planning_pipeline
  moveit_core
  moveit_msgs
  rclcpp
  Boost
  pluginlib
)

install(DIRECTORY include/ DESTINATION include)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_planning_pipeline_export.h DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(planning_pipeline_tests
    test/planning_pipeline_tests.cpp
  )
  target_link_libraries(planning_pipeline_tests
    moveit_planning_pipeline
  )
endif()
