# Base class
add_library(moveit_smoothing_base SHARED
  src/smoothing_base_class.cpp
)
target_include_directories(moveit_smoothing_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include/moveit_core>
)
target_link_libraries(moveit_smoothing_base moveit_macros)
include(GenerateExportHeader)
generate_export_header(moveit_smoothing_base)
set_target_properties(moveit_smoothing_base PROPERTIES VERSION
  "${${PROJECT_NAME}_VERSION}"
)
ament_target_dependencies(moveit_smoothing_base
  rclcpp
)

# Plugin implementations
add_library(moveit_butterworth_filter SHARED
  src/butterworth_filter.cpp
)
generate_export_header(moveit_butterworth_filter)
target_include_directories(moveit_butterworth_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)
set_target_properties(moveit_butterworth_filter PROPERTIES VERSION
  "${${PROJECT_NAME}_VERSION}"
)
target_link_libraries(moveit_butterworth_filter
  moveit_smoothing_base
  moveit_robot_model
)
ament_target_dependencies(moveit_butterworth_filter
  srdfdom  # include dependency from moveit_robot_model
)

# Installation

install(DIRECTORY include/ DESTINATION include/moveit_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_smoothing_base_export.h DESTINATION include/moveit_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_butterworth_filter_export.h DESTINATION include/moveit_core)

# Testing

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Lowpass filter unit test
  ament_add_gtest(test_butterworth_filter test/test_butterworth_filter.cpp)
  target_link_libraries(test_butterworth_filter moveit_butterworth_filter)
endif()
