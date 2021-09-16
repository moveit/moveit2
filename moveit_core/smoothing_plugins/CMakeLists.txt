# Plugin implementations

add_library(moveit_butterworth_filter SHARED
  src/butterworth_filter.cpp
)
include(GenerateExportHeader)
generate_export_header(moveit_butterworth_filter)
target_include_directories(moveit_butterworth_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)
set_target_properties(moveit_butterworth_filter PROPERTIES VERSION
  "${${PROJECT_NAME}_VERSION}"
)
target_link_libraries(moveit_butterworth_filter
  moveit_robot_model
  ruckig::ruckig
)

add_library(moveit_ruckig_filter SHARED
  src/ruckig_filter.cpp
)
include(GenerateExportHeader)
generate_export_header(moveit_ruckig_filter)
target_include_directories(moveit_ruckig_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)
set_target_properties(moveit_ruckig_filter PROPERTIES VERSION
  "${${PROJECT_NAME}_VERSION}"
)
target_link_libraries(moveit_ruckig_filter
  moveit_robot_model
)

# Installation

install(DIRECTORY include/ DESTINATION include)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_butterworth_filter_export.h DESTINATION include)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_ruckig_filter_export.h DESTINATION include)
install(TARGETS moveit_butterworth_filter EXPORT moveit_butterworth_filter)
install(TARGETS moveit_ruckig_filter EXPORT moveit_ruckig_filter)
install(
  TARGETS moveit_butterworth_filter EXPORT moveit_butterworth_filter
  TARGETS moveit_ruckig_filter EXPORT moveit_ruckig_filter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Testing

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  # Lowpass filter unit test
  ament_add_gtest(test_butterworth_filter test/test_butterworth_filter.cpp)
  target_link_libraries(test_butterworth_filter moveit_butterworth_filter)
endif()
