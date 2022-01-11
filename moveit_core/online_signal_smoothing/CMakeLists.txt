# Base class
set(SMOOTHING_BASE_LIB moveit_smoothing_base)
add_library(${SMOOTHING_BASE_LIB} SHARED
  src/smoothing_base_class.cpp
)
include(GenerateExportHeader)
generate_export_header(${SMOOTHING_BASE_LIB})
target_include_directories(${SMOOTHING_BASE_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)
set_target_properties(${SMOOTHING_BASE_LIB} PROPERTIES VERSION
  "${${PROJECT_NAME}_VERSION}"
)
ament_target_dependencies(${SMOOTHING_BASE_LIB}
  rclcpp
)

# Plugin implementations
set(BUTTERWORTH_FILTER_LIB moveit_butterworth_filter)
add_library(${BUTTERWORTH_FILTER_LIB} SHARED
  src/butterworth_filter.cpp
)
generate_export_header(${BUTTERWORTH_FILTER_LIB})
target_include_directories(${BUTTERWORTH_FILTER_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)
set_target_properties(${BUTTERWORTH_FILTER_LIB} PROPERTIES VERSION
  "${${PROJECT_NAME}_VERSION}"
)
target_link_libraries(${BUTTERWORTH_FILTER_LIB}
  ${SMOOTHING_BASE_LIB}
  moveit_robot_model
)
ament_target_dependencies(${BUTTERWORTH_FILTER_LIB}
  srdfdom  # include dependency from moveit_robot_model
)

# Installation

install(DIRECTORY include/ DESTINATION include)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${SMOOTHING_BASE_LIB}_export.h DESTINATION include)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${BUTTERWORTH_FILTER_LIB}_export.h DESTINATION include)

# Testing

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Lowpass filter unit test
  ament_add_gtest(test_butterworth_filter test/test_butterworth_filter.cpp)
  target_link_libraries(test_butterworth_filter ${BUTTERWORTH_FILTER_LIB})
endif()
