# Base class
target_sources(moveit_core PRIVATE
  src/smoothing_base_class.cpp
)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(moveit_core
  rclcpp
)

# Plugin implementations
target_sources(moveit_core PRIVATE
  src/butterworth_filter.cpp
)
ament_target_dependencies(moveit_core
  srdfdom 
)

# Installation

install(DIRECTORY include/ DESTINATION include/moveit_core)

# Testing

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Lowpass filter unit test
  ament_add_gtest(test_butterworth_filter test/test_butterworth_filter.cpp)
  target_link_libraries(test_butterworth_filter moveit_core)
endif()
