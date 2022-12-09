target_sources(moveit_core PRIVATE
  src/smoothing_base_class.cpp
  src/butterworth_filter.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_butterworth_filter test/test_butterworth_filter.cpp)
  target_link_libraries(test_butterworth_filter moveit::core)
endif()
