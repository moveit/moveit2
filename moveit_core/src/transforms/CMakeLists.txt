target_sources(moveit_core PRIVATE transforms.cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_transforms test/test_transforms.cpp)
  target_link_libraries(test_transforms moveit::core)
endif()
