target_sources(moveit_core PRIVATE src/transforms.cpp)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(DIRECTORY include/ DESTINATION include/moveit_core)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_transforms test/test_transforms.cpp)
  target_link_libraries(test_transforms moveit::core)
endif()
