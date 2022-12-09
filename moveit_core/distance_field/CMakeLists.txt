target_sources(moveit_core PRIVATE
  src/distance_field.cpp
  src/find_internal_points.cpp
  src/propagation_distance_field.cpp
)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(DIRECTORY include/ DESTINATION include/moveit_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_voxel_grid test/test_voxel_grid.cpp)
  target_link_libraries(test_voxel_grid moveit::core)

  ament_add_gtest(test_distance_field test/test_distance_field.cpp)
  target_link_libraries(test_distance_field moveit::core)
endif()
