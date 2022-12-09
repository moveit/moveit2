target_sources(moveit_core PRIVATE
  distance_field.cpp
  find_internal_points.cpp
  propagation_distance_field.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_voxel_grid test/test_voxel_grid.cpp)
  target_link_libraries(test_voxel_grid moveit::core)

  ament_add_gtest(test_distance_field test/test_distance_field.cpp)
  target_link_libraries(test_distance_field moveit::core)
endif()
