add_library(moveit_distance_field SHARED
  src/distance_field.cpp
  src/find_internal_points.cpp
  src/propagation_distance_field.cpp
)
target_include_directories(moveit_distance_field PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
target_link_libraries(moveit_distance_field moveit_macros moveit_utils)
set_target_properties(moveit_distance_field PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_distance_field
  Boost
  urdfdom
  urdfdom_headers
  visualization_msgs
  geometric_shapes
  tf2_eigen
  OCTOMAP
)

install(DIRECTORY include/ DESTINATION include/moveit_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_voxel_grid test/test_voxel_grid.cpp)
  target_link_libraries(test_voxel_grid moveit_distance_field)

  ament_add_gtest(test_distance_field test/test_distance_field.cpp)
  target_link_libraries(test_distance_field moveit_distance_field)
endif()
