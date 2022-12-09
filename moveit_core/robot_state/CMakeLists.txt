target_sources(moveit_core PRIVATE
  src/attached_body.cpp
  src/conversions.cpp
  src/robot_state.cpp
  src/cartesian_interpolator.cpp
)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(moveit_core
  urdf
  tf2_geometry_msgs
  geometric_shapes
  urdfdom_headers
  Boost
)

install(DIRECTORY include/ DESTINATION include/moveit_core)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(resource_retriever REQUIRED)

  if(WIN32)
    # TODO add windows paths
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  ament_add_gtest(test_robot_state test/robot_state_test.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")

  target_link_libraries(test_robot_state moveit_core)

  # As an executable, this benchmark is not run as a test by default
  ament_add_gtest(test_robot_state_benchmark test/robot_state_benchmark.cpp)
  target_link_libraries(test_robot_state_benchmark moveit_core)

  ament_add_gtest(test_robot_state_complex test/test_kinematic_complex.cpp)
  target_link_libraries(test_robot_state_complex moveit_core)

  ament_add_gtest(test_aabb test/test_aabb.cpp)
  target_link_libraries(test_aabb moveit_core)

  ament_add_gtest(test_cartesian_interpolator test/test_cartesian_interpolator.cpp)
  target_link_libraries(test_cartesian_interpolator moveit_core)
endif()
