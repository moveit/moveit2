add_library(
  moveit_robot_state SHARED src/attached_body.cpp src/conversions.cpp
                            src/robot_state.cpp src/cartesian_interpolator.cpp)
target_include_directories(
  moveit_robot_state
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_core>)
set_target_properties(moveit_robot_state PROPERTIES VERSION
                                                    ${${PROJECT_NAME}_VERSION})
target_link_libraries(
  moveit_robot_state
  moveit_robot_model
  moveit_kinematics_base
  moveit_transforms
  eigen_stl_containers::eigen_stl_containers
  urdf::urdf
  ${tf2_geometry_msgs_TARGETS}
  geometric_shapes::geometric_shapes
  urdfdom_headers::urdfdom_headers
  Boost::headers)

install(DIRECTORY include/ DESTINATION include/moveit_core)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_cmake_google_benchmark REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(benchmark REQUIRED)
  find_package(kdl_parser REQUIRED)

  find_package(tf2_geometry_msgs REQUIRED)
  find_package(resource_retriever REQUIRED)

  if(WIN32)
    # TODO add windows paths
  else()
    set(APPEND_LIBRARY_DIRS
        "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  ament_add_gmock(test_robot_state test/robot_state_test.cpp
                  APPEND_LIBRARY_DIRS "${APPEND_LIBRARY_DIRS}")

  target_link_libraries(test_robot_state moveit_test_utils moveit_utils
                        moveit_exceptions moveit_robot_state)

  ament_add_gtest(test_robot_state_complex test/test_kinematic_complex.cpp)
  target_link_libraries(test_robot_state_complex moveit_test_utils moveit_utils
                        moveit_exceptions moveit_robot_state)

  ament_add_gtest(test_aabb test/test_aabb.cpp)
  target_link_libraries(test_aabb moveit_test_utils moveit_utils
                        moveit_exceptions moveit_robot_state)

  ament_add_gtest(test_cartesian_interpolator
                  test/test_cartesian_interpolator.cpp)
  target_link_libraries(test_cartesian_interpolator moveit_test_utils
                        moveit_robot_state moveit_kinematics_base)

  ament_add_google_benchmark(robot_state_benchmark
                             test/robot_state_benchmark.cpp)
  target_link_libraries(
    robot_state_benchmark moveit_robot_model moveit_test_utils
    moveit_robot_state kdl_parser::kdl_parser)
endif()
