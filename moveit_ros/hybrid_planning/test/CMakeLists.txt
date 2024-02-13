add_executable(cancel_action cancel_action.cpp)
ament_target_dependencies(cancel_action PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem)

  # TODO (vatanaksoytezer / andyze: Flaky behaviour, investigate and re-enable
  # this test asap) Basic integration tests
  # ament_add_gtest_executable(test_basic_integration test_basic_integration.cpp
  # ) ament_target_dependencies(test_basic_integration
  # ${THIS_PACKAGE_INCLUDE_DEPENDS})
  # add_ros_test(launch/test_basic_integration.test.py TIMEOUT 50 ARGS
  # "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

endif()
