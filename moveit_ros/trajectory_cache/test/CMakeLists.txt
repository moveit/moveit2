if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(warehouse_ros_sqlite REQUIRED)

  # Fixtures.
  add_library(warehouse_fixture SHARED fixtures/warehouse_fixture.cpp)
  target_include_directories(
    warehouse_fixture PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
  ament_target_dependencies(warehouse_fixture ${TRAJECTORY_CACHE_DEPENDENCIES}
                            ament_cmake_gtest warehouse_ros_sqlite)

  add_library(move_group_fixture SHARED fixtures/move_group_fixture.cpp)
  target_include_directories(
    move_group_fixture PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
  ament_target_dependencies(move_group_fixture ${TRAJECTORY_CACHE_DEPENDENCIES}
                            ament_cmake_gtest warehouse_ros_sqlite)

  # Test utils library.
  ament_add_gtest(test_utils utils/test_utils.cpp)
  target_link_libraries(test_utils moveit_ros_trajectory_cache_utils_lib
                        warehouse_fixture)

  ament_add_gtest_executable(test_utils_with_move_group
                             utils/test_utils_with_move_group.cpp)
  target_link_libraries(
    test_utils_with_move_group moveit_ros_trajectory_cache_utils_lib
    move_group_fixture)
  add_ros_test(
    fixtures/gtest_with_move_group.py
    TARGET
    test_utils_with_move_group
    TIMEOUT
    30
    ARGS
    "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}"
    "test_executable:=test_utils_with_move_group")

  # Features ===================================================================

  # Test constant features library.
  ament_add_gtest_executable(
    test_constant_features_with_move_group
    features/test_constant_features_with_move_group.cpp)
  target_link_libraries(
    test_constant_features_with_move_group
    moveit_ros_trajectory_cache_utils_lib move_group_fixture)
  add_ros_test(
    fixtures/gtest_with_move_group.py
    TARGET
    test_constant_features_with_move_group
    TIMEOUT
    30
    ARGS
    "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}"
    "test_executable:=test_constant_features_with_move_group")

  # Test GetCartesianPath::Request features library.
  ament_add_gtest_executable(
    test_get_cartesian_path_request_features_with_move_group
    features/test_get_cartesian_path_request_features_with_move_group.cpp)
  target_link_libraries(
    test_get_cartesian_path_request_features_with_move_group
    moveit_ros_trajectory_cache_features_lib move_group_fixture)
  add_ros_test(
    fixtures/gtest_with_move_group.py
    TARGET
    test_get_cartesian_path_request_features_with_move_group
    TIMEOUT
    30
    ARGS
    "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}"
    "test_executable:=test_get_cartesian_path_request_features_with_move_group")

  # Test MotionPlanRequest features library.
  ament_add_gtest_executable(
    test_motion_plan_request_features_with_move_group
    features/test_motion_plan_request_features_with_move_group.cpp)
  target_link_libraries(
    test_motion_plan_request_features_with_move_group
    moveit_ros_trajectory_cache_features_lib move_group_fixture)
  add_ros_test(
    fixtures/gtest_with_move_group.py
    TARGET
    test_motion_plan_request_features_with_move_group
    TIMEOUT
    30
    ARGS
    "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}"
    "test_executable:=test_motion_plan_request_features_with_move_group")

  # Cache Insert Policies ======================================================

  # Test always_insert_never_prune policies library.
  ament_add_gtest_executable(
    test_always_insert_never_prune_policy_with_move_group
    cache_insert_policies/test_always_insert_never_prune_policy_with_move_group.cpp
  )
  target_link_libraries(
    test_always_insert_never_prune_policy_with_move_group
    moveit_ros_trajectory_cache_cache_insert_policies_lib
    moveit_ros_trajectory_cache_features_lib
    moveit_ros_trajectory_cache_utils_lib
    move_group_fixture)
  add_ros_test(
    fixtures/gtest_with_move_group.py
    TARGET
    test_always_insert_never_prune_policy_with_move_group
    TIMEOUT
    30
    ARGS
    "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}"
    "test_executable:=test_always_insert_never_prune_policy_with_move_group")

  # Test best_seen_execution_time policies library.
  ament_add_gtest_executable(
    test_best_seen_execution_time_policy_with_move_group
    cache_insert_policies/test_best_seen_execution_time_policy_with_move_group.cpp
  )
  target_link_libraries(
    test_best_seen_execution_time_policy_with_move_group
    moveit_ros_trajectory_cache_cache_insert_policies_lib
    moveit_ros_trajectory_cache_features_lib
    moveit_ros_trajectory_cache_utils_lib
    move_group_fixture)
  add_ros_test(
    fixtures/gtest_with_move_group.py
    TARGET
    test_best_seen_execution_time_policy_with_move_group
    TIMEOUT
    30
    ARGS
    "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}"
    "test_executable:=test_best_seen_execution_time_policy_with_move_group")

  # Integration Tests ==========================================================

  # This test executable is run by the pytest_test, since a node is required for
  # testing move groups add_executable(test_trajectory_cache
  # test_trajectory_cache.cpp) target_link_libraries(test_trajectory_cache
  # moveit_ros_trajectory_cache_lib)

  # install(TARGETS test_trajectory_cache RUNTIME DESTINATION
  # lib/${PROJECT_NAME})

  # ament_add_pytest_test( test_trajectory_cache_py "test_trajectory_cache.py"
  # WORKING_DIRECTORY
  # "${CMAKE_CURRENT_BINARY_DIR}")

endif()
