target_sources(moveit_core PRIVATE
  bullet_integration/bullet_utils.cpp
  bullet_integration/bullet_discrete_bvh_manager.cpp
  bullet_integration/bullet_cast_bvh_manager.cpp
  collision_env_bullet.cpp
  bullet_integration/bullet_bvh_manager.cpp
  bullet_integration/contact_checker_common.cpp
  bullet_integration/ros_bullet_utils.cpp
)
target_sources(moveit_core PRIVATE collision_detector_bullet_plugin_loader.cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  if(WIN32)
    # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../collision_detection;${CMAKE_CURRENT_BINARY_DIR}/../robot_state;${CMAKE_CURRENT_BINARY_DIR}/../robot_model;${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  # TODO: remove if transition to gtest's new API TYPED_TEST_SUITE_P is finished
  if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wno-deprecated-declarations)
  endif()

  ament_add_gtest(test_bullet_collision_detection test/test_bullet_collision_detection_pr2.cpp)
  target_link_libraries(test_bullet_collision_detection moveit::core)

  ament_add_gtest(test_bullet_collision_detection_panda test/test_bullet_collision_detection_panda.cpp)
  target_link_libraries(test_bullet_collision_detection_panda moveit::core)

  ament_add_gtest(test_bullet_continuous_collision_checking test/test_bullet_continuous_collision_checking.cpp)
  target_link_libraries(test_bullet_continuous_collision_checking moveit::core)
endif()
