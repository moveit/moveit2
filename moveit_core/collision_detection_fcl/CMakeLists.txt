target_sources(moveit_core PRIVATE
  src/collision_common.cpp
  src/collision_env_fcl.cpp
)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(moveit_core
  rclcpp
  rmw_implementation
  urdf
  urdfdom
  urdfdom_headers
  LIBFCL
  visualization_msgs
)

target_sources(moveit_core PRIVATE src/collision_detector_fcl_plugin_loader.cpp)
ament_target_dependencies(moveit_core
  rclcpp
  urdf
  visualization_msgs
  pluginlib
  rmw_implementation
)

install(DIRECTORY include/ DESTINATION include/moveit_core)

if(BUILD_TESTING)
  if(WIN32)
    # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../collision_detection;${CMAKE_CURRENT_BINARY_DIR}/../robot_state;${CMAKE_CURRENT_BINARY_DIR}/../robot_model;${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  # TODO: remove if transition to gtest's new API TYPED_TEST_SUITE_P is finished
  if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wno-deprecated-declarations)
  endif()

  ament_add_gtest(test_fcl_collision_env test/test_fcl_env.cpp)
  target_link_libraries(test_fcl_collision_env moveit_core)

  ament_add_gtest(test_fcl_collision_detection test/test_fcl_collision_detection_pr2.cpp)
  target_link_libraries(test_fcl_collision_detection moveit_core)

  ament_add_gtest(test_fcl_collision_detection_panda test/test_fcl_collision_detection_panda.cpp)
  target_link_libraries(test_fcl_collision_detection_panda moveit_core)
endif()
