add_library(moveit_collision_detection_fcl SHARED
  src/collision_common.cpp
  src/collision_env_fcl.cpp
)
target_include_directories(moveit_collision_detection_fcl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
include(GenerateExportHeader)
generate_export_header(moveit_collision_detection_fcl)
target_include_directories(moveit_collision_detection_fcl PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_collision_detection_fcl PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_collision_detection_fcl
  rclcpp
  rmw_implementation
  urdf
  urdfdom
  urdfdom_headers
  visualization_msgs
)
target_link_libraries(moveit_collision_detection_fcl
  moveit_collision_detection
  moveit_utils
  fcl
)

add_library(collision_detector_fcl_plugin SHARED src/collision_detector_fcl_plugin_loader.cpp)
set_target_properties(collision_detector_fcl_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(collision_detector_fcl_plugin
  rclcpp
  urdf
  visualization_msgs
  pluginlib
  rmw_implementation
)
target_link_libraries(collision_detector_fcl_plugin
  moveit_collision_detection_fcl
  moveit_planning_scene
  moveit_utils
)

install(DIRECTORY include/ DESTINATION include/moveit_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_collision_detection_fcl_export.h DESTINATION include/moveit_core)

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
  target_link_libraries(test_fcl_collision_env moveit_test_utils moveit_collision_detection_fcl)

  ament_add_gtest(test_fcl_collision_detection test/test_fcl_collision_detection_pr2.cpp)
  target_link_libraries(test_fcl_collision_detection moveit_test_utils moveit_collision_detection_fcl)

  ament_add_gtest(test_fcl_collision_detection_panda test/test_fcl_collision_detection_panda.cpp)
  target_link_libraries(test_fcl_collision_detection_panda moveit_test_utils moveit_collision_detection_fcl)
endif()
