add_library(
  moveit_collision_detection_bullet SHARED
  src/bullet_integration/bullet_utils.cpp
  src/bullet_integration/bullet_discrete_bvh_manager.cpp
  src/bullet_integration/bullet_cast_bvh_manager.cpp
  src/collision_env_bullet.cpp
  src/bullet_integration/bullet_bvh_manager.cpp
  src/bullet_integration/contact_checker_common.cpp
  src/bullet_integration/ros_bullet_utils.cpp)
target_include_directories(
  moveit_collision_detection_bullet
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_core>)
include(GenerateExportHeader)
generate_export_header(moveit_collision_detection_bullet)
target_include_directories(
  moveit_collision_detection_bullet
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> ${BULLET_INCLUDE_DIRS})
set_target_properties(moveit_collision_detection_bullet
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  moveit_collision_detection_bullet
  PUBLIC ${BULLET_LIBRARIES}
         ${octomap_msgs_TARGETS}
         ${visualization_msgs_TARGETS}
         moveit_collision_detection
         moveit_utils
         rclcpp::rclcpp
         rmw_implementation::rmw_implementation
         urdf::urdf)

add_library(collision_detector_bullet_plugin SHARED
            src/collision_detector_bullet_plugin_loader.cpp)
set_target_properties(collision_detector_bullet_plugin
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(
  collision_detector_bullet_plugin
  PUBLIC ${BULLET_LIBRARIES}
         ${octomap_msgs_TARGETS}
         ${visualization_msgs_TARGETS}
         pluginlib::pluginlib
         moveit_collision_detection_bullet
         moveit_planning_scene
         moveit_utils
         rclcpp::rclcpp
         rmw_implementation::rmw_implementation
         urdf::urdf)

install(DIRECTORY include/ DESTINATION include/moveit_core)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_collision_detection_bullet_export.h
  DESTINATION include/moveit_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  if(WIN32)
    # set(APPEND_LIBRARY_DIRS
    # "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
  else()
    set(APPEND_LIBRARY_DIRS
        "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../collision_detection;${CMAKE_CURRENT_BINARY_DIR}/../robot_state;${CMAKE_CURRENT_BINARY_DIR}/../robot_model;${CMAKE_CURRENT_BINARY_DIR}/../utils"
    )
  endif()

  # TODO: remove if transition to gtest's new API TYPED_TEST_SUITE_P is finished
  if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wno-deprecated-declarations)
  endif()

  ament_add_gtest(test_bullet_collision_detection
                  test/test_bullet_collision_detection_pr2.cpp)
  target_link_libraries(test_bullet_collision_detection moveit_test_utils
                        moveit_collision_detection_bullet)

  ament_add_gtest(test_bullet_collision_detection_panda
                  test/test_bullet_collision_detection_panda.cpp)
  target_link_libraries(test_bullet_collision_detection_panda moveit_test_utils
                        moveit_collision_detection_bullet)

  ament_add_gtest(test_bullet_continuous_collision_checking
                  test/test_bullet_continuous_collision_checking.cpp)
  target_link_libraries(test_bullet_continuous_collision_checking
                        moveit_test_utils moveit_collision_detection_bullet)
endif()
