set(MOVEIT_LIB_NAME moveit_collision_detection_bullet)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/bullet_integration/bullet_utils.cpp
  src/bullet_integration/bullet_discrete_bvh_manager.cpp
  src/bullet_integration/bullet_cast_bvh_manager.cpp
  src/collision_env_bullet.cpp
  src/bullet_integration/bullet_bvh_manager.cpp
  src/bullet_integration/contact_checker_common.cpp
  src/bullet_integration/ros_bullet_utils.cpp
)
include(GenerateExportHeader)
generate_export_header(${MOVEIT_LIB_NAME})
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME} SYSTEM
  BULLET
)
ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  rmw_implementation
  urdf
  urdfdom
  urdfdom_headers
  visualization_msgs
  octomap_msgs
)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_collision_detection
)

add_library(collision_detector_bullet_plugin SHARED src/collision_detector_bullet_plugin_loader.cpp)
set_target_properties(collision_detector_bullet_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(collision_detector_bullet_plugin SYSTEM
  BULLET
)
ament_target_dependencies(collision_detector_bullet_plugin
  rclcpp
  urdf
  visualization_msgs
  pluginlib
  rmw_implementation
  octomap_msgs
)
target_link_libraries(collision_detector_bullet_plugin
  ${MOVEIT_LIB_NAME}
  moveit_planning_scene
)

install(DIRECTORY include/ DESTINATION include)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${MOVEIT_LIB_NAME}_export.h DESTINATION include)
install(TARGETS ${MOVEIT_LIB_NAME} collision_detector_bullet_plugin EXPORT export_${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_${MOVEIT_LIB_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  if(WIN32)
    # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../collision_detection;${CMAKE_CURRENT_BINARY_DIR}/../robot_state;${CMAKE_CURRENT_BINARY_DIR}/../robot_model;${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  ament_add_gtest(test_bullet_collision_detection test/test_bullet_collision_detection_pr2.cpp)
  target_link_libraries(test_bullet_collision_detection moveit_test_utils ${MOVEIT_LIB_NAME})
  # TODO: remove if transition to gtest's new API TYPED_TEST_SUITE_P is finished
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(test_bullet_collision_detection PRIVATE -Wno-deprecated-declarations)
  endif()

  ament_add_gtest(test_bullet_collision_detection_panda test/test_bullet_collision_detection_panda.cpp)
  target_link_libraries(test_bullet_collision_detection_panda moveit_test_utils ${MOVEIT_LIB_NAME})
  # TODO: remove if transition to gtest's new API TYPED_TEST_SUITE_P is finished
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(test_bullet_collision_detection_panda PRIVATE -Wno-deprecated-declarations)
  endif()

  ament_add_gtest(test_bullet_continuous_collision_checking test/test_bullet_continuous_collision_checking.cpp)
  target_link_libraries(test_bullet_continuous_collision_checking moveit_test_utils ${MOVEIT_LIB_NAME})
  # TODO: remove if transition to gtest's new API TYPED_TEST_SUITE_P is finished
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(test_bullet_continuous_collision_checking PRIVATE -Wno-deprecated-declarations)
  endif()
endif()
