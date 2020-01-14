set(MOVEIT_LIB_NAME moveit_collision_detection_fcl)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/collision_common.cpp
  src/collision_env_fcl.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  rmw_implementation
  urdf
  urdfdom
  urdfdom_headers
  LIBFCL
  Boost
  visualization_msgs
)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_collision_detection
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
  ${MOVEIT_LIB_NAME}
)

install(TARGETS ${MOVEIT_LIB_NAME} collision_detector_fcl_plugin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})
  find_package(resource_retriever REQUIRED)

  if(WIN32)
    # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../collision_detection;${CMAKE_CURRENT_BINARY_DIR}/../robot_state;${CMAKE_CURRENT_BINARY_DIR}/../robot_model;${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  ament_add_gtest(test_fcl_collision_detection test/test_fcl_collision_detection.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")

  target_link_libraries(test_fcl_collision_detection
    ${urdf_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${srdfdom_LIBRARIES}
    moveit_test_utils
    moveit_robot_model
    moveit_robot_state
    moveit_collision_detection
    moveit_collision_detection_fcl
    ${MOVEIT_LIB_NAME}
    ${Boost_LIBRARIES}
    ${geometric_shapes_LIBRARIES}
    resource_retriever::resource_retriever
  )

  ament_add_gtest(test_fcl_collision_env test/test_fcl_env.cpp)
  target_link_libraries(test_fcl_collision_env
    moveit_test_utils
    ${MOVEIT_LIB_NAME}
    ${Boost_LIBRARIES}
    moveit_robot_state
  )
endif()
