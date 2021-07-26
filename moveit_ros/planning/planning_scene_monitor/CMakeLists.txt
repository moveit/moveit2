set(MOVEIT_LIB_NAME moveit_planning_scene_monitor)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/planning_scene_monitor.cpp
  src/current_state_monitor.cpp
  src/current_state_monitor_middleware_handle.cpp
  src/trajectory_monitor.cpp
  src/trajectory_monitor_middleware_handle.cpp
)
include(GenerateExportHeader)
generate_export_header(${MOVEIT_LIB_NAME})
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  moveit_ros_occupancy_map_monitor
  message_filters
  urdf
  pluginlib
  rclcpp
  Boost
  moveit_msgs
)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model_loader
  moveit_collision_plugin_loader
)

add_executable(demo_scene demos/demo_scene.cpp)
ament_target_dependencies(demo_scene
  rclcpp
  Boost
  moveit_msgs
  urdf
  message_filters
  pluginlib
)
target_link_libraries(demo_scene ${MOVEIT_LIB_NAME})

install(TARGETS demo_scene
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${MOVEIT_LIB_NAME}_export.h DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(current_state_monitor_tests
    test/current_state_monitor_tests.cpp
  )
  target_link_libraries(current_state_monitor_tests
    ${MOVEIT_LIB_NAME}
  )
  ament_add_gmock(trajectory_monitor_tests
    test/trajectory_monitor_tests.cpp
  )
  target_link_libraries(trajectory_monitor_tests
    ${MOVEIT_LIB_NAME}
  )
endif()
