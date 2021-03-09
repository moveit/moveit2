set(MOVEIT_LIB_NAME moveit_planning_scene)

add_library(${MOVEIT_LIB_NAME} SHARED src/planning_scene.cpp)
#TODO: Fix the versioning
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  Boost
  rclcpp
  urdfdom
  urdfdom_headers
  octomap_msgs
  OCTOMAP
)

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model
  moveit_robot_state
  moveit_exceptions
  moveit_transforms
  moveit_collision_detection_fcl
  moveit_collision_detection
  moveit_kinematic_constraints
  moveit_robot_trajectory
  moveit_trajectory_processing
  moveit_utils
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  if(UNIX OR APPLE)
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_CURRENT_BINARY_DIR}/../utils:${CMAKE_CURRENT_BINARY_DIR}/../collision_detection_fcl:${CMAKE_CURRENT_BINARY_DIR}/../collision_detection")
  endif()

  ament_add_gtest(test_planning_scene test/test_planning_scene.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  ament_target_dependencies(test_planning_scene
    geometric_shapes
    srdfdom
  )
  target_link_libraries(test_planning_scene moveit_test_utils ${MOVEIT_LIB_NAME})

  ament_add_gtest(test_multi_threaded test/test_multi_threaded.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_multi_threaded moveit_test_utils ${MOVEIT_LIB_NAME})
endif()
