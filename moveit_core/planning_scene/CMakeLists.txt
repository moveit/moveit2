set(MOVEIT_LIB_NAME moveit_planning_scene)

add_library(${MOVEIT_LIB_NAME} SHARED src/planning_scene.cpp)
#TODO: Fix the versioning
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_include_directories(${MOVEIT_LIB_NAME} PUBLIC
  ${octomap_msgs_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS}
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
  ${LIBOCTOMAP_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rmw_implementation_LIBRARIES}
  ${urdfdom_LIBRARIES}
  ${urdfdom_headers_LIBRARIES}
  ${Boost_LIBRARIES}
  ${octomap_msgs_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(moveit_resources REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  if(UNIX OR APPLE)
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_CURRENT_BINARY_DIR}/../utils:${CMAKE_CURRENT_BINARY_DIR}/../collision_detection_fcl:${CMAKE_CURRENT_BINARY_DIR}/../collision_detection")
  endif()

  ament_add_gtest(test_planning_scene test/test_planning_scene.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_planning_scene
    ${MOVEIT_LIB_NAME}
    ${srdfdom_LIBRARIES}
    ${urdf_LIBRARIES}
    ${geometric_shapes_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${ament_index_cpp_LIBRARIES}
  )

  ament_add_gtest(test_multi_threaded test/test_multi_threaded.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_multi_threaded
    moveit_planning_scene
    moveit_test_utils
    ${MOVEIT_LIB_NAME}
    ${urdfdom_LIBRARIES}
    ${urdfdom_headers_LIBRARIES})
endif()
