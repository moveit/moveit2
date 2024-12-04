add_library(moveit_planning_scene SHARED src/planning_scene.cpp)
target_include_directories(
  moveit_planning_scene
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_core>)
include(GenerateExportHeader)
generate_export_header(moveit_planning_scene)
target_include_directories(
  moveit_planning_scene PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
# TODO: Fix the versioning
set_target_properties(moveit_planning_scene
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(
  moveit_planning_scene
  Boost
  rclcpp
  urdfdom
  urdfdom_headers
  octomap_msgs
  octomap)

target_link_libraries(
  moveit_planning_scene
  moveit_robot_model
  moveit_robot_state
  moveit_exceptions
  moveit_transforms
  moveit_collision_detection_fcl
  moveit_collision_detection
  moveit_kinematic_constraints
  moveit_robot_trajectory
  moveit_trajectory_processing
  moveit_utils)

install(DIRECTORY include/ DESTINATION include/moveit_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_planning_scene_export.h
        DESTINATION include/moveit_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  if(UNIX OR APPLE)
    set(APPEND_LIBRARY_DIRS
        "${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_CURRENT_BINARY_DIR}/../utils:${CMAKE_CURRENT_BINARY_DIR}/../collision_detection_fcl:${CMAKE_CURRENT_BINARY_DIR}/../collision_detection"
    )
  endif()

  ament_add_gtest(test_planning_scene test/test_planning_scene.cpp
                  APPEND_LIBRARY_DIRS "${APPEND_LIBRARY_DIRS}")
  ament_target_dependencies(test_planning_scene geometric_shapes srdfdom)
  target_link_libraries(test_planning_scene moveit_test_utils
                        moveit_planning_scene)

  ament_add_gtest(test_collision_objects test/test_collision_objects.cpp
                  APPEND_LIBRARY_DIRS "${APPEND_LIBRARY_DIRS}")
  target_link_libraries(test_collision_objects moveit_test_utils
                        moveit_planning_scene)

  ament_add_gtest(test_multi_threaded test/test_multi_threaded.cpp
                  APPEND_LIBRARY_DIRS "${APPEND_LIBRARY_DIRS}")
  target_link_libraries(test_multi_threaded moveit_test_utils
                        moveit_planning_scene)
endif()
