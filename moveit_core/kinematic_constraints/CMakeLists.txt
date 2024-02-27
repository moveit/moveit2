if(WIN32)
  # set(APPEND_LIBRARY_DIRS
  # "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
else()
  set(APPEND_LIBRARY_DIRS
      "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../utils;${CMAKE_CURRENT_BINARY_DIR}/../robot_model;${CMAKE_CURRENT_BINARY_DIR}/../robot_state;${CMAKE_CURRENT_BINARY_DIR}/../collision_detection_fcl"
  )
endif()

add_library(moveit_kinematic_constraints SHARED src/kinematic_constraint.cpp
                                                src/utils.cpp)
target_include_directories(
  moveit_kinematic_constraints
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_core>)
set_target_properties(moveit_kinematic_constraints
                      PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(
  moveit_kinematic_constraints
  urdf
  urdfdom
  urdfdom_headers
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  tf2_eigen)

target_link_libraries(
  moveit_kinematic_constraints moveit_collision_detection_fcl
  moveit_kinematics_base moveit_robot_state moveit_robot_model moveit_utils)

install(DIRECTORY include/ DESTINATION include/moveit_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_constraints test/test_constraints.cpp
                  APPEND_LIBRARY_DIRS "${APPEND_LIBRARY_DIRS}")
  target_link_libraries(test_constraints moveit_test_utils
                        moveit_kinematic_constraints)

  ament_add_gtest(test_orientation_constraints
                  test/test_orientation_constraints.cpp)
  target_link_libraries(test_orientation_constraints moveit_test_utils
                        moveit_kinematic_constraints)
endif()
