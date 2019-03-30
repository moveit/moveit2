set(MOVEIT_LIB_NAME moveit_kinematic_constraints)

add_library(${MOVEIT_LIB_NAME}
  src/kinematic_constraint.cpp
  src/utils.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  urdf
  urdfdom
  urdfdom_headers
  moveit_robot_model
  tf2_geometry_msgs
  visualization_msgs
  moveit_kinematics_base
  moveit_robot_state
  tf2_eigen
  moveit_collision_detection_fcl)

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(moveit_resources REQUIRED)
#   include_directories(${moveit_resources_INCLUDE_DIRS})
#   ament_add_gtest(test_constraints test/test_constraints.cpp)
#   target_link_libraries(test_constraints
#     moveit_test_utils
#     moveit_collision_detection
#     moveit_collision_detection_fcl
#     ${MOVEIT_LIB_NAME})
# endif()
