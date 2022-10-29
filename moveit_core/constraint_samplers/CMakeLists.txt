set(MOVEIT_LIB_NAME moveit_constraint_samplers)

add_library(${MOVEIT_LIB_NAME} SHARED
  src/constraint_sampler.cpp
  src/constraint_sampler_manager.cpp
  src/constraint_sampler_tools.cpp
  src/default_constraint_samplers.cpp
  src/union_constraint_sampler.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  urdf
  urdfdom
  urdfdom_headers
  visualization_msgs
)
target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_trajectory
  moveit_robot_state
  moveit_kinematic_constraints
  moveit_kinematics_base
  moveit_planning_scene
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(orocos_kdl REQUIRED)
  find_package(angles REQUIRED)
  find_package(tf2_kdl REQUIRED)
  include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS} ${angles_INCLUDE_DIRS} ${tf2_kdl_INCLUDE_DIRS})

  if(WIN32)
    # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../utils;${CMAKE_CURRENT_BINARY_DIR}/../robot_state;${CMAKE_CURRENT_BINARY_DIR}/../planning_scene;${CMAKE_CURRENT_BINARY_DIR}/../robot_model;${CMAKE_CURRENT_BINARY_DIR}/../kinematics_constraint")
  endif()

  ament_add_gmock(test_constraint_samplers
    test/test_constraint_samplers.cpp
    test/pr2_arm_kinematics_plugin.cpp
    test/pr2_arm_ik.cpp
  )
  target_include_directories(test_constraint_samplers PUBLIC
    ${geometry_msgs_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_constraint_samplers kdl_parser)
  target_link_libraries(test_constraint_samplers
    moveit_test_utils
    moveit_utils
    ${MOVEIT_LIB_NAME}
  )
endif()
