set(MOVEIT_LIB_NAME moveit_kinematic_constraints)

if(WIN32)
  # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
else()
  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../utils;${CMAKE_CURRENT_BINARY_DIR}/../robot_model;${CMAKE_CURRENT_BINARY_DIR}/../robot_state;${CMAKE_CURRENT_BINARY_DIR}/../collision_detection_fcl")
endif()

add_library(${MOVEIT_LIB_NAME} SHARED
  src/kinematic_constraint.cpp
  src/utils.cpp
)

set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${MOVEIT_LIB_NAME}
  urdf
  urdfdom
  urdfdom_headers
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  tf2_eigen
)

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_collision_detection_fcl
  moveit_kinematics_base
  moveit_robot_state
  moveit_robot_model
  moveit_utils
)

install(TARGETS ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(moveit_resources REQUIRED)
  find_package(resource_retriever REQUIRED)

  include_directories(${moveit_resources_INCLUDE_DIRS})

  ament_add_gtest(test_constraints test/test_constraints.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}"
  )

  target_link_libraries(test_constraints
    moveit_kinematic_constraints
    moveit_collision_detection_fcl
    moveit_robot_model
    moveit_robot_state
    moveit_utils
    moveit_test_utils
    ${geometric_shapes_LIBRARIES}
    resource_retriever::resource_retriever
    ${MOVEIT_LIB_NAME}
  )
endif()
