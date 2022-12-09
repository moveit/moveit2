if(WIN32)
  # set(append_library_dirs "$<TARGET_FILE_DIR:${PROJECT_NAME}>;$<TARGET_FILE_DIR:${PROJECT_NAME}_TestPlugins1>")
else()
  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../utils;${CMAKE_CURRENT_BINARY_DIR}/../robot_model;${CMAKE_CURRENT_BINARY_DIR}/../robot_state;${CMAKE_CURRENT_BINARY_DIR}/../collision_detection_fcl")
endif()

target_sources(moveit_core PRIVATE
  src/kinematic_constraint.cpp
  src/utils.cpp
)
target_include_directories(moveit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(moveit_core
  urdf
  urdfdom
  urdfdom_headers
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  tf2_eigen
)

install(DIRECTORY include/ DESTINATION include/moveit_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_constraints test/test_constraints.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}"
  )
  target_link_libraries(test_constraints moveit_core)

  ament_add_gtest(test_orientation_constraints test/test_orientation_constraints.cpp)
  target_link_libraries(test_orientation_constraints moveit_core)
endif()
