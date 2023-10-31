add_library(moveit_collision_detection SHARED
  src/allvalid/collision_env_allvalid.cpp
  src/collision_common.cpp
  src/collision_matrix.cpp
  src/collision_octomap_filter.cpp
  src/collision_tools.cpp
  src/world.cpp
  src/world_diff.cpp
  src/collision_env.cpp
  src/collision_plugin_cache.cpp
)
target_include_directories(moveit_collision_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
include(GenerateExportHeader)
generate_export_header(moveit_collision_detection)

ament_target_dependencies(moveit_collision_detection
  rclcpp
  rmw_implementation
  urdf
  urdfdom
  urdfdom_headers
  srdfdom
  visualization_msgs
  tf2_eigen
  geometric_shapes
  OCTOMAP
)
target_include_directories(moveit_collision_detection BEFORE PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
target_include_directories(moveit_collision_detection PUBLIC $<TARGET_PROPERTY:moveit_planning_scene,INCLUDE_DIRECTORIES>)

set_target_properties(moveit_collision_detection PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(moveit_collision_detection
  moveit_robot_state
  moveit_utils
)

# unit tests
if(BUILD_TESTING)
  if(WIN32)
    # TODO add window paths
  else()
    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_BINARY_DIR}/../robot_model;${CMAKE_CURRENT_BINARY_DIR}/../utils")
  endif()

  ament_add_gtest(test_world test/test_world.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_world moveit_collision_detection)

  ament_add_gtest(test_world_diff test/test_world_diff.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_world_diff moveit_collision_detection)

  ament_add_gtest(test_all_valid test/test_all_valid.cpp
    APPEND_LIBRARY_DIRS "${append_library_dirs}")
  target_link_libraries(test_all_valid moveit_collision_detection moveit_robot_model)
endif()

install(DIRECTORY include/ DESTINATION include/moveit_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_collision_detection_export.h DESTINATION include/moveit_core)
