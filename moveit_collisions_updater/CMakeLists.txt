cmake_minimum_required(VERSION 3.10.2)
project(moveit_collisions_updater)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(srdfdom REQUIRED)
find_package(ompl REQUIRED)


# Tools Library
add_library(${PROJECT_NAME}_tools
  src/tools/compute_default_collisions.cpp
  src/tools/moveit_config_data.cpp
)
target_include_directories(${PROJECT_NAME}_tools PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OMPL_INCLUDE_DIRS}
)
ament_target_dependencies(
  ${PROJECT_NAME}_tools
  "moveit_core"
  "moveit_ros_planning"
  "moveit_ros_visualization"
  "rclcpp"
  "urdf"
  "srdfdom"
)

add_executable(${PROJECT_NAME} src/collisions_updater.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_tools
)

ament_target_dependencies(
  ${PROJECT_NAME}
  "moveit_core"
  "moveit_ros_planning"
  "moveit_ros_visualization"
  "rclcpp"
  "urdf"
  "srdfdom"
)

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION
    lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_tools
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Unit tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
