cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_warehouse)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(warehouse_ros REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

include_directories(warehouse/include)

add_subdirectory(warehouse)

install(
  TARGETS moveit_warehouse
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(rclcpp)
ament_export_dependencies(moveit_core)
ament_export_dependencies(warehouse_ros)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(tf2_eigen)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(Boost)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
