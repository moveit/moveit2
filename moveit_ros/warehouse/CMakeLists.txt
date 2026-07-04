cmake_minimum_required(VERSION 3.22)
project(moveit_ros_warehouse LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(warehouse_ros REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

# Finds Boost Components
include(ConfigExtras.cmake)

include_directories(include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    fmt
    Boost
    moveit_core
    rclcpp
    warehouse_ros
    moveit_ros_planning
    tf2_eigen
    tf2_ros)

# Libraries
add_library(
  moveit_warehouse SHARED
  src/moveit_message_storage.cpp
  src/planning_scene_storage.cpp
  src/planning_scene_world_storage.cpp
  src/constraints_storage.cpp
  src/trajectory_constraints_storage.cpp
  src/state_storage.cpp
  src/warehouse_connector.cpp)
include(GenerateExportHeader)
generate_export_header(moveit_warehouse)
target_include_directories(
  moveit_warehouse PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_warehouse
                      PROPERTIES VERSION "${moveit_ros_warehouse_VERSION}")
target_link_libraries(
  moveit_warehouse
  fmt::fmt
  Boost::headers
  moveit_core::moveit_core
  rclcpp::rclcpp
  warehouse_ros::warehouse_ros
  moveit_ros_planning::moveit_ros_planning
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros)

# Executables
add_executable(moveit_warehouse_broadcast src/broadcast.cpp)
target_link_libraries(moveit_warehouse_broadcast moveit_warehouse
                      Boost::program_options)

add_executable(moveit_save_to_warehouse src/save_to_warehouse.cpp)
target_link_libraries(moveit_save_to_warehouse moveit_warehouse fmt::fmt
                      Boost::program_options)

add_executable(moveit_warehouse_import_from_text src/import_from_text.cpp)
target_link_libraries(moveit_warehouse_import_from_text moveit_warehouse
                      Boost::program_options)

add_executable(moveit_warehouse_save_as_text src/save_as_text.cpp)
target_link_libraries(moveit_warehouse_save_as_text moveit_warehouse
                      Boost::program_options)

add_executable(moveit_init_demo_warehouse src/initialize_demo_db.cpp)
target_link_libraries(moveit_init_demo_warehouse moveit_warehouse
                      Boost::program_options)

add_executable(moveit_warehouse_services src/warehouse_services.cpp)
target_link_libraries(moveit_warehouse_services moveit_warehouse)

install(
  TARGETS moveit_save_to_warehouse moveit_warehouse_broadcast
          moveit_warehouse_import_from_text moveit_warehouse_save_as_text
          moveit_init_demo_warehouse moveit_warehouse_services
  RUNTIME DESTINATION lib/moveit_ros_warehouse)
install(DIRECTORY include/ DESTINATION include/moveit_ros_warehouse)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_warehouse_export.h
        DESTINATION include/moveit_ros_warehouse)

# Create an umbrella INTERFACE target so downstream packages can use
# moveit_ros_warehouse::moveit_ros_warehouse to link all libraries at once.
add_library(moveit_ros_warehouse INTERFACE)
target_link_libraries(moveit_ros_warehouse INTERFACE moveit_warehouse)

install(
  TARGETS moveit_ros_warehouse moveit_warehouse
  EXPORT moveit_ros_warehouseTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_warehouse)

ament_export_targets(moveit_ros_warehouseTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
