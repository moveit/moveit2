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
ament_target_dependencies(moveit_warehouse ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Executables
add_executable(moveit_warehouse_broadcast src/broadcast.cpp)
ament_target_dependencies(moveit_warehouse_broadcast
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_warehouse_broadcast moveit_warehouse)

add_executable(moveit_save_to_warehouse src/save_to_warehouse.cpp)
ament_target_dependencies(moveit_save_to_warehouse
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_save_to_warehouse moveit_warehouse fmt)

add_executable(moveit_warehouse_import_from_text src/import_from_text.cpp)
ament_target_dependencies(moveit_warehouse_import_from_text
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_warehouse_import_from_text moveit_warehouse)

add_executable(moveit_warehouse_save_as_text src/save_as_text.cpp)
ament_target_dependencies(moveit_warehouse_save_as_text
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_warehouse_save_as_text moveit_warehouse)

add_executable(moveit_init_demo_warehouse src/initialize_demo_db.cpp)
ament_target_dependencies(moveit_init_demo_warehouse
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_init_demo_warehouse moveit_warehouse)

add_executable(moveit_warehouse_services src/warehouse_services.cpp)
ament_target_dependencies(moveit_warehouse_services
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(moveit_warehouse_services moveit_warehouse)

install(
  TARGETS moveit_save_to_warehouse moveit_warehouse_broadcast
          moveit_warehouse_import_from_text moveit_warehouse_save_as_text
          moveit_init_demo_warehouse moveit_warehouse_services
  RUNTIME DESTINATION lib/moveit_ros_warehouse)
install(DIRECTORY include/ DESTINATION include/moveit_ros_warehouse)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_warehouse_export.h
        DESTINATION include/moveit_ros_warehouse)

install(
  TARGETS moveit_warehouse
  EXPORT moveit_ros_warehouseTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_ros_warehouse)

ament_export_targets(moveit_ros_warehouseTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
