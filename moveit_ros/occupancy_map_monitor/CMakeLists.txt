cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_occupancy_map_monitor)
set(MOVEIT_LIB_NAME ${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_EXTENSIONS OFF)

# Disable -Wpedantic warnings due to the warnings in ros-dashing-octomap
# TODO: add -Wpedantic warnings back when PR(https://github.com/OctoMap/octomap/pull/275) is merged
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-pedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)

if(APPLE)
  find_package(X11 REQUIRED)
endif(APPLE)

find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(geometric_shapes REQUIRED)

include_directories(include)
include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS}
                    ${X11_INCLUDE_DIR}
                    )

add_library(${MOVEIT_LIB_NAME} SHARED
  src/occupancy_map_monitor.cpp
  src/occupancy_map_updater.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${MOVEIT_LIB_NAME}
  rclcpp
  moveit_core
  moveit_msgs
  pluginlib
  octomap
  geometric_shapes
  Boost
)

add_executable(moveit_ros_occupancy_map_server src/occupancy_map_server.cpp)
ament_target_dependencies(moveit_ros_occupancy_map_server
  rclcpp
  moveit_core
  moveit_msgs
  pluginlib
  octomap
  geometric_shapes
  Boost
)
target_link_libraries(moveit_ros_occupancy_map_server ${MOVEIT_LIB_NAME})

ament_export_include_directories(include)
ament_export_interfaces(${MOVEIT_LIB_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${MOVEIT_LIB_NAME})
ament_export_dependencies(rclcpp)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_msgs)
ament_export_dependencies(pluginlib)
ament_export_dependencies(octomap)
ament_export_dependencies(geometric_shapes)
ament_export_dependencies(Boost)

install(
  TARGETS ${MOVEIT_LIB_NAME}
  EXPORT  ${MOVEIT_LIB_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS moveit_ros_occupancy_map_server
  DESTINATION lib/${PROJECT_NAME}/
)
install(DIRECTORY include/ DESTINATION include)

ament_package()