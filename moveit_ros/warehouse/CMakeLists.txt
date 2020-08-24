cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_warehouse)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(warehouse_ros REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED thread system filesystem regex date_time program_options)
find_package(OpenSSL)

include_directories(warehouse/include ${OPENSSL_INCLUDE_DIR})

add_subdirectory(warehouse)

ament_export_include_directories(include)
ament_export_libraries(moveit_warehouse)
ament_export_dependencies(rclcpp)
ament_export_dependencies(moveit_core)
ament_export_dependencies(warehouse_ros)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(tf2_eigen)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(Boost)

ament_package()
