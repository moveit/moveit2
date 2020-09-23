cmake_minimum_required(VERSION 3.10.2)
project(moveit_ros_warehouse)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
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
