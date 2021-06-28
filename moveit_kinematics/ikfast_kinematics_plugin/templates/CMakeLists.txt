cmake_minimum_required(VERSION 3.10.2)
project(_PACKAGE_NAME_)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(LAPACK REQUIRED)

include_directories(include)

set(IKFAST_LIBRARY_NAME _LIBRARY_NAME_)
add_library(${IKFAST_LIBRARY_NAME} SHARED src/_ROBOT_NAME___GROUP_NAME__ikfast_moveit_plugin.cpp)
ament_target_dependencies(${IKFAST_LIBRARY_NAME}
  rclcpp
  moveit_core
  pluginlib
  tf2_kdl
  orocos_kdl
  tf2_eigen
)
# suppress warnings about unused variables in OpenRave's solver code
target_compile_options(${IKFAST_LIBRARY_NAME} PRIVATE -Wno-unused-variable -Wno-unused-parameter)

install(TARGETS ${IKFAST_LIBRARY_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(moveit_core _ROBOT_NAME___GROUP_NAME__moveit_ikfast_plugin_description.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core)
ament_export_dependencies(pluginlib)
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2_kdl)
ament_export_dependencies(tf2_eigen)
ament_package()
