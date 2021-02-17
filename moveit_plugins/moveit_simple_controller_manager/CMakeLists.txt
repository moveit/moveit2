cmake_minimum_required(VERSION 3.10.2)
project(moveit_simple_controller_manager)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(Boost REQUIRED thread)
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  rclcpp
  rclcpp_action
  moveit_core
  pluginlib
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/moveit_simple_controller_manager.cpp
  src/follow_joint_trajectory_controller_handle.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  Boost
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

pluginlib_export_plugin_description_file(moveit_core moveit_simple_controller_manager_plugin_description.xml)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  Boost
)

ament_export_include_directories(include)

ament_package()
