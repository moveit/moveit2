cmake_minimum_required(VERSION 3.10.2)
project(moveit_fake_controller_manager)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/moveit_fake_controller_manager.cpp
  src/moveit_fake_controllers.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(${PROJECT_NAME}
  Boost
  moveit_core
  moveit_ros_planning
  pluginlib
  rclcpp
)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

pluginlib_export_plugin_description_file(moveit_core moveit_fake_controller_manager_plugin_description.xml)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${PROJECT_NAME})
ament_package()
