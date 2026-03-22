cmake_minimum_required(VERSION 3.22)
project(moveit_setup_framework LANGUAGES CXX)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(fmt REQUIRED)
# Try Qt6 first, fall back to Qt5. Avoid find_package(QT NAMES Qt6 Qt5 ...):
# CMake's ascending path resolution finds Qt5 before Qt6 when both are
# installed.
find_package(Qt6 QUIET COMPONENTS Core Widgets)
if(Qt6_FOUND)
  set(QT_VERSION_MAJOR 6)
else()
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
  set(QT_VERSION_MAJOR 5)
endif()
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_library(
  moveit_setup_framework
  src/helper_widgets.cpp
  src/double_list_widget.cpp
  src/urdf_config.cpp
  src/package_settings_config.cpp
  src/srdf_config.cpp
  src/rviz_panel.cpp
  src/data_warehouse.cpp
  src/templates.cpp
  src/utilities.cpp
  src/xml_syntax_highlighter.cpp
  include/moveit_setup_framework/qt/helper_widgets.hpp
  include/moveit_setup_framework/qt/setup_step_widget.hpp
  include/moveit_setup_framework/qt/rviz_panel.hpp
  include/moveit_setup_framework/qt/double_list_widget.hpp)
target_include_directories(
  moveit_setup_framework
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_setup_framework>)
ament_target_dependencies(
  moveit_setup_framework
  ament_index_cpp
  moveit_core
  moveit_ros_planning
  moveit_ros_visualization
  pluginlib
  Qt${QT_VERSION_MAJOR}Core
  Qt${QT_VERSION_MAJOR}Widgets
  rclcpp
  rviz_common
  rviz_rendering
  srdfdom
  urdf)

install(DIRECTORY include/ DESTINATION include/moveit_setup_framework)
install(FILES moveit_setup_framework_plugins.xml
        DESTINATION share/moveit_setup_framework)

install(DIRECTORY templates DESTINATION share/moveit_setup_framework)

ament_export_targets(moveit_setup_frameworkTargets HAS_LIBRARY_TARGET)
install(
  TARGETS moveit_setup_framework
  EXPORT moveit_setup_frameworkTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_setup_framework)

ament_export_dependencies(fmt)
ament_export_dependencies(rclcpp)
ament_export_dependencies(Qt${QT_VERSION_MAJOR})
ament_export_dependencies(moveit_common)
ament_export_dependencies(moveit_core)
ament_export_dependencies(moveit_ros_planning)
ament_export_dependencies(moveit_ros_visualization)
ament_export_dependencies(rviz_common)
ament_export_dependencies(rviz_rendering)
pluginlib_export_plugin_description_file(moveit_setup_framework
                                         moveit_setup_framework_plugins.xml)

ament_package()
