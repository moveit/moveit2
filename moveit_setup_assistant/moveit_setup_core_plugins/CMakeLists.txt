cmake_minimum_required(VERSION 3.22)
project(moveit_setup_core_plugins LANGUAGES CXX)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
if(Qt${QT_VERSION_MAJOR} STREQUAL "Qt6")
  find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Core5Compat)
elseif()
  find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets )
endif()

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(moveit_setup_framework REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

qt_wrap_cpp(
  MOC_FILES include/moveit_setup_core_plugins/start_screen_widget.hpp
  include/moveit_setup_core_plugins/configuration_files_widget.hpp
  include/moveit_setup_core_plugins/author_information_widget.hpp)

add_library(
  ${PROJECT_NAME}
  src/start_screen.cpp
  src/start_screen_widget.cpp
  src/configuration_files.cpp
  src/configuration_files_widget.cpp
  src/author_information.cpp
  src/author_information_widget.cpp
  ${MOC_FILES})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/moveit_setup_core_plugins>)
if(Qt${QT_VERSION_MAJOR} STREQUAL "Qt6")
  target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
      ament_index_cpp::ament_index_cpp
      pluginlib::pluginlib
      rclcpp::rclcpp
      srdfdom::srdfdom
      urdf::urdf
      Qt${QT_VERSION_MAJOR}::Widgets
      Qt${QT_VERSION_MAJOR}::Core5Compat
      moveit_setup_framework::moveit_setup_framework)
elseif()
  target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
      ament_index_cpp::ament_index_cpp
      pluginlib::pluginlib
      rclcpp::rclcpp
      srdfdom::srdfdom
      urdf::urdf
      Qt${QT_VERSION_MAJOR}::Widgets
      moveit_setup_framework::moveit_setup_framework)
endif()


install(
  TARGETS moveit_setup_core_plugins
  EXPORT moveit_setup_core_pluginsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_setup_core_plugins)
install(FILES moveit_setup_framework_plugins.xml
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/moveit_setup_core_plugins)

ament_export_targets(moveit_setup_core_pluginsTargets HAS_LIBRARY_TARGET)
pluginlib_export_plugin_description_file(moveit_setup_framework
                                         moveit_setup_framework_plugins.xml)

ament_package()
