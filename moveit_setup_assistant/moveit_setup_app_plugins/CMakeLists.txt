cmake_minimum_required(VERSION 3.22)
project(moveit_setup_app_plugins LANGUAGES CXX)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Core)
if(Qt${QT_VERSION_MAJOR} STREQUAL "Qt6")
  find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Test Widgets Core5Compat)
elseif()
  find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Test Widgets )
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

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

qt_wrap_cpp(MOC_FILES include/moveit_setup_app_plugins/launches_widget.hpp
            include/moveit_setup_app_plugins/perception_widget.hpp)

add_library(
  moveit_setup_app_plugins
  src/launches.cpp
  src/launches_widget.cpp
  src/launches_config.cpp
  src/perception_config.cpp
  src/perception.cpp
  src/perception_widget.cpp
  ${MOC_FILES})
target_include_directories(
  moveit_setup_app_plugins
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/moveit_setup_app_plugins>)

if(Qt${QT_VERSION_MAJOR} STREQUAL "Qt6")
  target_link_libraries(
    moveit_setup_app_plugins
    PUBLIC ament_index_cpp::ament_index_cpp
        moveit_setup_framework::moveit_setup_framework pluginlib::pluginlib
        rclcpp::rclcpp
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::Core5Compat)
elseif()
  target_link_libraries(
    moveit_setup_app_plugins
    PUBLIC ament_index_cpp::ament_index_cpp
        moveit_setup_framework::moveit_setup_framework pluginlib::pluginlib
        rclcpp::rclcpp
        Qt${QT_VERSION_MAJOR}::Widgets)
endif()


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception test/test_perception.cpp)
  target_link_libraries(test_perception moveit_setup_app_plugins)
endif()

install(DIRECTORY templates DESTINATION share/moveit_setup_app_plugins)

install(
  TARGETS moveit_setup_app_plugins
  EXPORT moveit_setup_app_pluginsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include/moveit_setup_app_plugins)
install(FILES moveit_setup_framework_plugins.xml
        DESTINATION share/moveit_setup_app_plugins)
install(DIRECTORY include/ DESTINATION include/moveit_setup_app_plugins)

ament_export_targets(moveit_setup_app_pluginsTargets HAS_LIBRARY_TARGET)
pluginlib_export_plugin_description_file(moveit_setup_framework
                                         moveit_setup_framework_plugins.xml)

ament_package()
